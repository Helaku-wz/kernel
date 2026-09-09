// Copyright (c) 2025 vivo Mobile Communication Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use crate::{
    devices::{Device, DeviceClass},
    error::{code, Error},
    vfs::{
        dcache::Dcache,
        dirent::DirBufferReader,
        file::FileAttr,
        fs::{FileSystem, FileSystemInfo},
        inode::{InodeAttr, InodeNo, InodeOps},
        inode_mode::{InodeFileType, InodeMode},
        utils::NAME_MAX,
    },
};
use alloc::{
    boxed::Box,
    collections::BTreeMap,
    string::String,
    sync::{Arc, Weak},
    vec::Vec,
};
use core::{
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    time::Duration,
};
use delegate::delegate;
use log::{debug, trace, warn};
use spin::RwLock;

static MAGIC: usize = 0x01021994;
const ROOT_INO: InodeNo = 1;
const BLOCK_SIZE: usize = 4096;

/// Chunked backing store for a tmpfs regular file.
///
/// File contents are held as a `Vec` of fixed-size `BLOCK_SIZE` chunks rather
/// than a single contiguous `Vec<u8>`. Appending data therefore only ever
/// allocates one 4 KiB block at a time; the previous design grew one `Vec`
/// whose doubling strategy (`resize`/`reserve`) reached a 64 KiB realloc that
/// could not be satisfied on small, fragmented heaps (e.g. the 449 KiB HP RAM
/// on esp32c6), aborting with "memory allocation of 65536 bytes failed".
///
/// `len` is the logical file length in bytes (`<= chunks * BLOCK_SIZE`); the
/// tail of the last chunk is zero-filled up to the chunk boundary.
#[derive(Debug, Default)]
struct ChunkedFile {
    chunks: Vec<Box<[u8; BLOCK_SIZE]>>,
    len: usize,
}

impl ChunkedFile {
    fn new() -> Self {
        Self::default()
    }

    fn len(&self) -> usize {
        self.len
    }

    /// Logical capacity across all allocated chunks.
    #[allow(dead_code)]
    fn capacity(&self) -> usize {
        self.chunks.len() * BLOCK_SIZE
    }

    /// Extend the chunk list so that `needed` bytes are addressable,
    /// appending zero-filled chunks as needed. Never reallocates an existing
    /// chunk; each new chunk is an independent `Box` allocation.
    fn ensure_capacity(&mut self, needed: usize) {
        let needed_chunks = needed.div_ceil(BLOCK_SIZE);
        while self.chunks.len() < needed_chunks {
            self.chunks.push(Box::new([0u8; BLOCK_SIZE]));
        }
    }

    /// Grow or shrink the logical file length to `new_len`, allocating or
    /// freeing whole chunks as appropriate and zero-filling the tail.
    fn resize(&mut self, new_len: usize) {
        self.ensure_capacity(new_len);
        if new_len < self.len {
            // Zero the freed tail within the last kept chunk so subsequent
            // growth reads back zeros instead of stale data.
            let last_chunk = new_len / BLOCK_SIZE;
            let tail_start = new_len % BLOCK_SIZE;
            if tail_start != 0 {
                self.chunks[last_chunk][tail_start..].fill(0);
            }
            let drop_from = new_len.div_ceil(BLOCK_SIZE);
            self.chunks.truncate(drop_from);
        }
        self.len = new_len;
    }

    /// Copy `buf` into the file at `offset`, growing the store as needed.
    fn write_at(&mut self, offset: usize, buf: &[u8]) {
        let end = offset + buf.len();
        if end > self.len {
            self.ensure_capacity(end);
            self.len = end;
        }
        let mut written = 0;
        while written < buf.len() {
            let abs = offset + written;
            let ci = abs / BLOCK_SIZE;
            let off = abs % BLOCK_SIZE;
            let take = (BLOCK_SIZE - off).min(buf.len() - written);
            self.chunks[ci][off..off + take].copy_from_slice(&buf[written..written + take]);
            written += take;
        }
    }

    /// Copy up to `buf.len()` bytes starting at `offset` into `buf`, clamped to
    /// the current file length.
    fn read_at(&self, offset: usize, buf: &mut [u8]) -> usize {
        let read_pos = self.len.min(offset);
        let read_end = self.len.min(offset + buf.len());
        let read_size = read_end - read_pos;
        let mut read = 0;
        while read < read_size {
            let abs = read_pos + read;
            let ci = abs / BLOCK_SIZE;
            let off = abs % BLOCK_SIZE;
            let take = (BLOCK_SIZE - off).min(read_size - read);
            buf[read..read + take].copy_from_slice(&self.chunks[ci][off..off + take]);
            read += take;
        }
        read_size
    }
}

#[derive(Debug)]
enum TmpFileData {
    Directory(TmpDir),
    File(ChunkedFile),
    Device(Arc<dyn Device>),
    // TODO: support symlink
    // SymLink(String),
    Socket(),
}

#[derive(Debug)]
pub struct TmpFileSystem {
    root: Arc<TmpInode>,
    // Next available inode number
    next_inode_no: AtomicUsize,
    fs_info: FileSystemInfo,
    is_mounted: AtomicBool,
}

impl TmpFileSystem {
    pub fn new() -> Arc<Self> {
        Arc::new_cyclic(|weak_fs| Self {
            root: Arc::new_cyclic(|weak_root| TmpInode {
                inner: RwLock::new(InnerNode {
                    attr: InodeAttr::new(
                        ROOT_INO,
                        InodeFileType::Directory,
                        InodeMode::from_bits_truncate(0o755),
                        0,
                        0,
                        0,
                    ),
                    data: TmpFileData::Directory(TmpDir::new(weak_root)),
                }),
                this: weak_root.clone(),
                fs: weak_fs.clone(),
            }),
            next_inode_no: AtomicUsize::new(ROOT_INO + 1),
            is_mounted: AtomicBool::new(false),
            fs_info: FileSystemInfo::new(MAGIC, 0, NAME_MAX, BLOCK_SIZE, 0),
        })
    }

    /// Allocate new inode number
    fn alloc_inode_no(&self) -> InodeNo {
        self.next_inode_no.fetch_add(1, Ordering::Relaxed)
    }

    fn check_mounted(&self) -> bool {
        self.is_mounted.load(Ordering::Relaxed)
    }
}

impl FileSystem for TmpFileSystem {
    fn mount(&self, _mount_point: Arc<Dcache>) -> Result<(), Error> {
        if self.check_mounted() {
            warn!("Filesystem already mounted {:?}", self);
            return Err(code::EBUSY);
        }

        self.is_mounted.store(true, Ordering::Relaxed);
        Ok(())
    }
    fn unmount(&self) -> Result<(), Error> {
        if !self.check_mounted() {
            return Err(code::EINVAL);
        }
        self.is_mounted.store(false, Ordering::Relaxed);
        Ok(())
    }
    fn sync(&self) -> Result<(), Error> {
        // do nothing
        Ok(())
    }
    fn root_inode(&self) -> Arc<dyn InodeOps> {
        self.root.clone()
    }
    fn fs_info(&self) -> FileSystemInfo {
        self.fs_info.clone()
    }
    fn fs_type(&self) -> &str {
        "tmpfs"
    }
}

#[derive(Debug)]
struct TmpDir {
    parent: Weak<TmpInode>,
    children: BTreeMap<String, Arc<TmpInode>>,
}

impl TmpDir {
    fn new(parent: &Weak<TmpInode>) -> Self {
        Self {
            parent: parent.clone(),
            children: BTreeMap::new(),
        }
    }

    fn find(&self, name: &str) -> Option<Arc<TmpInode>> {
        self.children.get(name).cloned()
    }

    fn insert(&mut self, name: &str, inode: &Arc<TmpInode>) {
        self.children.insert(String::from(name), inode.clone());
    }

    fn remove(&mut self, name: &str) {
        self.children.remove(name);
    }
}

/// Inode in temporary filesystem
#[derive(Debug)]
struct TmpInode {
    inner: RwLock<InnerNode>,
    // use for link and unlink
    this: Weak<TmpInode>,
    // fs is hold by mount point, so we use weak reference here
    fs: Weak<TmpFileSystem>,
}

impl TmpInode {
    fn new_file(
        fs: &Weak<TmpFileSystem>,
        inode_no: InodeNo,
        mode: InodeMode,
        uid: u32,
        gid: u32,
    ) -> Arc<Self> {
        Arc::new_cyclic(|weak_inode| Self {
            inner: RwLock::new(InnerNode {
                attr: InodeAttr::new(inode_no, InodeFileType::Regular, mode, uid, gid, 0),
                data: TmpFileData::File(ChunkedFile::new()),
            }),
            this: weak_inode.clone(),
            fs: fs.clone(),
        })
    }

    fn new_dir(
        fs: &Weak<TmpFileSystem>,
        inode_no: InodeNo,
        mode: InodeMode,
        uid: u32,
        gid: u32,
        parent: &Weak<TmpInode>,
    ) -> Arc<Self> {
        Arc::new_cyclic(|weak_inode| Self {
            inner: RwLock::new(InnerNode {
                attr: InodeAttr::new(inode_no, InodeFileType::Directory, mode, uid, gid, 0),
                data: TmpFileData::Directory(TmpDir::new(parent)),
            }),
            this: weak_inode.clone(),
            fs: fs.clone(),
        })
    }

    fn new_device(
        fs: &Weak<TmpFileSystem>,
        inode_no: InodeNo,
        mode: InodeMode,
        uid: u32,
        gid: u32,
        device: Arc<dyn Device>,
    ) -> Arc<Self> {
        Arc::new_cyclic(|weak_inode| Self {
            inner: RwLock::new(InnerNode {
                attr: InodeAttr::new(
                    inode_no,
                    InodeFileType::from(device.class()),
                    mode,
                    uid,
                    gid,
                    0,
                ),
                data: TmpFileData::Device(device),
            }),
            this: weak_inode.clone(),
            fs: fs.clone(),
        })
    }

    fn new_socket(
        fs: &Weak<TmpFileSystem>,
        inode_no: InodeNo,
        mode: InodeMode,
        uid: u32,
        gid: u32,
    ) -> Arc<Self> {
        Arc::new_cyclic(|weak_inode| Self {
            inner: RwLock::new(InnerNode {
                attr: InodeAttr::new(inode_no, InodeFileType::Socket, mode, uid, gid, 0),
                data: TmpFileData::Socket(),
            }),
            this: weak_inode.clone(),
            fs: fs.clone(),
        })
    }
}

#[derive(Debug)]
struct InnerNode {
    attr: InodeAttr,
    data: TmpFileData,
}

impl InnerNode {
    fn reported_size(&self) -> usize {
        let Some(device) = self.as_device() else {
            return self.attr.size;
        };
        if device.class() != DeviceClass::Char {
            return self.attr.size;
        }

        device
            .capacity()
            .ok()
            .and_then(|capacity| usize::try_from(capacity).ok())
            .unwrap_or(self.attr.size)
    }

    fn as_dir(&self) -> Option<&TmpDir> {
        match &self.data {
            TmpFileData::Directory(dir) => Some(dir),
            _ => None,
        }
    }

    fn as_dir_mut(&mut self) -> Option<&mut TmpDir> {
        match &mut self.data {
            TmpFileData::Directory(dir) => Some(dir),
            _ => None,
        }
    }

    fn as_device(&self) -> Option<&Arc<dyn Device>> {
        match &self.data {
            TmpFileData::Device(device) => Some(device),
            _ => None,
        }
    }

    fn as_file(&self) -> Option<&ChunkedFile> {
        match &self.data {
            TmpFileData::File(file) => Some(file),
            _ => None,
        }
    }

    fn as_file_mut(&mut self) -> Option<&mut ChunkedFile> {
        match &mut self.data {
            TmpFileData::File(file) => Some(file),
            _ => None,
        }
    }

    fn inc_nlinks(&mut self) {
        self.attr.nlinks += 1;
    }

    fn dec_nlinks(&mut self) {
        self.attr.nlinks -= 1;
    }

    fn inc_size(&mut self) {
        self.attr.size += 1;
        self.attr.blocks = self.attr.size.div_ceil(BLOCK_SIZE);
    }

    fn dec_size(&mut self) {
        self.attr.size -= 1;
        self.attr.blocks = self.attr.size.div_ceil(BLOCK_SIZE);
    }
}

impl InodeOps for TmpInode {
    fn create(
        &self,
        name: &str,
        type_: InodeFileType,
        mode: InodeMode,
    ) -> Result<Arc<dyn InodeOps>, Error> {
        debug_assert!(self.type_() == InodeFileType::Directory);
        if name.len() > NAME_MAX {
            return Err(code::ENAMETOOLONG);
        }
        if name == "." || name == ".." {
            return Err(code::EEXIST);
        }

        let mut inner = self.inner.write();
        let dir = inner.as_dir_mut().unwrap();
        if dir.find(name).is_some() {
            return Err(code::EEXIST);
        }

        let ino = self.fs.upgrade().unwrap().alloc_inode_no();
        let inode = match type_ {
            InodeFileType::Directory => TmpInode::new_dir(&self.fs, ino, mode, 0, 0, &self.this),
            InodeFileType::Regular => TmpInode::new_file(&self.fs, ino, mode, 0, 0),
            _ => {
                warn!("create: unsupported file type: {:?}", type_);
                return Err(code::EINVAL);
            }
        };
        dir.insert(name, &inode);
        if type_ == InodeFileType::Directory {
            inner.inc_nlinks();
        }
        inner.inc_size();

        Ok(inode)
    }

    fn create_device(
        &self,
        name: &str,
        mode: InodeMode,
        device: Arc<dyn Device>,
    ) -> Result<Arc<dyn InodeOps>, Error> {
        debug_assert!(self.type_() == InodeFileType::Directory);
        if name.len() > NAME_MAX {
            return Err(code::ENAMETOOLONG);
        }

        if name == "." || name == ".." {
            return Err(code::EEXIST);
        }

        let mut inner = self.inner.write();
        let dir = inner.as_dir_mut().unwrap();
        if dir.find(name).is_some() {
            return Err(code::EEXIST);
        }

        device.open()?;

        let ino = self.fs.upgrade().unwrap().alloc_inode_no();
        let inode = TmpInode::new_device(&self.fs, ino, mode, 0, 0, device);
        dir.insert(name, &inode);
        inner.inc_size();

        Ok(inode)
    }

    fn create_socket(&self, mode: InodeMode) -> Result<Arc<dyn InodeOps>, Error> {
        debug_assert!(self.type_() == InodeFileType::Directory);
        let mut inner = self.inner.write();
        let ino = self.fs.upgrade().unwrap().alloc_inode_no();
        let inode = TmpInode::new_socket(&self.fs, ino, mode, 0, 0);
        inner.inc_size();

        Ok(inode)
    }

    fn ioctl(&self, cmd: u32, arg: usize) -> Result<i32, Error> {
        let inner = self.inner.read();
        // when operate an socket, this line should change
        let Some(device) = inner.as_device() else {
            return Err(code::ENOTTY);
        };
        device.ioctl(cmd, arg).map_err(Error::from)?;
        Ok(0)
    }

    fn close(&self) -> Result<(), Error> {
        let inner = self.inner.read();
        if let Some(device) = inner.as_device() {
            device.close()?;
        }
        Ok(())
    }

    fn read_at(&self, offset: usize, buf: &mut [u8], nonblock: bool) -> Result<usize, Error> {
        let inner = self.inner.read();
        if let Some(device) = inner.as_device() {
            return device
                .read(offset as u64, buf, nonblock)
                .map_err(Error::from);
        }

        let Some(data) = inner.as_file() else {
            warn!("read_at: inode is not a file");
            return Err(code::EISDIR);
        };
        debug_assert!(data.len() == inner.attr.size);
        let read_size = data.read_at(offset, buf);

        Ok(read_size)
    }

    fn write_at(&self, offset: usize, buf: &[u8], nonblock: bool) -> Result<usize, Error> {
        let mut inner = self.inner.write();
        if let Some(device) = inner.as_device() {
            return device
                .write(offset as u64, buf, nonblock)
                .map_err(Error::from);
        }

        let write_end = offset + buf.len();
        let need_resize;
        {
            let file_size = inner.attr.size;
            let Some(data) = inner.as_file_mut() else {
                warn!("write_at: inode is not a file");
                return Err(code::EISDIR);
            };

            need_resize = write_end > file_size;
            data.write_at(offset, buf);
        }
        if need_resize {
            inner.attr.size = write_end;
            inner.attr.blocks = inner.attr.size.div_ceil(BLOCK_SIZE);
        }

        Ok(buf.len())
    }

    fn link(&self, old: &Arc<dyn InodeOps>, name: &str) -> Result<(), Error> {
        if let Some(fs) = self.fs() {
            if let Some(old_fs) = old.fs() {
                if !Arc::ptr_eq(&fs, &old_fs) {
                    debug!("link: cannot link across filesystems");
                    return Err(code::EXDEV);
                }
            } else {
                return Err(code::EAGAIN);
            }
        } else {
            return Err(code::EAGAIN);
        }

        let mut inner = self.inner.write();
        let Some(dir) = inner.as_dir_mut() else {
            debug!("link: inode is not a directory");
            return Err(code::ENOTDIR);
        };

        if name == "." || name == ".." {
            return Err(code::EEXIST);
        }

        if dir.find(name).is_some() {
            debug!("link: {} already exists", name);
            return Err(code::EEXIST);
        }

        let old = old.downcast_ref::<TmpInode>().unwrap();
        let mut old_inner = old.inner.write();
        if old_inner.attr.type_() == InodeFileType::Directory {
            debug!("link: cannot link directory");
            return Err(code::EPERM);
        }

        dir.insert(name, &old.this.upgrade().unwrap());
        inner.inc_size();
        old_inner.inc_nlinks();
        Ok(())
    }

    fn unlink(&self, name: &str) -> Result<(), Error> {
        if name == "." || name == ".." {
            return Err(code::EISDIR);
        }

        let mut inner = self.inner.write();
        let Some(dir) = inner.as_dir_mut() else {
            debug!("unlink: inode is not a directory");
            return Err(code::ENOTDIR);
        };

        let inode = dir.find(name).ok_or(code::ENOENT)?;
        let mut target = inode.inner.write();
        if target.attr.type_() == InodeFileType::Directory {
            debug!("unlink: cannot unlink directory");
            return Err(code::EPERM);
        }

        dir.remove(name);
        inner.dec_size();
        target.dec_nlinks();
        Ok(())
    }
    fn rmdir(&self, name: &str) -> Result<(), Error> {
        if name == "." || name == ".." {
            debug!("rmdir: cannot remove on {}", name);
            return Err(code::EINVAL);
        }

        let mut inner = self.inner.write();
        let Some(dir) = inner.as_dir_mut() else {
            debug!("rmdir: inode is not a directory");
            return Err(code::ENOTDIR);
        };

        let inode = dir.find(name).ok_or(code::ENOENT)?;
        let mut target = inode.inner.write();
        if target.attr.type_() != InodeFileType::Directory {
            debug!("rmdir: not a directory");
            return Err(code::ENOTDIR);
        }
        let target_dir = target.as_dir().unwrap();
        if !target_dir.children.is_empty() {
            debug!("rmdir: directory not empty");
            return Err(code::ENOTEMPTY);
        }
        dir.remove(name);
        inner.dec_size();
        inner.dec_nlinks();
        // dir is start with 2 links
        target.dec_nlinks();
        target.dec_nlinks();
        Ok(())
    }

    fn getdents_at(&self, offset: usize, reader: &mut DirBufferReader) -> Result<usize, Error> {
        let inner = self.inner.read();
        let Some(dir) = inner.as_dir() else {
            warn!("getdents_at: inode is not a directory");
            return Err(code::ENOTDIR);
        };

        let mut count = 0;
        let mut current_offset = offset;

        // Handle special entries (., ..)
        if current_offset == 0 {
            match reader.write_node(
                inner.attr.ino(),
                current_offset as i64,
                inner.attr.type_(),
                ".",
            ) {
                Ok(_) => {
                    count += 1;
                    current_offset += 1;
                }
                Err(e) => return Err(e),
            }
        }

        if current_offset == 1 {
            if let Err(e) = reader.write_node(
                inner.attr.ino(),
                current_offset as i64,
                inner.attr.type_(),
                "..",
            ) {
                if count == 0 {
                    return Err(e);
                }
                return Ok(count);
            }
            count += 1;
            current_offset += 1;
        }

        let start_idx = current_offset.saturating_sub(2);
        for (name, inode) in dir.children.iter().skip(start_idx) {
            let attr = inode.inode_attr();
            match reader.write_node(attr.ino(), current_offset as i64, attr.type_(), name) {
                Ok(_) => {
                    count += 1;
                    current_offset += 1;
                }
                Err(e) => {
                    if count == 0 {
                        return Err(e);
                    }
                    return Ok(count);
                }
            }
        }

        Ok(count)
    }

    fn resize(&self, size: usize) -> Result<(), Error> {
        let mut inner = self.inner.write();
        let Some(data) = inner.as_file_mut() else {
            warn!("resize: inode is not a file");
            return Err(code::EISDIR);
        };
        data.resize(size);
        inner.attr.size = size;
        inner.attr.blocks = size.div_ceil(BLOCK_SIZE);
        Ok(())
    }

    fn inode_attr(&self) -> InodeAttr {
        self.inner.read().attr.clone()
    }

    fn file_attr(&self) -> FileAttr {
        let inner = self.inner.read();
        let mut attr = inner.attr.clone();
        attr.size = inner.reported_size();

        match self.fs() {
            Some(fs) => {
                let dev = fs.fs_info().dev;
                let rdev: usize = if let Some(device) = inner.as_device() {
                    device.id().into()
                } else {
                    0
                };
                FileAttr::new(dev, rdev, &attr)
            }
            None => FileAttr::new(0, 0, &attr),
        }
    }

    fn lookup(&self, name: &str) -> Result<Arc<dyn InodeOps>, Error> {
        let inner = self.inner.read();
        let Some(dir) = inner.as_dir() else {
            debug!("lookup: inode is not a directory");
            return Err(code::ENOTDIR);
        };
        let inode = dir.find(name).ok_or(code::ENOENT)?;
        Ok(inode)
    }

    fn fs(&self) -> Option<Arc<dyn FileSystem>> {
        match self.fs.upgrade() {
            Some(fs) => Some(fs),
            None => None,
        }
    }

    fn size(&self) -> usize {
        self.inner.read().reported_size()
    }

    delegate! {
        to self.inner.read().attr {
            fn ino(&self) -> InodeNo;
            fn type_(&self) -> InodeFileType;
            fn mode(&self) -> InodeMode;
            fn atime(&self) -> Duration;
            fn mtime(&self) -> Duration;
        }

        to self.inner.write().attr {
            fn set_atime(&self, time: Duration);
            fn set_mtime(&self, time: Duration);
        }
    }
}

impl Drop for TmpInode {
    fn drop(&mut self) {
        trace!("Drop {:?}", self);
    }
}

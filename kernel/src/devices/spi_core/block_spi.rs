// Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
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

use crate::sync::KernelDelay;
use blueos_driver::spi::SpiConfig;
use blueos_hal::PlatPeri;
use embedded_hal::spi::Operation;

use crate::devices::bus::{BusInterface, BusWrapper};

pub struct BlockSpi<T: PlatPeri, G: blueos_hal::gpio::OutputPin> {
    inner: &'static T,
    cs: &'static G,
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>, G: blueos_hal::gpio::OutputPin> BlockSpi<T, G> {
    pub fn new(
        inner: &'static T,
        cs: &'static G,
        config: &SpiConfig,
    ) -> Result<Self, blueos_hal::err::HalError> {
        inner.configure(config)?;
        Ok(BlockSpi { inner, cs })
    }

    pub fn assert_cs(&self) {
        self.cs.set_low().ok();
    }

    pub fn deassert_cs(&self) {
        self.cs.set_high().ok();
    }

    pub fn read(&mut self, words: &mut [u8]) -> Result<(), crate::error::Error> {
        self.inner.read(words).map_err(|_| crate::error::code::EIO)
    }

    pub fn write(&mut self, words: &[u8]) -> Result<(), crate::error::Error> {
        self.inner.write(words).map_err(|_| crate::error::code::EIO)
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), crate::error::Error> {
        self.inner
            .transfer(read, write)
            .map_err(|_| crate::error::code::EIO)
    }

    pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), crate::error::Error> {
        self.inner
            .write(words)
            .map_err(|_| crate::error::code::EIO)?;
        self.inner.read(words).map_err(|_| crate::error::code::EIO)
    }
}

// QSPI forwarding. Available only when the inner peripheral implements Qspi
// (e.g. ESP32 GPSPI2). CS is NOT managed here — the caller (QSPI LCD driver)
// holds CS low across command + pixel stream via assert_cs/deassert_cs, so a
// single CS-low window spans the whole CASET+RASET+RAMWR+pixel sequence.
impl<T: blueos_hal::spi::Spi<SpiConfig, ()> + blueos_driver::spi::Qspi, G: blueos_hal::gpio::OutputPin>
    BlockSpi<T, G>
{
    pub fn qspi_write_command(&self, cmd: u8, params: &[u8]) -> Result<(), crate::error::Error> {
        self.inner
            .qspi_write_command(cmd, params)
            .map_err(|_| crate::error::code::EIO)
    }

    pub fn qspi_write_pixels(&self, pixels: &[u8]) -> Result<(), crate::error::Error> {
        self.inner
            .qspi_write_pixels(pixels)
            .map_err(|_| crate::error::code::EIO)
    }

    pub fn qspi_read_command(&self, cmd: u8, buf: &mut [u8]) -> Result<(), crate::error::Error> {
        self.inner
            .qspi_read_command(cmd, buf)
            .map_err(|_| crate::error::code::EIO)
    }
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>, G: blueos_hal::gpio::OutputPin> BusInterface
    for BlockSpi<T, G>
{
}

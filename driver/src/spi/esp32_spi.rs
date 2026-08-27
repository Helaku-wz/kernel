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

//! ESP32-C3 GPSPI2 (SPI2) register-level driver.

use crate::spi::{SpiBitOrder, SpiConfig, SpiPhase, SpiPolarity};
use blueos_hal::{Configuration, PlatPeri};
use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::ReadWrite,
};

const SPI2_DATA_BUF_SIZE: usize = 64;

// Pad byte for full-duplex reads where read length exceeds write length.
const EMPTY_WRITE_PAD: u8 = 0x00;

// Command bits are hardware-owned and must clear before the next operation.
const SPI_CMD_TIMEOUT: usize = 10_000;

// Diagnostic counters: how many times start_transfer / wait_done ran their
// busy-wait to the SPI_CMD_TIMEOUT cap without the expected bit clearing.
// Non-zero means SPI transactions are not completing -- the USR bit never
// self-cleared (start_transfer) or TRANS_DONE never asserted (wait_done). The
// CO5300 init() path swallows wait_done timeouts via .ok(), so without these
// counters the only symptom is "init takes very long then prints Ok" or hangs
// silently. Boards init reads these back via kearly_println to localize the
// fault from serial output alone.
static USR_TIMEOUTS: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
static TRANS_DONE_TIMEOUTS: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

pub fn spi_diag_timeouts() -> (usize, usize) {
    (
        USR_TIMEOUTS.load(core::sync::atomic::Ordering::Relaxed),
        TRANS_DONE_TIMEOUTS.load(core::sync::atomic::Ordering::Relaxed),
    )
}

fn wait_until_clear(
    mut is_set: impl FnMut() -> bool,
    max_polls: usize,
) -> blueos_hal::err::Result<()> {
    for _ in 0..max_polls {
        if !is_set() {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(blueos_hal::err::HalError::Timeout)
}

register_bitfields! [
    u32,

    pub CMD [
        USR OFFSET(24) NUMBITS(1) [],
        UPDATE OFFSET(23) NUMBITS(1) [],
    ],

    pub CTRL [
        // Quad/Dual phase width controls. C3 and C6 share identical bit positions.
        // Address/command phase widths live in CTRL; write-data width lives in USER (FWRITE_*).
        FADDR_DUAL OFFSET(5) NUMBITS(1) [],
        FADDR_QUAD OFFSET(6) NUMBITS(1) [],
        FCMD_DUAL OFFSET(8) NUMBITS(1) [],
        FCMD_QUAD OFFSET(9) NUMBITS(1) [],
        FREAD_DUAL OFFSET(14) NUMBITS(1) [],
        FREAD_QUAD OFFSET(15) NUMBITS(1) [],
        // C3: 1-bit; C6: 2-bit (bits 23:24 / 25:26). Low bit semantics identical (0=MSB).
        RD_BIT_ORDER OFFSET(25) NUMBITS(1) [
            MsbFirst = 0,
            LsbFirst = 1,
        ],
        WR_BIT_ORDER OFFSET(26) NUMBITS(1) [
            MsbFirst = 0,
            LsbFirst = 1,
        ],
    ],

    pub CLOCK [
        CLKCNT_L OFFSET(0) NUMBITS(6) [],
        CLKCNT_H OFFSET(6) NUMBITS(6) [],
        CLKCNT_N OFFSET(12) NUMBITS(6) [],
        CLKDIV_PRE OFFSET(18) NUMBITS(4) [],
        CLK_EQU_SYSCLK OFFSET(31) NUMBITS(1) [],
    ],

    pub USER [
        DOUTDIN OFFSET(0) NUMBITS(1) [
            HalfDuplex = 0,
            FullDuplex = 1,
        ],
        // Write-data phase width. Quad write (0x32 pixel stream) sets FWRITE_QUAD.
        FWRITE_DUAL OFFSET(12) NUMBITS(1) [],
        FWRITE_QUAD OFFSET(13) NUMBITS(1) [],
        CK_OUT_EDGE OFFSET(9) NUMBITS(1) [
            LeadingEdge = 0,
            TrailingEdge = 1,
        ],
        USR_MOSI OFFSET(27) NUMBITS(1) [],
        USR_MISO OFFSET(28) NUMBITS(1) [],
        USR_DUMMY OFFSET(29) NUMBITS(1) [],
        USR_ADDR OFFSET(30) NUMBITS(1) [],
        USR_COMMAND OFFSET(31) NUMBITS(1) [],
    ],

    pub USER1 [
        USR_DUMMY_CYCLELEN OFFSET(0) NUMBITS(8) [],
        USR_ADDR_BITLEN OFFSET(27) NUMBITS(5) [],
    ],

    pub USER2 [
        USR_COMMAND_VALUE OFFSET(0) NUMBITS(16) [],
        USR_COMMAND_BITLEN OFFSET(28) NUMBITS(4) [],
    ],

    pub MS_DLEN [
        MS_DATA_BITLEN OFFSET(0) NUMBITS(18) [],
    ],

    pub MISC [
        CS0_DIS OFFSET(0) NUMBITS(1) [
            Enabled = 0,
            Disabled = 1,
        ],
        CS1_DIS OFFSET(1) NUMBITS(1) [
            Enabled = 0,
            Disabled = 1,
        ],
        CS2_DIS OFFSET(2) NUMBITS(1) [
            Enabled = 0,
            Disabled = 1,
        ],
        CS3_DIS OFFSET(3) NUMBITS(1) [
            Enabled = 0,
            Disabled = 1,
        ],
        CS4_DIS OFFSET(4) NUMBITS(1) [
            Enabled = 0,
            Disabled = 1,
        ],
        CS5_DIS OFFSET(5) NUMBITS(1) [
            Enabled = 0,
            Disabled = 1,
        ],
        CK_IDLE_EDGE OFFSET(29) NUMBITS(1) [
            Low = 0,
            High = 1,
        ],
        CS_KEEP_ACTIVE OFFSET(30) NUMBITS(1) [],
    ],

    pub SLAVE [
        CLK_MODE OFFSET(0) NUMBITS(2) [],
        CLK_MODE_13 OFFSET(2) NUMBITS(1) [],
        SLAVE_MODE OFFSET(26) NUMBITS(1) [
            Master = 0,
            Slave = 1,
        ],
        SOFT_RESET OFFSET(27) NUMBITS(1) [],
    ],

    pub CLK_GATE [
        CLK_EN OFFSET(0) NUMBITS(1) [],
        MST_CLK_ACTIVE OFFSET(1) NUMBITS(1) [],
        MST_CLK_SEL OFFSET(2) NUMBITS(1) [
            XtalClock = 0,
            PllClock = 1,
        ],
    ],

    pub DMA_CONF [
        DMA_RX_ENA OFFSET(27) NUMBITS(1) [],
        DMA_TX_ENA OFFSET(28) NUMBITS(1) [],
        RX_AFIFO_RST OFFSET(29) NUMBITS(1) [],
        BUF_AFIFO_RST OFFSET(30) NUMBITS(1) [],
    ],

    pub DMA_INT_RAW [
        TRANS_DONE OFFSET(12) NUMBITS(1) [],
    ],

    pub DMA_INT_CLR [
        TRANS_DONE OFFSET(12) NUMBITS(1) [],
    ],

    pub PERIP_CLK_EN0 [
        SPI2_CLK_EN OFFSET(6) NUMBITS(1) [
            Enabled = 1,
            Disabled = 0,
        ],
    ],

    pub PERIP_RST_EN0 [
        SPI2_RST OFFSET(6) NUMBITS(1) [
            NoReset = 0,
            Reset = 1,
        ],
    ],

    // C6 PCR `spi2_conf` register (at PCR base + 0xC0).
    // NOTE the inverted reset polarity vs C3: RST_EN 0=reset, 1=de-reset.
    pub PCR_SPI2_CONF [
        SPI2_CLK_EN OFFSET(0) NUMBITS(1) [
            Enabled = 1,
            Disabled = 0,
        ],
        SPI2_RST_EN OFFSET(1) NUMBITS(1) [
            Reset = 0,
            NoReset = 1,
        ],
    ],

    // C6 PCR `spi2_clkm_conf` register (at PCR base + 0xC4).
    // No clkm_div_num field exposed in PAC 0.23; div_num stays at reset 0 (passthrough).
    pub PCR_SPI2_CLKM_CONF [
        SPI2_CLKM_SEL OFFSET(20) NUMBITS(2) [
            Xtal = 0,
            Pll80M = 1,
            Fosc = 2,
        ],
        SPI2_CLKM_EN OFFSET(22) NUMBITS(1) [
            Enabled = 1,
            Disabled = 0,
        ],
    ],
];

register_structs! {
    Spi2Registers {
        (0x00 => cmd: ReadWrite<u32, CMD::Register>),
        (0x04 => addr: ReadWrite<u32>),
        (0x08 => ctrl: ReadWrite<u32, CTRL::Register>),
        (0x0C => clock: ReadWrite<u32, CLOCK::Register>),
        (0x10 => user: ReadWrite<u32, USER::Register>),
        (0x14 => user1: ReadWrite<u32, USER1::Register>),
        (0x18 => user2: ReadWrite<u32, USER2::Register>),
        (0x1C => ms_dlen: ReadWrite<u32, MS_DLEN::Register>),
        (0x20 => misc: ReadWrite<u32, MISC::Register>),
        (0x24 => din_mode: ReadWrite<u32>),
        (0x28 => din_num: ReadWrite<u32>),
        (0x2C => dout_mode: ReadWrite<u32>),
        (0x30 => dma_conf: ReadWrite<u32, DMA_CONF::Register>),
        (0x34 => dma_int_ena: ReadWrite<u32>),
        (0x38 => dma_int_clr: ReadWrite<u32, DMA_INT_CLR::Register>),
        (0x3C => dma_int_raw: ReadWrite<u32, DMA_INT_RAW::Register>),
        (0x40 => dma_int_st: ReadWrite<u32>),
        (0x44 => _reserved0),
        (0x98 => w0: ReadWrite<u32>),
        (0x9C => w1: ReadWrite<u32>),
        (0xA0 => w2: ReadWrite<u32>),
        (0xA4 => w3: ReadWrite<u32>),
        (0xA8 => w4: ReadWrite<u32>),
        (0xAC => w5: ReadWrite<u32>),
        (0xB0 => w6: ReadWrite<u32>),
        (0xB4 => w7: ReadWrite<u32>),
        (0xB8 => w8: ReadWrite<u32>),
        (0xBC => w9: ReadWrite<u32>),
        (0xC0 => w10: ReadWrite<u32>),
        (0xC4 => w11: ReadWrite<u32>),
        (0xC8 => w12: ReadWrite<u32>),
        (0xCC => w13: ReadWrite<u32>),
        (0xD0 => w14: ReadWrite<u32>),
        (0xD4 => w15: ReadWrite<u32>),
        (0xD8 => _reserved1),
        (0xE0 => slave: ReadWrite<u32, SLAVE::Register>),
        (0xE4 => slave1: ReadWrite<u32>),
        (0xE8 => clk_gate: ReadWrite<u32, CLK_GATE::Register>),
        (0xEC => _reserved2),
        (0xF0 => @END),
    }
}

// System registers for SPI2 clock gating and reset.
// C3: SYSTEM peripheral (perip_clk_en0 @ +0x10, perip_rst_en0 @ +0x18, bit6).
// C6: PCR peripheral (spi2_conf @ +0xC0, spi2_clkm_conf @ +0xC4).
#[cfg(soc_esp32c3)]
register_structs! {
    SystemRegisters {
        (0x00 => _reserved_sys0),
        (0x10 => perip_clk_en0: ReadWrite<u32, PERIP_CLK_EN0::Register>),
        (0x14 => _reserved_sys1),
        (0x18 => perip_rst_en0: ReadWrite<u32, PERIP_RST_EN0::Register>),
        (0x1C => @END),
    }
}

#[cfg(soc_esp32c6)]
register_structs! {
    SystemRegisters {
        (0x00 => _reserved_sys0),
        (0xC0 => spi2_conf: ReadWrite<u32, PCR_SPI2_CONF::Register>),
        (0xC4 => spi2_clkm_conf: ReadWrite<u32, PCR_SPI2_CLKM_CONF::Register>),
        (0xC8 => @END),
    }
}

/// ESP32-C3 GPSPI2 (SPI2) peripheral, generic over register base and APB clock.
pub struct Esp32Spi2<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32> {}

unsafe impl<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32> Send
    for Esp32Spi2<SPI_BASE, SYS_BASE, APB_HZ>
{
}
unsafe impl<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32> Sync
    for Esp32Spi2<SPI_BASE, SYS_BASE, APB_HZ>
{
}

impl<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32>
    Esp32Spi2<SPI_BASE, SYS_BASE, APB_HZ>
{
    pub const fn new() -> Self {
        Self {}
    }

    fn spi_regs() -> &'static Spi2Registers {
        unsafe { &*(SPI_BASE as *const Spi2Registers) }
    }

    fn sys_regs() -> &'static SystemRegisters {
        unsafe { &*(SYS_BASE as *const SystemRegisters) }
    }

    fn write_buf(&self, data: &[u8]) {
        debug_assert!(data.len() <= SPI2_DATA_BUF_SIZE);
        let regs = Self::spi_regs();
        let words = data.chunks(4);
        for (i, chunk) in words.enumerate() {
            let mut word = 0u32;
            for (j, byte) in chunk.iter().enumerate() {
                word |= (*byte as u32) << (j * 8);
            }
            match i {
                0 => regs.w0.set(word),
                1 => regs.w1.set(word),
                2 => regs.w2.set(word),
                3 => regs.w3.set(word),
                4 => regs.w4.set(word),
                5 => regs.w5.set(word),
                6 => regs.w6.set(word),
                7 => regs.w7.set(word),
                8 => regs.w8.set(word),
                9 => regs.w9.set(word),
                10 => regs.w10.set(word),
                11 => regs.w11.set(word),
                12 => regs.w12.set(word),
                13 => regs.w13.set(word),
                14 => regs.w14.set(word),
                15 => regs.w15.set(word),
                _ => break,
            }
        }
    }

    fn read_buf(&self, data: &mut [u8]) {
        let regs = Self::spi_regs();
        let words = [
            regs.w0.get(),
            regs.w1.get(),
            regs.w2.get(),
            regs.w3.get(),
            regs.w4.get(),
            regs.w5.get(),
            regs.w6.get(),
            regs.w7.get(),
            regs.w8.get(),
            regs.w9.get(),
            regs.w10.get(),
            regs.w11.get(),
            regs.w12.get(),
            regs.w13.get(),
            regs.w14.get(),
            regs.w15.get(),
        ];
        for (i, byte) in data.iter_mut().enumerate() {
            let word_idx = i / 4;
            let byte_idx = i % 4;
            if word_idx < words.len() {
                *byte = ((words[word_idx] >> (byte_idx * 8)) & 0xFF) as u8;
            }
        }
    }

    fn apply_config(&self) -> blueos_hal::err::Result<()> {
        let regs = Self::spi_regs();
        regs.cmd.write(CMD::UPDATE.val(1));
        wait_until_clear(|| regs.cmd.is_set(CMD::UPDATE), SPI_CMD_TIMEOUT)
    }

    // AFIFO reset is a SET-then-CLEAR pulse.
    fn reset_tx_fifo(&self) {
        let regs = Self::spi_regs();
        regs.dma_conf.modify(DMA_CONF::BUF_AFIFO_RST::SET);
        regs.dma_conf.modify(DMA_CONF::BUF_AFIFO_RST::CLEAR);
    }

    fn reset_rx_fifo(&self) {
        let regs = Self::spi_regs();
        regs.dma_conf.modify(DMA_CONF::RX_AFIFO_RST::SET);
        regs.dma_conf.modify(DMA_CONF::RX_AFIFO_RST::CLEAR);
    }

    fn reset_tx_rx_fifo(&self) {
        let regs = Self::spi_regs();
        regs.dma_conf
            .modify(DMA_CONF::BUF_AFIFO_RST::SET + DMA_CONF::RX_AFIFO_RST::SET);
        regs.dma_conf
            .modify(DMA_CONF::BUF_AFIFO_RST::CLEAR + DMA_CONF::RX_AFIFO_RST::CLEAR);
    }

    fn start_transfer(&self) -> blueos_hal::err::Result<()> {
        let regs = Self::spi_regs();
        // Sync shadow registers, clear stale TRANS_DONE, then start (USR self-clears).
        regs.cmd.modify(CMD::UPDATE.val(1));
        wait_until_clear(|| regs.cmd.is_set(CMD::UPDATE), SPI_CMD_TIMEOUT)?;
        regs.dma_int_clr.write(DMA_INT_CLR::TRANS_DONE::SET);
        regs.cmd.modify(CMD::USR.val(1));
        let res = wait_until_clear(|| regs.cmd.is_set(CMD::USR), SPI_CMD_TIMEOUT);
        if res.is_err() {
            // USR never self-cleared: the peripheral never accepted the USER
            // command. Usually means the SPI function clock is off, the
            // peripheral is held in reset, or MS_DLEN is misconfigured.
            USR_TIMEOUTS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        res
    }

    fn wait_done(&self) {
        // The USR bit self-clears when the USER transaction is *accepted*, but the
        // SPI peripheral is still shifting data out of the TX FIFO at that point.
        // ESP-IDF's spi_device_polling_transmit waits on the TRANS_DONE interrupt
        // (dma_int_raw bit12), which asserts only after the last bit has left the
        // shift register. Without this, back-to-back USER transactions (header
        // then data chunks in do_qspi_write) race: the next reset_tx_fifo can
        // purge the still-shifting bytes, silently truncating the transfer. Poll
        // TRANS_DONE here so a subsequent FIFO reset / next USR is always safe.
        //
        // C6 note: dma_int_raw.trans_done (bit12) is R/WTC/SS -- hardware sets it
        // on transaction completion independent of dma_int_ena (which only gates
        // the CPU interrupt). So if this poll times out, the transaction never
        // completed: the SPI clock/CS/quad-mode config is wrong, not the int_ena.
        let regs = Self::spi_regs();
        let res =
            wait_until_clear(|| !regs.dma_int_raw.is_set(DMA_INT_RAW::TRANS_DONE), SPI_CMD_TIMEOUT);
        if res.is_err() {
            TRANS_DONE_TIMEOUTS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        regs.dma_int_clr.write(DMA_INT_CLR::TRANS_DONE::SET);
    }

    fn configure_clock(&self, baudrate: u32) -> blueos_hal::err::Result<()> {
        let regs = Self::spi_regs();
        if baudrate >= APB_HZ {
            regs.clock.write(CLOCK::CLK_EQU_SYSCLK.val(1));
            return Ok(());
        }

        // f_spi = f_apb / ((pre+1) * (n+1)), minimum divisor = 2; prefer larger n for duty cycle.
        let divisor = (APB_HZ / baudrate).max(2);

        let mut best_pre = 0u32;
        let mut best_n_plus_one = 0u32;
        for pre in 0..16u32 {
            let n_plus_one = divisor / (pre + 1);
            if n_plus_one >= 2 && n_plus_one <= 64 {
                let actual = (pre + 1) * n_plus_one;
                if actual == divisor {
                    best_pre = pre;
                    best_n_plus_one = n_plus_one;
                    break;
                }
                if best_n_plus_one == 0 {
                    best_pre = pre;
                    best_n_plus_one = n_plus_one;
                }
            }
        }

        // No valid combination: minimum is APB/1024 (~78kHz).
        if best_n_plus_one == 0 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }

        let n = best_n_plus_one - 1;
        let h = ((best_n_plus_one / 2).max(1) - 1) as u32;
        regs.clock.write(
            CLOCK::CLKCNT_L.val(n as u32)
                + CLOCK::CLKCNT_H.val(h)
                + CLOCK::CLKCNT_N.val(n as u32)
                + CLOCK::CLKDIV_PRE.val(best_pre as u32)
                + CLOCK::CLK_EQU_SYSCLK.val(0),
        );
        Ok(())
    }

    fn do_half_duplex_write(&self, data: &[u8]) -> blueos_hal::err::Result<()> {
        if data.is_empty() {
            return Ok(());
        }
        let regs = Self::spi_regs();

        regs.user.modify(
            USER::DOUTDIN.val(0)
                + USER::USR_MOSI::SET
                + USER::USR_MISO::CLEAR
                + USER::USR_COMMAND::CLEAR
                + USER::USR_ADDR::CLEAR
                + USER::USR_DUMMY::CLEAR,
        );

        for chunk in data.chunks(SPI2_DATA_BUF_SIZE) {
            self.reset_tx_fifo();
            regs.ms_dlen
                .write(MS_DLEN::MS_DATA_BITLEN.val((chunk.len() as u32 * 8 - 1)));
            self.write_buf(chunk);
            self.start_transfer()?;
            self.wait_done();
        }
        Ok(())
    }

    fn do_half_duplex_read(&self, data: &mut [u8]) -> blueos_hal::err::Result<()> {
        if data.is_empty() {
            return Ok(());
        }
        let regs = Self::spi_regs();

        // Full-duplex dummy read (write 0x00 while reading): half-duplex MISO-only
        // returned misaligned data on real hardware.
        regs.user.modify(
            USER::DOUTDIN.val(1)
                + USER::USR_MOSI::SET
                + USER::USR_MISO::SET
                + USER::USR_COMMAND::CLEAR
                + USER::USR_ADDR::CLEAR
                + USER::USR_DUMMY::CLEAR,
        );

        for chunk in data.chunks_mut(SPI2_DATA_BUF_SIZE) {
            self.reset_tx_rx_fifo();
            regs.ms_dlen
                .write(MS_DLEN::MS_DATA_BITLEN.val((chunk.len() as u32 * 8 - 1)));
            let dummy = [EMPTY_WRITE_PAD; SPI2_DATA_BUF_SIZE];
            self.write_buf(&dummy[..chunk.len()]);
            self.start_transfer()?;
            self.wait_done();
            self.read_buf(chunk);
        }
        Ok(())
    }

    fn do_full_duplex_transfer(
        &self,
        read: &mut [u8],
        write: &[u8],
    ) -> blueos_hal::err::Result<()> {
        if read.is_empty() && write.is_empty() {
            return Ok(());
        }
        let regs = Self::spi_regs();

        regs.user.modify(
            USER::DOUTDIN.val(1)
                + USER::USR_MOSI::SET
                + USER::USR_MISO::SET
                + USER::USR_COMMAND::CLEAR
                + USER::USR_ADDR::CLEAR
                + USER::USR_DUMMY::CLEAR,
        );

        // Independent read/write cursors; pad the shorter side with EMPTY_WRITE_PAD.
        let mut write_from = 0usize;
        let mut read_from = 0usize;
        loop {
            let write_inc = core::cmp::min(SPI2_DATA_BUF_SIZE, write.len() - write_from);
            let read_inc = core::cmp::min(SPI2_DATA_BUF_SIZE, read.len() - read_from);
            if write_inc == 0 && read_inc == 0 {
                break;
            }

            let this_len = write_inc.max(read_inc);
            self.reset_tx_rx_fifo();
            regs.ms_dlen
                .write(MS_DLEN::MS_DATA_BITLEN.val((this_len as u32 * 8 - 1)));

            if write_inc < read_inc {
                // Read more than we write: pad write side up to read_inc bytes.
                let mut buf = [EMPTY_WRITE_PAD; SPI2_DATA_BUF_SIZE];
                buf[..write_inc].copy_from_slice(&write[write_from..][..write_inc]);
                self.write_buf(&buf[..read_inc]);
            } else {
                self.write_buf(&write[write_from..][..write_inc]);
            }

            self.start_transfer()?;
            self.wait_done();

            if read_inc > 0 {
                let mut tmp = [0u8; SPI2_DATA_BUF_SIZE];
                self.read_buf(&mut tmp[..read_inc]);
                read[read_from..][..read_inc].copy_from_slice(&tmp[..read_inc]);
            }

            write_from += write_inc;
            read_from += read_inc;
        }
        Ok(())
    }

    // --- QSPI support ---

    // QSPI command header format (CO5300/SH8601 "no D/CX" QSPI panels).
    // CO5300 datasheet p20 (5.2 QUAD SPI Interface) defines command-write as:
    //   Instruction[7:0] = 0x02                       (1st bus byte)
    //   AD[23:0]       = {8'h00, CMD[7:0], 8'h00}     (next 3 bus bytes)
    //   PAM[7:0]       = parameters                    (following bytes)
    // so the panel expects the 4-byte header on the bus as:
    //   [0x02, 0x00, CMD, 0x00]   -- CMD sits at byte index 2, NOT index 1.
    //
    // ESP32 SPI shifts the data buffer out low-address-byte first, so the
    // in-memory buffer must already be in bus order: [opcode, 0x00, cmd, 0x00].
    // This matches what ESP-IDF esp_lcd `tx_param` actually puts on the wire:
    // it forms the 32-bit value 0x0200_<cmd>_00 (e.g. Set_Backlight builds
    // 0x02005100), stores it little-endian as [0x00, <cmd>, 0x00, 0x02], then
    // spi_lcd_prepare_cmd_buffer reverses the whole 4-byte run (because
    // lcd_cmd_bits=32 > 8) to [0x02, 0x00, <cmd>, 0x00] before the hardware
    // shifts it out. The LE-store + reverse compose to land CMD at index 2.
    // We write the final bus order directly, skipping both steps.
    //
    // Framing is split across two USER transactions held under one CS-low span:
    //   (1) 4-byte header  -> 1-wire (FWRITE_QUAD=0), USR_MOSI only.
    //   (2) payload/pixels -> quad (FWRITE_QUAD=1) for pixel writes, else 1-wire.
    // This matches esp_lcd: command phase is always 1-wire; only the data
    // phase of a tx_color (0x32) goes 4-wire.
    const QSPI_HEADER_LEN: usize = 4;

    // Build the 4-byte command header in bus order (opcode first, CMD at idx 2).
    // See the format block above for the datasheet/ESP-IDF derivation.
    fn qspi_header(&self, opcode: u8, cmd_code: u8) -> [u8; 4] {
        [opcode, 0x00, cmd_code, 0x00]
    }

    // Send the 4-byte command header as a standalone 1-wire MOSI transaction.
    // CS is assumed held low by the caller across this and the following data
    // transaction. USR_COMMAND/USR_ADDR stay CLEAR: the header is plain data.
    fn qspi_send_header(&self, opcode: u8, cmd_code: u8) -> blueos_hal::err::Result<()> {
        let regs = Self::spi_regs();
        let header = self.qspi_header(opcode, cmd_code);
        regs.user.modify(
            USER::DOUTDIN.val(0)
                + USER::USR_MOSI::SET
                + USER::USR_MISO::CLEAR
                + USER::USR_COMMAND::CLEAR
                + USER::USR_ADDR::CLEAR
                + USER::USR_DUMMY::CLEAR
                + USER::FWRITE_QUAD.val(0),
        );
        self.reset_tx_fifo();
        regs.ms_dlen
            .write(MS_DLEN::MS_DATA_BITLEN.val((Self::QSPI_HEADER_LEN as u32 * 8 - 1)));
        self.write_buf(&header);
        self.start_transfer()?;
        self.wait_done();
        Ok(())
    }

    // QSPI write: 4-byte header [opcode, cmd, 0,0] (1-wire) then payload data.
    // `quad_data` selects 4-wire (FWRITE_QUAD=1, pixel stream via 0x32) vs
    // 1-wire for the data phase; the header phase is always 1-wire. CS is held
    // low by the caller across header + all data chunks. Large data is chunked
    // to SPI2_DATA_BUF_SIZE; the header is sent in its own transaction first.
    fn do_qspi_write(
        &self,
        opcode: u8,
        cmd_code: u8,
        data: &[u8],
        quad_data: bool,
    ) -> blueos_hal::err::Result<()> {
        // Phase 1: command header (1-wire), CS still low.
        self.qspi_send_header(opcode, cmd_code)?;

        if data.is_empty() {
            return Ok(());
        }

        // Phase 2: payload data, quad/1-wire per quad_data. CS stays low across
        // all chunks (caller holds CS); each chunk is one USER transaction.
        let regs = Self::spi_regs();
        // CTRL.FREAD_QUAD is the master enable for the quad output path: on the
        // C6 GPSPI it gates D1/D2/D3's connection to the shift register for both
        // read and write phases (read and write share one quad output enable).
        // Setting only USER.FWRITE_QUAD (as before) left D1/D2/D3 physically
        // dead while D0 still toggled, so transactions completed but the panel
        // never received a real 4-wire stream. IDF's spi_ll_master_set_line_mode
        // and esp-hal's init_spi_data_mode both set FREAD_QUAD+FWRITE_QUAD
        // together for quad writes; mirror that here.
        regs.ctrl
            .modify(CTRL::FREAD_QUAD.val(if quad_data { 1 } else { 0 }));
        for chunk in data.chunks(SPI2_DATA_BUF_SIZE) {
            regs.user.modify(
                USER::DOUTDIN.val(0)
                    + USER::USR_MOSI::SET
                    + USER::USR_MISO::CLEAR
                    + USER::USR_COMMAND::CLEAR
                    + USER::USR_ADDR::CLEAR
                    + USER::USR_DUMMY::CLEAR
                    + USER::FWRITE_QUAD.val(if quad_data { 1 } else { 0 }),
            );
            self.reset_tx_fifo();
            regs.ms_dlen
                .write(MS_DLEN::MS_DATA_BITLEN.val((chunk.len() as u32 * 8 - 1)));
            self.write_buf(chunk);
            self.start_transfer()?;
            self.wait_done();
        }
        Ok(())
    }

    // QSPI read: 4-byte header [0x03, cmd, 0,0] (1-wire MOSI) + dummy turnaround
    // + MISO read (1-wire). Header and read share one USER transaction: we
    // send the 4 header bytes on MOSI, insert 8 dummy cycles, then read N bytes
    // from MISO. CS held low by caller. Always 1-wire (FREAD_QUAD=0).
    fn do_qspi_read(
        &self,
        opcode: u8,
        cmd_code: u8,
        buf: &mut [u8],
    ) -> blueos_hal::err::Result<()> {
        if buf.is_empty() {
            // No read requested; still send the header so the command lands.
            return self.qspi_send_header(opcode, cmd_code);
        }
        let regs = Self::spi_regs();
        let header = self.qspi_header(opcode, cmd_code);

        // Half-duplex: write the 4-byte header on MOSI, 8 dummy cycles for
        // turnaround, then read N bytes from MISO. One transaction per chunk.
        // USR_DUMMY_CYCLELEN = 8-1 = 7; header uses USR_MOSI, response USR_MISO.
        regs.user1
            .write(USER1::USR_DUMMY_CYCLELEN.val(7) + USER1::USR_ADDR_BITLEN.val(0));
        regs.user.modify(
            USER::DOUTDIN.val(0)
                + USER::USR_MOSI::SET
                + USER::USR_MISO::SET
                + USER::USR_COMMAND::CLEAR
                + USER::USR_ADDR::CLEAR
                + USER::USR_DUMMY::SET
                + USER::FWRITE_QUAD.val(0),
        );

        for chunk in buf.chunks_mut(SPI2_DATA_BUF_SIZE) {
            self.reset_tx_rx_fifo();
            // In half-duplex, MS_DATA_BITLEN defines BOTH the MOSI output bits
            // and the MISO input bits of the data phase. So this read path only
            // matches when the response length equals the header length (4
            // bytes) -- which is exactly the READ_ID (0x04) case. Set the data
            // phase length to the header size; the panel returns 4 bytes on MISO
            // within the same window, overwriting W0..W1 which we then read back.
            regs.ms_dlen
                .write(MS_DLEN::MS_DATA_BITLEN.val((Self::QSPI_HEADER_LEN as u32 * 8 - 1)));
            self.write_buf(&header);
            self.start_transfer()?;
            self.wait_done();
            self.read_buf(chunk);
        }
        Ok(())
    }
}

impl<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32> PlatPeri
    for Esp32Spi2<SPI_BASE, SYS_BASE, APB_HZ>
{
    fn enable(&self) {
        let sys = Self::sys_regs();

        // --- C3: SYSTEM peripheral clock gating + reset pulse (bit6, RST 1=reset) ---
        #[cfg(soc_esp32c3)]
        {
            sys.perip_clk_en0.modify(PERIP_CLK_EN0::SPI2_CLK_EN::SET);
            // Reset pulse; without it CMD::UPDATE may never clear.
            sys.perip_rst_en0.modify(PERIP_RST_EN0::SPI2_RST::SET);
            sys.perip_rst_en0.modify(PERIP_RST_EN0::SPI2_RST::CLEAR);
        }

        // --- C6: PCR clock gating + reset + function clock source (RST 0=reset) ---
        #[cfg(soc_esp32c6)]
        {
            // spi2_conf: enable APB clock, pulse reset (polarity inverted: 0=reset, 1=de-reset).
            sys.spi2_conf.modify(
                PCR_SPI2_CONF::SPI2_CLK_EN::Enabled
                    + PCR_SPI2_CONF::SPI2_RST_EN::Reset,
            );
            sys.spi2_conf
                .modify(PCR_SPI2_CONF::SPI2_RST_EN::NoReset);
            // spi2_clkm_conf: select 80MHz PLL source, enable function clock.
            sys.spi2_clkm_conf.modify(
                PCR_SPI2_CLKM_CONF::SPI2_CLKM_SEL::Pll80M
                    + PCR_SPI2_CLKM_CONF::SPI2_CLKM_EN::Enabled,
            );
        }

        let regs = Self::spi_regs();
        regs.clk_gate.write(
            CLK_GATE::CLK_EN.val(1)
                + CLK_GATE::MST_CLK_ACTIVE.val(1)
                + CLK_GATE::MST_CLK_SEL.val(1),
        );
    }

    fn disable(&self) {
        let regs = Self::spi_regs();
        regs.clk_gate.modify(CLK_GATE::MST_CLK_ACTIVE::CLEAR);
        let sys = Self::sys_regs();

        #[cfg(soc_esp32c3)]
        sys.perip_clk_en0.modify(PERIP_CLK_EN0::SPI2_CLK_EN::CLEAR);

        #[cfg(soc_esp32c6)]
        {
            sys.spi2_clkm_conf
                .modify(PCR_SPI2_CLKM_CONF::SPI2_CLKM_EN::Disabled);
            sys.spi2_conf
                .modify(PCR_SPI2_CONF::SPI2_CLK_EN::Disabled);
        }
    }
}

impl<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32> Configuration<SpiConfig>
    for Esp32Spi2<SPI_BASE, SYS_BASE, APB_HZ>
{
    type Target = ();

    fn configure(&self, config: &SpiConfig) -> blueos_hal::err::Result<Self::Target> {
        let regs = Self::spi_regs();

        // Ensure peripheral is enabled
        self.enable();

        // Master mode, soft reset first
        regs.slave
            .write(SLAVE::SLAVE_MODE.val(0) + SLAVE::SOFT_RESET.val(1));
        regs.slave.modify(SLAVE::SOFT_RESET::CLEAR);

        // No DMA, clear FIFOs
        regs.dma_conf
            .write(DMA_CONF::DMA_RX_ENA::CLEAR + DMA_CONF::DMA_TX_ENA::CLEAR);
        self.reset_tx_rx_fifo();

        // SPI mode from phase + polarity
        let ck_idle_edge = match config.polarity {
            SpiPolarity::Low => MISC::CK_IDLE_EDGE::Low,
            SpiPolarity::High => MISC::CK_IDLE_EDGE::High,
        };
        let ck_out_edge = match (config.polarity, config.phase) {
            (SpiPolarity::Low, SpiPhase::Phase0) => USER::CK_OUT_EDGE::LeadingEdge, // Mode 0
            (SpiPolarity::Low, SpiPhase::Phase1) => USER::CK_OUT_EDGE::TrailingEdge, // Mode 1
            (SpiPolarity::High, SpiPhase::Phase0) => USER::CK_OUT_EDGE::TrailingEdge, // Mode 2
            (SpiPolarity::High, SpiPhase::Phase1) => USER::CK_OUT_EDGE::LeadingEdge, // Mode 3
        };
        regs.misc.modify(ck_idle_edge);
        regs.user.modify(ck_out_edge);

        // Bit order
        let rd_bit_order = match config.bit_order {
            SpiBitOrder::MsbFirst => CTRL::RD_BIT_ORDER::MsbFirst,
            SpiBitOrder::LsbFirst => CTRL::RD_BIT_ORDER::LsbFirst,
        };
        let wr_bit_order = match config.bit_order {
            SpiBitOrder::MsbFirst => CTRL::WR_BIT_ORDER::MsbFirst,
            SpiBitOrder::LsbFirst => CTRL::WR_BIT_ORDER::LsbFirst,
        };
        regs.ctrl.modify(rd_bit_order + wr_bit_order);

        // Clock divider
        self.configure_clock(config.baudrate)?;

        // All HW CS lines disabled; CS managed by software GPIO via ExclusiveDevice
        regs.misc.modify(
            MISC::CS0_DIS.val(1)
                + MISC::CS1_DIS.val(1)
                + MISC::CS2_DIS.val(1)
                + MISC::CS3_DIS.val(1)
                + MISC::CS4_DIS.val(1)
                + MISC::CS5_DIS.val(1)
                + MISC::CS_KEEP_ACTIVE::CLEAR,
        );

        Ok(())
    }
}

impl<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32>
    blueos_hal::spi::Spi<SpiConfig, ()> for Esp32Spi2<SPI_BASE, SYS_BASE, APB_HZ>
{
    fn transfer(&self, read: &mut [u8], write: &[u8]) -> blueos_hal::err::Result<()> {
        self.do_full_duplex_transfer(read, write)
    }

    fn read(&self, buf: &mut [u8]) -> blueos_hal::err::Result<()> {
        self.do_half_duplex_read(buf)
    }

    fn write(&self, buf: &[u8]) -> blueos_hal::err::Result<()> {
        self.do_half_duplex_write(buf)
    }
}

// QSPI opcodes per CO5300 datasheet (1-wire command/address, data width per method).
const QSPI_CMD_WRITE: u8 = 0x02; // command write (1-wire data)
const QSPI_CMD_PIXEL_WRITE: u8 = 0x32; // pixel write (4-wire data)
const QSPI_CMD_READ: u8 = 0x03; // command read (1-wire data)

impl<const SPI_BASE: usize, const SYS_BASE: usize, const APB_HZ: u32> crate::spi::Qspi
    for Esp32Spi2<SPI_BASE, SYS_BASE, APB_HZ>
{
    fn qspi_write_command(&self, cmd: u8, params: &[u8]) -> blueos_hal::err::Result<()> {
        // Command write: opcode 0x02 + addr {0x00, cmd, 0x00} + params, 1-wire.
        self.do_qspi_write(QSPI_CMD_WRITE, cmd, params, false)
    }

    fn qspi_write_pixels(&self, pixels: &[u8]) -> blueos_hal::err::Result<()> {
        // Pixel write: opcode 0x32 + addr {0x00, 0x2C, 0x00} + pixel stream, 4-wire data.
        const RAMWR: u8 = 0x2C;
        self.do_qspi_write(QSPI_CMD_PIXEL_WRITE, RAMWR, pixels, true)
    }

    fn qspi_read_command(&self, cmd: u8, buf: &mut [u8]) -> blueos_hal::err::Result<()> {
        // Command read: opcode 0x03 + addr {0x00, cmd, 0x00} + dummy + read, 1-wire.
        self.do_qspi_read(QSPI_CMD_READ, cmd, buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use blueos_test_macro::test;

    #[test]
    fn test_wait_until_clear_times_out() {
        assert_eq!(
            wait_until_clear(|| true, 3),
            Err(blueos_hal::err::HalError::Timeout)
        );
    }

    #[test]
    fn test_wait_until_clear_returns_when_ready() {
        let mut polls = 0;
        assert_eq!(
            wait_until_clear(
                || {
                    polls += 1;
                    polls < 3
                },
                3
            ),
            Ok(())
        );
        assert_eq!(polls, 3);
    }
}

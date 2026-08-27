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

#[cfg(any(soc_esp32c3, soc_esp32c6))]
pub mod esp32_spi;

/// SPI clock phase (CPHA).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiPhase {
    Phase0,
    Phase1,
}

/// SPI clock polarity (CPOL).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiPolarity {
    Low,
    High,
}

/// SPI bit order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiBitOrder {
    MsbFirst,
    LsbFirst,
}

/// SPI peripheral configuration — used as `P` for HAL `Spi<P, T>` trait
pub struct SpiConfig {
    pub baudrate: u32,
    pub phase: SpiPhase,
    pub polarity: SpiPolarity,
    pub bit_order: SpiBitOrder,
    pub cs_pin: Option<u8>, // Unused — CS managed by ExclusiveDevice via GPIO OutputPin
}

impl SpiConfig {
    /// Mode 0 (CPOL=0, CPHA=0), MSB-first, 40MHz. Matches the
    /// 08_LVGL_V8_Test reference demo pclk_hz = 40 * 1000 * 1000.
    pub fn spi_flash_default() -> Self {
        SpiConfig {
            baudrate: 40_000_000,
            phase: SpiPhase::Phase0,
            polarity: SpiPolarity::Low,
            bit_order: SpiBitOrder::MsbFirst,
            cs_pin: None,
        }
    }
}

/// QSPI (4-wire) extension for SPI peripherals that support it (e.g. ESP32
/// GPSPI2). Used by QSPI LCD drivers such as CO5300.
///
/// Each transaction sends a QSPI opcode (0x02 command write / 0x32 pixel write
/// / 0x03 command read) plus a 24-bit address segment `{0x00, CMD, 0x00}` where
/// the middle byte carries the MIPI DCS command code. Command/address segments
/// run 1-wire; the data segment width is chosen per transaction (4-wire for
/// pixel writes, 1-wire otherwise). CS is managed by the caller so a single
/// CS-low window can span command + pixel stream.
pub trait Qspi {
    /// QSPI command write (1-wire): opcode 0x02 + addr {0x00, cmd, 0x00} + params.
    fn qspi_write_command(&self, cmd: u8, params: &[u8]) -> blueos_hal::err::Result<()>;
    /// QSPI pixel write (4-wire data): opcode 0x32 + addr {0x00, 0x2C, 0x00} + pixel stream.
    fn qspi_write_pixels(&self, pixels: &[u8]) -> blueos_hal::err::Result<()>;
    /// QSPI command read (1-wire): opcode 0x03 + addr {0x00, cmd, 0x00} + dummy + read.
    fn qspi_read_command(&self, cmd: u8, buf: &mut [u8]) -> blueos_hal::err::Result<()>;
}

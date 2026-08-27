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

//! ChipOne CO5300 AMOLED driver (480x480, QSPI 4-wire, no D/CX pin).
//!
//! Unlike the ST7789/ST7796 drivers, this does NOT use the `mipidsi` crate:
//! the CO5300 uses a QSPI interface where D/CX is tied low and the command/
//! data distinction is encoded in the QSPI address phase, so DCS commands are
//! issued directly through the `Qspi` trait. CS is held low by this driver
//! across the whole CASET+RASET+RAMWR+pixel sequence (the `Qspi` trait leaves
//! CS to the caller by design).

use crate::{
    devices::{bus::Bus, bus::BusWrapper, spi_core::block_spi::BlockSpi, DeviceData},
    drivers::{DriverModule, InitDriver},
    sync::KernelDelay,
};
use blueos_driver::spi::{Qspi, SpiConfig};
use blueos_hal::gpio::OutputPin;
use embedded_hal::delay::DelayNs;

// MIPI DCS standard command set.
const SWRESET: u8 = 0x01;
const SLPOUT: u8 = 0x11;
const DISPON: u8 = 0x29;
const CASET: u8 = 0x2A;
const RASET: u8 = 0x2B;
const RAMWR: u8 = 0x2C;
const TEON: u8 = 0x35; // Tearing effect line on
const MADCTL: u8 = 0x36;
const COLMOD: u8 = 0x3A;
const WRCTRLD: u8 = 0x53; // Write CTRL Display
const WRDISBV: u8 = 0x51; // Write Display Brightness Value (normal mode)
const HBM_DISBV: u8 = 0x63; // Write Display Brightness Value (HBM mode)

// CO5300 vendor-specific commands. 0xFE switches the vendor register page;
// page 0x00 is the DCS standard command page, page 0x20 is the vendor page
// where the analog-channel setup commands (0x19/0x1C) live.
const PAGE_SWITCH: u8 = 0xFE;
const VENDOR_REG_19: u8 = 0x19; // page-0x20 analog channel config
const VENDOR_REG_1C: u8 = 0x1C; // page-0x20 analog channel config
const SPI_MODE_REG: u8 = 0xC4;

// MADCTL: 0x30 matches the Waveshare ESP32-C6-Touch-AMOLED-2.16 reference
// demo (08_LVGL_V8_Test). The panel only lights up in this orientation.
const MADCTL_VALUE: u8 = 0x30;
// COLMOD: RGB565 = 0x55 (IFPF = 101).
const COLMOD_RGB565: u8 = 0x55;
// WRCTRLD: 0x20 = bit5 DISPON (display on, BCTRL off).
const WRCTRLD_VALUE: u8 = 0x20;
// 0xC4 value: 0x80 selects the QSPI 4-wire data path for pixel streaming.
const SPI_MODE_VALUE: u8 = 0x80;
// Page 0x00 = DCS standard command page.
const PAGE_DCS_STANDARD: u8 = 0x00;
// Page 0x20 = vendor command page (analog channel setup).
const PAGE_VENDOR: u8 = 0x20;
// Full brightness (0xFF) for both normal and HBM modes.
const BRIGHTNESS_MAX: u8 = 0xFF;

// Address window for the Waveshare 2.16 panel (480x480). CASET/RASET use the
// full 0..479 range (0x0000..0x01DF), matching the official waveshare board
// config. The CO5300 reference driver's 0x0016..0x01AF window targets a
// different 410x502 panel and is NOT used here.
const COL_START: u32 = 0;
const COL_END: u32 = 479; // 0x1DF
const ROW_START: u32 = 0;
const ROW_END: u32 = 479; // 0x1DF

// 480x480 panel.
const PANEL_WIDTH: u32 = 480;
const PANEL_HEIGHT: u32 = 480;

// Init delays, matching the Waveshare ESP32-C6-Touch-AMOLED-2.16 reference
// demo (08_LVGL_V8_Test): SLPOUT needs the full 600ms for the booster to
// settle before any vendor command is accepted.
const SWRESET_DELAY_MS: u32 = 80;
const SLPOUT_DELAY_MS: u32 = 600;
const DISPON_DELAY_MS: u32 = 100;

pub struct Co5300Config<G: OutputPin> {
    pub cs: &'static G,
}

/// Driver state for an initialized CO5300/SH8601. Holds the SPI bus wrapper
/// and the CS pin; CS is toggled manually around each draw sequence. The panel
/// is power-cycled (not GPIO-reset) via the AXP2101 ALDO3 rail by the board's
/// init_panel_via_pmic() before this driver is probed -- RST is NC on this
/// board, matching the reference demo (08_LVGL_V8_Test DisplayPort_DispReset).
pub struct Co5300Driver<T: blueos_hal::spi::Spi<SpiConfig, ()> + Qspi, G: OutputPin> {
    bus: BusWrapper<BlockSpi<T, G>>,
    cs: &'static G,
    width: u32,
    height: u32,
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()> + Qspi, G: OutputPin> Co5300Driver<T, G> {
    /// Issue a DCS command with optional parameters over the 1-wire QSPI path.
    /// CS is assumed already asserted by the caller.
    fn write_command(&self, cmd: u8, params: &[u8]) -> Result<(), crate::error::Error> {
        let inner = self.bus.0.lock();
        inner.qspi_write_command(cmd, params)
    }

    /// Stream pixels over the 4-wire QSPI path. CS is assumed already asserted.
    fn write_pixels(&self, pixels: &[u8]) -> Result<(), crate::error::Error> {
        let inner = self.bus.0.lock();
        inner.qspi_write_pixels(pixels)
    }

    /// Set the active draw window then start RAMWR. CS must be held low across
    /// this whole sequence and the subsequent pixel stream.
    fn set_window(&self, col_start: u32, col_end: u32, row_start: u32, row_end: u32) -> Result<(), crate::error::Error> {
        let caset = [
            ((col_start >> 8) & 0xFF) as u8,
            (col_start & 0xFF) as u8,
            ((col_end >> 8) & 0xFF) as u8,
            (col_end & 0xFF) as u8,
        ];
        let raset = [
            ((row_start >> 8) & 0xFF) as u8,
            (row_start & 0xFF) as u8,
            ((row_end >> 8) & 0xFF) as u8,
            (row_end & 0xFF) as u8,
        ];
        self.write_command(CASET, &caset)?;
        self.write_command(RASET, &raset)?;
        // RAMWR opens the pixel window; pixels follow without deasserting CS.
        self.write_command(RAMWR, &[])
    }

    /// CO5300 vendor + panel-config initialization. Mirrors the init_cmds[]
    /// from the Waveshare ESP32-C6-Touch-AMOLED-2.16 reference demo
    /// (08_LVGL_V8_Test display_bsp.cpp). The critical difference from a
    /// DCS-only init is the page-0x20 vendor block: 0x19/0x1C set up the
    /// panel's analog channel and the panel stays dark without them.
    ///   0xFE {0x20}        page switch -> vendor page
    ///   0x19 {0x10}        analog channel config (vendor page)
    ///   0x1C {0xA0}        analog channel config (vendor page)
    ///   0xFE {0x00}        page switch -> DCS standard page
    ///   0xC4 {0x80}        SPI mode = QSPI 4-wire
    ///   0x3A {0x55}        COLMOD RGB565
    ///   0x35 {0x00}        TEON (tearing effect on)
    ///   0x36 {0x30}        MADCTL (panel orientation that lights up)
    ///   0x53 {0x20}        WRCTRLD (DISPON, BCTRL off)
    ///   0x51 {0xFF}        WRDISBV normal brightness = max
    ///   0x63 {0xFF}        HBM brightness = max
    ///   0x2A {0..0x1DF}    CASET  (column window, 480px)
    ///   0x2B {0..0x1DF}    RASET  (row window, 480px)
    /// CS is assumed already asserted by the caller. No inter-command delays.
    fn vendor_init(&self) -> Result<(), crate::error::Error> {
        // Vendor page block: the 0x19/0x1C analog setup only takes effect
        // while page 0x20 is selected, so it must be bracketed by the two
        // 0xFE page switches.
        self.write_command(PAGE_SWITCH, &[PAGE_VENDOR])?;
        self.write_command(VENDOR_REG_19, &[0x10])?;
        self.write_command(VENDOR_REG_1C, &[0xA0])?;
        self.write_command(PAGE_SWITCH, &[PAGE_DCS_STANDARD])?;
        self.write_command(SPI_MODE_REG, &[SPI_MODE_VALUE])?;
        self.write_command(COLMOD, &[COLMOD_RGB565])?;
        self.write_command(TEON, &[0x00])?;
        self.write_command(MADCTL, &[MADCTL_VALUE])?;
        self.write_command(WRCTRLD, &[WRCTRLD_VALUE])?;
        self.write_command(WRDISBV, &[BRIGHTNESS_MAX])?;
        self.write_command(HBM_DISBV, &[BRIGHTNESS_MAX])?;
        // Address window for the full 480x480 panel.
        let caset = [
            ((COL_START >> 8) & 0xFF) as u8,
            (COL_START & 0xFF) as u8,
            ((COL_END >> 8) & 0xFF) as u8,
            (COL_END & 0xFF) as u8,
        ];
        let raset = [
            ((ROW_START >> 8) & 0xFF) as u8,
            (ROW_START & 0xFF) as u8,
            ((ROW_END >> 8) & 0xFF) as u8,
            (ROW_END & 0xFF) as u8,
        ];
        self.write_command(CASET, &caset)?;
        self.write_command(RASET, &raset)?;
        Ok(())
    }
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()> + Qspi, G: OutputPin> InitDriver<BlockSpi<T, G>>
    for Co5300Config<G>
{
    type Data = ();
    fn init(self, bus: &Bus<BlockSpi<T, G>>) -> crate::drivers::Result<Self::Data> {
        let mut delay = KernelDelay;

        // 1. Software reset. The panel was already power-cycled via AXP2101
        // ALDO3 by the board's init_panel_via_pmic(); SWRESET here is the
        // belt-and-suspenders path the official SH8601 driver takes after a
        // power-on. Official delay after SWRESET is 80ms.
        let bus_wrapper = bus.intf.clone();
        let driver = Co5300Driver {
            bus: bus_wrapper,
            cs: self.cs,
            width: PANEL_WIDTH,
            height: PANEL_HEIGHT,
        };

        crate::kearly_println!("[CO5300] init step 1/4: SWRESET");
        driver.cs.set_low().ok();
        let swreset_res = driver.write_command(SWRESET, &[]);
        driver.cs.set_high().ok();
        swreset_res?;
        delay.delay_ms(SWRESET_DELAY_MS);
        crate::kearly_println!("[CO5300] init step 1 ok");

        // 2. SLPOUT -> wait 600ms for the booster to settle. The reference
        // demo gives the panel the full 600ms here; a shorter delay was leaving
        // the panel not fully awake before the vendor commands that follow.
        crate::kearly_println!("[CO5300] init step 2/4: SLPOUT (+600ms)");
        driver.cs.set_low().ok();
        let slpout_res = driver.write_command(SLPOUT, &[]);
        driver.cs.set_high().ok();
        slpout_res?;
        delay.delay_ms(SLPOUT_DELAY_MS);
        crate::kearly_println!("[CO5300] init step 2 ok");

        // 3. Vendor init: page-0x20 analog setup + SPI mode + COLMOD/MADCTL +
        // brightness + the full 480x480 address window. All under one CS-low
        // span. WRDISBV is sent inside vendor_init, so it is not repeated here.
        crate::kearly_println!("[CO5300] init step 3/4: vendor_init (11 cmds)");
        driver.cs.set_low().ok();
        let cfg_res = driver.vendor_init();
        driver.cs.set_high().ok();
        cfg_res?;
        crate::kearly_println!("[CO5300] init step 3 ok");

        // 4. DISPON -> wait 100ms for the display to stabilize.
        crate::kearly_println!("[CO5300] init step 4/4: DISPON (+100ms)");
        driver.cs.set_low().ok();
        let dispon_res = driver.write_command(DISPON, &[]);
        driver.cs.set_high().ok();
        dispon_res?;
        delay.delay_ms(DISPON_DELAY_MS);
        crate::kearly_println!("[CO5300] init step 4 ok");

        super::LcdFramebuffer::register_lcd(driver, PANEL_WIDTH, PANEL_HEIGHT)
            .map_err(|_| crate::error::code::EINVAL)?;
        // Use kearly_println, not log::info!: CO5300 init runs from
        // init_spi_bus() in boot.rs:119, which executes BEFORE logger_init()
        // at boot.rs:128, so any log::info!/log::warn! here is silently
        // dropped. kearly_println writes the UART registers directly and is
        // available this early (console_uart is configured at boot.rs:96).
        crate::kearly_println!("[CO5300] initialized successfully (480x480, fb0 registered)");

        Ok(())
    }
}

pub struct Co5300DriverModule<G> {
    _marker: core::marker::PhantomData<G>,
}

impl<G> Co5300DriverModule<G> {
    pub const fn new() -> Self {
        Co5300DriverModule {
            _marker: core::marker::PhantomData,
        }
    }
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()> + Qspi, G: OutputPin> DriverModule<BlockSpi<T, G>>
    for Co5300DriverModule<G>
{
    type Data = Co5300Config<G>;
    fn probe(dev: &DeviceData) -> crate::drivers::Result<Self::Data> {
        match dev {
            DeviceData::Native(native_dev) => {
                if native_dev.is_attached() {
                    return Err(crate::error::code::ENODEV);
                }

                if let Some(config) = native_dev.config::<Co5300Config<G>>() {
                    Ok(Co5300Config::<G> {
                        cs: config.cs,
                    })
                } else {
                    Err(crate::error::code::ENODEV)
                }
            }
            _ => Err(crate::error::code::ENODEV),
        }
    }
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()> + Qspi, G: OutputPin> super::Lcd
    for Co5300Driver<T, G>
{
    fn draw_area(&mut self, area: super::DrawArea, color: &[u8]) -> Result<(), super::LcdError> {
        let area_width = area
            .col_end
            .checked_sub(area.col_start)
            .ok_or(super::LcdError::InvalidArea)?
            + 1;
        let area_height = area
            .row_end
            .checked_sub(area.row_start)
            .ok_or(super::LcdError::InvalidArea)?
            + 1;

        if area.col_start >= self.width || area.row_start >= self.height {
            return Ok(());
        }

        let col_start = area.col_start;
        let row_start = area.row_start;
        let col_end = area.col_end.min(self.width - 1);
        let row_end = area.row_end.min(self.height - 1);
        let draw_width = col_end - col_start + 1;
        let draw_height = row_end - row_start + 1;
        let draw_pixels = draw_width
            .checked_mul(draw_height)
            .ok_or(super::LcdError::InvalidColorData)?;
        let area_pixels = area_width
            .checked_mul(area_height)
            .ok_or(super::LcdError::InvalidColorData)?;
        let expected_len = usize::try_from(area_pixels)
            .ok()
            .and_then(|count| count.checked_mul(super::LCD_BYTES_PER_PIXEL as usize))
            .ok_or(super::LcdError::InvalidColorData)?;
        if color.len() != expected_len {
            return Err(super::LcdError::InvalidColorData);
        }

        // Single CS-low window spans set_window + pixel stream.
        self.cs.set_low().ok();
        let res = (|| {
            self.set_window(col_start, col_end, row_start, row_end)?;

            // If the draw area is smaller than the supplied color buffer, skip
            // the per-row lead-in and tail pixels to only send the visible
            // region. For the common full-area case this is a straight copy.
            if draw_pixels == area_pixels {
                self.write_pixels(color)?;
                return Ok(());
            }

            let initial_skip = (row_start - area.row_start) * area_width
                + (col_start - area.col_start);
            let skip_per_row = area_width - draw_width;
            let mut offset = initial_skip as usize * super::LCD_BYTES_PER_PIXEL as usize;
            for _ in 0..draw_height {
                let row_len = draw_width as usize * super::LCD_BYTES_PER_PIXEL as usize;
                self.write_pixels(&color[offset..offset + row_len])?;
                offset += row_len + skip_per_row as usize * super::LCD_BYTES_PER_PIXEL as usize;
            }
            Ok::<(), crate::error::Error>(())
        })();
        self.cs.set_high().ok();

        res.map_err(|_| super::LcdError::Bus)
    }
}

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

//! ESP32 IO_MUX + GPIO Matrix pin controller (C3/C6).

use crate::{
    gpio::esp32_gpio::{GpioEnable, GpioOut, GpioRegisters, GPIO_BASE},
    static_ref::StaticRef,
};
use blueos_hal::pinctrl::AlterFuncPin;
use tock_registers::{
    interfaces::{ReadWriteable, Writeable},
    register_bitfields,
    registers::ReadWrite,
};

// SPI2/FSPI signal indices routed through the GPIO Matrix (gpio_sig_map.h).
// C3 and C6 share identical signal indices (63..68). On ESP32 a signal's IN
// and OUT variants share the same index: the distinction is which register
// group (FUNC_IN_SEL_CFG vs FUNC_OUT_SEL_CFG) the index is written to.
pub const FSPICLK_OUT_IDX: u32 = 63;
pub const FSPICLK_IN_IDX: u32 = 63;
pub const FSPIQ_OUT_IDX: u32 = 64;
pub const FSPIQ_IN_IDX: u32 = 64;
pub const FSPID_OUT_IDX: u32 = 65;
pub const FSPID_IN_IDX: u32 = 65;
pub const FSPIHD_OUT_IDX: u32 = 66;
pub const FSPIHD_IN_IDX: u32 = 66;
pub const FSPIWP_OUT_IDX: u32 = 67;
pub const FSPIWP_IN_IDX: u32 = 67;
pub const FSPICS0_OUT_IDX: u32 = 68;
pub const FSPICS0_IN_IDX: u32 = 68;

// I2CEXT0 (I2C0) signal indices for the C6 GPIO Matrix (gpio_sig_map.h). C6
// only; C3 does not use this board's I2C path. Like the FSPI signals, a given
// I2C signal's IN and OUT variants share the same index.
#[cfg(soc_esp32c6)]
pub const I2CEXT0_SCL_OUT_IDX: u32 = 45;
#[cfg(soc_esp32c6)]
pub const I2CEXT0_SCL_IN_IDX: u32 = 45;
#[cfg(soc_esp32c6)]
pub const I2CEXT0_SDA_OUT_IDX: u32 = 46;
#[cfg(soc_esp32c6)]
pub const I2CEXT0_SDA_IN_IDX: u32 = 46;

// GPIO Matrix register offsets relative to GPIO base (identical on C3/C6).
const GPIO_PIN_REG_OFFSET: usize = 0x74; // per-pin PAD_DRIVER config
const FUNC_IN_SEL_CFG_OFFSET: usize = 0x154; // FUNCx_IN_SEL_CFG
const FUNC_OUT_SEL_CFG_OFFSET: usize = 0x554; // FUNCx_OUT_SEL_CFG

// GPIO Matrix base address differs per SoC (same layout as GPIO_BASE in
// esp32_gpio, but accessed via raw offsets beyond the typed GpioRegisters).
#[cfg(soc_esp32c3)]
const GPIO_MATRIX_BASE: usize = 0x60004000;
#[cfg(soc_esp32c6)]
const GPIO_MATRIX_BASE: usize = 0x60091000;

// IO_MUX per-pin register offsets relative to IO_MUX base: 0x04 + 4*pin.
// Covers the full pin range of both SoCs (C3: 22 pins, C6: 30 pins).
const IO_MUX_OFFSETS: [u32; 30] = [
    0x04, // GPIO0
    0x08, // GPIO1
    0x0C, // GPIO2
    0x10, // GPIO3
    0x14, // GPIO4
    0x18, // GPIO5
    0x1C, // GPIO6
    0x20, // GPIO7
    0x24, // GPIO8
    0x28, // GPIO9
    0x2C, // GPIO10
    0x30, // GPIO11
    0x34, // GPIO12
    0x38, // GPIO13
    0x3C, // GPIO14
    0x40, // GPIO15
    0x44, // GPIO16
    0x48, // GPIO17
    0x4C, // GPIO18
    0x50, // GPIO19
    0x54, // GPIO20
    0x58, // GPIO21
    0x5C, // GPIO22
    0x60, // GPIO23
    0x64, // GPIO24
    0x68, // GPIO25
    0x6C, // GPIO26
    0x70, // GPIO27
    0x74, // GPIO28
    0x78, // GPIO29
];

// IO_MUX base address differs per SoC.
#[cfg(soc_esp32c3)]
const IO_MUX_BASE: usize = 0x60009000;
#[cfg(soc_esp32c6)]
const IO_MUX_BASE: usize = 0x60090000;

register_bitfields! [
    u32,

    pub IoMuxFields [
        MCU_SEL OFFSET(12) NUMBITS(3) [
            Func0 = 0,  // Default (JTAG/special)
            Func1 = 1,  // GPIO function
            Func2 = 2,  // FSPI (SPI2) function for some pins
        ],
        FUN_DRV OFFSET(10) NUMBITS(2) [
            Drive0 = 0,
            Drive1 = 1,
            Drive2 = 2,
            Drive3 = 3,
        ],
        FUN_IE  OFFSET(9)  NUMBITS(1) [],  // Input enable
        FUN_PU  OFFSET(8)  NUMBITS(1) [],  // Pull-up
        FUN_PD  OFFSET(7)  NUMBITS(1) [],  // Pull-down
    ],
];

register_bitfields! [
    u32,

    pub GpioPinFields [
        PAD_DRIVER OFFSET(2) NUMBITS(1) [], // 0 = push-pull, 1 = open-drain
    ],

    // GPIO_FUNCx_OUT_SEL_CFG_REG: route a peripheral output signal to GPIO x.
    pub FuncOutSelCfg [
        OUT_SEL     OFFSET(0)  NUMBITS(8) [],
        INV_SEL     OFFSET(8)  NUMBITS(1) [],
        OEN_SEL     OFFSET(9)  NUMBITS(1) [
            Peripheral = 0,  // Peripheral controls output enable
            GpioReg     = 1,  // GPIO_ENABLE_REG controls output enable
        ],
        OEN_INV_SEL OFFSET(10) NUMBITS(1) [],
    ],
];

fn configure_open_drain(pin: u8, open_drain: bool) {
    let addr = GPIO_MATRIX_BASE + GPIO_PIN_REG_OFFSET + 4 * pin as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, GpioPinFields::Register>) };
    reg.modify(GpioPinFields::PAD_DRIVER.val(open_drain as u32));
}

register_bitfields! [
    u32,

    // GPIO_FUNCx_IN_SEL_CFG_REG: route GPIO pin to a peripheral input signal.
    // C6 layout (gpio_reg.h:2455-2472): IN_SEL [5:0] (6 bits, NOT 5 -- indices
    // go up to 46 for I2C0_SDA, 5 bits can't hold that), IN_INV_SEL bit6, and
    // the input-routing enable is SIG_IN_SEL at bit7 (1 = route the peripheral
    // input via the GPIO Matrix; 0 = bypass/don't route). The old layout put
    // IN_INV_SEL at bit5 and SEL at bit6: writing SEL=1 set IN_INV_SEL (input
    // inversion) while the real enable bit7 stayed 0, so the peripheral never
    // received the pad input at all -- for I2C this starved the FSM of SCL/SDA
    // feedback and tripped SCL_ST_TO (status 0x00002000) on the first transfer.
    pub FuncInSelCfg [
        IN_SEL     OFFSET(0) NUMBITS(6) [],
        IN_INV_SEL OFFSET(6) NUMBITS(1) [],
        SEL        OFFSET(7) NUMBITS(1) [],  // SIG_IN_SEL: 1 = route via GPIO Matrix, 0 = bypass
    ],
];

fn write_io_mux(pin: u8, mcu_sel: u32, ie: bool, pu: bool, pd: bool, drv: u32) {
    let addr = IO_MUX_BASE + IO_MUX_OFFSETS[pin as usize] as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, IoMuxFields::Register>) };
    reg.write(
        IoMuxFields::MCU_SEL.val(mcu_sel)
            + IoMuxFields::FUN_IE.val(if ie { 1 } else { 0 })
            + IoMuxFields::FUN_PU.val(if pu { 1 } else { 0 })
            + IoMuxFields::FUN_PD.val(if pd { 1 } else { 0 })
            + IoMuxFields::FUN_DRV.val(drv),
    );
}

fn route_signal_out(pin: u8, signal_idx: u32, oen_sel: u32) {
    let addr = GPIO_MATRIX_BASE + FUNC_OUT_SEL_CFG_OFFSET + 4 * pin as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, FuncOutSelCfg::Register>) };
    reg.write(
        FuncOutSelCfg::OUT_SEL.val(signal_idx)
            + FuncOutSelCfg::INV_SEL.val(0)
            + FuncOutSelCfg::OEN_SEL.val(oen_sel)
            + FuncOutSelCfg::OEN_INV_SEL.val(0),
    );
}

fn route_signal_in(signal_idx: u32, pin: u32) {
    let addr = GPIO_MATRIX_BASE + FUNC_IN_SEL_CFG_OFFSET + 4 * signal_idx as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, FuncInSelCfg::Register>) };
    reg.write(
        FuncInSelCfg::SEL.val(1) + FuncInSelCfg::IN_INV_SEL.val(0) + FuncInSelCfg::IN_SEL.val(pin),
    );
}

/// PinMux configuration entry used with `define_pin_states!`.
pub struct Esp32IoMuxPinctrl {
    pin: u8,
    mcu_sel: u32,
    ie: bool,
    pu: bool,
    pd: bool,
    drv: u32,
    out_signal: Option<u32>,
    in_signal: Option<u32>,
    gpio_output: bool,
    open_drain: bool,
}

impl Esp32IoMuxPinctrl {
    pub const fn new(
        pin: u8,
        mcu_sel: u32,
        ie: bool,
        pu: bool,
        pd: bool,
        drv: u32,
        out_signal: Option<u32>,
        in_signal: Option<u32>,
        gpio_output: bool,
        open_drain: bool,
    ) -> Self {
        Esp32IoMuxPinctrl {
            pin,
            mcu_sel,
            ie,
            pu,
            pd,
            drv,
            out_signal,
            in_signal,
            gpio_output,
            open_drain,
        }
    }
}

impl AlterFuncPin for Esp32IoMuxPinctrl {
    fn init(&self) {
        write_io_mux(self.pin, self.mcu_sel, self.ie, self.pu, self.pd, self.drv);
        configure_open_drain(self.pin, self.open_drain);

        // GPIO_ENABLE is a separate per-pin output-enable latch that ANDs with
        // the peripheral's OEN before the pad actually drives. ESP-IDF sets it
        // for every output line: esp_rom_gpio_connect_out_signal writes
        // GPIO_ENABLE_W1TS unconditionally, and spi_common.c calls
        // gpio_set_direction(GPIO_MODE_INPUT_OUTPUT) which does the same. Without
        // this, OEN_SEL=0 peripheral lines (SPI D0-D3/SCK) stayed high-Z: the
        // SPI controller toggled its OEN but the pad never drove, so D1/D2/D3
        // never transitioned -- only D0/SCK (active in every transaction) looked
        // alive. Software CS (gpio_output=true) already sets this below.
        if let Some(signal_idx) = self.out_signal {
            // Software-controlled pins (CS) use OEN_SEL=1; peripheral pins use OEN_SEL=0.
            let oen_sel = if self.gpio_output { 1u32 } else { 0u32 };
            route_signal_out(self.pin, signal_idx, oen_sel);
            if !self.gpio_output {
                let gpio_regs = &*GPIO_BASE;
                gpio_regs
                    .enable_w1ts
                    .write(GpioEnable::DATA.val(1 << self.pin));
            }
        }

        if let Some(signal_idx) = self.in_signal {
            route_signal_in(signal_idx, self.pin as u32);
        }

        // For software-controlled pins, pre-set output HIGH before enabling to
        // avoid a low glitch on CS at the enable moment.
        if self.gpio_output {
            let gpio_regs = &*GPIO_BASE;
            gpio_regs.out_w1ts.write(GpioOut::DATA.val(1 << self.pin));
            gpio_regs
                .enable_w1ts
                .write(GpioEnable::DATA.val(1 << self.pin));
        }
    }
}

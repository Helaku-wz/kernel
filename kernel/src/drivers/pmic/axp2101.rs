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

//! X-Powers AXP2101 PMIC driver (I2C, 7-bit address 0x34).
//!
//! Minimal driver covering what the Waveshare ESP32-C6-Touch-AMOLED-2.16
//! board needs. The vendor reference demo (`08_LVGL_V8_Test`,
//! `Custom_PmicRegisterInit`) sets five 3.3V rails before touching the panel:
//! DCDC1 + ALDO1/2/3/4. ALDO3 is then power-cycled as the panel's only reset
//! (the panel RST line is NC, exactly like `DisplayPort_DispReset()`).
//!
//! Register layout verified against the Waveshare-bundled XPowersLib source
//! (REG/AXP2101Constants.h + XPowersAXP2101.tpp):
//! - 0x80  DC_ONOFF_DVM_CTRL: bit0=DCDC1, bit1=DCDC2 ...
//! - 0x82  DC_VOL0_CTRL (DCDC1 voltage): low 5 bits = (mV-1500)/100;
//!         3300mV -> 0x12. High 3 bits reserved (read-modify-write preserved).
//! - 0x90  LDO_ONOFF_CTRL0: bit0=ALDO1, bit1=ALDO2, bit2=ALDO3, bit3=ALDO4.
//! - 0x92  LDO_VOL0_CTRL (ALDO1 voltage): low 5 bits = (mV-500)/100;
//! - 0x93  LDO_VOL1_CTRL (ALDO2 voltage): same encoding;
//! - 0x94  LDO_VOL2_CTRL (ALDO3 voltage): 3300mV -> 0x1C;
//! - 0x95  LDO_VOL3_CTRL (ALDO4 voltage): same encoding.
//!   All LDO voltage registers: high 3 bits reserved (preserved).

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::sync::KernelDelay;

/// AXP2101 I2C 7-bit slave address.
const AXP2101_I2C_ADDR: u8 = 0x34;

/// IC type register; reads back the chip ID. Per XPowersLib
/// (AXP2101Constants.h) a genuine AXP2101 returns 0x4A here, which also
/// proves the I2C bus is alive at address 0x34.
const REG_IC_TYPE: u8 = 0x03;
const AXP2101_CHIP_ID: u8 = 0x4A;

/// Voltage field mask shared by all DCDC and ALDO voltage registers: low 5
/// bits hold the encoded voltage, high 3 bits are reserved.
const VOLTAGE_MASK: u8 = 0x1F;

/// DCDC1 on/off control register (DC_ONOFF_DVM_CTRL): bit0 = DCDC1 enable.
const REG_DC_ONOFF: u8 = 0x80;
const DCDC1_ON_BIT: u8 = 1 << 0;
/// DCDC1 voltage register (DC_VOL0_CTRL): low 5 bits = (mV-1500)/100.
const REG_DCDC1_VOLTAGE: u8 = 0x82;
/// 3300mV -> (3300 - 1500) / 100 = 0x12.
const DCDC1_3300MV_CODE: u8 = 0x12;

/// LDO on/off control register 0: bit0=ALDO1, bit1=ALDO2, bit2=ALDO3, bit3=ALDO4.
const REG_LDO_ONOFF_CTRL0: u8 = 0x90;
const ALDO1_ON_BIT: u8 = 1 << 0;
const ALDO2_ON_BIT: u8 = 1 << 1;
const ALDO3_ON_BIT: u8 = 1 << 2;
const ALDO4_ON_BIT: u8 = 1 << 3;

/// ALDO voltage registers (LDO_VOL*_CTRL): low 5 bits = (mV-500)/100; 3300mV -> 0x1C.
const REG_ALDO1_VOLTAGE: u8 = 0x92;
const REG_ALDO2_VOLTAGE: u8 = 0x93;
const REG_ALDO3_VOLTAGE: u8 = 0x94;
const REG_ALDO4_VOLTAGE: u8 = 0x95;
/// 3300mV -> (3300 - 500) / 100 = 0x1C (shared by all ALDOs).
const ALDO_3300MV_CODE: u8 = 0x1C;
/// Backwards-compat alias for the old single-rail code path.
const ALDO3_3300MV_CODE: u8 = ALDO_3300MV_CODE;

/// Power-cycle timing for the AMOLED panel reset (vendor sequence).
const POWER_CYCLE_DELAY_MS: u32 = 100;

/// AXP2101 PMIC handle backed by any embedded-hal 1.0 I2c backend.
///
/// All register access goes through the `embedded_hal::i2c::I2c` trait
/// implemented by `BusWrapper<BlockI2c<T>>` in the kernel. Construct this from
/// a board's I2C bus interface (`bus.intf.clone()`) and call `init_panel_supply`
/// before probing the LCD driver.
pub struct Axp2101<I2C: I2c> {
    bus: I2C,
}

impl<I2C: I2c> Axp2101<I2C> {
    pub fn new(bus: I2C) -> Self {
        Self { bus }
    }

    /// Read a single register via write_read (register addr -> 1 byte).
    fn read_register(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.bus.write_read(AXP2101_I2C_ADDR, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    /// Write a single byte to a register.
    fn write_register(&mut self, reg: u8, val: u8) -> Result<(), I2C::Error> {
        self.bus.write(AXP2101_I2C_ADDR, &[reg, val])
    }

    /// Set a voltage register's low-5-bit field via read-modify-write,
    /// preserving the high 3 reserved bits. Works for any DCDC or ALDO voltage
    /// register (they all share the low-5-bit layout). `code` is a pre-encoded
    /// constant, so no runtime range check is needed (avoids constructing an
    /// I2C::Error value on the bad-input path, which has no generic
    /// constructor).
    fn set_voltage_code(&mut self, reg: u8, code: u8) -> Result<(), I2C::Error> {
        let current = self.read_register(reg)?;
        let updated = (current & !VOLTAGE_MASK) | (code & VOLTAGE_MASK);
        self.write_register(reg, updated)
    }

    /// Enable (bit=1) or disable (bit=0) a single LDO via the on/off control
    /// register 0x90, preserving all other enable bits.
    fn set_ldo_enable(&mut self, bit: u8, enable: bool) -> Result<(), I2C::Error> {
        let current = self.read_register(REG_LDO_ONOFF_CTRL0)?;
        let updated = if enable {
            current | bit
        } else {
            current & !bit
        };
        self.write_register(REG_LDO_ONOFF_CTRL0, updated)
    }

    /// Enable/disable DCDC1 via DC_ONOFF_DVM_CTRL bit0.
    fn set_dcdc1_enable(&mut self, enable: bool) -> Result<(), I2C::Error> {
        let current = self.read_register(REG_DC_ONOFF)?;
        let updated = if enable {
            current | DCDC1_ON_BIT
        } else {
            current & !DCDC1_ON_BIT
        };
        self.write_register(REG_DC_ONOFF, updated)
    }

    /// Power-cycle ALDO3 to reset the AMOLED panel (vendor sequence):
    /// on -> 100ms -> off -> 100ms -> on -> 100ms. The panel's RST line is NC,
    /// so this supply toggle is the only reset mechanism.
    ///
    /// Assumes ALDO3 voltage has already been configured.
    fn power_cycle_aldo3(&mut self, delay: &mut KernelDelay) -> Result<(), I2C::Error> {
        self.set_ldo_enable(ALDO3_ON_BIT, true)?;
        delay.delay_ms(POWER_CYCLE_DELAY_MS);
        self.set_ldo_enable(ALDO3_ON_BIT, false)?;
        delay.delay_ms(POWER_CYCLE_DELAY_MS);
        self.set_ldo_enable(ALDO3_ON_BIT, true)?;
        delay.delay_ms(POWER_CYCLE_DELAY_MS);
        Ok(())
    }

    /// Read back a voltage register's low-5-bit field and log it. `name` is the
    /// human-readable rail label, `reg` the register address, `expected_code`
    /// the value that should have latched.
    fn log_voltage_readback(&mut self, name: &str, reg: u8, expected_code: u8) {
        match self.read_register(reg) {
            Ok(v) => crate::kearly_println!(
                "[AXP2101] {} voltage reg {:#04x} = {:#04x} (low5={:#04x}, expected {:#04x}) {}",
                name,
                reg,
                v,
                v & VOLTAGE_MASK,
                expected_code,
                if (v & VOLTAGE_MASK) == expected_code { "OK" } else { "NOT LATCHED" }
            ),
            Err(_) => crate::kearly_println!("[AXP2101] {} voltage readback FAILED", name),
        }
    }

    /// Bring up all five 3.3V rails the reference demo sets
    /// (Custom_PmicRegisterInit: DCDC1 + ALDO1/2/3/4), then power-cycle ALDO3
    /// to reset the AMOLED panel. Call this once during board init, before
    /// driving the panel.
    ///
    /// Diagnostic: each rail's voltage and the final enable state is logged
    /// with kearly_println (this runs before logger_init, just like
    /// init_spi_bus). The goal is to answer, from serial output alone, exactly
    /// where the panel supply bring-up fails -- whether the I2C bus is dead
    /// (chip ID read fails / wrong ID), a rail voltage never latched, or a rail
    /// enable never latched. The panel stays dark until all rails are up.
    pub fn init_panel_supply(&mut self, delay: &mut KernelDelay) -> Result<(), I2C::Error> {
        // Step 0: chip-ID sanity check. If this fails or returns the wrong
        // value, every subsequent register write is meaningless -- the I2C
        // bus or address is wrong, so no rail was actually configured.
        match self.read_register(REG_IC_TYPE) {
            Ok(id) => crate::kearly_println!(
                "[AXP2101] chip ID (reg 0x03) = {:#04x} (expected {:#04x}) {}",
                id,
                AXP2101_CHIP_ID,
                if id == AXP2101_CHIP_ID { "OK" } else { "MISMATCH / WRONG CHIP" }
            ),
            Err(_) => crate::kearly_println!("[AXP2101] chip ID read FAILED (I2C bus dead?)"),
        }

        // Step 1: set all five rails to 3.3V (mirrors the demo's
        // Custom_PmicRegisterInit). Read back each voltage register to
        // confirm the low-5-bit code latched; a rail that reads back the
        // reset default instead of the code was never written and feeds the
        // panel nothing.
        self.set_voltage_code(REG_DCDC1_VOLTAGE, DCDC1_3300MV_CODE)?;
        self.log_voltage_readback("DCDC1", REG_DCDC1_VOLTAGE, DCDC1_3300MV_CODE);

        self.set_voltage_code(REG_ALDO1_VOLTAGE, ALDO_3300MV_CODE)?;
        self.log_voltage_readback("ALDO1", REG_ALDO1_VOLTAGE, ALDO_3300MV_CODE);

        self.set_voltage_code(REG_ALDO2_VOLTAGE, ALDO_3300MV_CODE)?;
        self.log_voltage_readback("ALDO2", REG_ALDO2_VOLTAGE, ALDO_3300MV_CODE);

        self.set_voltage_code(REG_ALDO3_VOLTAGE, ALDO_3300MV_CODE)?;
        self.log_voltage_readback("ALDO3", REG_ALDO3_VOLTAGE, ALDO_3300MV_CODE);

        self.set_voltage_code(REG_ALDO4_VOLTAGE, ALDO_3300MV_CODE)?;
        self.log_voltage_readback("ALDO4", REG_ALDO4_VOLTAGE, ALDO_3300MV_CODE);

        // Step 2: ensure DCDC1 is enabled (it is the likely main panel supply;
        // the demo relies on it being on). Read back to confirm.
        self.set_dcdc1_enable(true)?;
        match self.read_register(REG_DC_ONOFF) {
            Ok(v) => crate::kearly_println!(
                "[AXP2101] DC on/off reg 0x80 = {:#04x} (DCDC1 bit0={}) {}",
                v,
                v & DCDC1_ON_BIT,
                if (v & DCDC1_ON_BIT) != 0 { "DCDC1 ENABLED" } else { "DCDC1 STILL OFF" }
            ),
            Err(_) => crate::kearly_println!("[AXP2101] DC on/off readback FAILED"),
        }

        // Step 3: enable ALDO1/2/4 (ALDO3 is power-cycled next). Read back 0x90
        // after enabling to confirm the LDO enable bits are set.
        self.set_ldo_enable(ALDO1_ON_BIT, true)?;
        self.set_ldo_enable(ALDO2_ON_BIT, true)?;
        self.set_ldo_enable(ALDO4_ON_BIT, true)?;

        // Step 4: power-cycle ALDO3 (on->off->on) to reset the panel. Read the
        // enable bit after the final on to confirm ALDO3 is enabled.
        self.power_cycle_aldo3(delay)?;
        match self.read_register(REG_LDO_ONOFF_CTRL0) {
            Ok(v) => crate::kearly_println!(
                "[AXP2101] LDO on/off reg 0x90 = {:#04x} (ALDO1/2/3/4={:#04x}) {}",
                v,
                v & (ALDO1_ON_BIT | ALDO2_ON_BIT | ALDO3_ON_BIT | ALDO4_ON_BIT),
                if (v & (ALDO1_ON_BIT | ALDO2_ON_BIT | ALDO3_ON_BIT | ALDO4_ON_BIT))
                    == (ALDO1_ON_BIT | ALDO2_ON_BIT | ALDO3_ON_BIT | ALDO4_ON_BIT)
                {
                    "ALL ALDO ENABLED"
                } else {
                    "SOME ALDO OFF"
                }
            ),
            Err(_) => crate::kearly_println!("[AXP2101] LDO on/off readback FAILED"),
        }

        Ok(())
    }
}

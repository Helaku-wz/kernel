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

use crate::{devices::console::get_console, support::DisableInterruptGuard};
use core::{fmt, str};

#[macro_export]
macro_rules! kprintln {
    ($fmt:expr) => ({
        use core::fmt::Write;
        let mut writer = $crate::console::Console {};
        writer.write_fmt(format_args!(concat!($fmt, "\n"))).unwrap();
    });
    ($fmt:expr, $($arg:tt)*) => ({
        use core::fmt::Write;
        let mut writer = $crate::console::Console {};
        writer.write_fmt(format_args!(concat!($fmt, "\n"), $($arg)*)).unwrap();
    });
}

/// Provides a way to print messages before the console device is initialized.
/// It directly writes to the UART registers, so it can be used in the early
/// stage of the kernel initialization.
#[macro_export]
macro_rules! kearly_println {
    ($fmt:expr) => ({
        use core::fmt::Write;
        let mut writer = $crate::console::EarlyConsole {};
        writer.write_fmt(format_args!(concat!($fmt, "\n"))).unwrap();
    });
    ($fmt:expr, $($arg:tt)*) => ({
        use core::fmt::Write;
        let mut writer = $crate::console::EarlyConsole {};
        writer.write_fmt(format_args!(concat!($fmt, "\n"), $($arg)*)).unwrap();
    });
}

pub struct Console;
impl fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let _ = get_console().write(0, s.as_bytes(), true);
        Ok(())
    }
}

pub struct EarlyConsole;
impl fmt::Write for EarlyConsole {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        use blueos_hal::{Has8bitDataReg, HasFifo, HasLineStatusReg};
        let _guard = DisableInterruptGuard::new();
        let uart = crate::boards::get_device!(console_uart);
        for byte in s.as_bytes() {
            // Cap the busy-wait: EarlyConsole runs with interrupts disabled, so
            // the USB Serial JTAG TX-done ISR never fires to drain IN_EP_DATA_FREE.
            // An unbounded `while is_bus_busy() {}` deadlocks once the host's IN
            // polling falls behind the print rate -- CO5300/AXP init prints fill
            // the FIFO and hang forever at this line, freezing the whole boot
            // before `init returned Ok` can print. Spin a bounded number of
            // iterations; on timeout drop the byte and continue rather than
            // hanging the entire kernel initialization.
            let mut spin = 0u32;
            while uart.is_bus_busy() {
                spin += 1;
                if spin >= 200_000 {
                    break;
                }
            }
            if spin >= 200_000 {
                // FIFO never drained; drop this byte to avoid blocking boot.
                continue;
            }
            uart.write_data8(*byte);
            uart.flush_tx_fifo();
        }
        Ok(())
    }
}

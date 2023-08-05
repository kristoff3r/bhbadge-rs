#![no_std]
#![no_main]

use core::panic::PanicInfo;

pub mod color;
pub mod nfc;
pub mod spinlocks;
pub mod usb_serial;
pub mod usb_serial_defmt;
pub mod y2022;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    defmt_panic();
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

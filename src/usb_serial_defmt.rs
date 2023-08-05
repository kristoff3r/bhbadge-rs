use critical_section::RestoreState;
use defmt::global_logger;

use crate::spinlocks::UsbSpinlock;

#[global_logger]
struct GlobalUsbLogger;

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut INTERRUPTS: RestoreState = RestoreState::invalid();

unsafe impl defmt::Logger for GlobalUsbLogger {
    fn acquire() {
        unsafe {
            INTERRUPTS = critical_section::acquire();
            ENCODER.start_frame(write_usb);
        }
    }

    unsafe fn release() {
        ENCODER.end_frame(write_usb);
        critical_section::release(INTERRUPTS);
    }

    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes, write_usb);
    }

    unsafe fn flush() {}
}

fn write_usb(buffer: &[u8]) {
    let mut lock = UsbSpinlock::claim();
    if let Some(usb) = &mut *lock {
        let _ = usb.write_all(buffer);
    }
}

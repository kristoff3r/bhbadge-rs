use defmt::global_logger;

use crate::{spinlocks::UsbSpinlock, usb_serial::UsbManager};

/*
// #[global_logger]
struct GlobalUsbLogger;

static mut ENCODER: defmt::Encoder = defmt::Encoder::new();
static mut USB_SPINLOCK: Option<UsbSpinlock> = None;
static mut INTERRUPTS: u8 = 0;

unsafe impl defmt::Logger for GlobalUsbLogger {
    fn acquire() {
        unsafe {
            INTERRUPTS = critical_section::acquire();
            let usb_spinlock = USB_SPINLOCK.insert(UsbSpinlock::claim());
            ENCODER.start_frame(make_closure(usb_spinlock));
        }
    }

    unsafe fn release() {
        let mut usb_spinlock = USB_SPINLOCK.take().unwrap();
        ENCODER.end_frame(make_closure(&mut usb_spinlock));
        critical_section::release(INTERRUPTS);
    }

    unsafe fn write(bytes: &[u8]) {
        let usb_spinlock = USB_SPINLOCK.as_mut().unwrap();
        ENCODER.write(bytes, make_closure(usb_spinlock));
    }

    unsafe fn flush() {}
}

fn make_closure<'a>(usb: &'a mut Option<UsbManager>) -> impl 'a + FnMut(&[u8]) {
    |buf| {
        if let Some(usb) = usb.as_mut() {
            let _ = usb.write_all(buf);
        }
    }
}
 */

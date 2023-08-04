use rp2040_hal::{
    clocks::UsbClock,
    pac::interrupt,
    pac::{self, RESETS, USBCTRL_DPRAM, USBCTRL_REGS},
    usb::{self, UsbBus},
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_serial::SerialPort;

use crate::spinlocks::UsbSpinlock;

static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;

pub struct UsbManager {
    usb_dev: UsbDevice<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
    buffer: [u8; 64],
}

impl UsbManager {
    pub fn maintain(&mut self) -> Result<(), UsbError> {
        if self.usb_dev.poll(&mut [&mut self.serial]) {
            loop {
                match self.serial.read(&mut self.buffer) {
                    Ok(0) | Err(UsbError::WouldBlock) => break,
                    Ok(_) => continue,
                    Err(e) => return Err(e),
                }
            }
        }
        Ok(())
    }

    pub fn write_all(&mut self, mut buf: &[u8]) -> Result<(), UsbError> {
        let mut failure_count = 0;
        while !buf.is_empty() {
            match self.serial.write(buf) {
                Ok(0) | Err(UsbError::WouldBlock) => {
                    self.maintain()?;
                    failure_count += 1;
                    if failure_count > 20 {
                        return Err(UsbError::WouldBlock);
                    }
                }
                Ok(n) => {
                    buf = buf.get(n..).unwrap_or(&[]);
                    failure_count = 0;
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    pub fn init(
        ctrl_reg: USBCTRL_REGS,
        ctrl_dpram: USBCTRL_DPRAM,
        usb_clock: UsbClock,
        resets: &mut RESETS,
    ) {
        let usb_bus: &'static mut UsbBusAllocator<UsbBus> = unsafe {
            USB_BUS_ALLOCATOR.insert(UsbBusAllocator::new(usb::UsbBus::new(
                ctrl_reg, ctrl_dpram, usb_clock, true, resets,
            )))
        };

        // Set up the USB Communications Class Device driver
        let serial = SerialPort::new(usb_bus);

        // Create a USB device with a fake VID and PID
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        *UsbSpinlock::claim() = Some(UsbManager {
            usb_dev: usb_dev,
            serial,
            buffer: [0; 64],
        });

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
        };
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    if let Some(mut usb) = UsbSpinlock::try_claim() {
        if let Some(usb) = &mut *usb {
            usb.maintain().unwrap();
        }
    }
}

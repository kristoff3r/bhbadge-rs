#![no_std]

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use rp2040_hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use rp2040_hal::pac;

rp2040_hal::bsp_pins!(
    Gpio0 {
        name: tx1,
        aliases: {
            FunctionUart: Gp0Uart0Tx
        }
    },
    Gpio1 {
        name: rx1,
        aliases: {
            FunctionUart: Gp1Uart0Rx
        }
    },
    Gpio4 {
        name: sda,
        aliases: {
            FunctionI2C: Gp4I2C0Sda
        }
    },
    Gpio5 {
        name: scl,
        aliases: {
            FunctionI2C: Gp5I2C0Scl
        }
    },
    Gpio6 {
        name: pn7150_irq,
        aliases: {
            FloatingInput: PnIrq
        }
    },
    Gpio7 {
        name: pn7150_enable,
        aliases: {
            PushPullOutput: PnEnable
        }
    },
    Gpio13 {
        name: led1,
        aliases: {
            FunctionPwm: Led1Pwm,
            PushPullOutput: Led1PushPull
        }
    },
    Gpio24 {
        name: led2,
        aliases: {
            FunctionPwm: Led2Pwm,
            PushPullOutput: Led2PushPull
        }
    },
    Gpio25 {
        name: led0,
        aliases: {
            PushPullOutput: Led0PushPull
        }
    },
    Gpio26 {
        name: a0,
        aliases: {
            PullUpInput: A0PullUp
        }
    },
    Gpio27 {
        name: a1,
        aliases: {
            PullUpInput: A1PullUp
        }
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#![no_std]
#![no_main]

use bhbadge::usb_serial::UsbManager;
use bhboard_2023 as bsp;
use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        sio::Sio,
        watchdog::Watchdog,
    },
};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::gpio::PushPull;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Initialize the USB early to get debug information up and running
    // It still takes around 800ms after this point before messages start
    // showing up on the host
    UsbManager::init(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        &mut pac.RESETS,
    );

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.led0.into_mode::<rp2040_hal::gpio::Output<PushPull>>();
    let mut counter = 0u32;
    let mut is_high = false;
    loop {
        if counter & 0xffffff == 0 {
            defmt::debug!("Frame count: {}", counter);
            if is_high {
                led.set_low().unwrap();
            } else {
                led.set_high().unwrap();
            }
            is_high ^= true;
        }
        counter = counter.wrapping_add(1);
    }
}

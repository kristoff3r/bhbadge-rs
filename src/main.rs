#![no_std]
#![no_main]

use bsp::{entry, hal::spi};

use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use st7735::color::{Color, DefaultColor};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let a_button = pins.gpio16.into_pull_up_input();
    let b_button = pins.gpio17.into_pull_up_input();
    let x_button = pins.gpio18.into_pull_up_input();
    let y_button = pins.gpio19.into_pull_up_input();

    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // Turn on backlight
    // let mut backlight = pins.gpio13.into_push_pull_output();
    // backlight.set_high();

    let dc = pins.gpio7.into_push_pull_output();
    let mut display = {
        let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
        st7735::ST7734::new_with_spi(spi, dc, delay)
    };
    // display.set_orientation(&st7735::Orientation::Landscape);
    display.fill_screen(&Color::from_default(DefaultColor::Blue));

    loop {
        if a_button.is_low().unwrap()
            || b_button.is_low().unwrap()
            || x_button.is_low().unwrap()
            || y_button.is_low().unwrap()
        {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
    }
}

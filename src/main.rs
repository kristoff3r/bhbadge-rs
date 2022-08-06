#![no_std]
#![no_main]

use core::cell::Cell;

use bsp::{entry, hal::spi};

use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{self, Interrupt},
    pac::{self, interrupt},
    sio::Sio,
    watchdog::Watchdog,
};
use st7735::color::{Color, DefaultColor};

type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::PushPullOutput>;
type ButtonAPin = gpio::Pin<gpio::bank0::Gpio16, gpio::PullUpInput>;
type ButtonBPin = gpio::Pin<gpio::bank0::Gpio17, gpio::PullUpInput>;
type ButtonXPin = gpio::Pin<gpio::bank0::Gpio18, gpio::PullUpInput>;
type ButtonYPin = gpio::Pin<gpio::bank0::Gpio19, gpio::PullUpInput>;

struct LedAndButtons {
    led: LedPin,
    button_a: ButtonAPin,
    button_b: ButtonBPin,
    button_x: ButtonXPin,
    button_y: ButtonYPin,
}

// Used for hand-off to the interrupt handler
static GLOBAL_PINS: Mutex<Cell<Option<LedAndButtons>>> = Mutex::new(Cell::new(None));

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

    let led_and_buttons = LedAndButtons {
        led: pins.led.into_mode(),
        button_a: pins.gpio16.into_mode(),
        button_b: pins.gpio17.into_mode(),
        button_x: pins.gpio18.into_mode(),
        button_y: pins.gpio19.into_mode(),
    };

    led_and_buttons
        .button_a
        .set_interrupt_enabled(Interrupt::EdgeLow, true);

    cortex_m::interrupt::free(|cs| {
        GLOBAL_PINS.borrow(cs).set(Some(led_and_buttons));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

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
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndButton>`
    static mut LED_AND_BUTTONS: Option<LedAndButtons> = None;

    // Lazy initialization: Steal the global button and led pins
    if LED_AND_BUTTONS.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED_AND_BUTTONS = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(LedAndButtons {
        led,
        button_a,
        button_b,
        button_x,
        button_y,
    }) = LED_AND_BUTTONS
    {
        button_a.clear_interrupt(Interrupt::EdgeLow);

        if button_b.is_low().unwrap_or(false)
            || button_x.is_low().unwrap_or(false)
            || button_y.is_low().unwrap_or(false)
        {
            let _ = led.set_high();
        } else {
            let _ = led.set_low();
        }
    }
}

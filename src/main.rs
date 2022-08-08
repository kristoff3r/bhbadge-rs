#![no_std]
#![no_main]

use core::cell::Cell;

use cortex_m::{delay::Delay, interrupt::Mutex};
use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use panic_probe as _;

use bhboard as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{self, Interrupt},
    pac::{self, interrupt},
    sio::Sio,
    watchdog::Watchdog,
};
use bsp::{
    entry,
    hal::{
        clocks::ClocksManager,
        gpio::Pin,
        multicore::{Multicore, Stack},
        pwm, spi, uart,
    },
    ButtonA, ButtonB, ButtonX, ButtonY, Led,
};
use st7735::{
    color::{Color, DefaultColor},
    fonts::font57::Font57,
};

fn init_uart(
    clocks: &ClocksManager,
    uart: pac::UART1,
    resets: &mut pac::RESETS,
    tx: Pin<gpio::bank0::Gpio8, gpio::FunctionUart>,
    rx: Pin<gpio::bank0::Gpio9, gpio::FunctionUart>,
) {
    let uart = uart::UartPeripheral::new(uart, (tx, rx), resets)
        .enable(
            uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    defmt_serial::defmt_serial(uart);

    defmt::info!("defmt initialized");
}

struct Display<SPI> {
    display: st7735::ST7735<
        display_interface_spi::SPIInterface<SPI, bsp::DisplaySpiDc, bsp::DisplaySpiCs>,
        1,
        2,
    >,
    _reset: bsp::DisplayReset,
    _sck: bsp::DisplaySpiSck,
    _copi: bsp::DisplaySpiCopi,
}

impl<SPI> Display<SPI>
where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    SPI::Error: core::fmt::Debug,
{
    fn new(
        delay: &mut Delay,
        spi: SPI,
        mut cs: bsp::DisplaySpiCs,
        dc: bsp::DisplaySpiDc,
        mut reset: bsp::DisplayReset,
        sck: bsp::DisplaySpiSck,
        copi: bsp::DisplaySpiCopi,
    ) -> Self {
        // Hard reset
        cs.set_low().unwrap();
        reset.set_low().unwrap();
        delay.delay_ms(50);
        reset.set_high().unwrap();
        delay.delay_ms(150);
        cs.set_high().unwrap();

        let mut display =
            st7735::ST7735::new(display_interface_spi::SPIInterface::new(spi, dc, cs), delay);
        display.clear_screen();

        Self {
            display,
            _reset: reset,
            _sck: sck,
            _copi: copi,
        }
    }
}

struct LedAndButtons {
    led: Led,
    button_a: ButtonA,
    button_b: ButtonB,
    button_x: ButtonX,
    button_y: ButtonY,
}

// Used for hand-off to the interrupt handler
static GLOBAL_PINS: Mutex<Cell<Option<LedAndButtons>>> = Mutex::new(Cell::new(None));

static mut CORE1_STACK: Stack<4096> = Stack::new();
fn core1_task() -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, core1_task);

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    init_uart(
        &clocks,
        pac.UART1,
        &mut pac.RESETS,
        pins.tx2.into_mode(),
        pins.rx2.into_mode(),
    );

    let led_and_buttons = LedAndButtons {
        led: pins.led.into_mode(),
        button_a: pins.button_a.into_mode(),
        button_b: pins.button_b.into_mode(),
        button_x: pins.button_x.into_mode(),
        button_y: pins.button_y.into_mode(),
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Turn on backlight
    let backlight = pins.display_backlight_pwm;
    let mut pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm6;
    pwm.set_ph_correct();
    pwm.set_div_int(20);
    pwm.enable();
    let channel = &mut pwm.channel_b;
    channel.output_to(backlight);
    channel.set_duty(60000);

    defmt::info!("pwm initialized");

    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        40_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut display = Display::new(
        &mut delay,
        spi,
        pins.display_spi_cs.into_mode(),
        pins.display_spi_dc.into_mode(),
        pins.display_reset.into_mode(),
        pins.display_spi_sck.into_mode(),
        pins.display_spi_copi.into_mode(),
    );

    let mut x = 0u16;
    let mut y = 0u16;
    loop {
        display.display.clear_screen();

        display.display.draw_string(
            "YOLOOOO",
            x,
            y,
            &Color::from_default(DefaultColor::Blue),
            Font57,
        );

        delay.delay_ms(80);
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
    }
}

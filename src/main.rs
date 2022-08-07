#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{clocks::ClocksManager, gpio::Pin, pwm, spi, uart},
};

use cortex_m::delay::Delay;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embedded_hal::{digital::v2::OutputPin, PwmPin};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use panic_probe as _;

use bhboard as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    watchdog::Watchdog,
};
use st7735::color::Color;

const INIT_SEQUENCE: &[&[u8]] = &[
    b"\x01\x80\x96",                     // SWRESET and Delay 150ms
    b"\x11\x80\xff",                     // SLPOUT and Delay
    b"\xb1\x03\x01\x2C\x2D",             // _FRMCTR1
    b"\xb2\x03\x01\x2C\x2D",             // _FRMCTR2
    b"\xb3\x06\x01\x2C\x2D\x01\x2C\x2D", // _FRMCTR3
    b"\xb4\x01\x07",                     // _INVCTR line inversion
    b"\xc0\x03\xa2\x02\x84",             // _PWCTR1 GVDD = 4.7V, 1.0uA
    b"\xc1\x01\xc5",                     // _PWCTR2 VGH=14.7V, VGL=-7.35V
    b"\xc2\x02\x0a\x00",                 // _PWCTR3 Opamp current small, Boost frequency
    b"\xc3\x02\x8a\x2a",
    b"\xc4\x02\x8a\xee",
    b"\xc5\x01\x0e", // _VMCTR1 VCOMH = 4V, VOML = -1.1V
    b"\x20\x00",     // _INVOFF
    b"\x36\x01\x18", // _MADCTL bottom to top refresh
    // 1 clk cycle nonoverlap, 2 cycle gate rise, 3 sycle osc equalie,
    // fix on VTL
    b"\x3a\x01\x05", // COLMOD - 16bit color
    b"\xe0\x10\x02\x1c\x07\x12\x37\x32\x29\x2d\x29\x25\x2B\x39\x00\x01\x03\x10", // _GMCTRP1 Gamma
    b"\xe1\x10\x03\x1d\x07\x06\x2E\x2C\x29\x2D\x2E\x2E\x37\x3F\x00\x00\x02\x10", // _GMCTRN1
    b"\x13\x80\x0a", // _NORON
    b"\x29\x80\x64", // _DISPON
    b"\x36\x01\xC0", // _MADCTL Default rotation plus BGR encoding;
];

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
    display: st7735::ST7734<
        display_interface_spi::SPIInterface<SPI, bsp::DisplaySpiDc, bsp::DisplaySpiCs>,
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
            st7735::ST7734::new(display_interface_spi::SPIInterface::new(spi, dc, cs), delay);
        display.clear_screen();

        Self {
            display,
            _reset: reset,
            _sck: sck,
            _copi: copi,
        }
    }
}

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

    let blue = Color::from_rgb(0, 0, 0x1f);
    loop {
        for y in 40..100 {
            display.display.clear_screen();
            display.display.draw_filled_circle(50, y, 25, &blue);
            delay.delay_ms(50);
        }
    }
}

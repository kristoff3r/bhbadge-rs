#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{
        clocks::ClocksManager,
        gpio::{Pin, PushPullOutput},
        pwm, spi, uart,
    },
};

use cortex_m::delay::Delay;
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

fn spi_write<SPI>(
    delay: &mut Delay,
    spi: &mut SPI,
    cs: &mut Pin<gpio::bank0::Gpio21, PushPullOutput>,
    dc: &mut Pin<gpio::bank0::Gpio7, PushPullOutput>,
    cmd: u8,
    data: &[u8],
) where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    SPI::Error: core::fmt::Debug,
{
    // Start transaction
    cs.set_high().unwrap();

    // Send command
    dc.set_low().unwrap();
    spi.write(&[cmd]).unwrap();

    // Toggle cs in case chip latches on that
    cs.set_high().unwrap();
    delay.delay_us(1);
    cs.set_low().unwrap();
    delay.delay_us(1);

    // Send command
    dc.set_high().unwrap();
    spi.write(data).unwrap();

    // Start transaction
    cs.set_low().unwrap();
}

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

fn init_display<SPI>(
    delay: &mut Delay,
    mut spi: SPI,
    mut cs: Pin<gpio::bank0::Gpio21, PushPullOutput>,
    mut dc: Pin<gpio::bank0::Gpio7, PushPullOutput>,
    mut reset: Pin<gpio::bank0::Gpio6, PushPullOutput>,
) where
    SPI: embedded_hal::blocking::spi::Write<u8>,
    SPI::Error: core::fmt::Debug,
{
    delay.delay_ms(1);
    reset.set_low().unwrap();
    delay.delay_ms(1);
    reset.set_high().unwrap();

    for bytes in INIT_SEQUENCE {
        let cmd = bytes[0];
        let data_size = bytes[1];
        let delay_byte = data_size & 0x80;
        let data_size = data_size & 0x7f;

        let data = &bytes[2..2 + data_size as usize];

        spi_write(delay, &mut spi, &mut cs, &mut dc, cmd, data);

        let mut delay_len_ms = 10;
        if delay_byte > 0 {
            delay_len_ms = *bytes.last().unwrap() as u32;
            if delay_len_ms == 255 {
                delay_len_ms = 500;
            }
        }

        delay.delay_ms(delay_len_ms);
    }

    spi_write(delay, &mut spi, &mut cs, &mut dc, 0x2a, b"\x00\x10\x00\x20");
    delay.delay_ms(1);
    spi_write(delay, &mut spi, &mut cs, &mut dc, 0x2b, b"\x00\x10\x00\x20");
    delay.delay_ms(1);
    let buffer = [0x80; 17 * 17];
    spi_write(delay, &mut spi, &mut cs, &mut dc, 0x2c, &buffer[..]);
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
        pins.gpio8.into_mode(),
        pins.gpio9.into_mode(),
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Turn on backlight
    let backlight = pins.gpio13;
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
        24_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // Initialize screen
    let mut dc = pins.gpio7.into_push_pull_output();
    let mut cs = pins.gpio21.into_push_pull_output();
    let mut reset = pins.gpio6.into_push_pull_output();

    dc.set_high().unwrap();
    cs.set_low().unwrap();
    reset.set_high().unwrap();

    init_display(&mut delay, spi, cs, dc, reset);

    defmt::info!("display inited");

    let mut n = 0u16;
    loop {
        defmt::info!("Hello from defmt! {}", n);
        n = n.wrapping_add(1);
        // uart.write_full_blocking(b"UART example\r\n");
        delay.delay_ms(1000);
    }
}

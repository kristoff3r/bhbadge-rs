#![no_std]
#![no_main]

use bhbadge::{nfc::Pn7150, usb_serial::UsbManager};
use bhboard_2023 as bsp;
use bsp::entry;
use cortex_m::delay::Delay;
use fugit::RateExtU32;

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    i2c,
    pac::{self},
    sio::Sio,
    watchdog::Watchdog,
};

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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Initialize the USB early to get debug information up and running
    // It still takes around 800ms after this point before messages start
    // showing up on the host
    UsbManager::init(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        &mut pac.RESETS,
    );
    delay.delay_ms(1000);
    defmt::debug!("usb");

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let i2c = i2c::I2C::i2c0(
        pac.I2C0,
        pins.sda.into_mode(),
        pins.scl.into_mode(),
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut pn7150 = Pn7150 {
        i2c,
        irq: pins.pn7150_irq.into_mode(),
        ven: pins.pn7150_enable.into_mode(),
    };
    pn7150.init(&mut delay);
    delay.delay_ms(50);
    pn7150.connect_nci(&mut delay).expect("connect_nci");

    read_card(&mut pn7150, &mut delay).unwrap();

    loop {}
}

fn read_card(pn7150: &mut Pn7150, delay: &mut Delay) -> Result<(), i2c::Error> {
    pn7150.cmd_prop_act(delay)?;
    pn7150.cmd_discover_map(delay)?;

    const BLK_NB_MFC: u8 = 4;
    loop {
        pn7150.cmd_discover_cmd(delay)?;

        delay.delay_ms(50);

        pn7150.wait_for_card()?;

        delay.delay_ms(50);

        pn7150.send_card_command(&[
            0x40,
            BLK_NB_MFC / 4,
            0x10,
            0xff,
            0xff,
            0xff,
            0xff,
            0xff,
            0xff,
        ])?;

        pn7150.cmd_deactivate(delay)?;

        pn7150.send_card_command(&[0x10, 0x00, BLK_NB_MFC])?;

        delay.delay_ms(500);
    }
}

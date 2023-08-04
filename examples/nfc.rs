#![no_std]
#![no_main]

use bhbadge::usb_serial::UsbManager;
use bhboard_2023 as bsp;
use bsp::{entry, Gp4I2C0Sda, Gp5I2C0Scl};
use cortex_m::delay::Delay;
use defmt::Format;
use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    prelude::{_embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write},
};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use heapless::Vec;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    i2c,
    pac::{self, I2C0},
    sio::Sio,
    watchdog::Watchdog,
    I2C,
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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

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

    delay.delay_ms(1000);

    let i2c = i2c::I2C::i2c0(
        pac.I2C0,
        pins.sda.into_mode(),
        pins.scl.into_mode(),
        100.kHz(),
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
    pn7150.connect_nci(&mut delay);
    loop {}
}

const READ_WRITE_ADDR: u8 = 40;

pub struct Pn7150 {
    pub i2c: I2C<I2C0, (Gp4I2C0Sda, Gp5I2C0Scl)>,
    pub ven: bsp::PnEnable,
    pub irq: bsp::PnIrq,
}

impl Pn7150 {
    pub fn init(&mut self, delay: &mut Delay) {
        self.ven.set_high().unwrap();
        delay.delay_ms(1);
        self.ven.set_low().unwrap();
        delay.delay_ms(1);
        self.ven.set_high().unwrap();
        delay.delay_ms(3);
        defmt::debug!("init done");
    }

    pub fn has_message(&mut self) -> bool {
        self.irq.is_high().unwrap()
    }

    pub fn write_data(&mut self, data: &[u8]) -> Result<(), i2c::Error> {
        self.i2c.write(READ_WRITE_ADDR, data)
    }

    pub fn read_data(&mut self) -> Result<Option<Message>, i2c::Error> {
        if self.has_message() {
            let mut header = [0; 3];
            let mut buf = [0; 255];

            self.i2c.read(READ_WRITE_ADDR, &mut header)?;
            let len = header[2];

            self.i2c.read(READ_WRITE_ADDR, &mut buf[..len as usize])?;

            Ok(Some(Message {
                header: [header[0], header[1]],
                msg: Vec::from_slice(&buf[..len as usize]).unwrap(),
            }))
        } else {
            Ok(None)
        }
    }

    pub fn connect_nci(&mut self, delay: &mut Delay) {
        defmt::debug!("connect_nci");
        let mut ready = false;
        for _ in 0..3 {
            if let Ok(msg) = self.wakeup_nci() {
                if msg.header == [0x40, 0] {
                    ready = true;
                    defmt::debug!("MESSAGE: {:?}", msg.msg.as_slice());
                    break;
                } else {
                    defmt::debug!("WRONG HEADER: {:?}", msg);
                }
            }
            delay.delay_ms(500);
        }
        if !ready {
            defmt::debug!("did not wake up :(");
        }
    }

    fn wakeup_nci<'a>(&mut self) -> Result<Message, i2c::Error> {
        self.write_data(&[0x20, 0x00, 0x01, 0x01])?;
        loop {
            if let Some(msg) = self.read_data()? {
                return Ok(msg);
            }
        }
    }
}

#[derive(Format)]
pub struct Message {
    pub header: [u8; 2],
    pub msg: Vec<u8, 255>,
}

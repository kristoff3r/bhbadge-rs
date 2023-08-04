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
    loop {}
}

const READ_WRITE_ADDR: u8 = 40;
// MT=1 GID=0 OID=0 PL=1 ResetType=1 (Reset Configuration)
pub const NCI_CORE_RESET_CMD: &[u8] = b"\x20\x00\x01\x01";
// MT=1 GID=0 OID=1 PL=0
pub const NCI_CORE_INIT_CMD: &[u8] = b"\x20\x01\x00";
// MT=1 GID=f OID=2 PL=0
pub const NCI_PROP_ACT_CMD: &[u8] = b"\x2f\x02\x00";
// MT=1 GID=1 OID=0
pub const NCI_RF_DISCOVER_MAP_RW: &[u8] =
    b"\x21\x00\x10\x05\x01\x01\x01\x02\x01\x01\x03\x01\x01\x04\x01\x02\x80\x01\x80";
// MT=1 GID=1 OID=3
pub const NCI_RF_DISCOVER_CMD_RW: &[u8] = b"\x21\x03\x09\x04\x00\x01\x02\x01\x01\x01\x06\x01";
// MODE_POLL | TECH_PASSIVE_NFCA,
// MODE_POLL | TECH_PASSIVE_NFCF,
// MODE_POLL | TECH_PASSIVE_NFCB,
// MODE_POLL | TECH_PASSIVE_15693,
pub const NCI_RF_DEACTIVATE_CMD: &[u8] = b"\x21\x06\x01\x00";

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
        while self.has_message() {
            let _ = self.read_data();
        }
        defmt::debug!("write: {:?}", data);
        self.i2c.write(READ_WRITE_ADDR, data)
    }

    pub fn read_data(&mut self) -> Result<Option<Message>, i2c::Error> {
        if self.has_message() {
            let mut header = [0; 3];
            let mut buf = [0; 255];

            self.i2c.read(READ_WRITE_ADDR, &mut header)?;
            let len = header[2];

            self.i2c.read(READ_WRITE_ADDR, &mut buf[..len as usize])?;
            let msg = Message {
                header: [header[0], header[1]],
                msg: Vec::from_slice(&buf[..len as usize]).unwrap(),
            };
            defmt::trace!("read: {:?}", msg);

            Ok(Some(msg))
        } else {
            Ok(None)
        }
    }

    pub fn read_data_timeout(
        &mut self,
        delay: &mut Delay,
        mut timeout: usize,
    ) -> Result<Option<Message>, i2c::Error> {
        while !self.has_message() {
            delay.delay_ms(10);
            if timeout < 10 {
                return Ok(None);
            }
            timeout -= 10;
        }

        self.read_data()
    }

    pub fn connect_nci(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        defmt::debug!("connect_nci");
        let mut ready = false;
        for _ in 0..3 {
            if let Ok(msg) = self.wakeup_nci() {
                if msg.header == [0x40, 0] {
                    ready = true;
                    defmt::debug!("MESSAGE: {:?}", msg);
                    break;
                } else {
                    defmt::debug!("WRONG HEADER: {:?}", msg);
                }
            }
            delay.delay_ms(500);
        }

        if !ready {
            defmt::error!("did not wake up :(");
            return Err(i2c::Error::Abort(1337));
        }

        self.write_data(NCI_CORE_INIT_CMD)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        let nrf_int = msg.msg[5] as usize;
        defmt::info!(
            "VERSION: romcode_v={} major_no={} minor_no={}",
            msg.msg[14 + nrf_int],
            msg.msg[15 + nrf_int],
            msg.msg[16 + nrf_int],
        );

        self.write_data(NCI_PROP_ACT_CMD)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.validate([0x4f, 0x02]);
        defmt::info!("BUILD NUMBER: {:?}", &msg.msg[1..]);

        self.write_data(NCI_RF_DISCOVER_MAP_RW)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.validate([0x41, 0x00]);

        loop {
            self.write_data(NCI_RF_DISCOVER_CMD_RW)?;
            let msg = self.read_data_timeout(delay, 15)?.unwrap();
            msg.validate([0x41, 0x03]);

            delay.delay_ms(50);

            loop {
                if let Some(msg) = self.read_data_timeout(delay, 15)? {
                    if msg.header[0] == 0x61 {
                        let card = Card::from_buf(&msg.msg);
                        defmt::info!("{:?}", card);
                        break;
                    }
                }
            }

            delay.delay_ms(50);

            self.write_data(NCI_RF_DEACTIVATE_CMD)?;
            let msg = self.read_data_timeout(delay, 15)?.unwrap();
            msg.validate([0x41, 0x06]);

            delay.delay_ms(500);
        }

        Ok(())
    }

    fn wakeup_nci<'a>(&mut self) -> Result<Message, i2c::Error> {
        self.write_data(NCI_CORE_RESET_CMD)?;
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

impl Message {
    fn validate(&self, header: [u8; 2]) {
        assert!(self.header == header && self.msg[0] == 0)
    }
}

#[derive(Format)]
pub struct Card {
    pub id: u8,
    pub interface: u8,
    pub protocol: u8,
    pub modetech: u8,
    pub maxpayload: u8,
    pub credits: u8,
    pub nrfparams: u8,
    pub rest: Vec<u8, 255>,
}

impl Card {
    pub fn from_buf(buf: &[u8]) -> Self {
        Card {
            id: buf[0],
            interface: buf[1],
            protocol: buf[2],
            modetech: buf[3],
            maxpayload: buf[4],
            credits: buf[5],
            nrfparams: buf[6],
            rest: Vec::from_slice(&buf[7..]).unwrap(),
        }
    }
}

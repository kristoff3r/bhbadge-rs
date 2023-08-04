use bhboard_2023 as bsp;
use bsp::{Gp4I2C0Sda, Gp5I2C0Scl};
use cortex_m::delay::Delay;
use defmt::Format;
use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    prelude::{_embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write},
};

use heapless::Vec;
use rp2040_hal::{i2c, pac::I2C0, I2C};

pub const READ_WRITE_ADDR: u8 = 40;
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

    pub fn cmd_prop_act(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_PROP_ACT_CMD)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.validate([0x4f, 0x02]);
        defmt::info!("BUILD NUMBER: {:?}", &msg.msg[1..]);

        Ok(())
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

        self.wakeup_until_ready(delay)?;
        self.cmd_core_init(delay)?;

        Ok(())
    }

    fn wakeup_until_ready(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        for _ in 0..3 {
            if let Ok(msg) = self.wakeup_nci() {
                if msg.header == [0x40, 0] {
                    return Ok(());
                } else {
                    defmt::debug!("WRONG HEADER: {:?}", msg);
                }
            }
            delay.delay_ms(500);
        }

        defmt::error!("did not wake up :(");
        return Err(i2c::Error::Abort(1337));
    }

    pub fn cmd_deactivate(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_RF_DEACTIVATE_CMD)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.validate([0x41, 0x06]);
        Ok(())
    }

    pub fn wait_for_card(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        loop {
            if let Some(msg) = self.read_data_timeout(delay, 15)? {
                if msg.header[0] == 0x61 {
                    let card = Card::from_buf(&msg.msg);
                    defmt::info!("{:?}", card);
                    break;
                }
            }
        }

        Ok(())
    }

    pub fn cmd_discover_cmd(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_RF_DISCOVER_CMD_RW)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.validate([0x41, 0x03]);
        Ok(())
    }

    pub fn cmd_discover_map(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_RF_DISCOVER_MAP_RW)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.validate([0x41, 0x00]);
        Ok(())
    }

    pub fn cmd_core_init(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_CORE_INIT_CMD)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        let nrf_int = msg.msg[5] as usize;
        defmt::info!(
            "VERSION: romcode_v={} major_no={} minor_no={}",
            msg.msg[14 + nrf_int],
            msg.msg[15 + nrf_int],
            msg.msg[16 + nrf_int],
        );
        Ok(())
    }

    pub fn wakeup_nci<'a>(&mut self) -> Result<Message, i2c::Error> {
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

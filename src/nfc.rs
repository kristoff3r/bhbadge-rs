use bhboard_2023 as bsp;
use bsp::{Gp4I2C0Sda, Gp5I2C0Scl};
use cortex_m::delay::Delay;
use defmt::{assert, panic, Format};
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
        defmt::debug!("write: {}", data);
        self.i2c.write(READ_WRITE_ADDR, data)
    }

    pub fn read_data(&mut self) -> Result<Option<Packet>, i2c::Error> {
        if self.has_message() {
            let mut header = [0; 3];
            let mut buf = [0; 255];

            self.i2c.read(READ_WRITE_ADDR, &mut header)?;
            let len = header[2] as usize;

            self.i2c.read(READ_WRITE_ADDR, &mut buf[..len])?;
            let msg = Packet::from_header_payload(header, &buf[..len]);
            defmt::trace!("read: {}", msg);

            Ok(Some(msg))
        } else {
            Ok(None)
        }
    }

    pub fn cmd_prop_act(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_PROP_ACT_CMD)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.header.assert_response(0xf, 2);
        defmt::info!("BUILD NUMBER: {}", &msg.payload[1..]);

        Ok(())
    }

    pub fn read_data_timeout(
        &mut self,
        delay: &mut Delay,
        mut timeout: usize,
    ) -> Result<Option<Packet>, i2c::Error> {
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
                if let Some(header) = msg.header.as_reponse_header() {
                    if header.gid == 0 && header.oid == 0 {
                        return Ok(());
                    } else {
                        defmt::debug!("WRONG HEADER: {:?}", msg);
                    }
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
        msg.header.assert_response(1, 6);
        Ok(())
    }

    pub fn wait_for_card(&mut self) -> Result<(), i2c::Error> {
        loop {
            if let Some(msg) = self.read_data()? {
                if let Some(header) = msg.header.as_notification_header() {
                    defmt::info!("Notification packet while waiting for card: {}", header);
                    if header.gid == 1 {
                        let card = Card::from_buf(&msg.payload);
                        defmt::info!("{:?}", card);
                        break;
                    }
                }
            }
        }

        Ok(())
    }

    pub fn cmd_discover_cmd(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_RF_DISCOVER_CMD_RW)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.header.assert_response(1, 3);
        Ok(())
    }

    pub fn cmd_discover_map(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_RF_DISCOVER_MAP_RW)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        msg.header.assert_response(1, 0);
        Ok(())
    }

    pub fn cmd_core_init(&mut self, delay: &mut Delay) -> Result<(), i2c::Error> {
        self.write_data(NCI_CORE_INIT_CMD)?;
        let msg = self.read_data_timeout(delay, 15)?.unwrap();
        let nrf_int = msg.payload[5] as usize;
        defmt::info!(
            "VERSION: romcode_v={} major_no={} minor_no={}",
            msg.payload[14 + nrf_int],
            msg.payload[15 + nrf_int],
            msg.payload[16 + nrf_int],
        );
        Ok(())
    }

    pub fn wakeup_nci<'a>(&mut self) -> Result<Packet, i2c::Error> {
        self.write_data(NCI_CORE_RESET_CMD)?;
        loop {
            if let Some(msg) = self.read_data()? {
                return Ok(msg);
            }
        }
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

impl ControlPacketHeader {
    fn from_header_type(header: [u8; 3], message_type: ControlMessageType) -> Self {
        assert!(
            header[1] & 0xc0 == 0,
            "control packet with non-zero RFU field"
        );
        Self {
            message_type,
            gid: header[0] & 0xf,
            oid: header[1] & 0x3f,
        }
    }
}
#[derive(Format)]
pub struct Packet {
    pub header: PacketHeader,
    pub payload: Vec<u8, 255>,
}

#[derive(Copy, Clone, Format)]
pub enum PacketHeader {
    Data(DataPacketHeader),
    Control(ControlPacketHeader),
}

#[derive(Copy, Clone, Format)]
pub struct DataPacketHeader {
    /// Connection Identifier
    pub conn_id: u8,
}

#[derive(Copy, Clone, Format)]
pub struct ControlPacketHeader {
    pub message_type: ControlMessageType,
    /// Group Identifier
    pub gid: u8,
    /// Opcode Identifier
    pub oid: u8,
}

#[repr(u8)]
#[derive(Copy, Clone, Format, PartialEq, Eq)]
pub enum ControlMessageType {
    Command = 1,
    Response = 2,
    Notification = 3,
}

impl PacketHeader {
    fn from_header(header: [u8; 3]) -> Self {
        assert!(header[0] & 0x10 == 0, "segmented packets are not supported");
        let msg_type = header[0] >> 5;

        let msg_type = match msg_type {
            0 => return PacketHeader::Data(DataPacketHeader::from_header(header)),
            1 => ControlMessageType::Command,
            2 => ControlMessageType::Response,
            3 => ControlMessageType::Notification,
            _ => panic!("Unknown packet type: {}", msg_type),
        };
        PacketHeader::Control(ControlPacketHeader::from_header_type(header, msg_type))
    }

    pub fn as_data_header(&self) -> Option<&DataPacketHeader> {
        if let PacketHeader::Data(h) = self {
            Some(h)
        } else {
            None
        }
    }

    pub fn as_control_header(&self) -> Option<&ControlPacketHeader> {
        if let PacketHeader::Control(h) = self {
            Some(h)
        } else {
            None
        }
    }

    pub fn as_command_header(&self) -> Option<&ControlPacketHeader> {
        let h = self.as_control_header()?;
        if h.message_type == ControlMessageType::Command {
            Some(h)
        } else {
            None
        }
    }

    pub fn as_reponse_header(&self) -> Option<&ControlPacketHeader> {
        let h = self.as_control_header()?;
        if h.message_type == ControlMessageType::Response {
            Some(h)
        } else {
            None
        }
    }

    pub fn as_notification_header(&self) -> Option<&ControlPacketHeader> {
        let h = self.as_control_header()?;
        if h.message_type == ControlMessageType::Notification {
            Some(h)
        } else {
            None
        }
    }

    pub fn assert_response(&self, gid: u8, oid: u8) {
        if let Some(h) = self.as_reponse_header() {
            if h.gid == gid && h.oid == oid {
                return;
            }
        }
        panic!(
            "Expected response packet with gid={} and oid={}, got: {}",
            gid, oid, self
        );
    }
}

impl DataPacketHeader {
    fn from_header(header: [u8; 3]) -> Self {
        assert!(header[1] == 0, "data packet with non-zero RFU field");
        Self {
            conn_id: header[0] & 0xf,
        }
    }
}

impl Packet {
    pub fn to_bytes(&self, out: &mut Vec<u8, 258>) {
        out.clear();
        match self.header {
            PacketHeader::Data(p) => {
                out.push(p.conn_id).unwrap();
                out.push(0).unwrap();
            }
            PacketHeader::Control(p) => {
                out.push(((p.message_type as u8) << 5) | p.gid).unwrap();
                out.push(p.oid).unwrap();
            }
        }
        out.push(self.payload.len() as u8).unwrap();
        out.extend_from_slice(&self.payload).unwrap();
    }

    pub fn from_header_payload(header: [u8; 3], payload: &[u8]) -> Self {
        Self {
            header: PacketHeader::from_header(header),
            payload: Vec::from_slice(payload).unwrap(),
        }
    }
}

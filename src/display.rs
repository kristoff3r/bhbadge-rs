use bhboard as bsp;
use bsp::hal::spi::Enabled;
use bsp::{hal::spi, pac::DMA};
use cortex_m::delay::Delay;
use display_interface::{DataFormat, WriteOnlyDataCommand};
use embedded_hal::digital::v2::OutputPin;
use num_traits::cast::ToPrimitive;
use panic_probe as _;
use st7735::command::Instruction;

type RawSpi = spi::Spi<Enabled, bsp::pac::SPI0, 8>;
pub type RawDisplayBufferRow = [u16; 128];
pub type RawDisplayBuffer = [RawDisplayBufferRow; 160];

pub struct DisplayBuffer {
    buffer: &'static mut RawDisplayBuffer,
}

unsafe fn transmute_ref<T, U>(r: &T) -> &U {
    assert_eq!(core::mem::size_of::<T>(), core::mem::size_of::<U>());
    assert!(core::mem::align_of::<T>() >= core::mem::align_of::<U>());
    let ptr = r as *const T;
    core::mem::forget(r);
    &*(ptr as *const U)
}

unsafe fn transmute_mut_ref<T, U>(r: &mut T) -> &mut U {
    assert_eq!(core::mem::size_of::<T>(), core::mem::size_of::<U>());
    assert!(core::mem::align_of::<T>() >= core::mem::align_of::<U>());
    let ptr = r as *mut T;
    core::mem::forget(r);
    &mut *(ptr as *mut U)
}

impl DisplayBuffer {
    pub fn new(buffer: &'static mut RawDisplayBuffer) -> Self {
        Self { buffer }
    }

    pub fn bytes(&self) -> &[u8; 2 * 128 * 160] {
        unsafe { transmute_ref(self.buffer) }
    }

    pub fn mut_bytes(&mut self) -> &mut [u8; 2 * 128 * 160] {
        unsafe { transmute_mut_ref(self.buffer) }
    }

    pub fn row(&self, y: usize) -> &RawDisplayBufferRow {
        &self.buffer[y]
    }

    pub fn row_mut(&mut self, y: usize) -> &mut RawDisplayBufferRow {
        &mut self.buffer[y]
    }

    pub fn pixel_mut(&mut self, y: usize, x: usize) -> &mut u16 {
        &mut self.buffer[y][x]
    }

    pub fn pixel(&mut self, y: usize, x: usize) -> u16 {
        self.buffer[y][x]
    }

    pub fn clear(&mut self) {
        for byte in self.mut_bytes() {
            *byte = 0;
        }
    }

    pub fn draw_rect(
        &mut self,
        ys: core::ops::Range<usize>,
        xs: core::ops::Range<usize>,
        value: u16,
    ) {
        for row in &mut self.buffer[ys.start.clamp(0, 160)..ys.end.clamp(0, 160)] {
            for pixel in &mut row[xs.start.clamp(0, 128)..xs.end.clamp(0, 128)] {
                *pixel = value;
            }
        }
    }
}

pub struct DisplayDevice {
    pub display: st7735::ST7735<
        display_interface_spi::SPIInterface<RawSpi, bsp::DisplaySpiDc, bsp::DisplaySpiCs>,
        1,
        2,
    >,
    _reset: bsp::DisplayReset,
    _sck: bsp::DisplaySpiSck,
    _copi: bsp::DisplaySpiCopi,
}

impl DisplayDevice {
    pub fn new(
        delay: &mut Delay,
        spi: RawSpi,
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

    pub fn draw(&mut self, dma: &mut DMA, buffer: &DisplayBuffer) {
        self.display
            .spi
            .send_commands(DataFormat::U8(&[Instruction::RAMWR.to_u8().unwrap()]))
            .unwrap();

        // Assert chip select pin
        self.display.spi.cs.set_low().unwrap();

        // 1 = data, 0 = command
        self.display.spi.dc.set_high().unwrap();

        // Send words over SPI
        let buffer_bytes = buffer.bytes();
        let buffer_ptr = buffer_bytes.as_ptr();
        let buffer_len = buffer_bytes.len();
        const SPI0_BASE: u32 = 0x4003c000;
        const SSPDR_OFFSET: u32 = 0x008;
        dma.ch[0]
            .ch_read_addr
            .write(|w| unsafe { w.bits(buffer_ptr as u32) });
        dma.ch[0]
            .ch_write_addr
            .write(|w| unsafe { w.bits(SPI0_BASE + SSPDR_OFFSET) });
        dma.ch[0]
            .ch_trans_count
            .write(|w| unsafe { w.bits(buffer_len as u32) });
        dma.ch[0].ch_ctrl_trig.write(|w| {
            w.incr_read().set_bit();
            w.incr_write().clear_bit();
            w.high_priority().clear_bit();
            w.data_size().size_byte();
            // treq sel?
            w.en().set_bit();
            w
        });
        const DMA_CHANNEL: u32 = 0;
        dma.multi_chan_trigger
            .write(|w| unsafe { w.multi_chan_trigger().bits(1 << DMA_CHANNEL) });

        while dma.ch[0].ch_ctrl_trig.read().busy().bit_is_set() {}

        // self.display
        //     .spi
        //     .spi
        //     .send_u8(&mut self.display.spi.spi, DataFormat::U8(buffer.bytes()))
        //     .unwrap();

        // Deassert chip select pin
        self.display.spi.cs.set_high().ok();
        self.display
            .spi
            .send_data(DataFormat::U8(&buffer.bytes()[..]))
            .unwrap();
    }
}

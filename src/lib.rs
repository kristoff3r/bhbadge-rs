#![no_std]
#![no_main]

use core::{
    panic::PanicInfo,
    sync::atomic::{AtomicU32, AtomicU8, Ordering},
};

use bhboard as bsp;
use bsp::{
    pac::{DMA, RESETS},
    ButtonA, ButtonB, ButtonX, ButtonY, Led,
};
use cortex_m::delay::Delay;
use display::RawSpi;
use embedded_hal::{digital::v2::OutputPin, spi::FullDuplex};
use num_traits::cast::ToPrimitive;
use rp2040_hal::dma::DREQ_SPI0_TX;
use st7735::command::Instruction;

use crate::{color::Pixel, display::DisplayBuffer};

pub mod color;
pub mod display;
pub mod gameboy;
pub mod spinlocks;
pub mod usb_serial;
pub mod usb_serial_defmt;

const TX_CHANNEL: u8 = 0;
const CLEAR_CHANNEL: u8 = 1;
// TODO: Wasting an entire channel to check for when the DMA done is a bad solution.
// We should use interrupts instead.
const MARK_NOT_BUSY_CHANNEL: u8 = 2;
static DMA_BUSY: AtomicU8 = AtomicU8::new(0);
static CLEAR_COLOR: AtomicU32 = AtomicU32::new(0);

pub fn set_clear_color(pixel: Pixel) {
    let raw_pixel = pixel.raw_pixel();
    let doubled_raw_pixel = (raw_pixel as u32) | (raw_pixel as u32) << 16;
    CLEAR_COLOR.store(doubled_raw_pixel, Ordering::Relaxed);
}

pub struct LedAndButtons {
    pub led: Led,
    pub button_a: ButtonA,
    pub button_b: ButtonB,
    pub button_x: ButtonX,
    pub button_y: ButtonY,
}

pub fn init_dma(resets: &mut RESETS, dma: &mut DMA) {
    resets.reset.modify(|_, w| w.dma().set_bit());
    resets.reset.modify(|_, w| w.dma().clear_bit());
    while resets.reset_done.read().dma().bit_is_clear() {}

    const SPI0_BASE: u32 = 0x4003c000;
    const SSPDR_OFFSET: u32 = 0x008;
    let tx_channel = &dma.ch[TX_CHANNEL as usize];
    let clear_channel = &dma.ch[CLEAR_CHANNEL as usize];
    let mark_not_busy_channel = &dma.ch[MARK_NOT_BUSY_CHANNEL as usize];

    tx_channel
        .ch_write_addr
        .write(|w| unsafe { w.bits(SPI0_BASE + SSPDR_OFFSET) });
    tx_channel
        .ch_trans_count
        .write(|w| unsafe { w.bits(128 * 160 * 2) });
    tx_channel.ch_al1_ctrl.write(|w| {
        w.incr_read().set_bit();
        w.incr_write().clear_bit();
        w.high_priority().clear_bit();
        w.data_size().size_byte();
        unsafe {
            w.treq_sel().bits(DREQ_SPI0_TX);
        }
        w.bswap().clear_bit();
        w.ring_sel().clear_bit();
        w.irq_quiet().clear_bit();
        w.sniff_en().clear_bit();
        unsafe {
            w.ring_size().bits(0);
            w.chain_to().bits(CLEAR_CHANNEL);
        }
        w.en().set_bit();
        w
    });

    clear_channel
        .ch_read_addr
        .write(|w| unsafe { w.bits(&CLEAR_COLOR as *const AtomicU32 as u32) });
    clear_channel
        .ch_trans_count
        .write(|w| unsafe { w.bits(128 * 160 / 2) }); // We write two pixels at a time
    clear_channel.ch_al1_ctrl.write(|w| {
        w.incr_read().clear_bit();
        w.incr_write().set_bit();
        w.high_priority().clear_bit();
        w.data_size().size_word();
        w.treq_sel().permanent();
        w.bswap().clear_bit();
        w.ring_sel().clear_bit();
        w.irq_quiet().clear_bit();
        w.sniff_en().clear_bit();
        unsafe {
            w.ring_size().bits(0);
            w.chain_to().bits(MARK_NOT_BUSY_CHANNEL);
        }
        w.en().set_bit();
        w
    });

    static ZERO: &'static u8 = &0;
    mark_not_busy_channel
        .ch_al1_read_addr
        .write(|w| unsafe { w.bits(ZERO as *const u8 as u32) });
    mark_not_busy_channel
        .ch_write_addr
        .write(|w| unsafe { w.bits(&DMA_BUSY as *const AtomicU8 as u32) });
    mark_not_busy_channel
        .ch_trans_count
        .write(|w| unsafe { w.bits(1) });
    mark_not_busy_channel.ch_al1_ctrl.write(|w| {
        w.incr_read().clear_bit();
        w.incr_write().clear_bit();
        w.high_priority().clear_bit();
        w.data_size().size_byte();
        w.treq_sel().permanent();
        w.bswap().clear_bit();
        w.ring_sel().clear_bit();
        w.irq_quiet().clear_bit();
        w.sniff_en().clear_bit();
        unsafe {
            w.ring_size().bits(0);
            w.chain_to().bits(MARK_NOT_BUSY_CHANNEL);
        }
        w.en().set_bit();
        w
    });
}

pub fn send_and_clear_buffer(
    dma: &mut DMA,
    spi: &mut RawSpi,
    delay: &mut Delay,
    dc: &mut bsp::DisplaySpiDc,
    buffer: &DisplayBuffer,
) {
    // 1 = data, 0 = command
    dc.set_low().unwrap();

    // Send words over SPI
    spi.send(Instruction::RAMWR.to_u8().unwrap()).unwrap();
    delay.delay_us(10);

    // 1 = data, 0 = command
    dc.set_high().unwrap();

    DMA_BUSY.store(1, Ordering::SeqCst);
    let tx_channel = &dma.ch[TX_CHANNEL as usize];
    let clear_channel = &dma.ch[CLEAR_CHANNEL as usize];
    clear_channel
        .ch_write_addr
        .write(|w| unsafe { w.bits(buffer.bytes() as *const u8 as u32) });
    tx_channel
        .ch_al3_read_addr_trig
        .write(|w| unsafe { w.bits(buffer.bytes() as *const u8 as u32) });
}

pub fn wait_for_dma_done() {
    while DMA_BUSY.load(Ordering::Acquire) != 0 {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    defmt_panic();
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

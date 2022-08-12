#![no_std]
#![no_main]

mod color;
pub mod display;
mod spinlocks;
mod usb_serial;
mod usb_serial_defmt;

use core::{
    panic::PanicInfo,
    sync::atomic::{AtomicU32, AtomicU8, Ordering},
};

use bhboard as bsp;
use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        multicore::{Multicore, Stack},
        pac, pwm,
        sio::Sio,
        spi,
        watchdog::Watchdog,
    },
    pac::{DMA, RESETS},
    ButtonA, ButtonB, ButtonX, ButtonY, Led,
};
use color::Color;
use cortex_m::delay::Delay;
use display::{RawDisplayBuffer, RawSpi};
use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    spi::FullDuplex,
    PwmPin,
};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use num_traits::cast::ToPrimitive;
use rp2040_hal::dma::DREQ_SPI0_TX;
use st7735::command::Instruction;
use usb_serial::UsbManager;

use crate::{
    color::Pixel,
    display::{DisplayBuffer, DisplayDevice},
};

pub struct LedAndButtons {
    pub led: Led,
    pub button_a: ButtonA,
    pub button_b: ButtonB,
    pub button_x: ButtonX,
    pub button_y: ButtonY,
}

static mut CORE1_STACK: Stack<4096> = Stack::new();
fn core1_task() -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    static mut RAW_DISPLAY_BUFFERS: [RawDisplayBuffer; 2] = [RawDisplayBuffer::new(); 2];
    let mut display_buffers: [DisplayBuffer; 2] = {
        let [buffer0, buffer1] = RAW_DISPLAY_BUFFERS;
        [DisplayBuffer::new(buffer0), DisplayBuffer::new(buffer1)]
    };

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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let led_and_buttons = LedAndButtons {
        led: pins.led.into_mode(),
        button_a: pins.button_a.into_mode(),
        button_b: pins.button_b.into_mode(),
        button_x: pins.button_x.into_mode(),
        button_y: pins.button_y.into_mode(),
    };

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

    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        40_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut display = DisplayDevice::new(
        &mut delay,
        spi,
        pins.display_spi_cs.into_mode(),
        pins.display_spi_dc.into_mode(),
        pins.display_reset.into_mode(),
        pins.display_spi_sck.into_mode(),
        pins.display_spi_copi.into_mode(),
    );

    let mut x = 40;
    let mut y = 40;
    display.display.set_address_window(0, 0, 127, 159);
    // Assert chip select pin
    let (mut spi, mut dc, mut cs) = display.display.spi.release();
    cs.set_low().unwrap();

    init_dma(&mut pac.RESETS, &mut pac.DMA);

    let mut clear_color = Color::BLACK;

    let mut counter = 0u16;

    loop {
        set_clear_color(clear_color.into());
        clear_color.red = clear_color.red.wrapping_add(1);
        if counter & 0x1f == 0 {
            defmt::debug!("Frame count: {}", counter);
        }
        counter = counter.wrapping_add(1);

        // At this point the DMA takes ownership over display_buffers[1]
        send_and_clear_buffer(
            &mut pac.DMA,
            &mut spi,
            &mut delay,
            &mut dc,
            &display_buffers[1],
        );

        // Simultaniously we update the world and draw to display_buffers[0]
        update_input(&mut x, &mut y, &led_and_buttons);
        display_buffers[0].draw_rect(y..y + 30, x..x + 30, Color::BLUE.into());

        // Wait for the dma to be done with display_buffers[1]
        wait_for_dma_done();

        // Swap the buffers to be ready for the next loop
        display_buffers.swap(0, 1);
    }
}

fn update_input(x: &mut usize, y: &mut usize, led_and_buttons: &LedAndButtons) {
    if led_and_buttons.button_a.is_high().unwrap_or(false) {
        *y = (*y + 160 + 1) % 160;
    }
    if led_and_buttons.button_b.is_high().unwrap_or(false) {
        *y = (*y + 160 - 1) % 160;
    }
    if led_and_buttons.button_x.is_high().unwrap_or(false) {
        *x = (*x + 128 + 1) % 128;
    }
    if led_and_buttons.button_y.is_high().unwrap_or(false) {
        *x = (*x + 128 - 1) % 128;
    }
}

const TX_CHANNEL: u8 = 0;
const CLEAR_CHANNEL: u8 = 1;
// TODO: Wasting an entire channel to check for when the DMA done is a bad solution.
// We should use interrupts instead.
const MARK_NOT_BUSY_CHANNEL: u8 = 2;
static DMA_BUSY: AtomicU8 = AtomicU8::new(0);
static CLEAR_COLOR: AtomicU32 = AtomicU32::new(0);

fn set_clear_color(pixel: Pixel) {
    let raw_pixel = pixel.raw_pixel();
    let doubled_raw_pixel = (raw_pixel as u32) | (raw_pixel as u32) << 16;
    CLEAR_COLOR.store(doubled_raw_pixel, Ordering::Relaxed);
}

fn init_dma(resets: &mut RESETS, dma: &mut DMA) {
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

fn send_and_clear_buffer(
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

fn wait_for_dma_done() {
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

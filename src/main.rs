#![no_std]
#![no_main]

pub mod display;
pub mod gameboy;

use core::sync::atomic::{AtomicU8, Ordering};

use bhboard as bsp;
use cortex_m::{delay::Delay, prelude::_embedded_hal_blocking_spi_Write};
use display::{RawDisplayBuffer, RawSpi};
use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use num_traits::cast::ToPrimitive;
use panic_probe as _;

use bsp::{
    entry,
    hal::{
        clocks::ClocksManager,
        gpio::Pin,
        multicore::{Multicore, Stack},
        pwm, spi, uart,
    },
    pac::DMA,
    ButtonA, ButtonB, ButtonX, ButtonY, Led,
};
use bsp::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio, pac,
        sio::Sio,
        watchdog::Watchdog,
    },
    pac::RESETS,
};
use rp2040_hal::dma::DREQ_SPI0_TX;
use st7735::command::Instruction;

use crate::display::{DisplayBuffer, DisplayDevice};

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
    static mut RAW_DISPLAY_BUFFERS: [RawDisplayBuffer; 2] = [[[0; 128]; 160]; 2];
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

    let led_and_buttons = LedAndButtons {
        led: pins.led.into_mode(),
        button_a: pins.button_a.into_mode(),
        button_b: pins.button_b.into_mode(),
        button_x: pins.button_x.into_mode(),
        button_y: pins.button_y.into_mode(),
    };

    // cortex_m::interrupt::free(|cs| {
    //     GLOBAL_PINS.borrow(cs).set(Some(led_and_buttons));
    // });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

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
    let rom = include_bytes!("../pokemon.gb");
    let rom = padme_core::Rom::load(rom.as_slice()).unwrap_or_else(|_| panic!());

    let mut emulator = padme_core::System::new(rom, gameboy::MySerialConsole, gameboy::MySpeaker);
    // Set the number of frame per seconds
    // This also sets the number of cycles needed per frame given the fixed CPU clock frequency
    emulator.set_frame_rate(5);

    loop {
        // At this point the DMA takes ownership over display_buffers[1]
        send_and_clear_buffer(
            &mut pac.DMA,
            &mut spi,
            &mut delay,
            &mut dc,
            &display_buffers[1],
            &0,
        );

        // Simultaniously we update the world and draw to display_buffers[0]
        emulator.set_button(
            padme_core::Button::A,
            led_and_buttons.button_x.is_high().unwrap(),
        );
        emulator.set_button(
            padme_core::Button::B,
            led_and_buttons.button_y.is_high().unwrap(),
        );
        emulator.set_button(
            padme_core::Button::Start,
            led_and_buttons.button_b.is_high().unwrap(),
        );

        emulator.update_frame(&mut display_buffers[0]);

        // update_input(&mut x, &mut y, &led_and_buttons);
        // display_buffers[0].draw_rect(y..y + 30, x..x + 30, 0xffff);

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
        .ch_trans_count
        .write(|w| unsafe { w.bits(128 * 160) });
    clear_channel.ch_al1_ctrl.write(|w| {
        w.incr_read().clear_bit();
        w.incr_write().set_bit();
        w.high_priority().clear_bit();
        w.data_size().size_halfword();
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
    clear_color: &u16,
) {
    // 1 = data, 0 = command
    dc.set_low().unwrap();

    // Send words over SPI
    spi.write(&[Instruction::RAMWR.to_u8().unwrap()]).unwrap();
    delay.delay_us(10);

    // 1 = data, 0 = command
    dc.set_high().unwrap();

    DMA_BUSY.store(1, Ordering::SeqCst);
    let tx_channel = &dma.ch[TX_CHANNEL as usize];
    let clear_channel = &dma.ch[CLEAR_CHANNEL as usize];
    clear_channel
        .ch_read_addr
        .write(|w| unsafe { w.bits(clear_color as *const u16 as u32) });
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

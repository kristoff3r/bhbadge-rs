#![no_std]
#![no_main]

use core::convert::Into;

use bhbadge::color::Color;
use bhbadge::usb_serial::UsbManager;
use bhbadge::y2022::display::RawDisplayBuffer;
use bhbadge::y2022::{
    display::{DisplayBuffer, DisplayDevice, RawSpi},
    get_current_dma_done, init_dma, send_and_clear_buffer, wait_for_dma_done, LedAndButtons,
    TX_CHANNEL,
};
use bhboard_2022 as bsp;
use bsp::{entry, pac::DMA};
use cortex_m::delay::Delay;
use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};
use padme_core::{AudioSpeaker, Screen, SerialOutput};
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    multicore::{Multicore, Stack},
    pac, pwm,
    sio::Sio,
    spi,
    watchdog::Watchdog,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();
fn core1_task() -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    static mut RAW_DISPLAY_BUFFERS: [RawDisplayBuffer; 2] = [RawDisplayBuffer::new(); 2];
    let display_buffers: [DisplayBuffer; 2] = {
        let [buffer0, buffer1] = RAW_DISPLAY_BUFFERS;
        [DisplayBuffer::new(buffer0), DisplayBuffer::new(buffer1)]
    };

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    defmt::timestamp!("{=u32:us}", unsafe {
        &(*rp2040_hal::pac::TIMER::ptr()).timelr.read().bits()
    });

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

    display.display.set_address_window(0, 0, 127, 159);

    // Assert chip select pin
    let (spi, dc, mut cs) = display.display.spi.release();
    cs.set_low().unwrap();

    init_dma(&mut pac.RESETS, &mut pac.DMA, false);
    let rom = core::include_bytes!("pokemon.gb");
    let rom = padme_core::Rom::load(rom.as_slice()).unwrap_or_else(|_| panic!());

    let mut emulator = padme_core::System::new(rom, MySerialConsole, MySpeaker);
    // Set the number of frame per seconds
    // This also sets the number of cycles needed per frame given the fixed CPU clock frequency
    emulator.set_frame_rate(60);

    let mut counter = 0u16;

    let mut my_display = MyDisplay {
        dma: pac.DMA,
        spi,
        delay,
        dc,
        display_buffers,
        expected: get_current_dma_done(TX_CHANNEL),
    };

    loop {
        defmt::debug!("Step {}", counter);
        counter = counter.wrapping_add(1);

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

        emulator.update_frame(&mut my_display);
    }
}

struct MyDisplay {
    dma: DMA,
    spi: RawSpi,
    delay: Delay,
    dc: bsp::DisplaySpiDc,
    display_buffers: [DisplayBuffer; 2],
    expected: bool,
}

impl Screen for MyDisplay {
    fn set_pixel(&mut self, pixel: &padme_core::Pixel, x: u8, y: u8) {
        if y >= 160 || x >= 128 {
            return;
        }
        let ptr = self.display_buffers[0].pixel_mut(y.into(), x.into());
        *ptr = Color {
            r: pixel.r,
            g: pixel.g,
            b: pixel.b,
        }
        .into();
    }

    fn update(&mut self) {
        // Wait for the dma to be done with display_buffers[1]
        wait_for_dma_done(TX_CHANNEL, self.expected);

        // Swap the buffers to be ready for the next loop
        self.display_buffers.swap(0, 1);

        self.expected = !get_current_dma_done(TX_CHANNEL);

        // At this point the DMA takes ownership over display_buffers[1]
        send_and_clear_buffer(
            &mut self.dma,
            &mut self.spi,
            &mut self.delay,
            &mut self.dc,
            &self.display_buffers[1],
        );
    }
}

pub struct MySpeaker;

impl AudioSpeaker for MySpeaker {
    fn set_samples(&mut self, _left: f32, _right: f32) {
        // add samples for left and right channels
    }
}

pub struct MySerialConsole;

impl SerialOutput for MySerialConsole {
    fn putchar(&mut self, _ch: u8) {
        // a byte has been transmitted
    }
}

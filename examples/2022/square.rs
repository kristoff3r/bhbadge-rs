#![no_std]
#![no_main]

use bhbadge::{
    color::Color,
    usb_serial::UsbManager,
    y2022::{
        display::{DisplayBuffer, DisplayDevice, RawDisplayBuffer},
        get_current_dma_done, init_dma, send_and_clear_buffer, set_clear_color, wait_for_dma_done,
        LedAndButtons, CLEAR_CHANNEL,
    },
};

use bhboard_2022 as bsp;
use bsp::entry;

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac, pwm,
    sio::Sio,
    spi,
    watchdog::Watchdog,
};

use embedded_hal::{
    digital::v2::{InputPin, OutputPin},
    PwmPin,
};
use embedded_time::{fixed_point::FixedPoint, rate::Extensions};

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

    init_dma(&mut pac.RESETS, &mut pac.DMA, true);

    let mut clear_color = Color::BLACK;

    let mut counter = 0u16;

    loop {
        set_clear_color(clear_color.into());
        clear_color.r = clear_color.r.wrapping_add(1);
        if counter & 0x1f == 0 {
            defmt::debug!("Frame count: {}", counter);
        }
        counter = counter.wrapping_add(1);

        let expected = !get_current_dma_done(CLEAR_CHANNEL);
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
        wait_for_dma_done(CLEAR_CHANNEL, expected);

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

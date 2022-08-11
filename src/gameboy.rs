use crate::display::DisplayBuffer;
use padme_core::{AudioSpeaker, Pixel, Screen, SerialOutput};

impl<'a> Screen for DisplayBuffer {
    fn set_pixel(&mut self, pixel: &Pixel, x: u8, y: u8) {
        if y >= 160 || x >= 128 {
            return;
        }
        let ptr = self.pixel_mut(y.into(), x.into());
        *ptr = pixel.r as u16 | (pixel.g as u16) << 6 | (pixel.b as u16) << 12;
    }
    fn update(&mut self) {}
}

pub struct MySpeaker;

impl AudioSpeaker for MySpeaker {
    fn set_samples(&mut self, left: f32, right: f32) {
        // add samples for left and right channels
    }
}

pub struct MySerialConsole;

impl SerialOutput for MySerialConsole {
    fn putchar(&mut self, ch: u8) {
        // a byte has been transmitted
    }
}

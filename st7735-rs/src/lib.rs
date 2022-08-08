#![no_std]

//! This crate provides a ST7735 driver to connect to TFT displays.
//!
//! Currently, there is support for using hardware SPI as well as software SPI to
//! communicate to the display. Note that using hardware SPI is much faster and
//! recommended to be used if supported by the connecting device.
//!
//! The driver also provides a simple graphics library which currently supports drawing the
//! following shapes:
//! * Rectangles (filled and border only)
//! * Circles (filled and border only)
//! * Lines (horizontal, vertical, and diagonal)
//! * Text (characters, strings)

pub mod color;
pub mod command;
pub mod fonts;

use crate::color::{Color, DefaultColor};
use crate::command::{Command, Instruction};
use crate::fonts::Font;

use core::cmp::{max, min};
use display_interface::DataFormat;
use embedded_hal::blocking::delay::DelayMs;
use num::ToPrimitive;
use num_derive::{FromPrimitive, ToPrimitive};

/// ST7735 driver to connect to TFT displays. The driver allows to draw simple shapes,
/// and reset the display.
pub struct ST7735<DISPLAY, const ROWSTART: u16, const COLSTART: u16> {
    pub spi: DISPLAY,
}

/// Display orientation.
#[derive(FromPrimitive, ToPrimitive)]
pub enum Orientation {
    Portrait = 0x00,
    Landscape = 0x60,
    PortraitSwapped = 0xC0,
    LandScapeSwapped = 0xA0,
}

impl<DISPLAY, const ROWSTART: u16, const COLSTART: u16> ST7735<DISPLAY, ROWSTART, COLSTART>
where
    DISPLAY: display_interface::WriteOnlyDataCommand,
{
    pub fn new<DELAY>(spi: DISPLAY, delay: &mut DELAY) -> Self
    where
        DELAY: DelayMs<u32>,
    {
        let mut display = ST7735 { spi };

        display.init(delay);
        display
    }

    /// Runs commands to initialize the display.
    fn init<DELAY>(&mut self, delay: &mut DELAY)
    where
        DELAY: DelayMs<u32>,
    {
        let init_commands: &[Command] = &[
            Command {
                instruction: Instruction::SWRESET,
                delay_ms: Some(150),
                arguments: &[],
            },
            Command {
                instruction: Instruction::SLPOUT,
                delay_ms: Some(500),
                arguments: &[],
            },
            Command {
                instruction: Instruction::FRMCTR1,
                delay_ms: None,
                arguments: &[0x01, 0x2C, 0x2D],
            },
            Command {
                instruction: Instruction::FRMCTR2,
                delay_ms: None,
                arguments: &[0x01, 0x2C, 0x2D],
            },
            Command {
                instruction: Instruction::FRMCTR3,
                delay_ms: None,
                arguments: &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D],
            },
            Command {
                instruction: Instruction::INVCTR,
                delay_ms: None,
                arguments: &[0x07],
            },
            Command {
                instruction: Instruction::PWCTR1,
                delay_ms: None,
                arguments: &[0xA2, 0x02, 0x84],
            },
            Command {
                instruction: Instruction::PWCTR2,
                delay_ms: None,
                arguments: &[0xC5],
            },
            Command {
                instruction: Instruction::PWCTR3,
                delay_ms: None,
                arguments: &[0x0A, 0x00],
            },
            Command {
                instruction: Instruction::PWCTR4,
                delay_ms: None,
                arguments: &[0x8A, 0x2A],
            },
            Command {
                instruction: Instruction::PWCTR5,
                delay_ms: None,
                arguments: &[0x8A, 0xEE],
            },
            Command {
                instruction: Instruction::VMCTR1,
                delay_ms: None,
                arguments: &[0x0E],
            },
            Command {
                instruction: Instruction::INVOFF,
                delay_ms: None,
                arguments: &[],
            },
            Command {
                instruction: Instruction::MADCTL,
                delay_ms: None,
                arguments: &[0x18],
            },
            Command {
                instruction: Instruction::COLMOD,
                delay_ms: None,
                arguments: &[0x05],
            },
            Command {
                instruction: Instruction::GMCTRP1,
                delay_ms: None,
                arguments: &[
                    0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00,
                    0x01, 0x03, 0x10,
                ],
            },
            Command {
                instruction: Instruction::GMCTRN1,
                delay_ms: None,
                arguments: &[
                    0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d, 0x2e, 0x2e, 0x37, 0x3f, 0x00,
                    0x00, 0x02, 0x10,
                ],
            },
            Command {
                instruction: Instruction::NORON,
                delay_ms: None,
                arguments: &[0x0A],
            },
            Command {
                instruction: Instruction::DISPON,
                delay_ms: Some(100),
                arguments: &[],
            },
            Command {
                instruction: Instruction::MADCTL,
                delay_ms: None,
                arguments: &[0xC0],
            },
        ];

        for cmd in init_commands {
            self.write_cmd(cmd.instruction, cmd.arguments);
            if let Some(delay_ms) = cmd.delay_ms {
                delay.delay_ms(delay_ms);
            }
        }
    }

    pub fn write_cmd(&mut self, instruction: Instruction, arguments: &[u8]) {
        self.spi
            .send_commands(DataFormat::U8(&[instruction.to_u8().unwrap()]))
            .unwrap();
        self.spi.send_data(DataFormat::U8(arguments)).unwrap();
    }

    fn write_data(&mut self, data: &[u8]) {
        let _ = self.spi.send_data(DataFormat::U8(data));
    }

    /// Writes a bulk of pixels to the display.
    fn write_bulk(&mut self, color: &Color, repetitions: u16, count: u16) {
        self.write_cmd(Instruction::RAMWR, &[]);

        for _ in 0..=count {
            let bytes: [u8; 2] = color.hex.to_be_bytes();
            let mut byte_array = [0; 1024];

            for i in 0..(repetitions as usize) {
                byte_array[2 * i] = bytes[0];
                byte_array[2 * i + 1] = bytes[1];
            }
            self.write_data(&byte_array[..repetitions as usize * 2]);
        }
    }

    /// Sets the color to be used.
    fn write_color(&mut self, color: &Color) {
        let bytes: [u8; 2] = color.hex.to_be_bytes();

        self.write_data(&bytes);
    }

    /// Sets the address window for the display.
    pub fn set_address_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        let x0 = (x0 + COLSTART).to_be_bytes();
        let x1 = (x1 + COLSTART).to_be_bytes();
        let y0 = (y0 + ROWSTART).to_be_bytes();
        let y1 = (y1 + ROWSTART).to_be_bytes();
        self.write_cmd(Instruction::CASET, &[x0[0], x0[1], x1[0], x1[1]]);
        self.write_cmd(Instruction::RASET, &[y0[0], y0[1], y1[0], y1[1]]);
    }

    /// Changes the display orientation.
    pub fn set_orientation(&mut self, orientation: &Orientation) {
        self.write_cmd(Instruction::MADCTL, &[orientation.to_u8().unwrap()]);
    }

    /// Draws a single pixel with the specified `color` at the defined coordinates on the display.
    pub fn draw_pixel(&mut self, x: u16, y: u16, color: &Color) {
        self.set_address_window(x, y, x, y);
        self.write_cmd(Instruction::RAMWR, &[]);
        self.write_color(color);
    }

    /// Draws a filled rectangle with the specified `color` on the display.
    pub fn draw_filled_rect(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: &Color) {
        let width = x1 - x0 + 1;
        let height = y1 - y0 + 1;
        self.set_address_window(x0, y0, x1, y1);
        self.write_bulk(color, width, height);
    }

    /// Draws a rectangle with the specified `color` as border color on the display.
    pub fn draw_rect(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: &Color) {
        self.draw_horizontal_line(x0, x1, y0, color);
        self.draw_horizontal_line(x0, x1, y1, color);
        self.draw_vertical_line(x0, y0, y1, color);
        self.draw_vertical_line(x1, y0, y1, color);
    }

    /// Draws a horizontal with the specified `color` between the provided coordinates on the display.
    pub fn draw_horizontal_line(&mut self, x0: u16, x1: u16, y: u16, color: &Color) {
        let length = x1 - x0 + 1;
        self.set_address_window(x0, y, x1, y);
        self.write_bulk(color, length, 1);
    }

    /// Draws a vertical with the specified `color` between the provided coordinates on the display.
    pub fn draw_vertical_line(&mut self, x: u16, y0: u16, y1: u16, color: &Color) {
        let length = y1 - y0 + 1;
        self.set_address_window(x, y0, x, y1);
        self.write_bulk(color, length, 1);
    }

    /// Draws a line with the specified `color` between the provided coordinates on the display.
    pub fn draw_line(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: &Color) {
        if x0 == x1 {
            self.draw_vertical_line(x0, y0, y1, color);
        } else if y0 == y1 {
            self.draw_horizontal_line(x0, x1, y1, color);
        } else {
            let m = ((max(y1, y0) - min(y0, y1)) as f32) / ((max(x1, x0) - min(x0, x1)) as f32);

            if m < 1.0 {
                for x in x0..=x1 {
                    let y = ((x - x0) as f32) * m + (y0 as f32);
                    self.draw_pixel(x, y as u16, color);
                }
            } else {
                for y in y0..=y1 {
                    let x = ((y - y0) as f32) / m + (x0 as f32);
                    self.draw_pixel(x as u16, y, color);
                }
            }
        }
    }

    /// Draws a circle whose border has the specified `color` around the provided coordinates on the display.
    pub fn draw_circle(&mut self, x_pos: u16, y_pos: u16, radius: u16, color: &Color) {
        let x_end = ((core::f32::consts::FRAC_1_SQRT_2 * (radius as f32)) + 1.0) as u16;

        for x in 0..x_end {
            let y = sqrt(radius * radius - x * x) as u16;
            let u_x = x as u16;
            self.draw_pixel(x_pos + u_x, y_pos + y, color);
            self.draw_pixel(x_pos + u_x, y_pos - y, color);
            self.draw_pixel(x_pos - u_x, y_pos + y, color);
            self.draw_pixel(x_pos - u_x, y_pos - y, color);
            self.draw_pixel(x_pos + y, y_pos + u_x, color);
            self.draw_pixel(x_pos + y, y_pos - u_x, color);
            self.draw_pixel(x_pos - y, y_pos + u_x, color);
            self.draw_pixel(x_pos - y, y_pos - u_x, color);
        }
    }

    /// Draws a circle filled with the specified `color` around the provided coordinates on the display.
    pub fn draw_filled_circle(&mut self, x_pos: u16, y_pos: u16, radius: u16, color: &Color) {
        let r2 = radius * radius;
        for x in 0..radius {
            let y = sqrt(r2 - x * x) as u16;
            let y0 = y_pos - y;
            let y1 = y_pos + y;
            self.draw_vertical_line(x_pos + x, y0, y1, color);
            self.draw_vertical_line(x_pos - x, y0, y1, color);
        }
    }

    /// Draws a character filled with the specified `color` and the defined font on the display.
    pub fn draw_character<F: Font>(&mut self, c: char, x: u16, y: u16, color: &Color, _font: F) {
        let character_data = <F as Font>::get_char(c);

        let mask = 0x01;

        for row in 0..7 {
            for col in 0..5 {
                let bit = character_data[col] & (mask << row);

                if bit != 0 {
                    self.draw_pixel(x + (col as u16), y + (row as u16), color);
                }
            }
        }
    }

    pub fn draw_string<F: Font>(&mut self, s: &str, x: u16, y: u16, color: &Color, _font: F) {
        for (i, c) in s.chars().enumerate() {
            self.draw_character(c, x + 5 * i as u16, y, color, _font);
        }
    }

    /// Fills the entire screen with the specified `color`.
    pub fn fill_screen(&mut self, color: &Color) {
        self.draw_filled_rect(0, 0, 127, 159, color);
    }

    /// Fills the entire screen black.
    pub fn clear_screen(&mut self) {
        self.fill_screen(&Color::from_default(DefaultColor::Black));
    }
}

fn ceil_log2(n: u16) -> u8 {
    16u8.saturating_sub(n.leading_zeros() as u8 + 1)
}

pub fn sqrt(s: u16) -> u8 {
    if s < 2 {
        return s as u8;
    }

    let n: u8 = (ceil_log2(s) + 1) / 2;
    let mut r: u16 = (1 << (n - 1)) + (s >> (n + 1));
    loop {
        if r * r <= s {
            match (r + 1).checked_mul(r + 1) {
                None => return r as u8,
                Some(p) if p > s => return r as u8,
                Some(_) => (),
            }
        }
        r = (r + s / r) / 2;
    }
}

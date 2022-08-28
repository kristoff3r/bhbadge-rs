#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Pixel(u16);

impl core::fmt::Debug for Pixel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let hex = u16::from(*self);
        f.debug_struct("Pixel")
            .field("r", &(hex >> 11))
            .field("g", &((hex >> 5) & 0x3f))
            .field("b", &(hex & 0x1f))
            .finish()
    }
}

impl From<u16> for Pixel {
    fn from(value: u16) -> Self {
        Self(value.to_be())
    }
}

impl From<Pixel> for u16 {
    fn from(pixel: Pixel) -> Self {
        u16::from_be(pixel.0)
    }
}

impl Pixel {
    pub const WHITE: Self = Self(0xffff);
    pub const BLACK: Self = Self(0);

    pub fn raw_pixel(&self) -> u16 {
        self.0
    }
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const WHITE: Self = Self::new(0xff, 0xff, 0xff);
    pub const BLACK: Self = Self::new(0, 0, 0);
    pub const RED: Self = Self::new(0xff, 0, 0);
    pub const GREEN: Self = Self::new(0, 0xff, 0);
    pub const BLUE: Self = Self::new(0, 0, 0xff);

    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self {
            r: red,
            g: green,
            b: blue,
        }
    }
}

impl From<Color> for Pixel {
    fn from(color: Color) -> Self {
        let r = ((color.r & 0xF8) as u16) >> 3;
        let g = ((color.g & 0xFC) as u16) >> 2;
        let b = ((color.b & 0xF8) as u16) >> 3;
        let hex: u16 = (r << 11) + (g << 5) + b;
        hex.into()
    }
}

impl From<Pixel> for Color {
    fn from(pixel: Pixel) -> Self {
        let hex: u16 = pixel.into();
        let blue = ((hex & 0x1f) as u8) << 3;
        let green = (((hex >> 5) & 0x3f) as u8) << 2;
        let red = (((hex >> 11) & 0x3f) as u8) << 3;
        Self {
            r: red,
            g: green,
            b: blue,
        }
    }
}

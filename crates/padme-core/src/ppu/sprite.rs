const FLAG_BGWIN_PRIO: u8 = 0b10000000;
const FLAG_Y_FLIP: u8 = 0b01000000;
const FLAG_X_FLIP: u8 = 0b00100000;
const FLAG_PALETTE_NUMBER: u8 = 0b00010000;

#[derive(Clone, Copy)]
pub struct Sprite {
    /// X coord
    pub x: u8,
    /// Y coord
    pub y: u8,
    /// Tile index in the data
    pub tile_index: u8,
    /// Tile attributes
    attrs: u8,
}

impl Sprite {
    pub fn new(x: u8, y: u8, tile_index: u8, attrs: u8) -> Self {
        Self {
            x,
            y,
            tile_index,
            attrs,
        }
    }

    #[inline]
    pub fn is_x_flipped(&self) -> bool {
        is_set!(self.attrs, FLAG_X_FLIP)
    }

    #[inline]
    pub fn is_y_flipped(&self) -> bool {
        is_set!(self.attrs, FLAG_Y_FLIP)
    }

    #[inline]
    pub fn is_bgwin_prio(&self) -> bool {
        is_set!(self.attrs, FLAG_BGWIN_PRIO)
    }

    #[inline]
    pub fn palette_number(&self) -> u8 {
        is_set!(self.attrs, FLAG_PALETTE_NUMBER) as u8
    }
}

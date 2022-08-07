pub mod font57;

/// Font trait implemented by fonts that can be used to display text on the display.
pub trait Font: Copy {
    /// Returns the bit representation of character `c` that can be displayed on the display.
    fn get_char(c: char) -> [u8; 5];
}

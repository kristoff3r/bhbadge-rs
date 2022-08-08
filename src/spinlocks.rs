use bhboard::hal::sio::Spinlock;

macro_rules! guards {
    ($($name:ident of $n:expr => $static:ident : $inner:ty = $default:expr;)*) => {
        $(
            pub struct $name {
                _spinlock: Spinlock<$n>,
            }

            static mut $static: $inner = $default;

            impl core::ops::Deref for $name {
                type Target = $inner;

                fn deref(&self) -> &$inner {
                    unsafe { &$static }
                }
            }

            impl core::ops::DerefMut for $name {
                fn deref_mut(&mut self) -> &mut $inner {
                    unsafe { &mut $static }
                }
            }

            impl $name {
                pub fn claim() -> $name {
                    $name { _spinlock: Spinlock::claim() }
                }
            }
        )*
    }
}

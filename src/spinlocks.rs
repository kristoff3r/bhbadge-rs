use rp2040_hal::sio::Spinlock;

macro_rules! spinlocks {
    ($($n:expr => $name:ident : $inner:ty = $default:expr;)*) => {
        $(
            pub(crate) struct $name {
                _spinlock: Spinlock<$n>,
            }

            mod spinlock_impl {
                use super::$name;

                static mut GLOBAL: $inner = $default;

                impl core::ops::Deref for $name {
                    type Target = $inner;

                    fn deref(&self) -> &$inner {
                        unsafe { &GLOBAL }
                    }
                }

                impl core::ops::DerefMut for $name {
                    fn deref_mut(&mut self) -> &mut $inner {
                        unsafe { &mut GLOBAL }
                    }
                }

                impl $name {
                    pub fn claim() -> $name {
                        $name { _spinlock: rp2040_hal::sio::Spinlock::<$n>::claim() }
                    }

                    pub fn try_claim() -> Option<$name> {
                        Some($name { _spinlock: rp2040_hal::sio::Spinlock::<$n>::try_claim()? })
                    }
                }
            }
        )*
    }
}

spinlocks!(
    0 => UsbSpinlock : Option<crate::usb_serial::UsbManager> = None;
);

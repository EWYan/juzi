use core::fmt;

struct QEMUOutput;

impl fmt::Write for QEMUOutput {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            unsafe {
                core::ptr::write_volatile(0xFF00_0030 as *mut u8, c as u8);
            }
        }
        Ok(())
    }
}

pub fn console_internal() -> impl interface::Write {
    QEMUOutput {}
}

pub mod interface {
    pub use core::fmt::Write;
}

pub fn console() -> impl interface::Write {
    console_internal()
}

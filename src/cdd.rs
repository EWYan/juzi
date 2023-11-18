// FFI CODE HERE

use crate::println;

extern "C" {
    fn c_hw_init();
}

pub fn hw_init() {
    unsafe {
        c_hw_init();
    }
}

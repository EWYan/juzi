// nothing

#![no_std]
#![no_main]
#![feature(format_args_nl)]
#![feature(asm_const)]
#![feature(panic_info_message)]

use core::panic::PanicInfo;
mod arch;
mod cdd;
mod console;
mod os;

pub mod print;

pub unsafe fn main() -> ! {
    cdd::hw_init();

    println!("ASR_RTOS Hello, Fisher!!!💖");

    loop {
        core::arch::asm!("nop");
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("Kernel panic !!!");
    println!("Reason    : {:?}", info.message().unwrap());
    println!("Location  : {:?}", info.location().unwrap());
    loop {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

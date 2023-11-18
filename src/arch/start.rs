use core::arch::global_asm;

global_asm!(".section .text.vec");
global_asm!(include_str!("vec.s"));

global_asm!(".section .text._start");
global_asm!(include_str!("boot.s"));

#[no_mangle]
pub unsafe fn _start_rs() -> ! {
    crate::main();
}

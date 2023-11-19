#include "gic.h"
static inline void set_low_vec(void)
{
    int val;
    __asm volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(val)::);
    val &= ~(1 << 13);
    __asm volatile("dsb");
    __asm volatile("mcr p15, 0, %0, c1, c0, 0" ::"r"(val) :);
    __asm volatile("isb");
}
extern void sys_clock_driver_init( void );
extern void uart_init(void);
void c_hw_init(void)
{
    // config uart for log print
    uart_init();
    // config gic
    arm_gic_init();
    // config timer device
    sys_clock_driver_init();
    // set irq vector address low mode
    set_low_vec();
    // enable cpu irq
    __asm volatile("cpsie i");
}

#include "gic.h"

#define NULL ((void *) 0)
#define IRQ_CNT     255
struct irq_entry {
    const void *arg;
    void (*isr) (const void *);
};

void z_irq_spurious(const void *unused) {
    while(1);
}
void undefined_irq(const void *unused) {
    while(1);
}

struct irq_entry _sw_isr_table[IRQ_CNT] = {0};

void arch_irq_enable(unsigned int irq) {
    arm_gic_irq_enable(irq);
}

void arch_irq_disable(unsigned int irq) {
    arm_gic_irq_disable(irq);
}

// void arch_gic_irq_set_priority(unsigned int irq,unsigned int priority, unsigned int flags) {
//     arm_gic_irq_set_priority(irq, priority, flags);
// }

void irq_connect(unsigned int irq, unsigned int priority, void *arg, void (*isr)(const void *)) {
    _sw_isr_table[irq].arg = arg;
    _sw_isr_table[irq].isr = isr;
    arm_gic_irq_set_priority(irq, priority, 0);
    arm_gic_irq_enable(irq);
}

void entry_init(void) {
    unsigned int cnt = 0;
    for(;cnt < IRQ_CNT; cnt++) {
        _sw_isr_table[cnt].arg = 0;
        _sw_isr_table[cnt].isr = undefined_irq;
    }
}

void vApplicationIRQHandler(unsigned int irq, void *arg) {
    if (irq > IRQ_CNT) return;
    _sw_isr_table[irq].isr(arg);
}

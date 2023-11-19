#include "common.h"
#include "gic.h"

struct irq_entry
{
	const void *arg;
	void (*isr)(const void *);
};

struct irq_entry _sw_isr_table[256] = {0};

void irq_connect(unsigned int irq_num, unsigned int prio, void *arg, void (*isr)(const void *))
{
	_sw_isr_table[irq_num].arg = arg;
	_sw_isr_table[irq_num].isr = isr;
	arm_gic_irq_set_priority(irq_num, prio, 0);
	arm_gic_irq_enable(irq_num);
}

void irq_handle(unsigned int irq_num, void *param)
{
	if (irq_num > 256)
		return;
	_sw_isr_table[irq_num].isr(param);
}

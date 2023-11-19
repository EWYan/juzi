/*
 * Copyright (c) 2018 Marvell
 * Copyright (c) 2018 Lexmark International, Inc.
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * NOTE: This driver implements the GICv1 and GICv2 interfaces.
 */

#include "common.h"
#include "gic.h"

__attribute__((used)) const unsigned int ulICCIAR = 0xF900100C;
__attribute__((used)) const unsigned int ulICCEOIR = 0xF9001010;


static const uint64_t cpu_mpid_list[] = {0};

void arm_gic_irq_enable(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (GICD_ISENABLERn + int_grp * 4));
}

void arm_gic_irq_disable(unsigned int irq)
{
	int int_grp, int_off;

	int_grp = irq / 32;
	int_off = irq % 32;

	sys_write32((1 << int_off), (GICD_ICENABLERn + int_grp * 4));
}

bool arm_gic_irq_is_enabled(unsigned int irq)
{
	int int_grp, int_off;
	unsigned int enabler;

	int_grp = irq / 32;
	int_off = irq % 32;

	enabler = sys_read32(GICD_ISENABLERn + int_grp * 4);

	return (enabler & (1 << int_off)) != 0;
}

void arm_gic_irq_set_priority(unsigned int irq, unsigned int prio, uint32_t flags)
{
	int int_grp, int_off;
	uint32_t val;

	/* Set priority */
	sys_write8(prio & 0xff, GICD_IPRIORITYRn + irq);

	/* Set interrupt type */
	int_grp = (irq / 16) * 4;
	int_off = (irq % 16) * 2;

	val = sys_read32(GICD_ICFGRn + int_grp);
	val &= ~(GICD_ICFGR_MASK << int_off);
	if (flags & IRQ_TYPE_EDGE)
	{
		val |= (GICD_ICFGR_TYPE << int_off);
	}

	sys_write32(val, GICD_ICFGRn + int_grp);
}

unsigned int arm_gic_get_active(void)
{
	int irq;

	irq = sys_read32(GICC_IAR) & 0x3ff;
	return irq;
}

void arm_gic_eoi(unsigned int irq)
{
	/*
	 * Ensure the write to peripheral registers are *complete* before the write
	 * to GIC_EOIR.
	 *
	 * Note: The completion gurantee depends on various factors of system design
	 * and the barrier is the best core can do by which execution of further
	 * instructions waits till the barrier is alive.
	 */
	__DSB();

	/* set to inactive */
	sys_write32(irq, GICC_EOIR);
}

void gic_raise_sgi(unsigned int sgi_id, uint64_t target_aff, uint16_t target_list)
{
	uint32_t sgi_val;

	sgi_val = GICD_SGIR_TGTFILT_CPULIST |
			  GICD_SGIR_CPULIST(target_list & GICD_SGIR_CPULIST_MASK) |
			  sgi_id;

	__DSB();
	sys_write32(sgi_val, GICD_SGIR);
	__ISB();
}

static void gic_dist_init(void)
{
	unsigned int gic_irqs, i;
	uint8_t cpu_mask = 0;
	uint32_t reg_val = 0;

	gic_irqs = sys_read32(GICD_TYPER) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32; // The maximum number of INTIDs. For the INTID range 32 to 1019, indicates the maximum SPI supported. If the value of this field is N, the maximum SPI INTID is 32(N+1) minus 1. For example, 00011 specifies that the maximum SPI INTID in is 127
	if (gic_irqs > 1020)
	{
		gic_irqs = 1020;
	}

	/*
	 * Disable the forwarding of pending interrupts
	 * from the Distributor to the CPU interfaces
	 */
	sys_write32(0, GICD_CTLR);

	/*
	 * Enable all global interrupts distributing to CPUs listed
	 * in dts with the count of CONFIG_MP_NUM_CPUS.
	 */
	for (i = 0; i < 1; i++)
	{
		cpu_mask |= BIT(cpu_mpid_list[i]);
	}
	/*
		In a uniprocessor implementation, all interrupts target the one processor, and the
		GICD_ITARGETSRs are RAZ/WI. (Read As Zero, Write Ignore)
	*/
	for (i = 0; i < 3; i++)
	{
		reg_val |= cpu_mask << (8 * i);
	}
	// uart_send(itoa(reg_val, "", 16));
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 4)
	{
		sys_write32(reg_val, GICD_ITARGETSRn + i);
	}

	/*
	 * Set all global interrupts to be level triggered, active low.
	 * 0b00: level senstive
	 * 0b10: edge-triggered
	 */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 16)
	{
		sys_write32(0, GICD_ICFGRn + i / 4);
	}

	/*  Set priority on all global interrupts.   */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 4)
	{
		sys_write32(0, GICD_IPRIORITYRn + i);
	}

	/* Set all interrupts to group 0 */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 32)
	{
		sys_write32(0, GICD_IGROUPRn + i / 8);
	}

	/*
	 * Disable all interrupts.  Leave the PPI and SGIs alone
	 * as these enables are banked registers.
	 */
	for (i = GIC_SPI_INT_BASE; i < gic_irqs; i += 32)
	{
#ifndef CONFIG_GIC_V1
		sys_write32(0xffffffff, GICD_ICACTIVERn + i / 8);
#endif
		sys_write32(0xffffffff, GICD_ICENABLERn + i / 8);
	}

	/*
	 * Enable the forwarding of pending interrupts
	 * from the Distributor to the CPU interfaces
	 */
	sys_write32(3, GICD_CTLR);
}

static void gic_cpu_init(void)
{
	int i;
	uint32_t val;

	/*
	 * Deal with the banked PPI and SGI interrupts - disable all
	 * PPI interrupts, ensure all SGI interrupts are enabled.
	 */
// V2
#ifndef CONFIG_GIC_V1
	sys_write32(0xffffffff, GICD_ICACTIVERn);
#endif
	sys_write32(0xffff0000, GICD_ICENABLERn);
	sys_write32(0x0000ffff, GICD_ISENABLERn);

	/*
	 * Set priority on PPI and SGI interrupts
	 */
	for (i = 0; i < 32; i += 4)
	{
		sys_write32(0xa0a0a0a0, GICD_IPRIORITYRn + i);
	}

	sys_write32(0xff, GICC_PMR); // 240

	/*
	 * Enable interrupts and signal them using the IRQ signal.
	 */
	val = sys_read32(GICC_CTLR);
#ifndef CONFIG_GIC_V1
	val &= ~GICC_CTLR_BYPASS_MASK; // 0b1 1110 0000   0b0 0001 1111
#endif
	val |= GICC_CTLR_ENABLE_MASK; // 0b11 GRP0, GRP1
	sys_write32(val, GICC_CTLR);
}

/**
 *
 * @brief Initialize the GIC device driver
 *
 *
 * @return N/A
 */
#define GIC_PARENT_IRQ 0
#define GIC_PARENT_IRQ_PRI 0
#define GIC_PARENT_IRQ_FLAGS 0
int arm_gic_init(void)
{
	/* Init of Distributor interface registers */
	gic_dist_init();

	/* Init CPU interface registers */
	gic_cpu_init();

	return 0;
}

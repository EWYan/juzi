/*
 * Copyright (c) 2020 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (c) 2018 Xilinx, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "common.h"
#include "xlnx_psttc_timer_priv.h"

#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 1000
#define CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC 0x2faf080 // 0x17d7840


uint32_t sys_clock_cycle_get_32(void);

extern void irq_connect(unsigned int irq_num, unsigned int prio, void *arg, void (*isr)(const void *));

#define TIMER_IRQ 0x44				   // DT_INST_IRQN(0)
#define TIMER_BASE_ADDR 0xFF110000	   // DT_INST_REG_ADDR(0)
#define TIMER_CLOCK_FREQUECY 0x2faf080 // 0x17d7840 		// DT_INST_PROP(0, clock_frequency)

#define TICKS_PER_SEC CONFIG_SYS_CLOCK_TICKS_PER_SEC	 // 1000
#define CYCLES_PER_SEC TIMER_CLOCK_FREQUECY				 // 25,000,000
#define CYCLES_PER_TICK (CYCLES_PER_SEC / TICKS_PER_SEC) // 25,000

/*
 * CYCLES_NEXT_MIN must be large enough to ensure that the timer does not miss
 * interrupts.  This value was conservatively set using the trial and error
 * method, and there is room for improvement.
 */
#define CYCLES_NEXT_MIN (10000)
#define CYCLES_NEXT_MAX (XTTC_MAX_INTERVAL_COUNT)

static uint32_t read_count(void)
{
	/* Read current counter value */
	return sys_read32(TIMER_BASE_ADDR + XTTCPS_COUNT_VALUE_OFFSET);
}

static void update_match(uint32_t cycles, uint32_t match)
{
	uint32_t delta = match - cycles;

	/* Ensure that the match value meets the minimum timing requirements */
	if (delta < CYCLES_NEXT_MIN)
	{
		match += CYCLES_NEXT_MIN - delta;
	}

	/* Write counter match value for interrupt generation */
	sys_write32(match, TIMER_BASE_ADDR + XTTCPS_MATCH_0_OFFSET);
}

static void ttc_isr(const void *args)
{
	uint32_t cycles;

	/* Acknowledge interrupt */
	sys_read32(TIMER_BASE_ADDR + XTTCPS_ISR_OFFSET);

	/* Read counter value */
	cycles = read_count();

	/* Update counter match value for the next interrupt */
	update_match(cycles, cycles + CYCLES_PER_TICK);

	extern void freertos_tick_handler(void);
	freertos_tick_handler();
}

void sys_clock_driver_init(void)
{
	uint32_t reg_val;

	/* Stop timer */
	sys_write32(XTTCPS_CNT_CNTRL_DIS_MASK, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

	/* Initialise timer registers */
	sys_write32(XTTCPS_CNT_CNTRL_RESET_VALUE, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET); // 0x11?
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_CLK_CNTRL_OFFSET);							  // 000 0000
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_INTERVAL_VAL_OFFSET);						  // Interval = 0
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_MATCH_0_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_MATCH_1_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_MATCH_2_OFFSET);
	sys_write32(0, TIMER_BASE_ADDR + XTTCPS_IER_OFFSET);				   // Interrupt enable = 0
	sys_write32(XTTCPS_IXR_ALL_MASK, TIMER_BASE_ADDR + XTTCPS_ISR_OFFSET); // Interrupt Status Register 01 1111, all but event overflow

	/* Reset counter value */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);
	reg_val |= XTTCPS_CNT_CNTRL_RST_MASK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

	/* Set match mode */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);
	reg_val |= XTTCPS_CNT_CNTRL_MATCH_MASK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

	/* Set initial timeout for match mode */
	reg_val = CYCLES_PER_TICK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_MATCH_0_OFFSET);

	/* Connect timer interrupt */
	irq_connect(TIMER_IRQ, 0, NULL, ttc_isr);

	/* Enable timer interrupt for match mode */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_IER_OFFSET);
	reg_val |= XTTCPS_IXR_MATCH_0_MASK;
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_IER_OFFSET);

	/* Start timer */
	reg_val = sys_read32(TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);
	reg_val &= (~XTTCPS_CNT_CNTRL_DIS_MASK);
	sys_write32(reg_val, TIMER_BASE_ADDR + XTTCPS_CNT_CNTRL_OFFSET);

	return;
}

uint32_t sys_clock_cycle_get_32(void)
{
	/* Return the current counter value */
	return read_count();
}

#include <stdint.h>
#include "common.h"

typedef uint32_t mem_addr_t;
#define dmb() __asm__ volatile("dmb sy" ::: "memory")
#define __DMB() dmb()

static inline uint32_t sys_read32(mem_addr_t addr)
{
	return *(volatile uint32_t *)addr;
}

static inline void sys_write32(uint32_t data, mem_addr_t addr)
{
	*(volatile uint32_t *)addr = data;
}

#define DT_DRV_COMPAT xlnx_xuartps
/** @brief Type used to represent devices and functions.
 *
 * The extreme values and zero have special significance. Negative
 * values identify functionality that does not correspond to a Zephyr
 * device, such as the system clock or a SYS_INIT() function.
 */
typedef int16_t device_handle_t;

/**
 * @brief Runtime device structure (in ROM) per driver instance
 */
struct device
{
	/** Name of the device instance */
	const char *name;
	/** Address of device instance config information */
	const void *config;
	/** Address of the API structure exposed by the device instance */
	const void *api;
	/** Address of the common device state */
	struct device_state *const state;
	/** Address of the device instance private data */
	void *const data;
	/** optional pointer to handles associated with the device.
	 *
	 * This encodes a sequence of sets of device handles that have
	 * some relationship to this node. The individual sets are
	 * extracted with dedicated API, such as
	 * device_required_handles_get().
	 */
	const device_handle_t *const handles;
#ifdef CONFIG_PM_DEVICE
	/** Power Management function */
	pm_device_control_callback_t pm_control;
	/** Pointer to device instance power management data */
	struct pm_device *const pm;
#endif
};

/** Device configuration structure */
struct uart_xlnx_ps_dev_config
{
	uint32_t reg;
	uint32_t sys_clk_freq;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
	uint32_t baud_rate;
};

/** Device data structure */
struct uart_xlnx_ps_dev_data_t
{
	uint32_t parity;
	uint32_t stopbits;
	uint32_t databits;
	uint32_t flowctrl;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
};

int uart_xlnx_ps_init(const struct device *dev);

int uart_xlnx_ps_poll_in(const struct device *dev, unsigned char *c);

void uart_xlnx_ps_poll_out(const struct device *dev, unsigned char c);
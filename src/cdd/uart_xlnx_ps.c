/* uart_xlnx_ps.c - Xilinx Zynq family serial driver */

/*
 * Copyright (c) 2018 Xilinx, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Xilinx Zynq Family Serial Driver
 *
 * This is the driver for the Xilinx Zynq family cadence serial device.
 *
 * Before individual UART port can be used, uart_xlnx_ps_init() has to be
 * called to setup the port.
 *
 * - the following macro for the number of bytes between register addresses:
 *
 *  UART_REG_ADDR_INTERVAL
 */

#include "common.h"
#include "uart_xlnx_ps.h"

/* For all register offsets and bits / bit masks:
 * Comp. Xilinx Zynq-7000 Technical Reference Manual (ug585), chap. B.33
 */

/* Register offsets within the UART device's register space */
#define XUARTPS_CR_OFFSET 0x0000U	   /**< Control Register [8:0] */
#define XUARTPS_MR_OFFSET 0x0004U	   /**< Mode Register [9:0] */
#define XUARTPS_IER_OFFSET 0x0008U	   /**< Interrupt Enable [12:0] */
#define XUARTPS_IDR_OFFSET 0x000CU	   /**< Interrupt Disable [12:0] */
#define XUARTPS_IMR_OFFSET 0x0010U	   /**< Interrupt Mask [12:0] */
#define XUARTPS_ISR_OFFSET 0x0014U	   /**< Interrupt Status [12:0]*/
#define XUARTPS_BAUDGEN_OFFSET 0x0018U /**< Baud Rate Generator [15:0] */
#define XUARTPS_RXTOUT_OFFSET 0x001CU  /**< RX Timeout [7:0] */
#define XUARTPS_RXWM_OFFSET 0x0020U	   /**< RX FIFO Trigger Level [5:0] */
#define XUARTPS_MODEMCR_OFFSET 0x0024U /**< Modem Control [5:0] */
#define XUARTPS_MODEMSR_OFFSET 0x0028U /**< Modem Status [8:0] */
#define XUARTPS_SR_OFFSET 0x002CU	   /**< Channel Status [14:0] */
#define XUARTPS_FIFO_OFFSET 0x0030U	   /**< FIFO [7:0] */
#define XUARTPS_BAUDDIV_OFFSET 0x0034U /**< Baud Rate Divider [7:0] */
#define XUARTPS_FLOWDEL_OFFSET 0x0038U /**< Flow Delay [5:0] */
#define XUARTPS_TXWM_OFFSET 0x0044U	   /**< TX FIFO Trigger Level [5:0] */
#define XUARTPS_RXBS_OFFSET 0x0048U	   /**< RX FIFO Byte Status [11:0] */

/* Control Register Bits Definition */
#define XUARTPS_CR_STOPBRK 0x00000100U	   /**< Stop transmission of break */
#define XUARTPS_CR_STARTBRK 0x00000080U	   /**< Set break */
#define XUARTPS_CR_TORST 0x00000040U	   /**< RX timeout counter restart */
#define XUARTPS_CR_TX_DIS 0x00000020U	   /**< TX disabled. */
#define XUARTPS_CR_TX_EN 0x00000010U	   /**< TX enabled */
#define XUARTPS_CR_RX_DIS 0x00000008U	   /**< RX disabled. */
#define XUARTPS_CR_RX_EN 0x00000004U	   /**< RX enabled */
#define XUARTPS_CR_EN_DIS_MASK 0x0000003CU /**< Enable/disable Mask */
#define XUARTPS_CR_TXRST 0x00000002U	   /**< TX logic reset */
#define XUARTPS_CR_RXRST 0x00000001U	   /**< RX logic reset */

/* Mode Register Bits Definition */
#define XUARTPS_MR_CCLK 0x00000400U				/**< Input clock select */
#define XUARTPS_MR_CHMODE_R_LOOP 0x00000300U	/**< Remote loopback mode */
#define XUARTPS_MR_CHMODE_L_LOOP 0x00000200U	/**< Local loopback mode */
#define XUARTPS_MR_CHMODE_ECHO 0x00000100U		/**< Auto echo mode */
#define XUARTPS_MR_CHMODE_NORM 0x00000000U		/**< Normal mode */
#define XUARTPS_MR_CHMODE_SHIFT 8U				/**< Mode shift */
#define XUARTPS_MR_CHMODE_MASK 0x00000300U		/**< Mode mask */
#define XUARTPS_MR_STOPMODE_2_BIT 0x00000080U	/**< 2 stop bits */
#define XUARTPS_MR_STOPMODE_1_5_BIT 0x00000040U /**< 1.5 stop bits */
#define XUARTPS_MR_STOPMODE_1_BIT 0x00000000U	/**< 1 stop bit */
#define XUARTPS_MR_STOPMODE_SHIFT 6U			/**< Stop bits shift */
#define XUARTPS_MR_STOPMODE_MASK 0x000000A0U	/**< Stop bits mask */
#define XUARTPS_MR_PARITY_NONE 0x00000020U		/**< No parity mode */
#define XUARTPS_MR_PARITY_MARK 0x00000018U		/**< Mark parity mode */
#define XUARTPS_MR_PARITY_SPACE 0x00000010U		/**< Space parity mode */
#define XUARTPS_MR_PARITY_ODD 0x00000008U		/**< Odd parity mode */
#define XUARTPS_MR_PARITY_EVEN 0x00000000U		/**< Even parity mode */
#define XUARTPS_MR_PARITY_SHIFT 3U				/**< Parity setting shift */
#define XUARTPS_MR_PARITY_MASK 0x00000038U		/**< Parity mask */
#define XUARTPS_MR_CHARLEN_6_BIT 0x00000006U	/**< 6 bits data */
#define XUARTPS_MR_CHARLEN_7_BIT 0x00000004U	/**< 7 bits data */
#define XUARTPS_MR_CHARLEN_8_BIT 0x00000000U	/**< 8 bits data */
#define XUARTPS_MR_CHARLEN_SHIFT 1U				/**< Data Length shift */
#define XUARTPS_MR_CHARLEN_MASK 0x00000006U		/**< Data length mask */
#define XUARTPS_MR_CLKSEL 0x00000001U			/**< Input clock select */

/* Interrupt Register Bits Definition */
#define XUARTPS_IXR_RBRK 0x00002000U	/**< Rx FIFO break detect interrupt */
#define XUARTPS_IXR_TOVR 0x00001000U	/**< Tx FIFO Overflow interrupt */
#define XUARTPS_IXR_TNFUL 0x00000800U	/**< Tx FIFO Nearly Full interrupt */
#define XUARTPS_IXR_TTRIG 0x00000400U	/**< Tx Trig interrupt */
#define XUARTPS_IXR_DMS 0x00000200U		/**< Modem status change interrupt */
#define XUARTPS_IXR_TOUT 0x00000100U	/**< Timeout error interrupt */
#define XUARTPS_IXR_PARITY 0x00000080U	/**< Parity error interrupt */
#define XUARTPS_IXR_FRAMING 0x00000040U /**< Framing error interrupt */
#define XUARTPS_IXR_RXOVR 0x00000020U	/**< Overrun error interrupt */
#define XUARTPS_IXR_TXFULL 0x00000010U	/**< TX FIFO full interrupt. */
#define XUARTPS_IXR_TXEMPTY 0x00000008U /**< TX FIFO empty interrupt. */
#define XUARTPS_IXR_RXFULL 0x00000004U	/**< RX FIFO full interrupt. */
#define XUARTPS_IXR_RXEMPTY 0x00000002U /**< RX FIFO empty interrupt. */
#define XUARTPS_IXR_RTRIG 0x00000001U	/**< RX FIFO trigger interrupt. */
#define XUARTPS_IXR_MASK 0x00003FFFU	/**< Valid bit mask */

/* Modem Control Register Bits Definition */
#define XUARTPS_MODEMCR_FCM_RTS_CTS 0x00000020 /**< RTS/CTS hardware flow control. */
#define XUARTPS_MODEMCR_FCM_NONE 0x00000000	   /**< No hardware flow control. */
#define XUARTPS_MODEMCR_FCM_MASK 0x00000020	   /**< Hardware flow control mask. */
#define XUARTPS_MODEMCR_RTS_SHIFT 1U		   /**< RTS bit shift */
#define XUARTPS_MODEMCR_DTR_SHIFT 0U		   /**< DTR bit shift */

/* Channel Status Register */
#define XUARTPS_SR_TNFUL 0x00004000U   /**< TX FIFO Nearly Full Status */
#define XUARTPS_SR_TTRIG 0x00002000U   /**< TX FIFO Trigger Status */
#define XUARTPS_SR_FLOWDEL 0x00001000U /**< RX FIFO fill over flow delay */
#define XUARTPS_SR_TACTIVE 0x00000800U /**< TX active */
#define XUARTPS_SR_RACTIVE 0x00000400U /**< RX active */
#define XUARTPS_SR_TXFULL 0x00000010U  /**< TX FIFO full */
#define XUARTPS_SR_TXEMPTY 0x00000008U /**< TX FIFO empty */
#define XUARTPS_SR_RXFULL 0x00000004U  /**< RX FIFO full */
#define XUARTPS_SR_RXEMPTY 0x00000002U /**< RX FIFO empty */
#define XUARTPS_SR_RTRIG 0x00000001U   /**< RX FIFO fill over trigger */

/**
 * @brief Disables the UART's RX and TX function.
 *
 * Writes 'Disable RX' and 'Disable TX' command bits into the respective
 * UART's Command Register, thus disabling the operation of the UART.
 *
 * While writing the disable command bits, the opposing enable command
 * bits, which are set when enabling the UART, are cleared.
 *
 * This function must be called before any configuration parameters
 * of the UART are modified at run-time.
 *
 * @param reg_base Base address of the respective UART's register space.
 */
static void xlnx_ps_disable_uart(uint32_t reg_base)
{
	uint32_t regval;

	regval = sys_read32(reg_base + XUARTPS_CR_OFFSET);
	regval &= (~XUARTPS_CR_EN_DIS_MASK);
	/* Set control register bits [5]: TX_DIS and [3]: RX_DIS */
	regval |= XUARTPS_CR_TX_DIS | XUARTPS_CR_RX_DIS;
	sys_write32(regval, reg_base + XUARTPS_CR_OFFSET);
}

/**
 * @brief Enables the UART's RX and TX function.
 *
 * Writes 'Enable RX' and 'Enable TX' command bits into the respective
 * UART's Command Register, thus enabling the operation of the UART.
 *
 * While writing the enable command bits, the opposing disable command
 * bits, which are set when disabling the UART, are cleared.
 *
 * This function must not be called while any configuration parameters
 * of the UART are being modified at run-time.
 *
 * @param reg_base Base address of the respective UART's register space.
 */
static void xlnx_ps_enable_uart(uint32_t reg_base)
{
	uint32_t regval;

	regval = sys_read32(reg_base + XUARTPS_CR_OFFSET);
	regval &= (~XUARTPS_CR_EN_DIS_MASK);
	/* Set control register bits [4]: TX_EN and [2]: RX_EN */
	regval |= XUARTPS_CR_TX_EN | XUARTPS_CR_RX_EN;
	sys_write32(regval, reg_base + XUARTPS_CR_OFFSET);
}

/**
 * @brief Calculates and sets the values of the BAUDDIV and BAUDGEN registers.
 *
 * Calculates and sets the values of the BAUDDIV and BAUDGEN registers, which
 * determine the prescaler applied to the clock driving the UART, based on
 * the target baud rate, which is provided as a decimal value.
 *
 * The calculation of the values to be written to the BAUDDIV and BAUDGEN
 * registers is described in the Zynq-7000 TRM, chapter 19.2.3 'Baud Rate
 * Generator'.
 *
 * @param dev UART device struct
 * @param baud_rate The desired baud rate as a decimal value
 */
static void set_baudrate(const struct device *dev, uint32_t baud_rate)
{
	const struct uart_xlnx_ps_dev_config *dev_cfg = dev->config;
	uint32_t divisor, generator;
	uint32_t baud;
	uint32_t clk_freq;
	uint32_t reg_base;

	baud = dev_cfg->baud_rate;
	clk_freq = dev_cfg->sys_clk_freq;

	/* Calculate divisor and baud rate generator value */
	if ((baud != 0) && (clk_freq != 0))
	{
		/* Covering case where input clock is so slow */
		if (clk_freq < 1000000U && baud > 4800U)
		{
			baud = 4800;
		}

		for (divisor = 4; divisor < 255; divisor++)
		{
			uint32_t tmpbaud, bauderr;

			generator = clk_freq / (baud * (divisor + 1));
			if (generator < 2 || generator > 65535)
			{
				continue;
			}
			tmpbaud = clk_freq / (generator * (divisor + 1));

			if (baud > tmpbaud)
			{
				bauderr = baud - tmpbaud;
			}
			else
			{
				bauderr = tmpbaud - baud;
			}
			if (((bauderr * 100) / baud) < 3)
			{
				break;
			}
		}

		/*
		 * Set baud rate divisor and generator.
		 * -> This function is always called from a context in which
		 * the receiver/transmitter is disabled, the baud rate can
		 * be changed safely at this time.
		 */

		reg_base = dev_cfg->reg;
		sys_write32(divisor, reg_base + XUARTPS_BAUDDIV_OFFSET);
		sys_write32(generator, reg_base + XUARTPS_BAUDGEN_OFFSET);
	}
}

/**
 * @brief Initialize individual UART port
 *
 * This routine is called to reset the chip in a quiescent state.
 *
 * @param dev UART device struct
 *
 * @return 0 if successful, failed otherwise
 */
int uart_xlnx_ps_init(const struct device *dev)
{
	const struct uart_xlnx_ps_dev_config *dev_cfg = dev->config;
	uint32_t reg_val;
	uint32_t reg_base;

	reg_base = dev_cfg->reg;

	/* Disable RX/TX before changing any configuration data */
	xlnx_ps_disable_uart(reg_base);

	/* Set initial character length / start/stop bit / parity configuration */
	reg_val = sys_read32(reg_base + XUARTPS_MR_OFFSET);
	reg_val &= (~(XUARTPS_MR_CHARLEN_MASK | XUARTPS_MR_STOPMODE_MASK |
				  XUARTPS_MR_PARITY_MASK));
	reg_val |= XUARTPS_MR_CHARLEN_8_BIT | XUARTPS_MR_STOPMODE_1_BIT |
			   XUARTPS_MR_PARITY_NONE;
	sys_write32(reg_val, reg_base + XUARTPS_MR_OFFSET);

	/* Set RX FIFO trigger at 1 data bytes. */
	sys_write32(0x01U, reg_base + XUARTPS_RXWM_OFFSET);

	/* Disable all interrupts, polling mode is default */
	sys_write32(XUARTPS_IXR_MASK, reg_base + XUARTPS_IDR_OFFSET);

	/* Set the baud rate */
	set_baudrate(dev, dev_cfg->baud_rate);
	xlnx_ps_enable_uart(reg_base);

	return 0;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
int uart_xlnx_ps_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_xlnx_ps_dev_config *dev_cfg = dev->config;
	uint32_t reg_val;
	uint32_t reg_base;

	reg_base = dev_cfg->reg;
	reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);
	if ((reg_val & XUARTPS_SR_RXEMPTY) == 0)
	{
		*c = (unsigned char)sys_read32(reg_base +
									   XUARTPS_FIFO_OFFSET);
		return 0;
	}
	else
	{
		return -1;
	}
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * If the hardware flow control is enabled then the handshake signal CTS has to
 * be asserted in order to send a character.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
void uart_xlnx_ps_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_xlnx_ps_dev_config *dev_cfg = dev->config;
	uint32_t reg_val;
	uint32_t reg_base;

	reg_base = dev_cfg->reg;
	/* wait for transmitter to ready to accept a character */
	do
	{
		reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);
	} while ((reg_val & XUARTPS_SR_TXEMPTY) == 0);

	sys_write32((uint32_t)(c & 0xFF), reg_base + XUARTPS_FIFO_OFFSET);

	do
	{
		reg_val = sys_read32(reg_base + XUARTPS_SR_OFFSET);
	} while ((reg_val & XUARTPS_SR_TXEMPTY) == 0);
}


#define CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC 0x17d7840
struct uart_xlnx_ps_dev_config qemu_uart_config;
struct device qemu_uart_device;
void uart_init(void) {
    qemu_uart_config.reg = 0xFF000000;
    qemu_uart_config.baud_rate = 115200;
    qemu_uart_config.sys_clk_freq = 0X17D7840;
    qemu_uart_device.name ="qemu_uart";
    qemu_uart_device.config = &qemu_uart_config;
    uart_xlnx_ps_init(&qemu_uart_device);
    return;
}
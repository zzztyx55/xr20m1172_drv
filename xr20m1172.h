
#define XRM117XX_NAME			"xrm117x"
#define XRM117XX_NAME_SPI		"xrm117x_spi"


/* XRM117XX register definitions */
#define XRM117XX_RHR_REG		(0x00) /* RX FIFO */
#define XRM117XX_THR_REG		(0x00) /* TX FIFO */
#define XRM117XX_IER_REG		(0x01) /* Interrupt enable */
#define XRM117XX_IIR_REG		(0x02) /* Interrupt Identification */
#define XRM117XX_FCR_REG		(0x02) /* FIFO control */
#define XRM117XX_LCR_REG		(0x03) /* Line Control */
#define XRM117XX_MCR_REG		(0x04) /* Modem Control */
#define XRM117XX_LSR_REG		(0x05) /* Line Status */
#define XRM117XX_MSR_REG		(0x06) /* Modem Status */
#define XRM117XX_SPR_REG		(0x07) /* Scratch Pad */
#define XRM117XX_TXLVL_REG		(0x08) /* TX FIFO level */
#define XRM117XX_RXLVL_REG		(0x09) /* RX FIFO level */
#define XRM117XX_IODIR_REG		(0x0a) /* I/O Direction
						* - only on 75x/76x
						*/
#define XRM117XX_IOSTATE_REG		(0x0b) /* I/O State
						* - only on 75x/76x
						*/
#define XRM117XX_IOINTENA_REG		(0x0c) /* I/O Interrupt Enable
						* - only on 75x/76x
						*/
#define XRM117XX_IOCONTROL_REG		(0x0e) /* I/O Control
						* - only on 75x/76x
						*/
#define XRM117XX_EFCR_REG		(0x0f) /* Extra Features Control */

/* TCR/TLR Register set: Only if ((MCR[2] == 1) && (EFR[4] == 1)) */
#define XRM117XX_TCR_REG		(0x06) /* Transmit control */
#define XRM117XX_TLR_REG		(0x07) /* Trigger level */

/* Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF)) */
#define XRM117XX_DLL_REG		(0x00) /* Divisor Latch Low */
#define XRM117XX_DLH_REG		(0x01) /* Divisor Latch High */

/* Enhanced Register set: Only if (LCR == 0xBF) */
#define XRM117XX_EFR_REG		(0x02) /* Enhanced Features */
#define XRM117XX_XON1_REG		(0x04) /* Xon1 word */
#define XRM117XX_XON2_REG		(0x05) /* Xon2 word */
#define XRM117XX_XOFF1_REG		(0x06) /* Xoff1 word */
#define XRM117XX_XOFF2_REG		(0x07) /* Xoff2 word */

/* IER register bits */
#define XRM117XX_IER_RDI_BIT		(1 << 0) /* Enable RX data interrupt */
#define XRM117XX_IER_THRI_BIT		(1 << 1) /* Enable TX holding register
						  * interrupt */
#define XRM117XX_IER_RLSI_BIT		(1 << 2) /* Enable RX line status
						  * interrupt */
#define XRM117XX_IER_MSI_BIT		(1 << 3) /* Enable Modem status
						  * interrupt */

/* IER register bits - write only if (EFR[4] == 1) */
#define XRM117XX_IER_SLEEP_BIT		(1 << 4) /* Enable Sleep mode */
#define XRM117XX_IER_XOFFI_BIT		(1 << 5) /* Enable Xoff interrupt */
#define XRM117XX_IER_RTSI_BIT		(1 << 6) /* Enable nRTS interrupt */
#define XRM117XX_IER_CTSI_BIT		(1 << 7) /* Enable nCTS interrupt */

/* FCR register bits */
#define XRM117XX_FCR_FIFO_BIT		(1 << 0) /* Enable FIFO */
#define XRM117XX_FCR_RXRESET_BIT	(1 << 1) /* Reset RX FIFO */
#define XRM117XX_FCR_TXRESET_BIT	(1 << 2) /* Reset TX FIFO */
#define XRM117XX_FCR_RXLVLL_BIT	(1 << 6) /* RX Trigger level LSB */
#define XRM117XX_FCR_RXLVLH_BIT	(1 << 7) /* RX Trigger level MSB */

/* FCR register bits - write only if (EFR[4] == 1) */
#define XRM117XX_FCR_TXLVLL_BIT	(1 << 4) /* TX Trigger level LSB */
#define XRM117XX_FCR_TXLVLH_BIT	(1 << 5) /* TX Trigger level MSB */

/* IIR register bits */
#define XRM117XX_IIR_NO_INT_BIT	(1 << 0) /* No interrupts pending */
#define XRM117XX_IIR_ID_MASK		0x3e     /* Mask for the interrupt ID */
#define XRM117XX_IIR_THRI_SRC		0x02     /* TX holding register empty */
#define XRM117XX_IIR_RDI_SRC		0x04     /* RX data interrupt */
#define XRM117XX_IIR_RLSE_SRC		0x06     /* RX line status error */
#define XRM117XX_IIR_RTOI_SRC		0x0c     /* RX time-out interrupt */
#define XRM117XX_IIR_MSI_SRC		0x00     /* Modem status interrupt
						  * - only on 75x/76x
						  */
#define XRM117XX_IIR_INPIN_SRC		0x30     /* Input pin change of state
						  * - only on 75x/76x
						  */
#define XRM117XX_IIR_XOFFI_SRC		0x10     /* Received Xoff */
#define XRM117XX_IIR_CTSRTS_SRC	0x20     /* nCTS,nRTS change of state
						  * from active (LOW)
						  * to inactive (HIGH)
						  */
/* LCR register bits */
#define XRM117XX_LCR_LENGTH0_BIT	(1 << 0) /* Word length bit 0 */
#define XRM117XX_LCR_LENGTH1_BIT	(1 << 1) /* Word length bit 1
						  *
						  * Word length bits table:
						  * 00 -> 5 bit words
						  * 01 -> 6 bit words
						  * 10 -> 7 bit words
						  * 11 -> 8 bit words
						  */
#define XRM117XX_LCR_STOPLEN_BIT	(1 << 2) /* STOP length bit
						  *
						  * STOP length bit table:
						  * 0 -> 1 stop bit
						  * 1 -> 1-1.5 stop bits if
						  *      word length is 5,
						  *      2 stop bits otherwise
						  */
#define XRM117XX_LCR_PARITY_BIT	(1 << 3) /* Parity bit enable */
#define XRM117XX_LCR_EVENPARITY_BIT	(1 << 4) /* Even parity bit enable */
#define XRM117XX_LCR_FORCEPARITY_BIT	(1 << 5) /* 9-bit multidrop parity */
#define XRM117XX_LCR_TXBREAK_BIT	(1 << 6) /* TX break enable */
#define XRM117XX_LCR_DLAB_BIT		(1 << 7) /* Divisor Latch enable */
#define XRM117XX_LCR_WORD_LEN_5	(0x00)
#define XRM117XX_LCR_WORD_LEN_6	(0x01)
#define XRM117XX_LCR_WORD_LEN_7	(0x02)
#define XRM117XX_LCR_WORD_LEN_8	(0x03)
#define XRM117XX_LCR_CONF_MODE_A	XRM117XX_LCR_DLAB_BIT /* Special
								* reg set */
#define XRM117XX_LCR_CONF_MODE_B	0xBF                   /* Enhanced
								* reg set */

/* MCR register bits */
#define XRM117XX_MCR_DTR_BIT		(1 << 0) /* DTR complement
						  * - only on 75x/76x
						  */
#define XRM117XX_MCR_RTS_BIT		(1 << 1) /* RTS complement */
#define XRM117XX_MCR_TCRTLR_BIT	(1 << 2) /* TCR/TLR register enable */
#define XRM117XX_MCR_LOOP_BIT		(1 << 4) /* Enable loopback test mode */
#define XRM117XX_MCR_XONANY_BIT	(1 << 5) /* Enable Xon Any
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define XRM117XX_MCR_IRDA_BIT		(1 << 6) /* Enable IrDA mode
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define XRM117XX_MCR_CLKSEL_BIT	(1 << 7) /* Divide clock by 4
						  * - write enabled
						  * if (EFR[4] == 1)
						  */

/* LSR register bits */
#define XRM117XX_LSR_DR_BIT		(1 << 0) /* Receiver data ready */
#define XRM117XX_LSR_OE_BIT		(1 << 1) /* Overrun Error */
#define XRM117XX_LSR_PE_BIT		(1 << 2) /* Parity Error */
#define XRM117XX_LSR_FE_BIT		(1 << 3) /* Frame Error */
#define XRM117XX_LSR_BI_BIT		(1 << 4) /* Break Interrupt */
#define XRM117XX_LSR_BRK_ERROR_MASK	0x1E     /* BI, FE, PE, OE bits */
#define XRM117XX_LSR_THRE_BIT		(1 << 5) /* TX holding register empty */
#define XRM117XX_LSR_TEMT_BIT		(1 << 6) /* Transmitter empty */
#define XRM117XX_LSR_FIFOE_BIT		(1 << 7) /* Fifo Error */

/* MSR register bits */
#define XRM117XX_MSR_DCTS_BIT		(1 << 0) /* Delta CTS Clear To Send */
#define XRM117XX_MSR_DDSR_BIT		(1 << 1) /* Delta DSR Data Set Ready
						  * or (IO4)
						  * - only on 75x/76x
						  */
#define XRM117XX_MSR_DRI_BIT		(1 << 2) /* Delta RI Ring Indicator
						  * or (IO7)
						  * - only on 75x/76x
						  */
#define XRM117XX_MSR_DCD_BIT		(1 << 3) /* Delta CD Carrier Detect
						  * or (IO6)
						  * - only on 75x/76x
						  */
#define XRM117XX_MSR_CTS_BIT		(1 << 0) /* CTS */
#define XRM117XX_MSR_DSR_BIT		(1 << 1) /* DSR (IO4)
						  * - only on 75x/76x
						  */
#define XRM117XX_MSR_RI_BIT		(1 << 2) /* RI (IO7)
						  * - only on 75x/76x
						  */
#define XRM117XX_MSR_CD_BIT		(1 << 3) /* CD (IO6)
						  * - only on 75x/76x
						  */
#define XRM117XX_MSR_DELTA_MASK	0x0F     /* Any of the delta bits! */

/*
 * TCR register bits
 * TCR trigger levels are available from 0 to 60 characters with a granularity
 * of four.
 * The programmer must program the TCR such that TCR[3:0] > TCR[7:4]. There is
 * no built-in hardware check to make sure this condition is met. Also, the TCR
 * must be programmed with this condition before auto RTS or software flow
 * control is enabled to avoid spurious operation of the device.
 */
#define XRM117XX_TCR_RX_HALT(words)	((((words) / 4) & 0x0f) << 0)
#define XRM117XX_TCR_RX_RESUME(words)	((((words) / 4) & 0x0f) << 4)

/*
 * TLR register bits
 * If TLR[3:0] or TLR[7:4] are logical 0, the selectable trigger levels via the
 * FIFO Control Register (FCR) are used for the transmit and receive FIFO
 * trigger levels. Trigger levels from 4 characters to 60 characters are
 * available with a granularity of four.
 *
 * When the trigger level setting in TLR is zero, the SC16IS740/750/760 uses the
 * trigger level setting defined in FCR. If TLR has non-zero trigger level value
 * the trigger level defined in FCR is discarded. This applies to both transmit
 * FIFO and receive FIFO trigger level setting.
 *
 * When TLR is used for RX trigger level control, FCR[7:6] should be left at the
 * default state, that is, '00'.
 */
#define XRM117XX_TLR_TX_TRIGGER(words)	((((words) / 4) & 0x0f) << 0)
#define XRM117XX_TLR_RX_TRIGGER(words)	((((words) / 4) & 0x0f) << 4)

/* IOControl register bits (Only 750/760) */
#define XRM117XX_IOCONTROL_LATCH_BIT	(1 << 0) /* Enable input latching */
#define XRM117XX_IOCONTROL_GPIO_BIT	(1 << 1) /* Enable GPIO[7:4] */
#define XRM117XX_IOCONTROL_SRESET_BIT	(1 << 3) /* Software Reset */

/* EFCR register bits */
#define XRM117XX_EFCR_9BIT_MODE_BIT	(1 << 0) /* Enable 9-bit or Multidrop
						  * mode (RS485) */
#define XRM117XX_EFCR_RXDISABLE_BIT	(1 << 1) /* Disable receiver */
#define XRM117XX_EFCR_TXDISABLE_BIT	(1 << 2) /* Disable transmitter */
#define XRM117XX_EFCR_AUTO_RS485_BIT	(1 << 4) /* Auto RS485 RTS direction */
#define XRM117XX_EFCR_RTS_INVERT_BIT	(1 << 5) /* RTS output inversion */
#define XRM117XX_EFCR_IRDA_MODE_BIT	(1 << 7) /* IrDA mode
						  * 0 = rate upto 115.2 kbit/s
						  *   - Only 750/760
						  * 1 = rate upto 1.152 Mbit/s
						  *   - Only 760
						  */

/* EFR register bits */
#define XRM117XX_EFR_AUTORTS_BIT	(1 << 6) /* Auto RTS flow ctrl enable */
#define XRM117XX_EFR_AUTOCTS_BIT	(1 << 7) /* Auto CTS flow ctrl enable */
#define XRM117XX_EFR_XOFF2_DETECT_BIT	(1 << 5) /* Enable Xoff2 detection */
#define XRM117XX_EFR_ENABLE_BIT	(1 << 4) /* Enable enhanced functions
						  * and writing to IER[7:4],
						  * FCR[5:4], MCR[7:5]
						  */
#define XRM117XX_EFR_SWFLOW3_BIT	(1 << 3) /* SWFLOW bit 3 */
#define XRM117XX_EFR_SWFLOW2_BIT	(1 << 2) /* SWFLOW bit 2
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no transmitter flow
						  *       control
						  * 01 -> transmitter generates
						  *       XON2 and XOFF2
						  * 10 -> transmitter generates
						  *       XON1 and XOFF1
						  * 11 -> transmitter generates
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */
#define XRM117XX_EFR_SWFLOW1_BIT	(1 << 1) /* SWFLOW bit 2 */
#define XRM117XX_EFR_SWFLOW0_BIT	(1 << 0) /* SWFLOW bit 3
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no received flow
						  *       control
						  * 01 -> receiver compares
						  *       XON2 and XOFF2
						  * 10 -> receiver compares
						  *       XON1 and XOFF1
						  * 11 -> receiver compares
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */

/* Misc definitions */
#define XRM117XX_FIFO_SIZE		(64)
#define XRM117XX_REG_SHIFT		2

struct xrm117x_devtype {
	char	name[10];
	int	nr_gpio;
	int	nr_uart;
};

struct xrm117x_one {
	struct uart_port		port;
	struct work_struct		tx_work;
	struct work_struct		md_work;
    struct work_struct      stop_rx_work;
	struct work_struct      stop_tx_work;
	struct serial_rs485		rs485;
	unsigned char           msr_reg;
};

struct xrm117x_port {
	struct uart_driver		uart;
	struct xrm117x_devtype	*devtype;
	struct mutex			mutex;
	struct mutex			mutex_bus_access;
	struct clk			*clk;

#ifdef CONFIG_GPIOLIB
	struct gpio_chip		gpio;
#endif
	unsigned char			buf[XRM117XX_FIFO_SIZE];
	struct xrm117x_one		p[0];
};
#ifndef __XR20m1172_H__
#define __XR20m1172_H__

#endif

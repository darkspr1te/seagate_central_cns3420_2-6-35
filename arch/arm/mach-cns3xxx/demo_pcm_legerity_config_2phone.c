/*******************************************************************************
 *
 *  Copyright (c) 2008 Cavium Networks
 *  Copyright (c) 2011 Cavium
 *
 *  This file is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, Version 2, as
 *  published by the Free Software Foundation.
 *
 *  This file is distributed in the hope that it will be useful,
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 *  NONINFRINGEMENT.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this file; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or
 *  visit http://www.gnu.org/licenses/.
 *
 *  This file may also be available under a different license from Cavium.
 *  Contact Cavium Networks for more information
 *
 ******************************************************************************/

/*
 * Options
 * #define DEBUG_PRINT
 */
/* ========================================================================== */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include <asm/system.h>

#include <mach/pcm.h>
#include <asm/dma.h>
#include <linux/spi/spi.h>

#include <mach/cns3xxx.h>
#include <mach/dmac.h>
#include <linux/gpio.h>
#include <mach/misc.h>
#include <mach/pm.h>

#define u_int32 u32
#define u_int16 u16
#define u_int8 u8

#define CNS3XXX_PCM_INTERRUPT
#define CNS3XXX_PCM_DMA

#if defined(CNS3XXX_PCM_DMA) && defined(CNS3XXX_PCM_INTERRUPT)
#undef CNS3XXX_PCM_INTERRUPT
#endif

#if defined(CONFIG_LE88221_CONTROL)
extern void Pcm_Initial_Legerity_Le88221(void);
#else
unsigned short int (*si3226_demo_hook)(void) = NULL;
EXPORT_SYMBOL(si3226_demo_hook);
#endif

#define Sys_Interrupt_Disable_Save_Flags(_p)	local_irq_save(*(_p))
#define Sys_Interrupt_Restore_Flags		local_irq_restore

#define Hal_Timer_Timer3_Delay(_p)	mdelay(_p / 1000)
/*int debug = 0; */
#ifdef DEBUG_PRINT
#undef DEBUG_PRINT
/*#define DEBUG_PRINT(arg...)		if (debug) printk(arg); */
#define DEBUG_PRINT			printk
#else
/*#define DEBUG_PRINT(arg...)		if (debug) printk(arg); */
#define DEBUG_PRINT(arg...)		printk(KERN_DEBUG arg);
#endif

/*
 * public variable declarations
 */
PCM_OBJECT_T				pcm_object;

PCM_CHANNEL_OBJECT_T			pcm_channel_object;

const u_int16				pcm_data_mask[2] = {0x00FF, 0xFFFF};

#ifdef CNS3XXX_PCM_DMA
static u32 *pcm_dma_buffer0;
static u32 *pcm_dma_buffer1;
static dma_addr_t pcm_dma_buffer_p0;
static dma_addr_t pcm_dma_buffer_p1;
#endif

#ifdef CNS3XXX_PCM_INTERRUPT
static u16 pcm_intr_buffer_tx;
static u16 pcm_intr_buffer_rx;
#endif

/*
 * public variable declarations
 */
PCM_OBJECT_T				pcm_object;
PCM_CHANNEL_OBJECT_T			pcm_channel_object;

/*#define BUFFER_SIZE			0x1000 */

/*
 * For Legerity's Le88221
 */
#define	CH0_TX_Le88221_DELAY		(0)
#define	CH0_RX_Le88221_DELAY		(0)

#define	CH1_TX_Le88221_DELAY		(8)
#define	CH1_RX_Le88221_DELAY		(8)

#define	CH2_TX_Le88221_DELAY		(32)
#define	CH2_RX_Le88221_DELAY		(32)

#define	CH3_TX_Le88221_DELAY		(48)
#define	CH3_RX_Le88221_DELAY		(48)

static struct proc_dir_entry *pcm_proc_entry;

/******************************************************************************
 *
 * FUNCTION:  Hal_Pcm_Initialize
 * PURPOSE:
 *
 ******************************************************************************/
void Hal_Pcm_Initialize(PCM_OBJECT_T *pPcm_Object)
{

	/*
	 * Note PCM is NOT enabled after this function is invoked!!
	 */
	PCM_CONFIGURATION_0_REG = pPcm_Object->config_0;
	PCM_CONFIGURATION_1_REG = pPcm_Object->config_1;
	PCM_CHANNEL_0_CONFIG_REG = pPcm_Object->channel_0_config;
	PCM_CHANNEL_1_CONFIG_REG = pPcm_Object->channel_1_config;
	PCM_CHANNEL_2_CONFIG_REG = pPcm_Object->channel_2_config;
	PCM_CHANNEL_3_CONFIG_REG = pPcm_Object->channel_3_config;
	PCM_INTERRUPT_ENABLE_REG = pPcm_Object->interrupt_config;

	/*
	if (pPcm_Object->interrupt_config)
		Hal_Intc_Register_Interrupt(&pPcm_Object->intc_obj);
	*/

	/* Clear spurious interrupt sources */
	PCM_INTERRUPT_STATUS_REG = PCM_RXBUF_OVERRUN_FG | PCM_TXBUF_UNDERRUN_FG;

	/* Disable PCM */
	HAL_PCM_DISABLE_PCM();
}

#ifdef CNS3XXX_PCM_DMA
/******************************************************************************
 *
 * FUNCTION:  Pcm_Configure_DMA_Hardware_Handshake_For_Legerity
 * PURPOSE:
 *
 ******************************************************************************/
static int pcm_dma_rx_ch;
static int pcm_dma_tx_ch;
static int pcm_dma_lpfe;
static uint32_t	pcm_rx_ev = 11;
static uint32_t	pcm_tx_ev = 12;
void Pcm_Configure_DMA_Hardware_Handshake_For_Legerity(void)
{
	uint16_t	lc0sz;
	uint32_t	pcm_rx_reg_addr = CNS3XXX_SSP_BASE + 0xA0;
	uint32_t	pcm_tx_reg_addr = CNS3XXX_SSP_BASE + 0x98;
	uint32_t	pcm_periph_id = DMAC_PCM_PERIPH_ID_0;

	pcm_dma_lpfe = 1;

	lc0sz = DMAWFP_INSTR_SIZE + DMALDP_INSTR_SIZE + DMAST_INSTR_SIZE +
		DMAWMB_INSTR_SIZE + DMASEV_INSTR_SIZE + DMAWFE_INSTR_SIZE;

	/* Periperal to Memory (PCM RX) */
	/* ctrl == src fix, dst fix, ss-4, sl-1, ds-4, dl-1, no swap */
	DMAC_DMAMOV(pcm_dma_rx_ch, CCR, dmac_create_ctrlval(0, 2, 1, 0, 2, 1,
				0));
	DMAC_DMAMOV(pcm_dma_rx_ch, SAR, pcm_rx_reg_addr);
	DMAC_DMAMOV(pcm_dma_rx_ch, DAR, pcm_dma_buffer_p0);
	DMAC_DMALP(pcm_dma_rx_ch, 0, 1);
		DMAC_DMAWFP(pcm_dma_rx_ch, pcm_periph_id, 2);
		DMAC_DMALDP(pcm_dma_rx_ch, pcm_periph_id, 0);
		DMAC_DMASTS(pcm_dma_rx_ch);
		DMAC_DMAWMB(pcm_dma_rx_ch);
		DMAC_DMASEV(pcm_dma_rx_ch, pcm_rx_ev);
		DMAC_WFE(pcm_dma_rx_ch, pcm_tx_ev);
	DMAC_DMALPEND(pcm_dma_rx_ch, 0, lc0sz, ~pcm_dma_lpfe);	/* loop
								   forever */
	DMAC_DMAEND(pcm_dma_rx_ch);
	/* End of PCM RX */

	lc0sz = DMAWFE_INSTR_SIZE + DMALD_INSTR_SIZE
	  + DMAST_INSTR_SIZE + DMAWMB_INSTR_SIZE
	  + DMASEV_INSTR_SIZE;
	/* Memory to Peripheral (PCM TX) */
	/* ctrl == src fix, dst fix, ss-4, sl-1, ds-4, dl-1, swap 16 */
	/*
	DMAC_DMAMOV(pcm_dma_tx_ch, CCR, dmac_create_ctrlval(0, 2, 1, 0, 2, 1,
	1));
	*/
	/* no swap */
	DMAC_DMAMOV(pcm_dma_tx_ch, CCR, dmac_create_ctrlval(0, 2, 1, 0, 2, 1,
				0));
	DMAC_DMAMOV(pcm_dma_tx_ch, SAR, pcm_dma_buffer_p0);
	DMAC_DMAMOV(pcm_dma_tx_ch, DAR, pcm_tx_reg_addr);
	DMAC_DMALP(pcm_dma_tx_ch, 0, 1);
		DMAC_WFE(pcm_dma_tx_ch, pcm_rx_ev);
		DMAC_DMALD(pcm_dma_tx_ch);
		DMAC_DMASTS(pcm_dma_tx_ch);
		DMAC_DMAWMB(pcm_dma_tx_ch);
		DMAC_DMASEV(pcm_dma_tx_ch, pcm_tx_ev);
	DMAC_DMALPEND(pcm_dma_tx_ch, 0, lc0sz, ~pcm_dma_lpfe);	/* loop
								   forever */
	DMAC_DMAEND(pcm_dma_tx_ch);
	/* End of PCM TX */

	DMAC_DMAGO(pcm_dma_rx_ch);
	DMAC_DMAGO(pcm_dma_tx_ch);

	return;
}
#endif

u32 bu_count;
u32 to_count;
#ifdef CNS3XXX_PCM_INTERRUPT
u32 rx_full;
u32 tx_empty;
#endif
static irqreturn_t cns3xxx_pcm_irq_handler(int this_irq, void *dev_id)
{
	if (PCM_INTERRUPT_STATUS_REG & 0x8) {
		PCM_INTERRUPT_STATUS_REG |= 0x8;
		to_count++;
/*		printk(KERN_DEBUG "TXBUF Underrun\n"); */
	}
	if (PCM_INTERRUPT_STATUS_REG & 0x4) {
		PCM_INTERRUPT_STATUS_REG |= 0x4;
		bu_count++;
/*		printk(KERN_DEBUG "RXBUF Overrun\n"); */
	}
#ifdef CNS3XXX_PCM_INTERRUPT
	if (PCM_INTERRUPT_STATUS_REG & 0x2) {
		PCM_TX_DATA_31_0_REG = pcm_intr_buffer_tx;
		tx_empty++;
/*		printk(KERN_DEBUG "Tx Empty\n"); */
	}
	if (PCM_INTERRUPT_STATUS_REG & 0x1) {
		pcm_intr_buffer_rx = PCM_RX_DATA_31_0_REG & 0xFF;
		pcm_intr_buffer_tx = (pcm_intr_buffer_rx >> 8 & 0x0F) |
			(pcm_intr_buffer_rx << 8 & 0xF0);
		rx_full++;
/*		printk(KERN_DEBUG "Rx Full\n"); */
	}
#endif

	return IRQ_HANDLED;
}

/*this is for PCM0 init... */
static void pcm_init(void)
{
	unsigned long flags;
	local_irq_save(flags);

	PM_PLL_HM_PD_CTRL_REG &= ~(0x1 << 5);
	HAL_MISC_ENABLE_PCM_PINS();
	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(SPI_PCM_I2C));
	cns3xxx_pwr_soft_rst(0x1 << PM_SOFT_RST_REG_OFFST_SPI_PCM_I2C);

	/*
	 * PCM0 Pin Drive Strength
	 *   0xC0: 21mA
	 *   0x80: 15.7mA
	 *   0x40: 10.5mA
	 *   0x00: 5.2mA
	 */
/*	MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B &= ~0xC0; */
/*	MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B |= 0xC0; */ /*21mA... */


	/*
	 * Check CLK_OUT_SEL_Pin for 8.192 MHz
	 */
	DEBUG_PRINT("%s:\n", __func__);
/*	HAL_MISC_DISABLE_SPI_SERIAL_FLASH_BANK_ACCESS(); */
/*	HAL_PWRMGT_CONFIGURE_CLOCK_OUT_PIN(5, 0); */

	/*
	 * For IDL Case:
	 *	 UDCLK	: 4.096 MHz
	 *	 PCMCLK   : 2.048 MHz = 4.096/(1 + 1)
	 *	 FSYNCCLK : 8 KHz = 2048000/(255 + 1)
	 */

	pcm_object.config_0 = ((0 & 0x7) << 0) |   /* Configure master clock
						      rate */
			  (0 << 12) |		   /* Disable loopback mode */
			  (1 << 13) |		   /* Enable master mode */
			  (0 << 14) |		   /* Select IDL mode */
			  /*(1 << 24) | */	   /* Enable PCM data swap */
			  (0 << 24) |		   /* Disable PCM data swap */
			  (0 << 31);		   /* Disable PCM */

	/*
	 * Note FSYNC_WIDE will be ignored when the PCM is configured as Slave
	 * or as Master with GCI mode
	 */
	/* Select FSYNC mode , 0 : short FSYNC, 1 : long FSYNC */
	pcm_object.config_1 = ((0 & 0x1) << 15);


	/*
	 * Configure the settings of PCM's channel
	 */
	pcm_object.channel_0_config = ((CH0_TX_Le88221_DELAY & 0x7F) << 0) |
				  ((CH0_RX_Le88221_DELAY & 0x7F) << 8) |
				  ((PCM_DATA_BIT_8 & 0x1) << 22) |
				  (1 << 23);	/* Enable this channel */

	pcm_object.channel_1_config = ((CH1_TX_Le88221_DELAY & 0x7F) << 0) |
				  ((CH1_RX_Le88221_DELAY & 0x7F) << 8) |
				  ((PCM_DATA_BIT_8 & 0x1) << 22) |
				  (1 << 23);	/* Enable this channel */

	pcm_object.channel_2_config = ((CH2_TX_Le88221_DELAY & 0x7F) << 0) |
				  ((CH2_RX_Le88221_DELAY & 0x7F) << 8) |
				  ((0 & 0x1) << 22) |
				  (0 << 23);	/* Disable this channel */

	pcm_object.channel_3_config = ((CH3_TX_Le88221_DELAY & 0x7F) << 0) |
				  ((CH3_RX_Le88221_DELAY & 0x7F) << 8) |
				  ((0 & 0x1) << 22) |
				  (0 << 23);	/* Disable this channel */


	/* Enable PCM's interrupt sources */
	pcm_object.interrupt_config = 0;
	pcm_object.interrupt_config = PCM_RXBUF_OVERRUN_FG |
			PCM_TXBUF_UNDERRUN_FG;

	/* Initialize PCM's setting */
	Hal_Pcm_Initialize(&pcm_object);

	/*
	 * Disable PCM interrupt since GDMA hardware handshake interrupt will be
	 * used.
	 */
	/*HAL_INTC_DISABLE_INTERRUPT_SOURCE(INTC_PCM_BIT_INDEX); */


	/*
	 * PCM will start to transmit and receive data once PCM is enabled. To
	 * avoid PCM Transmit Buffer underrun, we have to put one transmit data
	 * into PCM Transmit Buffer before PCM is enabled!!
	 * Note PCM channel 0 is used.
	 */
	PCM_TX_DATA_31_0_REG = 0;

#ifdef CNS3XXX_PCM_DMA
	/* Disable PCM interrupt */
	__raw_writel(0, CNS3XXX_SSP_BASE_VIRT + 0xAC);

	/* Initialize GDMA hardware handshake for PCM */
	Pcm_Configure_DMA_Hardware_Handshake_For_Legerity();
#endif

	/* Write 1 to clear interrupt status */
	HAL_PCM_CLEAR_INTERRUPT_STATUS((PCM_RXBUF_OVERRUN_FG |
				PCM_TXBUF_UNDERRUN_FG));

	HAL_PCM_ENABLE_PCM();
	/*
	 * Configure Legerity's Le88221 MPI Interface
	 */
#if defined(CONFIG_LE88221_CONTROL)
	Pcm_Initial_Legerity_Le88221();
#else
	si3226_demo_hook();
#endif

	/*HAL_PCM_ENABLE_PCM(); */

	local_irq_restore(flags);

	DEBUG_PRINT("%s: end =>\n", __func__);

}

#ifdef CNS3XXX_PCM_DMA
/* ========================================================================== */
static int pcm_dma_irq_rx_handler (void *pdata)
{
/*	u32 volatile reg; */
	printk(KERN_INFO "Got the IRQ interrupt for the rx channel\n");

#if 0
	/* read interrupt status */
	reg = __raw_readl(CNS3XXX_GDMAC_BASE_VIRT + 0x28);
	printk(KERN_INFO "interrupt status: 0x%x\n", reg);

	/* disable interrupt */
	__raw_writel(0, CNS3XXX_GDMAC_BASE_VIRT + 0x20);

	/* clear interrupt status */
	__raw_writel(reg, CNS3XXX_GDMAC_BASE_VIRT + 0x2C);

	/* enable interrupt */
	__raw_writel(0xFF, CNS3XXX_GDMAC_BASE_VIRT + 0x20);
#endif
	return 0;
}

static int pcm_dma_irq_tx_handler (void *pdata)
{
	printk(KERN_INFO "Got the IRQ interrupt for the tx channel\n");
	return 0;
}
/* ========================================================================== */
#endif

extern void si3226_test_spi(void);
#define SPI_MEM_MAP_VALUE(reg_offset)		\
	(*((u32 volatile *)(CNS3XXX_SSP_BASE_VIRT + reg_offset)))

static int proc_read_pcm(char *buf, char **start, off_t offset,
		   int count, int *eof, void *data)
{
	int len = 0;

	DEBUG_PRINT("%s:\n", __func__);
#if 0
	u32 reg;
	reg = si3226_test_spi();
#endif

	len += sprintf(buf + len, "SPI_CONFIGURATION_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x40));
	len += sprintf(buf + len, "SPI_SERVICE_STATUS_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x44));
	len += sprintf(buf + len, "SPI_BIT_RATE_CONTROL_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x48));
	len += sprintf(buf + len, "SPI_TRANSMIT_CONTROL_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x4C));
	len += sprintf(buf + len, "SPI_TRANSMIT_BUFFER_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x50));
	len += sprintf(buf + len, "SPI_RECEIVE_CONTROL_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x54));
	len += sprintf(buf + len, "SPI_RECEIVE_BUFFER_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x58));
	len += sprintf(buf + len, "SPI_FIFO_TRANSMIT_CONFIG_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x5C));
	len += sprintf(buf + len, "SPI_FIFO_TRANSMIT_CONTROL_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x60));
	len += sprintf(buf + len, "SPI_FIFO_RECEIVE_CONFIG_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x64));
	len += sprintf(buf + len, "SPI_INTERRUPT_STATUS_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x68));
	len += sprintf(buf + len, "SPI_INTERRUPT_ENABLE_REG: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x6C));
	len += sprintf(buf + len, "SPI_ACCESS_CMD: 0x%x\n",
			SPI_MEM_MAP_VALUE(0x70));

	len += sprintf(buf + len, "PCM_CONFIGURATION_0_REG: 0x%x\n",
			PCM_MEM_MAP_VALUE(0x80));
	len += sprintf(buf + len, "PCM_CONFIGURATION_1_REG: 0x%x\n",
			PCM_MEM_MAP_VALUE(0x84));
	len += sprintf(buf + len, "PCM_CHANNEL_0_CONFIG_REG: 0x%x\n",
			PCM_MEM_MAP_VALUE(0x88));
	len += sprintf(buf + len, "PCM_CHANNEL_1_CONFIG_REG: 0x%x\n",
			PCM_MEM_MAP_VALUE(0x8C));
	len += sprintf(buf + len, "PCM_RX_DATA_31_0_REG: 0x%x\n",
			PCM_MEM_MAP_VALUE(0xA0));

#ifdef CNS3XXX_PCM_INTERRUPT
	len += sprintf(buf + len, "PCM_INTERRUPT_STATUS_REG: 0x%x\n",
			PCM_MEM_MAP_VALUE(0xA8));
	len += sprintf(buf + len, "PCM_INTERRUPT_ENABLE_REG: 0x%x\n",
			PCM_MEM_MAP_VALUE(0xAC));
	len += sprintf(buf + len, "to_count: %d\n", to_count);
	len += sprintf(buf + len, "bu_count: %d\n", bu_count);
	len += sprintf(buf + len, "tx_empty: %d\n", tx_empty);
	len += sprintf(buf + len, "rx_full: %d\n", rx_full);
#endif

#ifdef CNS3XXX_PCM_DMA
	len += sprintf(buf + len, "DMAC_FTM: 0x%x\n",
			__raw_readl(CNS3XXX_DMAC_BASE_VIRT + 0x38));
	len += sprintf(buf + len, "DMAC_FTC0: 0x%x\n",
			__raw_readl(CNS3XXX_DMAC_BASE_VIRT + 0x40));
	len += sprintf(buf + len, "DMAC_FTC1: 0x%x\n",
			__raw_readl(CNS3XXX_DMAC_BASE_VIRT + 0x44));
	len += sprintf(buf + len, "DMAC_CS0: 0x%x\n",
			__raw_readl(CNS3XXX_DMAC_BASE_VIRT + 0x100));
	len += sprintf(buf + len, "DMAC_CS1: 0x%x\n",
			__raw_readl(CNS3XXX_DMAC_BASE_VIRT + 0x108));
	len += sprintf(buf + len, "DMAC_LC0: 0x%x\n",
			__raw_readl(CNS3XXX_DMAC_BASE_VIRT + 0x40C));
#endif

	*eof = 1;
	return len;
}

static int proc_write_pcm(struct file *file, const char *buffer,
		unsigned long count, void *data)
{
	int gpio_pin = 50;
	DEBUG_PRINT("%s: count=%ld\n", __func__, count);

#if !defined(CONFIG_LE88221_CONTROL)
	if (!si3226_demo_hook) {
		printk(KERN_INFO "Si3226 driver is not ready.\n");
		return -1;
	}

	/* gpio B pin 18 */
	if (gpio_request(gpio_pin, "cns3xxx_gpio") == 0) {
		gpio_direction_output(gpio_pin, 1);
		gpio_free(gpio_pin);
	}
#endif


#ifdef CNS3XXX_PCM_DMA
	pcm_dma_buffer0 = pci_alloc_consistent(NULL, 2 , &pcm_dma_buffer_p0);
	if (!pcm_dma_buffer0) {
		printk(KERN_INFO "%s: alloc pcm_dma_buffer0 failed.\n",
				__func__);
		return 0;
	}
	pcm_dma_buffer1 = pci_alloc_consistent(NULL, 2, &pcm_dma_buffer_p1);
	if (!pcm_dma_buffer1) {
		printk(KERN_INFO "%s: alloc pcm_dma_buffer1 failed.\n",
				__func__);
		return 0;
	}

	pcm_dma_rx_ch = dmac_get_channel(pcm_dma_irq_rx_handler, NULL);
	DEBUG_PRINT("CNS3XXX PCM: rx ch no: %d\n", pcm_dma_rx_ch);
	   if (pcm_dma_rx_ch == -1) {
		printk(KERN_INFO "Could not get free channel\n");
		return -1;
	}
	pcm_dma_tx_ch = dmac_get_channel(pcm_dma_irq_tx_handler, NULL);
	DEBUG_PRINT("CNS3XXX PCM: tx ch no: %d\n", pcm_dma_tx_ch);
	if (pcm_dma_tx_ch == -1) {
		printk(KERN_INFO "Could not get free channel\n");
		return -1;
	}
	if (dmac_get_event(pcm_dma_rx_ch, pcm_rx_ev) == -1) {
		printk(KERN_INFO "Event Failure .. Bailing\n");
		return -1;
	}
	if (dmac_get_event(pcm_dma_tx_ch, pcm_tx_ev) == -1) {
		printk(KERN_INFO "Event Failure .. Bailing\n");
		return -1;
	}
#endif

	pcm_init();

/*	Pcm_Loopback_Test_Polling(); */

	return count;
}

extern struct platform_driver cns3xxx_spi_driver;

static int __init pcm_init_module(void)
{
	int ret = 0;

	printk(KERN_INFO "%s: CSN3XXX PCM0 (SLIC) demo driver\n", __func__);
	printk(KERN_INFO "%s: SPI bus driver is %s\n", __func__,
			cns3xxx_spi_driver.driver.name);

	if (!cns3xxx_proc_dir)
		return -ENOENT;

	/* Initial /proc/cns3xxx/pcm entry. */
	pcm_proc_entry = create_proc_entry("pcm", S_IFREG | S_IRUGO,
			cns3xxx_proc_dir);
	if (!pcm_proc_entry)
		return -EBUSY;
	pcm_proc_entry->read_proc  = proc_read_pcm;
	pcm_proc_entry->write_proc = proc_write_pcm;


	ret = request_irq(IRQ_CNS3XXX_PCM, cns3xxx_pcm_irq_handler, 0, "pcm",
			NULL);
	if (ret) {
		printk(KERN_INFO "%s: request_irq %d failed(error=0x%x)\n",
				__func__, IRQ_CNS3XXX_PCM, ret);
		goto exit1;
	}

	return 0;
exit1:
	/*pcm_exit_module(); */
	remove_proc_entry(pcm_proc_entry->name, cns3xxx_proc_dir);
	return ret;
}

static void __exit pcm_exit_module(void){
	printk(KERN_INFO "%s:\n", __func__);
	remove_proc_entry(pcm_proc_entry->name, cns3xxx_proc_dir);
	free_irq(IRQ_CNS3XXX_PCM, NULL);

	/* Disable PCM interrupt */
	__raw_writel(0, CNS3XXX_SSP_BASE_VIRT+0xAC);
	/* Disable PCM0 */
	__raw_writel(~(0x1 << 31), CNS3XXX_SSP_BASE_VIRT+0x80);
	/* Disable PCM1 */
/*	__raw_writel(~(0x1 << 31), CNS3XXX_SSP_BASE_VIRT+0x84); */

#ifdef CNS3XXX_PCM_DMA
	/* Disable GDMA interrupt */
	__raw_writel(0, CNS3XXX_DMAC_BASE_VIRT+0x20);
	pcm_dma_lpfe = 0;	/* stop DMA loop */

	dmac_release_event(pcm_dma_rx_ch, pcm_rx_ev);
	dmac_release_event(pcm_dma_tx_ch, pcm_tx_ev);
	dmac_release_channel(pcm_dma_rx_ch);
	dmac_release_channel(pcm_dma_tx_ch);

	if (pcm_dma_buffer0) {
		pci_free_consistent(NULL, 4, pcm_dma_buffer0,
				pcm_dma_buffer_p0);
		pcm_dma_buffer0 = NULL;
	}

	if (pcm_dma_buffer1) {
		pci_free_consistent(NULL, 4, pcm_dma_buffer1,
				pcm_dma_buffer_p1);
		pcm_dma_buffer1 = NULL;
	}
#endif /* CNS3XXX_PCM_DMA */
}

MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("2 phones demo driver via PCM interface");
MODULE_LICENSE("GPL");

module_init(pcm_init_module);
module_exit(pcm_exit_module);

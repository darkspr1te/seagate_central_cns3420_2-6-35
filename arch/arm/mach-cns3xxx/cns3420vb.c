/*
 * Cavium Networks CNS3420 Validation Board
 *
 * Copyright 2000 Deep Blue Solutions Ltd
 * Copyright 2008 ARM Limited
 * Copyright 2008 Cavium Networks
 *		  Scott Shu
 * Copyright 2010 MontaVista Software, LLC.
 *		  Anton Vorontsov <avorontsov@mvista.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/cns3xxx.h>
#include <mach/irqs.h>
#include <mach/lm.h>
#ifdef CONFIG_CNS3XXX_DMAC
#include <asm/dma.h>
#include <mach/dmac.h>
#endif

#if defined(CONFIG_VIDEO_CNS3XXX) || defined(CONFIG_VIDEO_CNS3XXX_MODULE)
#include <media/soc_camera.h>
#include <mach/camera.h>
#include <linux/dma-mapping.h>
#include <mach/gpio.h>
#endif
#include <linux/gpio.h>


#include "core.h"
#include "rdma.h"
#include "devices.h"

struct spi_flash_platform_data {
	char		*name;
	struct mtd_partition *parts;
	unsigned int	nr_parts;

	char		*type;

	/* we'll likely add more ... use JEDEC IDs, etc */
};

/*
 * NOR Flash
 */
static struct mtd_partition cns3420_nor_partitions[] = {
	{
		.name		= "uboot",
		.size		= 0x00040000,
		.offset		= 0,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "kernel",
		.size		= 0x004C0000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "filesystem",
		.size		= 0x7000000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "filesystem2",
		.size		= 0x0AE0000,
		.offset		= MTDPART_OFS_APPEND,
	}, {
		.name		= "ubootenv",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};

static struct physmap_flash_data cns3420_nor_pdata = {
	.width		= 2,
	.parts		= cns3420_nor_partitions,
	.nr_parts	= ARRAY_SIZE(cns3420_nor_partitions),
};

static struct resource cns3420_nor_res = {
	.start		= CNS3XXX_FLASH_BASE,
	.end		= CNS3XXX_FLASH_BASE + SZ_128M - 1,
	.flags		= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
};

static struct platform_device cns3420_nor_pdev = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data = &cns3420_nor_pdata,
	},
	.num_resources	= 1,
	.resource	= &cns3420_nor_res,
};

/*
 * UART
 */
static void __init cns3420_early_serial_setup(void)
{
#ifdef CONFIG_SERIAL_8250_CONSOLE
	static struct uart_port cns3420_serial_port = {
		.membase	= (void __iomem *)CNS3XXX_UART0_BASE_VIRT,
		.mapbase	= CNS3XXX_UART0_BASE,
		.irq	    = IRQ_CNS3XXX_UART0,
		.iotype	 = UPIO_MEM,
		.flags	  = UPF_BOOT_AUTOCONF | UPF_FIXED_TYPE,
		.regshift       = 2,
		.uartclk	= 24000000,
		.line	   = 0,
		.type	   = PORT_16550A,
		.fifosize       = 16,
	};

	early_serial_setup(&cns3420_serial_port);
#endif
}

#if defined(CONFIG_VIDEO_CNS3XXX) || defined(CONFIG_VIDEO_CNS3XXX_MODULE)
static int cns3xxx_camera_set_bus_param(struct soc_camera_link *link,
		unsigned long flags)
{
	return 0;
}

static unsigned long cns3xxx_camera_query_bus_param(
		struct soc_camera_link *link)
{
	/* Now, the platform always sets 10-bits data bus */
	return SOCAM_DATAWIDTH_10;
}

static struct soc_camera_link iclink = {
	.bus_id = 0, /* Must match with the camera ID above */
	.query_bus_param = cns3xxx_camera_query_bus_param,
	.set_bus_param = cns3xxx_camera_set_bus_param,
};
#endif


/*
 * I2C
 */
static struct i2c_board_info __initdata cns3xxx_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c16", 0x50),
	},
#if defined(CONFIG_VIDEO_CNS3XXX) || defined(CONFIG_VIDEO_CNS3XXX_MODULE)
	{
		I2C_BOARD_INFO("mt9v111", 0x48),
		.platform_data = &iclink, /* With extender */
	},
#endif

#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
	{
		I2C_BOARD_INFO("wm8991 ch1", 0x1a),
	},
	{
		I2C_BOARD_INFO("wm8991 ch2", 0x1b),
	},
#endif
#if defined(CONFIG_SND_SOC_WM8580) || defined(CONFIG_SND_SOC_WM8580_MODULE)
	{
		/*
		I2C_BOARD_INFO("wm8580", 0x1b),
		*/
		I2C_BOARD_INFO("wm8580", 0x1a),
	},
#endif

#if defined(CONFIG_SND_SOC_WM9081) || defined(CONFIG_SND_SOC_WM9081_MODULE)
	{
		I2C_BOARD_INFO("wm9081", 0x6c),
	},
#endif
#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)
	{
		I2C_BOARD_INFO("wau8822", 0x1a),
		/*
		I2C_BOARD_INFO("wau8822", 0x0d),
		I2C_BOARD_INFO("wau8822", 0x6c),
		*/
	},
#endif

#if defined(CONFIG_SENSORS_ADT7475)
	{
		I2C_BOARD_INFO("adt7473", 0x2e),
	},
#endif

};

static struct resource cns3xxx_i2c_resource[] = {
	[0] = {
		.start		= CNS3XXX_SSP_BASE + 0x20,
		.end		= 0x7100003f,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_CNS3XXX_I2C,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device cns3xxx_i2c_controller_device = {
	.name		= "cns3xxx-i2c",
	.num_resources	= 2,
	.resource	= cns3xxx_i2c_resource,
};

static struct i2c_board_info __initdata cns3xxx_i2c_gpio_devices[] = {

#if defined(CONFIG_VIDEO_CNS3XXX) || defined(CONFIG_VIDEO_CNS3XXX_MODULE)
	{
		I2C_BOARD_INFO("mt9v111", 0x48),
		.platform_data = &iclink, /* With extender */
	},
	{
		I2C_BOARD_INFO("ov2655", (0x60 >> 1)), /* slave address is
							  0x60 */
		.platform_data = &iclink, /* With extender */
	},
#endif

};

/*
 * I2C GPIO
 */
static struct i2c_gpio_platform_data cns3xxx_i2c_data = {
	.sda_pin		= 1,
	.sda_is_open_drain	= 0,
	.scl_pin		= 0,
	.scl_is_open_drain	= 0,
	.udelay			= 2,
};

static struct platform_device cns3xxx_i2c_gpio_controller_device = {
	.name		= "i2c-gpio",
	.id		= 1,
	.dev		= {
		.platform_data	= &cns3xxx_i2c_data,
	},
};

/* PCM */
static struct resource cns3xxx_pcm_resource = {
	.start	  = CNS3XXX_SSP_BASE + 0x180,
	.end	    = CNS3XXX_SSP_BASE + 0x1ff,
	.flags	  = IORESOURCE_MEM,
};

static struct platform_device cns3xxx_pcm_device = {
	.name	   = "cns3xxx-pcm1",
	.num_resources  = 1,
	.resource       = &cns3xxx_pcm_resource,
};


/*
 * USB
 */
static struct resource cns3xxx_usb_ehci_resources[] = {
	[0] = {
		.start = CNS3XXX_USB_BASE,
		.end   = CNS3XXX_USB_BASE + SZ_16M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CNS3XXX_USB_EHCI,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 cns3xxx_usb_ehci_dma_mask = DMA_BIT_MASK(32);

static struct platform_device cns3xxx_usb_ehci_device = {
	.name	  = "cns3xxx-ehci",
	.num_resources = ARRAY_SIZE(cns3xxx_usb_ehci_resources),
	.resource      = cns3xxx_usb_ehci_resources,
	.dev	   = {
		.dma_mask	  = &cns3xxx_usb_ehci_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource cns3xxx_usb_ohci_resources[] = {
	[0] = {
		.start = CNS3XXX_USB_OHCI_BASE,
		.end   = CNS3XXX_USB_OHCI_BASE + SZ_16M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CNS3XXX_USB_OHCI,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 cns3xxx_usb_ohci_dma_mask = DMA_BIT_MASK(32);

static struct platform_device cns3xxx_usb_ohci_device = {
	.name	  = "cns3xxx-ohci",
	.num_resources = ARRAY_SIZE(cns3xxx_usb_ohci_resources),
	.resource      = cns3xxx_usb_ohci_resources,
	.dev	   = {
		.dma_mask	  = &cns3xxx_usb_ohci_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

/* OTG */
static u64 cns3xxx_usbotg_dma_mask = 0xffffffffULL;
static struct lm_device cns3xxx_usb_otg_device = {
    .dev		= {
	.dma_mask       = &cns3xxx_usbotg_dma_mask,
	.coherent_dma_mask = 0xffffffffULL,
     },
    .resource	   = {
	.start	  = CNS3XXX_USBOTG_BASE,
	.end	    = CNS3XXX_USBOTG_BASE + SZ_16M - 1,
	.flags	  = IORESOURCE_MEM,
     },
    .irq      = IRQ_CNS3XXX_USB_OTG,
};

/* RTC */
static struct resource cns3xxx_rtc_resources[] = {
	[0] = {
		.start = CNS3XXX_RTC_BASE,
		.end   = CNS3XXX_RTC_BASE + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_CNS3XXX_RTC,
		.end   = IRQ_CNS3XXX_RTC,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device cns3xxx_rtc_device = {
	.name	   = "cns3xxx-rtc",
	.id	     = -1,
	.num_resources  = ARRAY_SIZE(cns3xxx_rtc_resources),
	.resource       = cns3xxx_rtc_resources,
};


/* GPIO */
static struct resource cns3xxx_gpio_resources[] = {
	[0] = {
		.start = CNS3XXX_GPIOA_BASE,
		.end   = CNS3XXX_GPIOA_BASE + GPIO_BOUNCE_PRESCALE_OFFSET + 3,
		.flags = IORESOURCE_MEM,
		.name  = "GPIOA",
	},
	[1] = {
		.start = CNS3XXX_GPIOB_BASE,
		.end   = CNS3XXX_GPIOB_BASE + GPIO_BOUNCE_PRESCALE_OFFSET + 3,
		.flags = IORESOURCE_MEM,
		.name  = "GPIOB",
	},
	[2] = {
		.start = IRQ_CNS3XXX_GPIOA,
		.flags = IORESOURCE_IRQ,
		.name  = "GPIOA",
	},
	[3] = {
		.start = IRQ_CNS3XXX_GPIOB,
		.flags = IORESOURCE_IRQ,
		.name  = "GPIOB",
	},
	[4] = {
		.start = CNS3XXX_MISC_BASE,
		.end   = CNS3XXX_MISC_BASE + 0x799,
		.flags = IORESOURCE_MEM,
		.name  = "MISC"
	},

};

static struct platform_device cns3xxx_gpio_device = {
	.name		= "cns3xxx-gpio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cns3xxx_gpio_resources),
	.resource	= cns3xxx_gpio_resources,
};

/* I2S */
static struct resource cns3xxx_i2s_resource = {
	.start	  = CNS3XXX_I2S_BASE,
	.end	    = CNS3XXX_I2S_BASE + 0x53,
	.flags	  = IORESOURCE_MEM,
};

static struct platform_device cns3xxx_i2s_device = {
	.name	   = "cns3xxx-i2s",
	.num_resources  = 1,
	.resource       = &cns3xxx_i2s_resource,
};


/* Watchdog */
static struct resource cns3xxx_watchdog_resources[] = {
	[0] = {
		.start = CNS3XXX_TC11MP_TWD_BASE,
		.end   = CNS3XXX_TC11MP_TWD_BASE + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LOCALWDOG,
		.end   = IRQ_LOCALWDOG,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device cns3xxx_watchdog_device = {
	.name		= "cns3xxx-wdt",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cns3xxx_watchdog_resources),
	.resource	= cns3xxx_watchdog_resources,
};

/* SPI */
static struct mtd_partition cns3xxx_spi_partitions[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 0x30000,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	{
		.name		= "u-bootenv",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x10000,
		.mask_flags	= 0
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x4c0000,
		.mask_flags	= 0
	},

	{
		.name		= "rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};
static struct spi_flash_platform_data cns3xxx_spi_flash_data = {
	.parts		= cns3xxx_spi_partitions,
	.nr_parts	= ARRAY_SIZE(cns3xxx_spi_partitions),
};

static struct spi_board_info __initdata cns3xxx_spi_devices[] = {
	[0] = {
		.modalias		= "m25p80",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 25 * 1000 * 1000,
		.platform_data		=  &cns3xxx_spi_flash_data,
	}, {
#if defined(CONFIG_LE8221_CONTROL)
		.modalias		= "le88221",
#elif defined(CONFIG_SI3226_CONTROL_API)
		.modalias		= "si3226",
#endif
		.bus_num		= 1,
		.chip_select		= 1,
		.max_speed_hz		= 25 * 1000 * 1000,
	},
};

static struct platform_device cns3xxx_spi_controller_device = {
	.name		= "cns3xxx_spi",
};

static struct platform_device cns3xxx_crypto_device = {
	.name	   = "cns3xxx_crypto",
	.id	     = -1,
};

/*
 * LEDS
 */
static struct platform_device cns3xxx_leds_device = {
    .name       = "cns3xxx_leds",
    .id     = 1,
    .dev        = {
        .platform_data  = NULL,
    },
};

/*
 * Initialization
 */
static struct platform_device *cns3420_pdevs[] __initdata = {
	&cns3xxx_rtc_device,
	&cns3xxx_gpio_device,
	&cns3xxx_leds_device,
	&cns3xxx_watchdog_device,
	&cns3xxx_spi_controller_device,
	&cns3420_nor_pdev,
	&cns3xxx_usb_ehci_device,
	&cns3xxx_usb_ohci_device,
	&cns3xxx_i2c_controller_device,
	&cns3xxx_i2c_gpio_controller_device,
	&cns3xxx_i2s_device,
	&cns3xxx_pcm_device,
	&cns3xxx_crypto_device,
};

/*
 * Cavium Networks ARM11 MPCore AMBA devices
 */

#define CLCD_IRQ	{ IRQ_CLCD, NO_IRQ }
#define CLCD_DMA	{ 0, 0 }

AMBA_DEVICE(clcd,       "dev:20",       CLCD,   &clcd_plat_data);

static struct amba_device *amba_devs[] __initdata = {
	&clcd_device,
};


struct proc_dir_entry *cns3xxx_proc_dir;
EXPORT_SYMBOL_GPL(cns3xxx_proc_dir);

#ifdef CONFIG_DEBUG_FS
struct dentry *cns3xxx_debugfs_dir;
#endif

static void __init cns3420_init(void)
{
	int i;

#ifdef CONFIG_CACHE_L2X0
	/* ...010..., 32KB, 8-way, Parity Disable*/
	l2x0_init((void __iomem *) CNS3XXX_L2C_BASE_VIRT, 0x00540000,
			0xfe000fff);
#endif

#ifdef CONFIG_CNS3XXX_DMAC
	dmac_init();
#endif

#ifdef CONFIG_CNS3XXX_RAID
	cns_rdma_init();
#endif

	printk(KERN_INFO "Cavium feature enabled: "
#ifdef CONFIG_AMP_ALL
		"SOP, "
#endif
#ifdef CONFIG_AMP_PURENAS
		"SOP-PURENAS, "
#endif
#ifdef CONFIG_CNS3XXX_SPEEDUP_NAS
		"UP-NAS, "
#endif
#ifdef CONFIG_PAGE_SIZE_64K
		"64KB page size, "
		"Jumbo frame, "
#endif
		"\n");
	platform_add_devices(cns3420_pdevs, ARRAY_SIZE(cns3420_pdevs));

	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		int ret;

		cns3xxx_pwr_power_up(CNS3XXX_PWR_PLL(PLL_LCD));
		cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(LCDC));
		cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(LCDC));

		ret = amba_device_register(d, &iomem_resource);
		if (ret)
			printk("%s=>%d: %d\n", __func__, __LINE__, ret);
	}


	cns3xxx_ahci_init();
#if defined(CONFIG_MMC_SDHCI_CNS3XXX) || defined(CONFIG_MMC_SDHCI_CNS3XXX_MODULE)
	cns3xxx_sdhci_init();
#endif

	i2c_register_board_info(0, cns3xxx_i2c_devices,
			ARRAY_SIZE(cns3xxx_i2c_devices));
	i2c_register_board_info(1, cns3xxx_i2c_gpio_devices,
			ARRAY_SIZE(cns3xxx_i2c_gpio_devices));

	spi_register_board_info(cns3xxx_spi_devices,
			ARRAY_SIZE(cns3xxx_spi_devices));

	pm_power_off = cns3xxx_power_off;
	
	cns3xxx_proc_dir = proc_mkdir("cns3xxx", NULL);
#ifdef CONFIG_DEBUG_FS
	cns3xxx_debugfs_dir = debugfs_create_dir("cns3xxx", NULL);
#endif

	lm_device_register(&cns3xxx_usb_otg_device);

}

static struct map_desc cns3420_io_desc[] __initdata = {
	{
		.virtual	= CNS3XXX_UART0_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_UART0_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_USB_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_USB_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_USB_OHCI_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_USB_OHCI_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_RAID_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_RAID_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	},
};

#ifdef CONFIG_CNS3XXX_HIGH_PHYS_OFFSET
static __init void cns3420_fixup(struct machine_desc *desc,
				struct tag *tags, char **cmdline,
				struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = SZ_512M;
	mi->bank[0].node = 0;
}
#endif

static void __init cns3420_map_io(void)
{
	cns3xxx_map_io();
	iotable_init(cns3420_io_desc, ARRAY_SIZE(cns3420_io_desc));

	cns3420_early_serial_setup();
}

MACHINE_START(CNS3420VB, "Cavium Networks CNS3420 Validation Board")
	.phys_io	= CNS3XXX_UART0_BASE,
	.io_pg_offst	= (CNS3XXX_UART0_BASE_VIRT >> 18) & 0xfffc,
	.boot_params	= PHYS_OFFSET + 0x00000100,
#ifdef CONFIG_CNS3XXX_HIGH_PHYS_OFFSET
	.fixup		= cns3420_fixup,
#endif
	.map_io		= cns3420_map_io,
	.init_irq	= cns3xxx_init_irq,
	.timer		= &cns3xxx_timer,
	.init_machine	= cns3420_init,
MACHINE_END

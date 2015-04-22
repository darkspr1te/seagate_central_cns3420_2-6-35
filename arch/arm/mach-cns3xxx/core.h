/*
 * Copyright 2000 Deep Blue Solutions Ltd
 * Copyright 2004 ARM Limited
 * Copyright 2008 Cavium Networks
 * Copyright 2011 Cavium
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __CNS3XXX_CORE_H
#define __CNS3XXX_CORE_H

#include <linux/amba/bus.h>


#define AMBA_DEVICE(name, busid, base, plat)			\
static struct amba_device name##_device = {			\
	.dev		= {					\
		.coherent_dma_mask = ~0,			\
		.init_name = busid,				\
		.platform_data = plat,				\
	},							\
	.res		= {					\
		.start  = CNS3XXX_##base##_BASE,		\
		.end    = (CNS3XXX_##base##_BASE) + SZ_4K - 1,	\
		.flags  = IORESOURCE_MEM,			\
	},							\
	.dma_mask	= ~0,					\
	.irq		= base##_IRQ,				\
	/* .dma		= base##_DMA,*/				\
}

extern struct clk cns3xxx_clcd_clk;
extern struct clcd_board clcd_plat_data;

extern void __iomem *gic_cpu_base_addr;
extern struct sys_timer cns3xxx_timer;

void __init cns3xxx_map_io(void);
void __init cns3xxx_init_irq(void);
void cns3xxx_power_off(void);
void cns3xxx_pwr_power_up(unsigned int block);
void cns3xxx_pwr_power_down(unsigned int block);

#endif /* __CNS3XXX_CORE_H */

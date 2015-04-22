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

#ifndef __CNS3XXX_PM_H
#define __CNS3XXX_PM_H

#include <linux/atomic.h>

void cns3xxx_pwr_clk_en(unsigned int block);
void cns3xxx_pwr_clk_disable(unsigned int block);
/* Software reset*/
void cns3xxx_pwr_soft_rst(unsigned int block);

void cns3xxx_pwr_power_up(unsigned int block);
void cns3xxx_pwr_power_down(unsigned int block);
void cns3xxx_pwr_mode(unsigned int pwr_mode);
void cns3xxx_wfi(void);

extern atomic_t usb_pwr_ref;

#ifdef CONFIG_AMP
#define CNSX_DP_PWR_EVENT_PENDING	1
#define CNSX_DP_PWR_EVENT_ACK		2
#define CNSX_DP_RESUME			0
#define CNSX_DP_SUSPEND			1

extern int cnsx_dp_power_event;
extern int cnsx_dp_power_event_ack;

#endif /* CONFIG_AMP */

#endif /* __CNS3XXX_PM_H */

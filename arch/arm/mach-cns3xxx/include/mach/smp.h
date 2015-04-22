/*
 *  Copyright (c) 2011 Cavium
 *  Copyright (c) 2008 Cavium Networks
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
 *  Contact Cavium for more information
 */

#ifndef ASMARM_ARCH_SMP_H
#define ASMARM_ARCH_SMP_H

#include <linux/init.h>
#include <asm/hardware/gic.h>

#define hard_smp_processor_id()			\
	({						\
		unsigned int cpunum;			\
		__asm__("mrc p15, 0, %0, c0, c0, 5"	\
			: "=r" (cpunum));		\
		cpunum &= 0x0F;				\
	})

/*
 * We use IRQ1 as the IPI
 */
static inline void smp_cross_call(const struct cpumask *mask)
{
	gic_raise_softirq(mask, 1);
}

void cns3xxx_secondary_startup(void);
int __cpuinit __cpu_up_amp(unsigned int cpu);

/* TTBRs for CPU0 */
extern unsigned long ttbr0;
extern unsigned long ttbr1;
extern unsigned long ttbcr;

void dp_csd_unlock(void);
int boot_secondary(unsigned int cpu, struct task_struct *);
void send_ipi_wakeup_event(int cpu);

/* This variable is used for WFI */
extern volatile int __cpuinitdata pen_release;

#endif

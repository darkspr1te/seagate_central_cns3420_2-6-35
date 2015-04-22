/*
 * Copyright 2003 ARM Limited
 * Copyright 2008 Cavium Networks
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#include <asm/mach-types.h>
#include <linux/serial_reg.h>
#include <mach/hardware.h>
#include <mach/cns3xxx.h>

/*
 * Return the UART base address
 */
static inline unsigned char * get_uart_base(void)
{
	if (machine_is_cns3420vb())
		return (unsigned char *)CNS3XXX_UART0_BASE;
	else
		return 0;
}

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
	volatile unsigned char *base = get_uart_base();
	int i;

	for (i = 0; i < 0x1000; i++) {
		if (base[UART_LSR << 2] & UART_LSR_THRE)
			break;
		barrier();
	}

	base[UART_TX << 2] = c;
}

static inline void flush(void)
{
	volatile unsigned char *base = get_uart_base();
	unsigned char mask;
	int i;

	mask = UART_LSR_TEMT | UART_LSR_THRE;

	for (i = 0; i < 0x1000; i++) {
		if ((base[UART_LSR << 2] & mask) == mask)
			break;
		barrier();
	}
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()

/*
 * Copyright 2008 Cavium Networks
 * Copyright 2011 Cavium
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_BOARD_CNS3XXXH_SUM
#define __MACH_BOARD_CNS3XXXH_SUM

#ifdef CONFIG_PAGE_SIZE_64K
#include <mach/cns3xxx-64k.h>
#else
#include <mach/cns3xxx-4k.h>
#endif

#include <mach/gpio-regs.h>

/* KH: 20110628 add for the SATA stagger support
 *     20110817 move to inside__MACH_BOARD_CNS3XXXH_SUM to prevent the
 *              nested header file including.
 *     20110830 add the STAGGER_SATA_PORT to 32 , and set the PORT disk
 *              from 5 to 9
 */
/* ------------------------------------------------- */
#define CNSX_STAGGER_SATA_PORT		32
#define CNSX_STAGGER_SATA_PORT_DISKS	9
#define CNSX_STAGGER_DELAY		2000
/* ------------------------------------------------- */

#endif

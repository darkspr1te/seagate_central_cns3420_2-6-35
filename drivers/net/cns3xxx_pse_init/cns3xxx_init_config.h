/*******************************************************************************
 *
 *   Copyright (c) 2009 Cavium Networks 
 *
 *   This program is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License as published by the Free
 *   Software Foundation; either version 2 of the License, or (at your option)
 *   any later version.
 *
 *   This program is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *   more details.
 *
 *   You should have received a copy of the GNU General Public License along with
 *   this program; if not, write to the Free Software Foundation, Inc., 59
 *   Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 *   The full GNU General Public License is included in this distribution in the
 *   file called LICENSE.
 *
 ********************************************************************************/
#include <linux/cns3xxx/pse_init_common.h>

#ifndef CNS3XXX_INIT_CONFIG_H
#define CNS3XXX_INIT_CONFIG_H

#define CNS3XXX_SET_ARL_TABLE
#define CNS3XXX_AGE_ENABLE
#define CNS3XXX_LEARN_ENABLE
#define CNS3XXX_CPU_PORT_FC
//#define CNS3XXX_CONFIG_SIM_MODE
//#define ACCEPT_CRC_BAD_PKT

#ifndef CONFIG_CNS3XXX_SPPE
#define IVL // if no define, use SVL
#else
#ifdef  CONFIG_VITESSE7395
#define IVL 
#endif
#endif

#ifdef CONFIG_CNS3XXX_SPEEDUP_NAS
#define CNS3XXX_DELAYED_INTERRUPT
#endif

#ifdef CNS3XXX_DELAYED_INTERRUPT
#define MAX_PEND_INT_CNT 0x06
#define MAX_PEND_TIME 0x20
#endif

#endif

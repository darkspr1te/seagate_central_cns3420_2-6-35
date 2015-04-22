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

#include <linux/version.h>
#include <linux/cns3xxx/pse_init_common.h>

#ifndef CNS3XXX_CONFIG_H
#define CNS3XXX_CONFIG_H

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,27)
#define LINUX2627 1
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
#define LINUX2631 1
#endif

//#define CONFIG_SWITCH_BIG_ENDIAN

//#define CONFIG_FPGA_FORCE

//#define CNS3XXX_GIGA_MODE

#define CNS3XXX_CPU_MIB_COUNTER
#define CNS3XXX_MAC0_MIB_COUNTER
#define CNS3XXX_MAC1_MIB_COUNTER
#define CNS3XXX_MAC2_MIB_COUNTER
//#define QOS_TEST
//#define CONFIG_FAST_BRIDGE
//#define CONFIG_HOLP_TEST


#define CONFIG_CNS3XXX_NAPI
#ifdef CONFIG_CNS3XXX_NAPI
#define CNS3XXX_NAPI_WEIGHT 64
#endif
//#define CONFIG_NIC_MODE
#define CNS3XXX_TX_HW_CHECKSUM
#define CNS3XXX_RX_HW_CHECKSUM

#ifdef CNS3XXX_TX_HW_CHECKSUM
/* XXX: this just for router performance test, 
 * should't be defined in formal release */
//#define CNS3XXX_SG_DISABLE 
#endif

//#define CNS3XXX_SHOW_LINK_STATUS

#ifdef CNS3XXX_SHOW_LINK_STATUS
#define CNS3XXX_STATUS_ISR
#endif
//#define CNS3XXX_STATUS_ISR
//#define CNS3XXX_TEST_ONE_LEG_VLAN
//#define CNS3XXX_TX_DSCP_PROC


#define CNS3XXX_FSQF_RING0_ISR
//#define CNS3XXX_TSTC_RING0_ISR
//#define CNS3XXX_TSTC_RING1_ISR

//#define CNS3XXX_COMPARE_PACKET
//#define CONFIG_FPGA_10

//#define CNS3XXX_4N // if don't define it, use 4N+2

//#define NCNB_TEST
//#define CNS3XXX_TEST_D_CACHE
#define CNS3XXX_FREE_TX_IN_RX_PATH

// This MACRO should not be defined, only for FPGA test.
//#define DO_CLEAR_FS_DMA

//#define DEBUG_RX
//#define DEBUG_TX
//#define DEBUG_PRIO_IPDSCR
#define DEBUG_RX_PROC
#define DEBUG_TX_PROC
//#define DEBUG_PHY_PROC
#define CNS3XXX_PVID_PROC
#define CNS3XXX_SARL_PROC


//#define DOUBLE_RING_TEST

//#define CNS3XXX_DOUBLE_RX_RING
//#define CNS3XXX_DOUBLE_TX_RING
#define CNS3XXX_USE_MASK

#define CNS3XXX_CONFIG_CHANGE_TX_RING

#ifdef CNS3XXX_DOUBLE_RX_RING
#define CNS3XXX_FSQF_RING1_ISR
#endif

//#define CNS3XXX_ENABLE_RINT1

#define PRINT_INFO printk

#endif

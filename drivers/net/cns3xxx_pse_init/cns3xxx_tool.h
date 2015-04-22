/*******************************************************************************
 *
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

#ifndef CNS3XXX_TOOL_H
#define CNS3XXX_TOOL_H

#define PRINT_INFO printk

#if defined(__KERNEL__)

#include "cns3xxx_symbol.h"
#include "cns3xxx.h"
#include <linux/kernel.h> // for printk

#else // u-boot

#endif

#define SHOW_DEBUG_MESSAGE
#ifdef SHOW_DEBUG_MESSAGE

extern int MSG_LEVEL;

#define NO_MSG 0
#define NORMAL_MSG 1
#define WARNING_MSG (1 << 1)
#define CRITICAL_MSG (1 << 2)
#define DUMP_RX_PKT_INFO (1 << 3)
#define DUMP_TX_PKT_INFO (1 << 4)

#define DEBUG_MSG(msg_level, fmt, args...)\
{ \
        int i=0; \
\
        for(i=0 ; i < 3 ; ++i) { \
                if ((MSG_LEVEL & msg_level) >> i) \
                        printk(KERN_INFO "*cns3xxx gsw debug* " fmt, ## args); \
        } \
}

#endif

#define GET_MAC_PORT_CFG(port, cfg) \
{ \
	switch (port) \
	{ \
		case MAC_PORT0: \
		{ \
			cfg = MAC0_CFG_REG; \
			break; \
		} \
		case MAC_PORT1: \
		{ \
			cfg = MAC1_CFG_REG; \
			break; \
		} \
		case MAC_PORT2: \
		{ \
			cfg = MAC2_CFG_REG; \
			break; \
		} \
	} \
}

#define SET_MAC_PORT_CFG(port, cfg) \
{ \
	switch (port) \
	{ \
		case MAC_PORT0: \
		{ \
			MAC0_CFG_REG = cfg; \
			break; \
		} \
		case MAC_PORT1: \
		{ \
			MAC1_CFG_REG = cfg; \
			break; \
		} \
		case MAC_PORT2: \
		{ \
			MAC2_CFG_REG = cfg; \
			break; \
		} \
	} \
}

#define GET_PORT_PHY_ADDR(port, phy_addr) \
{\
	switch (port) \
	{\
		case MAC_PORT0: \
			phy_addr = PHY_AUTO_ADDR_REG & 0x1f; \
			break; \
		case MAC_PORT1: \
			phy_addr = (PHY_AUTO_ADDR_REG >> 8) & 0x1f; \
			break; \
		case MAC_PORT2: \
			phy_addr = (PHY_AUTO_ADDR_REG >> 16) & 0x1f; \
			break; \
	} \
}

static inline void cns3xxx_gsw_power_enable(void)
{
        PLL_HM_PD_CTRL_REG &= (~(1 << 2)); // power up PLL_RGMII (for MAC)
        CLK_GATE_REG |= (1 << 11); // enable switch clock
}       

static inline void cns3xxx_gsw_software_reset(void)
{
        SOFT_RST_REG &= (~(1 << 11));
        SOFT_RST_REG |= (1 << 11); 
}




// port:
// 0 : mac port0
// 1 : mac port1
// 2 : mac port2
// 3 : cpu port
static inline void enable_port(u8 port, u8 enable)
{
	switch (port)
	{
		case 0:
		{
			(enable==1) ? (MAC0_CFG_REG &= (~(1 << 18)) ) : (MAC0_CFG_REG |= (1 << 18)) ;

			break;
		}
		case 1:
		{
			(enable==1) ? (MAC1_CFG_REG &= (~(1 << 18)) ) : (MAC1_CFG_REG |= (1 << 18)) ;
			break;
		}
		case 2:
		{
			(enable==1) ? (MAC2_CFG_REG &= (~(1 << 18)) ) : (MAC2_CFG_REG |= (1 << 18)) ;
			break;
		}
		case 3:
		{
			(enable==1) ? (CPU_CFG_REG &= (~(1 << 18)) ) : (CPU_CFG_REG |= (1 << 18)) ;
			break;
		}
	}
}

static inline int cns3xxx_vlan_table_lookup(VLANTableEntry *entry)
{
	VLAN_CTRL2_REG |= entry->vid;
	ARL_VLAN_CMD_REG |= (1 << 8); // look up vlan table command

	// wait for vlan command complete
	while(( (ARL_VLAN_CMD_REG >> 9) & 1) == 0) ;
	
	if (!((ARL_VLAN_CMD_REG >> 10) & 1)) {
		// not found any entry
		return CAVM_NOT_FOUND;
	}

        entry->valid = ((VLAN_CTRL0_REG >> 31) & 0x1);
        entry->vid = ((VLAN_CTRL2_REG >> 31) & 0xfff);
        entry->wan_side = ((VLAN_CTRL0_REG >> 30) & 0x1);
        entry->etag_pmap = ((VLAN_CTRL0_REG >> 25) & 0x1f);
        entry->mb_pmap = ((VLAN_CTRL0_REG >> 9) & 0x1f);

        entry->my_mac[0] = ((VLAN_CTRL1_REG >> 24) & 0xff);
        entry->my_mac[1] = ((VLAN_CTRL1_REG >> 16) & 0xff);
        entry->my_mac[2] = ((VLAN_CTRL1_REG >> 8) & 0xff);
        entry->my_mac[3] = (VLAN_CTRL1_REG & 0xff);

        entry->my_mac[4] = ((VLAN_CTRL2_REG >> 24) & 0xff);
        entry->my_mac[5] = ((VLAN_CTRL2_REG >> 16) & 0xff);

	return CAVM_FOUND;
}

static inline int cns3xxx_vlan_table_read(VLANTableEntry *entry)
{
        //printf("VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	ARL_VLAN_CMD_REG &= (~0x3f);
	ARL_VLAN_CMD_REG |= (entry->vlan_index);
	ARL_VLAN_CMD_REG |= (1 << 7); // read vlan table command
        //printf("after read ARL_VLAN_CMD_REG: %x\n", ARL_VLAN_CMD_REG);

	// wait for vlan command complete
	while(( (ARL_VLAN_CMD_REG >> 9) & 1) == 0) ;

        //printf("ARL_VLAN_CMD_REG: %x\n", ARL_VLAN_CMD_REG);
        //printf("VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);

        entry->valid = ((VLAN_CTRL0_REG >> 31) & 0x1);
        entry->vid = ((VLAN_CTRL2_REG) & 0xfff);
        entry->wan_side = ((VLAN_CTRL0_REG >> 30) & 0x1);
        entry->etag_pmap = ((VLAN_CTRL0_REG >> 25) & 0x1f);
        entry->mb_pmap = ((VLAN_CTRL0_REG >> 9) & 0x1f);

        entry->my_mac[0] = ((VLAN_CTRL1_REG >> 24) & 0xff);
        entry->my_mac[1] = ((VLAN_CTRL1_REG >> 16) & 0xff);
        entry->my_mac[2] = ((VLAN_CTRL1_REG >> 8) & 0xff);
        entry->my_mac[3] = (VLAN_CTRL1_REG & 0xff);

        entry->my_mac[4] = ((VLAN_CTRL2_REG >> 24) & 0xff);
        entry->my_mac[5] = ((VLAN_CTRL2_REG >> 16) & 0xff);

	printk("[kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	printk("[kernel mode] VLAN_CTRL1_REG: %x\n", VLAN_CTRL1_REG);
	printk("[kernel mode] VLAN_CTRL2_REG: %x\n", VLAN_CTRL2_REG);

	return CAVM_OK;

}


// add a entry in the vlan table
static inline int cns3xxx_vlan_table_add(VLANTableEntry *entry)
{
	VLAN_CTRL0_REG = 0;
	VLAN_CTRL1_REG = 0;
	VLAN_CTRL2_REG = 0;

#if 0
	printk("a [kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	printk("a [kernel mode] VLAN_CTRL1_REG: %x\n", VLAN_CTRL1_REG);
	printk("a [kernel mode] VLAN_CTRL2_REG: %x\n", VLAN_CTRL2_REG);
#endif

	//printk("vlan_index: %x\n", entry->vlan_index);
	VLAN_CTRL0_REG |= (entry->valid << 31);
	//DEBUG_MSG(NORMAL_MSG, "1 [kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	VLAN_CTRL0_REG |= (entry->wan_side << 30);
	//DEBUG_MSG(NORMAL_MSG, "2 [kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	//printk("entry->etag_pmap: %x\n", entry->etag_pmap);
	VLAN_CTRL0_REG |= (entry->etag_pmap << 25);
	//DEBUG_MSG(NORMAL_MSG, "3 [kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	//printk("entry->mb_pmap: %x\n", entry->mb_pmap);
	VLAN_CTRL0_REG |= (entry->mb_pmap << 9);
	//DEBUG_MSG(NORMAL_MSG, "4 [kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	//printk("bb [kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);

        //printf("vlan index: %d ## add VLAN_CTRL0_REG: %x\n", entry->vlan_index, VLAN_CTRL0_REG);


	VLAN_CTRL1_REG |= (entry->my_mac[0] << 24);
	VLAN_CTRL1_REG |= (entry->my_mac[1] << 16);
	VLAN_CTRL1_REG |= (entry->my_mac[2] << 8);
	VLAN_CTRL1_REG |= (entry->my_mac[3]);

	VLAN_CTRL2_REG |= (entry->my_mac[4] << 24);
	VLAN_CTRL2_REG |= (entry->my_mac[5] << 16);
	VLAN_CTRL2_REG |= entry->vid;

#if 0
	printk("b [kernel mode] VLAN_CTRL0_REG: %x\n", VLAN_CTRL0_REG);
	printk("b [kernel mode] VLAN_CTRL1_REG: %x\n", VLAN_CTRL1_REG);
	printk("b [kernel mode] VLAN_CTRL2_REG: %x\n", VLAN_CTRL2_REG);
#endif

	ARL_VLAN_CMD_REG &= (~0x3f);
	ARL_VLAN_CMD_REG |= (entry->vlan_index);
	ARL_VLAN_CMD_REG |= (1 << 6); // write vlan table command


        //printf("after write ARL_VLAN_CMD_REG: %x\n", ARL_VLAN_CMD_REG);

	// wait for vlan command complete
	while(( (ARL_VLAN_CMD_REG >> 9) & 1) == 0) ;

	return CAVM_OK;
}

static inline void print_arl_table_entry(ARLTableEntry *entry)
{
        printk("vid: %d\n", entry->vid);
        printk("pmap: %#x\n", entry->pmap);
        printk("age_field: %d\n", entry->age_field);
        printk("vlan_mac: %d\n", entry->vlan_mac);
        printk("filter: %d\n", entry->filter);
        printk("mac addr: %x:%x:%x:%x:%x:%x\n", entry->mac[0], entry->mac[1],entry->mac[2],entry->mac[3],entry->mac[4],entry->mac[5]);

}


static inline int cns3xxx_arl_table_lookup(ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	ARL_CTRL0_REG |= (entry->vid << 16);

	ARL_CTRL1_REG |= (entry->mac[0] << 24);
	ARL_CTRL1_REG |= (entry->mac[1] << 16);
	ARL_CTRL1_REG |= (entry->mac[2] << 8);
	ARL_CTRL1_REG |= entry->mac[3];

	ARL_CTRL2_REG |= (entry->mac[4] << 24);
	ARL_CTRL2_REG |= (entry->mac[5] << 16);

	ARL_VLAN_CMD_REG |= (1 << 18); // arl table lookup command

	// wait arl command complete
	while(( (ARL_VLAN_CMD_REG >> 21) & 1) == 0);

	if (( (ARL_VLAN_CMD_REG >> 23) & 1)) {
		// found

		entry->vid = ((ARL_CTRL0_REG >> 16) & 0xfff);
		entry->pmap = ((ARL_CTRL0_REG >> 9) & 0x1f);

		entry->age_field = ((ARL_CTRL2_REG >> 4 ) & 0x7);
		entry->vlan_mac = ((ARL_CTRL2_REG >> 1 ) & 0x1);
		entry->filter = (ARL_CTRL2_REG & 0x1);
	} else {
		// not found
		return CAVM_NOT_FOUND;
	}
#if 0
	printk("[kernel mode] ARL_VLAN_CMD_REG : %#x\n", ARL_VLAN_CMD_REG);
	printk("[kernel mode] ARL_CTRL0_REG : %#x\n", ARL_CTRL0_REG);
	printk("[kernel mode] ARL_CTRL1_REG : %#x\n", ARL_CTRL1_REG);
	printk("[kernel mode] ARL_CTRL2_REG : %#x\n", ARL_CTRL2_REG);
#endif

	return CAVM_FOUND;
}

static inline int cns3xxx_arl_table_search_again(ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	ARL_VLAN_CMD_REG |= (1 << 17); // arl table search again command

	// wait arl command complete
	while(( (ARL_VLAN_CMD_REG >> 21) & 1) == 0);

	if ((ARL_VLAN_CMD_REG >> 23) & 1) {

		// found
	#if 0
	printk("[kernel mode] ARL_VLAN_CMD_REG : %#x\n", ARL_VLAN_CMD_REG);
	printk("[kernel mode] ARL_CTRL0_REG : %#x\n", ARL_CTRL0_REG);
	printk("[kernel mode] ARL_CTRL1_REG : %#x\n", ARL_CTRL1_REG);
	printk("[kernel mode] ARL_CTRL2_REG : %#x\n", ARL_CTRL2_REG);
	#endif
		entry->vid = ((ARL_CTRL0_REG >> 16) & 0xfff);
		entry->pmap = ((ARL_CTRL0_REG >> 9) & 0x1f);

		entry->age_field = ((ARL_CTRL2_REG >> 4 ) & 0x7);
		entry->vlan_mac = ((ARL_CTRL2_REG >> 1 ) & 0x1);
		entry->filter = (ARL_CTRL2_REG & 0x1);

		entry->mac[0] = (ARL_CTRL1_REG >> 24);
		entry->mac[1] = (ARL_CTRL1_REG >> 16);
		entry->mac[2] = (ARL_CTRL1_REG >> 8);
		entry->mac[3] = ARL_CTRL1_REG;

		entry->mac[4] = (ARL_CTRL2_REG >> 24);
		entry->mac[5] = (ARL_CTRL2_REG >> 16);

		return CAVM_FOUND;
	} else {
		// not found
		return CAVM_NOT_FOUND;
	}
}

static inline int cns3xxx_is_arl_table_end(void)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	if (( (ARL_VLAN_CMD_REG >> 22) & 1)) { // search to table end
		return CAVM_OK;
	} else {
		return CAVM_ERR;
	}
}

static inline int cns3xxx_arl_table_search(ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

#if 0
	ARL_CTRL0_REG |= (entry->vid << 16);

	ARL_CTRL1_REG |= (entry->mac[0] << 24);
	ARL_CTRL1_REG |= (entry->mac[1] << 16);
	ARL_CTRL1_REG |= (entry->mac[2] << 8);
	ARL_CTRL1_REG |= entry->mac[3];

	ARL_CTRL2_REG |= (entry->mac[4] << 24);
	ARL_CTRL2_REG |= (entry->mac[5] << 16);
#endif
	printk("ARL_VLAN_CMD_REG: %x\n", ARL_VLAN_CMD_REG);
	ARL_VLAN_CMD_REG |= (1 << 16); // arl table search start command
	printk("11 ARL_VLAN_CMD_REG: %x\n", ARL_VLAN_CMD_REG);

	// wait arl command complete
	while(( (ARL_VLAN_CMD_REG >> 21) & 1) == 0);

	if (((ARL_VLAN_CMD_REG >> 23) & 1)) {
		// found
	#if 0
	printk("[kernel mode] ARL_VLAN_CMD_REG : %#x\n", ARL_VLAN_CMD_REG);
	printk("[kernel mode] ARL_CTRL0_REG : %#x\n", ARL_CTRL0_REG);
	printk("[kernel mode] ARL_CTRL1_REG : %#x\n", ARL_CTRL1_REG);
	printk("[kernel mode] ARL_CTRL2_REG : %#x\n", ARL_CTRL2_REG);
	#endif
		entry->vid = ((ARL_CTRL0_REG >> 16) & 0xfff);
		entry->pmap = ((ARL_CTRL0_REG >> 9) & 0x1f);

		entry->age_field = ((ARL_CTRL2_REG >> 4 ) & 0x7);
		entry->vlan_mac = ((ARL_CTRL2_REG >> 1 ) & 0x1);
		entry->filter = (ARL_CTRL2_REG & 0x1);

		entry->mac[0] = (ARL_CTRL1_REG >> 24);
		entry->mac[1] = (ARL_CTRL1_REG >> 16);
		entry->mac[2] = (ARL_CTRL1_REG >> 8);
		entry->mac[3] = ARL_CTRL1_REG;

		entry->mac[4] = (ARL_CTRL2_REG >> 24);
		entry->mac[5] = (ARL_CTRL2_REG >> 16);

		return CAVM_FOUND;
	} else {
		// not found
		return CAVM_NOT_FOUND;
	}
}


// flush all age out entries except static entries
static inline int cns3xxx_arl_table_flush(void)
{
	ARL_VLAN_CMD_REG |= (1 << 20); // flush arl table command

	// wait arl command complete
	while(( (ARL_VLAN_CMD_REG >> 21) & 1) == 0);


	return CAVM_OK;
}


// add a entry in the arl table
static inline int cns3xxx_arl_table_add(ARLTableEntry *entry)
{
	ARL_CTRL0_REG = 0;
	ARL_CTRL1_REG = 0;
	ARL_CTRL2_REG = 0;

	entry->age_field = 7; // static entry
	ARL_CTRL0_REG |= (entry->vid << 16);
	ARL_CTRL0_REG |= (entry->pmap << 9);

	ARL_CTRL1_REG |= (entry->mac[0] << 24);
	ARL_CTRL1_REG |= (entry->mac[1] << 16);
	ARL_CTRL1_REG |= (entry->mac[2] << 8);
	ARL_CTRL1_REG |= entry->mac[3];

	ARL_CTRL2_REG |= (entry->mac[4] << 24);
	ARL_CTRL2_REG |= (entry->mac[5] << 16);

	ARL_CTRL2_REG |= (entry->age_field << 4);
	ARL_CTRL2_REG |= (entry->vlan_mac << 1);
	ARL_CTRL2_REG |= (entry->filter);

	//printk("entry->age_field: %d\n", entry->age_field);
	//printk("ARL_CTRL2_REG: %x\n", ARL_CTRL2_REG);

	ARL_VLAN_CMD_REG |= (1 << 19); // arl table write command

	// wait arl command complete
	while(( (ARL_VLAN_CMD_REG >> 21) & 1) == 0);

	return CAVM_OK;
}

// invalid a entry in the arl table
static inline int cns3xxx_arl_table_invalid(ARLTableEntry *entry)
{
	entry->age_field = 0; // invalid
	return cns3xxx_arl_table_add(entry);
}

// port:
// 0 : mac port0
// 1 : mac port1
// 2 : mac port2
// 3 : cpu port
static inline void cns3xxx_set_pvid(u8 port, u16 pvid)
{
	switch (port)
	{
		case 0:
		{
			MAC1_MAC0_PVID_REG &= (~0x0fff);
			MAC1_MAC0_PVID_REG |= pvid;
			break;
		}
		case 1:
		{
			MAC1_MAC0_PVID_REG &= (~(0x0fff << 16));
			MAC1_MAC0_PVID_REG |= (pvid << 16);
			break;
		}
		case 2:
		{
			MAC2_CPU_PVID_REG &= (~(0x0fff << 16));
			MAC2_CPU_PVID_REG |= (pvid << 16);
			break;
		}
		case 3: // cpu port
		{
			MAC2_CPU_PVID_REG &= (~0x0fff);
			MAC2_CPU_PVID_REG |= pvid;
			break;
		}
	}


}

static inline u16 cns3xxx_get_pvid(u8 port)
{
		        // 0,     1,   2,    cpu port
	u16 port_offset[]={0x9c, 0x9c, 0xa0, 0xa0};
		      // 0, 1,   2,  cpu port
	u16 port_shift[]={0, 16, 16, 0};

	return ((SWITCH_REG_VALUE(port_offset[port]) >> port_shift[port]) & 0xfff);
}

// which : 0 or 1
// enable: 0 or 1
static inline int enable_rx_dma(u8 which, u8 enable)
{
	if (which == 0 ) {
		FS_DMA_CTRL0_REG = enable;
	} else if (which == 1 ) {
		FS_DMA_CTRL1_REG = enable;
	} else {
		return CAVM_ERR;
	}
	return CAVM_OK;
}


// which : 0 or 1
// enable: 0 or 1
static inline int enable_tx_dma(u8 which, u8 enable)
{
	if (which == 0 ) {
		TS_DMA_CTRL0_REG = enable;
	} else if (which == 1 ) {
		TS_DMA_CTRL1_REG = enable;
	} else {
		return CAVM_ERR;
	}
	return CAVM_OK;
}

static inline void rx_dma_suspend(u8 enable)
{
#if 1
	DMA_AUTO_POLL_CFG_REG &= (~0x00000001);
	if (enable == 1)
		DMA_AUTO_POLL_CFG_REG |= 1;
#endif
}


// clear: 0 normal
// clear: 1 clear
static inline void clear_fs_dma_state(u8 clear)
{
#ifdef DO_CLEAR_FS_DMA
	//printk("do clear fs dma\n");
	DMA_RING_CTRL_REG &= (~(1 << 31));
	if (clear==1) {
		DMA_RING_CTRL_REG |= (1 << 31);
		while(((DMA_RING_CTRL_REG >> 30) & 1) == 0);
		DMA_RING_CTRL_REG &= (~(1 << 31));
	}
	//printk("DMA_RING_CTRL_REG: %#x\n", DMA_RING_CTRL_REG);
#else
	//printk("in clear_fs_dma_state: do nothing.\n");
#endif
}

// enable: 1 -> IVL
// enable: 0 -> SVL
static inline void cns3xxx_ivl(u8 enable)
{
	// SVL
	MAC_GLOB_CFG_REG &= (~(0x1 << 7));
	if (enable == 1)
		MAC_GLOB_CFG_REG |= (0x1 << 7);
}

static inline void cns3xxx_nic_mode(u8 enable)
{
	VLAN_CFG &= (~(1<<15));
	if (enable == 1) 
		VLAN_CFG |= (1<<15);
}

#endif // CNS3XXX_TOOL_H

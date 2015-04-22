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

// This macro or function divide two part, 
// one is initial state, another is in netdev open (ifconfig up) function.

#ifndef  RTL8366RB_H
#define  RTL8366RB_H

#include <linux/types.h>

//#include "cns3xxx_config.h"
#include "cns3xxx.h"
#include "cns3xxx_phy.h"

#ifdef CONFIG_RTL8366RB
// init phy or switch chip
//#define INIT_PORT0_PHY bcm53115M_init(0, 0);
#define INIT_PORT0_PHY rtl8366rb_init(0, 0);
#define INIT_PORT1_PHY 
//#define INIT_PORT2_PHY icp_ip1001_init(2, 2);
#define INIT_PORT2_PHY
//#define INIT_PORT1_PHY 

// configure mac0/mac1 register
#define INIT_PORT0_MAC 
#define INIT_PORT1_MAC
#define INIT_PORT2_MAC 
//#define INIT_PORT1_MAC 

#define PORT0_LINK_DOWN disable_AN(0, 0);
#define PORT0_LINK_UP disable_AN(0, 1);

#define PORT1_LINK_DOWN disable_AN(1, 0);
#define PORT1_LINK_UP disable_AN(1, 1);

//#define PORT2_LINK_DOWN cns3xxx_std_phy_power_down(2, 1);
//#define PORT2_LINK_UP cns3xxx_std_phy_power_down(2, 0);
#define PORT2_LINK_DOWN disable_AN(1, 0);
#define PORT2_LINK_UP disable_AN(1, 1);

#define MODEL "CNS3XXX validation board"

static int rc_port0 = 0; // rc means reference counting, determine port open/close.

#define PRINT_INFO printk

// enable port
// link down
static inline void open_port0(void)
{
        if (rc_port0 == 0) {
		enable_port(0, 1);
		//PRINT_INFO("open mac port 0\n");
		// link up
		PORT0_LINK_UP
	} else {
		//PRINT_INFO("port 0 already open\n");
	}
	++rc_port0;
}

static inline void close_port0(void)
{
	--rc_port0;
        if (rc_port0 == 0) {
		// link down
		PORT0_LINK_DOWN
		enable_port(0, 0);
       		//PRINT_INFO("close mac port 0\n");
	}
}

static inline void open_port1(void)
{

	enable_port(1, 1);
	//PRINT_INFO("open mac port 1\n");
	// link up
	PORT1_LINK_UP
}

static inline void close_port1(void)
{
	enable_port(1, 0);
	//PRINT_INFO("close mac port 1\n");
	// link down
	PORT1_LINK_DOWN
}

static inline void open_port2(void)
{
	enable_port(2, 1);
	//PRINT_INFO("open mac port 2\n");
	// link up
	PORT2_LINK_UP
}

static inline void close_port2(void)
{
	enable_port(2, 0);
	//PRINT_INFO("close mac port 2\n");
	// link down
	PORT2_LINK_DOWN
}

#if defined (CONFIG_CNS3XXX_SPPE)
/* only for PPE PCI-to-WAN fast path */
static int fp_ref_cnt = 0;
static inline void open_fp(void)
{
	if (!fp_ref_cnt) {
		fp_ref_cnt++;
	}
}

static inline void close_fp(void)
{
	if (fp_ref_cnt) {
		fp_ref_cnt--;
	}
}
#endif

#if 0
static u8 my_vlan0_mac[] = {0x00, 0x11, 0x22, 0x33, 0x55, 0x00};
static u8 my_vlan1_mac[] = {0x00, 0x11, 0x22, 0x33, 0x55, 0x11};
static u8 my_vlan2_mac[] = {0x00, 0x11, 0xbb, 0xcc, 0xdd, 0x70};
static u8 my_vlan3_mac[] = {0x00, 0x11, 0xbb, 0xcc, 0xdd, 0x80};
#endif

#define my_vlan0_mac {0x00, 0x11, 0x22, 0x33, 0x55, 0x00}
#define my_vlan1_mac {0x00, 0x11, 0x22, 0x33, 0x55, 0x11}
#define my_vlan2_mac {0x00, 0x11, 0xbb, 0xcc, 0xdd, 0x70}
#define my_vlan3_mac {0x00, 0x11, 0xbb, 0xcc, 0xdd, 0x80}



// CNS3XXX_NIC_MODE_8021Q, CNS3XXX_NON_NIC_MODE_8021Q, CNS3XXX_VLAN_BASE_MODE and
// CNS3XXX_PORT_BASE_MODE, only one macro can be defined

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
	//#define CNS3XXX_NIC_MODE_8021Q
	#ifndef CNS3XXX_NIC_MODE_8021Q
		#define CNS3XXX_NON_NIC_MODE_8021Q
	#endif
#else
	//#define CNS3XXX_VLAN_BASE_MODE
	#define CNS3XXX_PORT_BASE_MODE
#endif

//#define CNS3XXX_PORT_BASE_MODE
//
#ifdef CNS3XXX_NON_NIC_MODE_8021Q

#define PORT0_PVID 50
#define PORT1_PVID 60
#define PORT2_PVID 70
#define CPU_PVID 80

#define CONFIG_CNS3XXX_PORT_BASE

VLANTableEntry cpu_vlan_table_entry = {0, 1, CPU_PVID, 0, 0, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan3_mac}; 

VLANTableEntry vlan_table_entry[] = 
{
	// vlan_index; valid; vid; wan_side; etag_pmap; mb_pmap; *my_mac;
	#if 0
	{1, 1, PORT0_PVID, 0, 0, MAC_PORT0_PMAP | CPU_PORT_PMAP, my_vlan0_mac},
	{2, 1, PORT1_PVID, 0, 0, MAC_PORT1_PMAP | CPU_PORT_PMAP, my_vlan1_mac},
	{3, 1, PORT2_PVID, 1, 0, MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan2_mac},
	#endif

	{1, 1, PORT0_PVID, 0, CPU_PORT_PMAP, MAC_PORT0_PMAP | CPU_PORT_PMAP, my_vlan0_mac},
	{2, 1, PORT1_PVID, 0, CPU_PORT_PMAP, MAC_PORT1_PMAP | CPU_PORT_PMAP, my_vlan1_mac},
	{3, 1, PORT2_PVID, 0, CPU_PORT_PMAP, MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan2_mac},
	//{2, 1, 4, 0, 0, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan3_mac}, // for cpu
};

ARLTableEntry arl_table_entry[] = 
{
	// vid; pmap; *mac; age_field; vlan_mac ; filter
	{PORT0_PVID, CPU_PORT_PMAP, my_vlan0_mac, 7, 1, 0},
	//{CPU_PVID, CPU_PORT_PMAP, my_vlan0_mac, 7, 1, 0},
	{PORT1_PVID, CPU_PORT_PMAP, my_vlan1_mac, 7, 1, 0},
	{PORT2_PVID, CPU_PORT_PMAP, my_vlan2_mac, 7, 1, 0},
	//{PORT0_PVID, MAC_PORT0_PMAP, my_vlan8_mac, 7, 0, 0},
	//{PORT0_PVID, MAC_PORT0_PMAP, my_vlan9_mac, 7, 0, 0},
	//{CPU_PVID, 0x4, my_vlan2_mac, 7, 1, 0},
	//{CPU_PVID, MAC_PORT2_PMAP, my_vlan2_mac, 7, 1, 0},
};

NetDevicePriv net_device_prive[]= {
	/*  pmap, is_wan, s-tag, vlan_tag or pvid, rx_func_ptr, tx_func_ptr, open_ptr, close_ptr, which port, mac, VLANTableEntry, ARLTableEntry, NICSetting, netdev s-tag, name */
	{MAC_PORT0_PMAP, 0, 1, PORT0_NETDEV_INDEX, rx_port_base, tx_port_base, open_port0, close_port0, MAC_PORT0, my_vlan0_mac, &vlan_table_entry[0], &arl_table_entry[0], 0, 0},   // eth0 
	{MAC_PORT1_PMAP, 0, 2, PORT1_NETDEV_INDEX, rx_port_base, tx_port_base, open_port1, close_port1, MAC_PORT1, my_vlan1_mac, &vlan_table_entry[1], &arl_table_entry[1], 0, 0},   // eth1 
	{MAC_PORT2_PMAP, 1, 3, PORT2_NETDEV_INDEX, rx_port_base, tx_port_base, open_port2, close_port2, MAC_PORT2, my_vlan2_mac,  &vlan_table_entry[2], &arl_table_entry[2], 0, 0}   // eth2
#if defined (CONFIG_CNS3XXX_SPPE)
	,{CPU_PORT_PMAP, 0, 1, FP_NETDEV_INDEX, NULL, fp_port_base,
		open_fp, close_fp, CPU_PORT, my_vlan3_mac, &cpu_vlan_table_entry,
		0, 0, "fp"}
#endif 
					 };

#endif // CNS3XXX_PORT_BASE_MODE

#ifdef CNS3XXX_PORT_BASE_MODE

#define PORT0_PVID 0x1
#define PORT1_PVID 0x2
#define PORT2_PVID 3
#define CPU_PVID 5

#define CONFIG_CNS3XXX_PORT_BASE

VLANTableEntry cpu_vlan_table_entry = {0, 1, CPU_PVID, 0, 0, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan3_mac}; 

VLANTableEntry vlan_table_entry[] = 
{
	// vlan_index; valid; vid; wan_side; etag_pmap; mb_pmap; *my_mac;
	//{0, 1, 1, 0, 0, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan0_mac},
	{1, 1, PORT0_PVID, 0, 0, MAC_PORT0_PMAP | CPU_PORT_PMAP, my_vlan0_mac},
	{2, 1, PORT1_PVID, 0, 0, MAC_PORT1_PMAP | CPU_PORT_PMAP, my_vlan1_mac},
	{3, 1, PORT2_PVID, 1, 0, MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan2_mac},
	//{2, 1, 4, 0, 0, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan3_mac}, // for cpu
};

ARLTableEntry arl_table_entry[] = 
{
	// vid; pmap; *mac; age_field; vlan_mac ; filter
	{PORT0_PVID, CPU_PORT_PMAP, my_vlan0_mac, 7, 1, 0},
	//{CPU_PVID, CPU_PORT_PMAP, my_vlan0_mac, 7, 1, 0},
	{PORT1_PVID, CPU_PORT_PMAP, my_vlan1_mac, 7, 1, 0},
	{PORT2_PVID, CPU_PORT_PMAP, my_vlan2_mac, 7, 1, 0},
	//{PORT0_PVID, MAC_PORT0_PMAP, my_vlan8_mac, 7, 0, 0},
	//{PORT0_PVID, MAC_PORT0_PMAP, my_vlan9_mac, 7, 0, 0},
	//{CPU_PVID, 0x4, my_vlan2_mac, 7, 1, 0},
	//{CPU_PVID, MAC_PORT2_PMAP, my_vlan2_mac, 7, 1, 0},
};

NetDevicePriv net_device_prive[]= {
	/*  pmap, is_wan, s-tag, vlan_tag or pvid, rx_func_ptr, tx_func_ptr, open_ptr, close_ptr, which port, mac, VLANTableEntry, ARLTableEntry, NICSetting, netdev s-tag, name */
	{MAC_PORT0_PMAP, 0, 1, PORT0_NETDEV_INDEX, rx_port_base, tx_port_base, open_port0, close_port0, MAC_PORT0, my_vlan0_mac, &vlan_table_entry[0], &arl_table_entry[0], 0, 0},   // eth0 
	{MAC_PORT1_PMAP, 0, 2, PORT1_NETDEV_INDEX, rx_port_base, tx_port_base, open_port1, close_port1, MAC_PORT1, my_vlan1_mac, &vlan_table_entry[1], &arl_table_entry[1], 0, 0},   // eth1 
	{MAC_PORT2_PMAP, 1, 3, PORT2_NETDEV_INDEX, rx_port_base, tx_port_base, open_port2, close_port2, MAC_PORT2, my_vlan2_mac,  &vlan_table_entry[2], &arl_table_entry[2], 0, 0}   // eth2 
					 };

#endif // CNS3XXX_PORT_BASE_MODE

#ifdef CNS3XXX_NIC_MODE_8021Q
//#error  "8021Q"
#define PORT0_PVID 1
#define PORT1_PVID 2
#define PORT2_PVID 9
#define CPU_PVID 5

#define CONFIG_CNS3XXX_PORT_BASE
//#define CONFIG_CNS3XXX_VLAN_BASE
//#define CONFIG_HAVE_VLAN_TAG

VLANTableEntry cpu_vlan_table_entry = {0, 1, CPU_PVID, 0, 0, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan3_mac}; // for cpu

VLANTableEntry vlan_table_entry[] = 
{
	// vlan_index; valid; vid; wan_side; etag_pmap; mb_pmap; *my_mac;C_PORT2_PMAP
	{1, 1, PORT0_PVID, 1, MAC_PORT0_PMAP|CPU_PORT_PMAP, MAC_PORT0_PMAP | CPU_PORT_PMAP, my_vlan0_mac},
	{2, 1, PORT1_PVID, 0, MAC_PORT1_PMAP|CPU_PORT_PMAP, MAC_PORT1_PMAP | CPU_PORT_PMAP, my_vlan1_mac},
	{3, 1, PORT2_PVID, 1, MAC_PORT2_PMAP|CPU_PORT_PMAP, MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan2_mac},
};

ARLTableEntry arl_table_entry[] = 
{
	// vid; pmap; *mac; age_field; vlan_mac ; filter
	{PORT0_PVID, CPU_PORT_PMAP, my_vlan0_mac, 7, 1, 0},
	{PORT1_PVID, CPU_PORT_PMAP, my_vlan1_mac, 7, 1, 0},
	{PORT2_PVID, CPU_PORT_PMAP, my_vlan2_mac, 7, 1, 0},
};


// if used 8021Q, use PORT0_NETDEV_INDEX, don't use VID
NetDevicePriv net_device_prive[]= {
	{MAC_PORT0_PMAP, 0, 1, PORT0_NETDEV_INDEX, rx_port_base, tx_port_base, open_port0, close_port0, MAC_PORT0, my_vlan0_mac, &vlan_table_entry[0], &arl_table_entry[0], 0, 0},   // eth0 
	{MAC_PORT1_PMAP, 0, 0, PORT1_NETDEV_INDEX, rx_port_base, tx_port_base, open_port1, close_port1, MAC_PORT1, my_vlan1_mac, &vlan_table_entry[1], &arl_table_entry[1], 0, 0},   // eth1 
	{MAC_PORT2_PMAP, 1, 3, PORT2_NETDEV_INDEX, rx_port_base, tx_port_base, open_port2, close_port2, MAC_PORT2, my_vlan2_mac,  &vlan_table_entry[2], &arl_table_entry[2], 0, 0}   // eth2 
					  };
#endif // CNS3XXX_NIC_MODE_8021Q

#ifdef CNS3XXX_VLAN_BASE_MODE
//#error  "vlan_base"
// vlan configuration

#define PORT0_PVID 1
#define PORT1_PVID 2
#define PORT2_PVID 3
#define CPU_PVID 5
#define CONFIG_CNS3XXX_VLAN_BASE
#define CONFIG_HAVE_VLAN_TAG

VLANTableEntry cpu_vlan_table_entry = {0, 1, CPU_PVID, 0, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, MAC_PORT0_PMAP | MAC_PORT1_PMAP | MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan3_mac}; // for cpu

VLANTableEntry vlan_table_entry[] = 
{
	// vlan_index; valid; vid; wan_side; etag_pmap; mb_pmap; *my_mac;
	{1, 1, PORT0_PVID, 0, MAC_PORT0_PMAP | CPU_PORT_PMAP, MAC_PORT0_PMAP | CPU_PORT_PMAP, my_vlan0_mac},
	{2, 1, PORT1_PVID, 0, MAC_PORT1_PMAP | CPU_PORT_PMAP, MAC_PORT1_PMAP | CPU_PORT_PMAP, my_vlan1_mac},
	{3, 1, PORT2_PVID, 1, MAC_PORT2_PMAP | CPU_PORT_PMAP, MAC_PORT2_PMAP | CPU_PORT_PMAP, my_vlan2_mac},
};

ARLTableEntry arl_table_entry[] = 
{
	// vid; pmap; *mac; age_field; vlan_mac ; filter
	{PORT0_PVID, CPU_PORT_PMAP, my_vlan0_mac, 7, 1, 0},
	{PORT1_PVID, CPU_PORT_PMAP, my_vlan1_mac, 7, 1, 0},
	{PORT2_PVID, CPU_PORT_PMAP, my_vlan2_mac, 7, 1, 0},
};

NetDevicePriv net_device_prive[]= {
	/*  pmap, is_wan, gid, vlan_tag or pvid, rx_func_ptr, tx_func_ptr, open_ptr, close_ptr, which port, mac, VLANTableEntry, ARLTableEntry, NICSetting, netdev name */
	{MAC_PORT0_PMAP, 0, 1, PORT0_PVID, rx_port_base, tx_vlan_base, open_port0, close_port0, MAC_PORT0, my_vlan0_mac, &vlan_table_entry[0], &arl_table_entry[0], 0, 0},   // eth0 
	{MAC_PORT1_PMAP, 0, 0, PORT1_PVID, rx_port_base, tx_vlan_base, open_port1, close_port1, MAC_PORT1, my_vlan1_mac, &vlan_table_entry[1], &arl_table_entry[1], 0, 0},   // eth1 
	{MAC_PORT2_PMAP, 1, 3, PORT2_PVID, rx_port_base, tx_vlan_base, open_port2, close_port2, MAC_PORT2, my_vlan2_mac,  &vlan_table_entry[2], &arl_table_entry[2], 0, 0}   // eth2 
					 };
#endif // CNS3XXX_VLAN_BASE_MODE



int is_config_cns3xxx_port_base(void)
{
#ifdef CONFIG_CNS3XXX_PORT_BASE
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(is_config_cns3xxx_port_base);

int is_config_cns3xxx_vlan_base(void)
{
#ifdef CONFIG_CNS3XXX_VLAN_BASE
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(is_config_cns3xxx_vlan_base);

int is_config_have_vlan_tag(void)
{
#ifdef CONFIG_HAVE_VLAN_TAG
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(is_config_have_vlan_tag);


int is_cns3xxx_nic_mode_8021q(void)
{
#ifdef CNS3XXX_NIC_MODE_8021Q
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(is_cns3xxx_nic_mode_8021q);

int is_cns3xxx_non_nic_mode_8021q(void)
{
#ifdef CNS3XXX_NON_NIC_MODE_8021Q
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(is_cns3xxx_non_nic_mode_8021q);

int is_cns3xxx_vlan_base_mode(void)
{
#ifdef CNS3XXX_VLAN_BASE_MODE
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(is_cns3xxx_vlan_base_mode);

int is_cns3xxx_port_base_mode(void)
{
#ifdef CNS3XXX_PORT_BASE_MODE
	return 1;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(is_cns3xxx_port_base_mode);

int num_net_dev_priv = sizeof(net_device_prive)/sizeof(NetDevicePriv);
EXPORT_SYMBOL_GPL(num_net_dev_priv);

EXPORT_SYMBOL_GPL(cpu_vlan_table_entry);
EXPORT_SYMBOL_GPL(vlan_table_entry);
EXPORT_SYMBOL_GPL(arl_table_entry);
EXPORT_SYMBOL_GPL(net_device_prive);

#endif // CONFIG_RTL8366RB
#endif // RTL8366RB_H

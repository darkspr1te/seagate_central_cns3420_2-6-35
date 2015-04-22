/*******************************************************************************
 *
 *   Copyright (c) 2008 Cavium Networks 
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
 *   Contact Information:
 *   Technology Support <tech@starsemi.com>
 *   Star Semiconductor 4F, No.1, Chin-Shan 8th St, Hsin-Chu,300 Taiwan, R.O.C
 *
 ********************************************************************************/

#include <linux/cns3xxx/switch_api.h>

#ifndef __PSE_INIT_COMMON_H
#define __PSE_INIT_COMMON_H

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define CNS3XXX_VLAN_8021Q
#endif

#ifdef CNS3XXX_VLAN_8021Q
//#define CNS3XXX_NIC_MODE_8021Q // use NIC mode to support 8021Q
#endif

#define CNS3XXX_8021Q_HW_TX

extern int is_cns3xxx_nic_mode_8021q(void);
extern int is_cns3xxx_non_nic_mode_8021q(void);
extern int is_cns3xxx_vlan_base_mode(void);
extern int is_cns3xxx_port_base_mode(void);
extern int is_config_cns3xxx_port_base(void);
extern int is_config_cns3xxx_vlan_base(void);
extern int is_config_have_vlan_tag(void);
extern void cns3xxx_gsw_up_init(void);
extern void cns3xxx_gsw_sop_init(void);
extern ushort jumbo_frame;
extern VLANTableEntry cpu_vlan_table_entry;
extern VLANTableEntry vlan_table_entry[];
extern ARLTableEntry arl_table_entry[];
//extern NetDevicePriv net_device_prive[10];
extern int num_net_dev_priv;

#endif //__PSE_INIT_COMMON_H

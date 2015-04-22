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

#include <linux/module.h>
#include <linux/cns3xxx/switch_api.h>

#include "../cns3xxx_pse_init/cns3xxx.h"
#include "cns3xxx_tool.h"
#include "cns3xxx_config.h"

static int set_fc_rls(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	FC_GLOB_THRS_REG &= (~(0x1ff << 16));
	FC_GLOB_THRS_REG |= (ctl.val << 16);

	return CAVM_OK;
}

static int get_fc_rls(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	ctl.val = ((FC_GLOB_THRS_REG >> 16) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_fc_set(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	FC_GLOB_THRS_REG &= (~0x1ff);
	FC_GLOB_THRS_REG |= ctl.val;

	return CAVM_OK;
}

static int get_fc_set(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	ctl.val = ((FC_GLOB_THRS_REG) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}


static int set_sarl_rls(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	SARL_CTRL_REG &= (~(0x1ff << 12));
	SARL_CTRL_REG |= (ctl.val << 12);

	return CAVM_OK;
}

static int get_sarl_rls(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	ctl.val = ((SARL_CTRL_REG >> 12) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_sarl_enable(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	SARL_CTRL_REG &= (~(0x1 << 31));
	SARL_CTRL_REG |= (ctl.val << 31);

	return CAVM_OK;
}

static int get_sarl_enable(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	ctl.val = ((SARL_CTRL_REG >> 31 ) & 0x1);
	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_sarl_set(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	SARL_CTRL_REG &= (~0x1ff);
	SARL_CTRL_REG |= ctl.val;

	return CAVM_OK;
}

static int get_sarl_set(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	ctl.val = ((SARL_CTRL_REG) & 0x1ff);

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_sarl_oq(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	switch (ctl.gyr) {
	case 0: // green
		SARL_OQ_GTH_REG &= (~(0xff << ctl.tc*8));
		SARL_OQ_GTH_REG |= (ctl.val << ctl.tc*8);
		break;
	case 1: // yellow
		SARL_OQ_YTH_REG &= (~(0xff << ctl.tc*8));
		SARL_OQ_YTH_REG |= (ctl.val << ctl.tc*8);
		break;
	case 2: // red
		SARL_OQ_RTH_REG &= (~(0xff << ctl.tc*8));
		SARL_OQ_RTH_REG |= (ctl.val << ctl.tc*8);
		break;
	default: 
		break;
	}
	return CAVM_OK;
}

static int get_sarl_oq(struct ifreq *ifr)
{
	CNS3XXXSARLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	switch (ctl.gyr) {
	case 0: // green
		ctl.val = ((SARL_OQ_GTH_REG >> ctl.tc*8) & 0xff);
		break;
	case 1: // yellow
		ctl.val = ((SARL_OQ_YTH_REG >> ctl.tc*8) & 0xff);
		break;
	case 2: // red
		ctl.val = ((SARL_OQ_RTH_REG >> ctl.tc*8) & 0xff);
		break;
	default:
		break;
	}

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXSARLEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

#define QUEUE_WEIGHT_SET(port, ctl) \
{ \
    MAC##port##_PRI_CTRL_REG &= ~(0x3ffff); \
    MAC##port##_PRI_CTRL_REG |= (ctl.sch_mode << 16); \
    MAC##port##_PRI_CTRL_REG |= (ctl.q0_w); \
    MAC##port##_PRI_CTRL_REG |= (ctl.q1_w << 4); \
    MAC##port##_PRI_CTRL_REG |= (ctl.q2_w << 8); \
    MAC##port##_PRI_CTRL_REG |= (ctl.q3_w << 12); \
}

#define QUEUE_WEIGHT_GET(port, ctl) \
{ \
    ctl.sch_mode = ((MAC##port##_PRI_CTRL_REG >> 16 ) & 0x3); \
    ctl.q0_w = ((MAC##port##_PRI_CTRL_REG >> 0 ) & 0x7); \
    ctl.q1_w = ((MAC##port##_PRI_CTRL_REG >> 4 ) & 0x7); \
    ctl.q2_w = ((MAC##port##_PRI_CTRL_REG >> 8 ) & 0x7); \
    ctl.q3_w = ((MAC##port##_PRI_CTRL_REG >> 12 ) & 0x7); \
}


static int set_queue_weight(struct ifreq *ifr)
{
	CNS3XXXQueueWeightEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXQueueWeightEntry)) )  
		return -EFAULT;
 
	switch (ctl.which_port) {
	case 0:
		QUEUE_WEIGHT_SET(0, ctl)
		return 0;
	case 1:
		QUEUE_WEIGHT_SET(1, ctl)
		return 0;
	case 2:
		QUEUE_WEIGHT_SET(2, ctl)
		return 0;
	case 3: // cpu port 
		CPU_PRI_CTRL_REG &= ~(0x3ffff);
		CPU_PRI_CTRL_REG |= (ctl.sch_mode << 16);
		CPU_PRI_CTRL_REG |= (ctl.q0_w);
		CPU_PRI_CTRL_REG |= (ctl.q1_w << 4);
		CPU_PRI_CTRL_REG |= (ctl.q2_w << 8);
		CPU_PRI_CTRL_REG |= (ctl.q3_w << 12);
		return 0;
	case 4: // PPE port 
		HNAT_PRI_CTRL_REG &= ~(0x3ffff);
		HNAT_PRI_CTRL_REG |= (ctl.sch_mode << 16);
		HNAT_PRI_CTRL_REG |= (ctl.q0_w);
		HNAT_PRI_CTRL_REG |= (ctl.q1_w << 4);
		HNAT_PRI_CTRL_REG |= (ctl.q2_w << 8);
		HNAT_PRI_CTRL_REG |= (ctl.q3_w << 12);
		return 0;
	default:
		return -EFAULT;
	}
}

static int get_queue_weight(struct ifreq *ifr)
{
	CNS3XXXQueueWeightEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXQueueWeightEntry)) )  
		return -EFAULT; 

	switch (ctl.which_port) {
	case 0:
		QUEUE_WEIGHT_GET(0, ctl)
		break;
	case 1:
		QUEUE_WEIGHT_GET(1, ctl)
		break;
	case 2:
		QUEUE_WEIGHT_GET(2, ctl)
		break;
	case 3:
		ctl.sch_mode = ((CPU_PRI_CTRL_REG >> 16 ) & 0x3); 
		ctl.q0_w = ((CPU_PRI_CTRL_REG >> 0 ) & 0x7); 
		ctl.q1_w = ((CPU_PRI_CTRL_REG >> 4 ) & 0x7);
		ctl.q2_w = ((CPU_PRI_CTRL_REG >> 8 ) & 0x7); 
		ctl.q3_w = ((CPU_PRI_CTRL_REG >> 12 ) & 0x7); 
		break;
	case 4:
		ctl.sch_mode = ((HNAT_PRI_CTRL_REG >> 16 ) & 0x3); 
		ctl.q0_w = ((HNAT_PRI_CTRL_REG >> 0 ) & 0x7); 
		ctl.q1_w = ((HNAT_PRI_CTRL_REG >> 4 ) & 0x7);
		ctl.q2_w = ((HNAT_PRI_CTRL_REG >> 8 ) & 0x7); 
		ctl.q3_w = ((HNAT_PRI_CTRL_REG >> 12 ) & 0x7); 
		break;
	default:
		return -EFAULT; 
	}

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXQueueWeightEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_rate_limit(struct ifreq *ifr)
{
	CNS3XXXRateLimitEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXRateLimitEntry)) )  
		return -EFAULT;
 
	switch (ctl.which_port) {
	case 0:
		RATE_CTRL_REG &= (~(0x7f << 8));
		RATE_CTRL_REG |= ( ctl.band_width << 8);
		RATE_CTRL_REG &= (~(0x3));
		RATE_CTRL_REG |= ctl.base_rate;
		return 0;
	case 1:
		RATE_CTRL_REG &= (~(0x7f << 16));
		RATE_CTRL_REG |= ( ctl.band_width << 16);
		RATE_CTRL_REG &= (~(0x3 << 2));
		RATE_CTRL_REG |= (ctl.base_rate << 2);
		return 0;
	case 2:
		RATE_CTRL_REG &= (~(0x7f << 24));
		RATE_CTRL_REG |= ( ctl.band_width << 24);
		RATE_CTRL_REG &= (~(0x3 << 4));
		RATE_CTRL_REG |= (ctl.base_rate << 4);
		return 0;
	case 3: // port 0 extra dma
		TC_CTRL_REG &= (~0x7f);
		TC_CTRL_REG |= ctl.band_width;
		RATE_CTRL_REG &= (~(0x3 << 6));
		RATE_CTRL_REG |= (ctl.base_rate << 6);
		return 0;
	default:
		return -EFAULT;
	}
}

static int get_rate_limit(struct ifreq *ifr)
{
	CNS3XXXRateLimitEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXRateLimitEntry)) )  
		return -EFAULT; 

	switch (ctl.which_port) {
	case 0:
		ctl.band_width = (RATE_CTRL_REG >> 8) & 0x7f;
		ctl.base_rate = RATE_CTRL_REG & 0x3;
		break;
	case 1:
		ctl.band_width = (RATE_CTRL_REG >> 16) & 0x7f;
		ctl.base_rate = (RATE_CTRL_REG >> 2) & 0x3;
		break;
	case 2:
		ctl.band_width = (RATE_CTRL_REG >> 24) & 0x7f;
		ctl.base_rate = (RATE_CTRL_REG >> 4) & 0x3;
		break;
	case 3: // port 0 extra dma
		ctl.band_width = (TC_CTRL_REG) & 0x7f;
		ctl.base_rate = (RATE_CTRL_REG >> 6) & 0x3;
		break;
	default:
		return -EFAULT;
	}

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXRateLimitEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_fc(struct ifreq *ifr)
{
	CNS3XXXFCEntry ctl;
	u32 port_offset[]={0x0c, 0x10, 0x18, 0x14}; // 0x14 is cpu port offset
	u32 val=0;


	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXFCEntry)) )  
		return -EFAULT; 

	val = SWITCH_REG_VALUE(port_offset[ctl.port]);
	if (ctl.port == 3) { // cpu port, only can set rx fc
		val &= (~(1 << 31));
		if (ctl.fc_en) 
			val |= (1 << 31);
	} else {
		val &= (~(1 << 11)); // disable rx fc
		val &= (~(1 << 12)); // disable tx fc
		val |= (ctl.fc_en << 11);
	}

	SWITCH_REG_VALUE(port_offset[ctl.port]) = val;
	return CAVM_OK;
}

static int get_fc(struct ifreq *ifr)
{
	CNS3XXXFCEntry ctl;
	u32 port_offset[]={0x0c, 0x10, 0x18, 0x14}; // 0x14 is cpu port offset
	u32 val=0;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXFCEntry)) )  
		return -EFAULT; 

	val = SWITCH_REG_VALUE(port_offset[ctl.port]);
	printk("port_offset[%d]: %x\n", ctl.port, port_offset[ctl.port]);
	printk("val: %x\n", val);

	if (ctl.port == 3) { // cpu port, only can set rx fc
		ctl.fc_en = ((val >> 31) & 1);
	} else {
		ctl.fc_en = ((val >> 11) & 3);
	}

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXFCEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_ivl(struct ifreq *ifr)
{
	CNS3XXXIVLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXIVLEntry)) )  
		return -EFAULT; 

	cns3xxx_ivl(ctl.enable);

	return CAVM_OK;
}

static int get_ivl(struct ifreq *ifr)
{
	CNS3XXXIVLEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXIVLEntry)) )  
		return -EFAULT; 

	ctl.enable = ((MAC_GLOB_CFG_REG >> 7) & 0x1);

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXIVLEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_wan_port(struct ifreq *ifr)
{
	CNS3XXXWANPortEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXWANPortEntry)) )  
		return -EFAULT; 
	VLAN_CFG &= (~(0x1f << 8));
	VLAN_CFG |= (ctl.wan_port << 8);

	return CAVM_OK;
}

static int get_wan_port(struct ifreq *ifr)
{
	CNS3XXXWANPortEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXWANPortEntry)) )  
		return -EFAULT; 

	ctl.wan_port = ((VLAN_CFG >> 8) & 0x1f);

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXWANPortEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_pvid(struct ifreq *ifr)
{
	CNS3XXXPVIDEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXPVIDEntry)) )  
		return -EFAULT; 
	cns3xxx_set_pvid(ctl.which_port, ctl.pvid);

	return CAVM_OK;
}

static int get_pvid(struct ifreq *ifr)
{
	CNS3XXXPVIDEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXPVIDEntry)) )  
		return -EFAULT; 

	ctl.pvid = cns3xxx_get_pvid(ctl.which_port);
	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXPVIDEntry)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_qa(struct ifreq *ifr)
{
	CNS3XXXQAEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXQAEntry)) )  
		return -EFAULT; 

	MAC_GLOB_CFG_EXT_REG &= ~(0x7 << 27);
	MAC_GLOB_CFG_EXT_REG |= (ctl.qa << 27);
	//printk("[kernel mode] ctl.qa: %d\n", ctl.qa);
	
	return CAVM_OK;
}

static int get_qa(struct ifreq *ifr)
{
	CNS3XXXQAEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXQAEntry)) )  
		return -EFAULT; 

	ctl.qa = (MAC_GLOB_CFG_EXT_REG >> 27) & 0x7;

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXQAEntry)) )  
		return -EFAULT; 
	return CAVM_OK;
}

static int get_packet_max_len(struct ifreq *ifr)
{
	CNS3XXXMaxLenEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXMaxLenEntry)) )  
		return -EFAULT; 

	ctl.max_len = (PHY_AUTO_ADDR_REG >> 30) & 0x3;

	if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXMaxLenEntry)) )  
		return -EFAULT; 
	return CAVM_OK;
}

static int set_packet_max_len(struct ifreq *ifr)
{
	CNS3XXXMaxLenEntry ctl;

	if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXMaxLenEntry)) )  
		return -EFAULT; 

	PHY_AUTO_ADDR_REG &= (~(3 << 30));
	PHY_AUTO_ADDR_REG |= (ctl.max_len << 30);

	return CAVM_OK;
}

static int set_udp_range(struct ifreq *ifr)
{
	CNS3XXXUdpRangeEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXUdpRangeEtypeControl)) )  
		return -EFAULT; 

	switch (conf.udp_range_num) {
	case 0:
		UDP_RANGE0_REG = 0;
		UDP_RANGE0_REG |= conf.port_start;
		UDP_RANGE0_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 16));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 16);
		break;
	case 1:
		UDP_RANGE1_REG = 0;
		UDP_RANGE1_REG |= conf.port_start;
		UDP_RANGE1_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 20));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 20);
		break;
	case 2:
		UDP_RANGE2_REG = 0;
		UDP_RANGE2_REG |= conf.port_start;
		UDP_RANGE2_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 24));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 24);
		break;
	case 3:
		UDP_RANGE3_REG = 0;
		UDP_RANGE3_REG |= conf.port_start;
		UDP_RANGE3_REG |= (conf.port_end << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 28));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 28);
		break;
	}

	return CAVM_OK;
}

static int get_udp_range(struct ifreq *ifr)
{
	CNS3XXXUdpRangeEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXUdpRangeEtypeControl)) )  
		return -EFAULT; 

	switch (conf.udp_range_num) {
	case 0:
		conf.port_start = (UDP_RANGE0_REG & 0xffff);
		conf.port_end = ((UDP_RANGE0_REG >> 16 )& 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 16) & 0x7);
		break;
	case 1:
		conf.port_start = (UDP_RANGE1_REG & 0xffff);
		conf.port_end = ((UDP_RANGE1_REG >> 16 )& 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 20) & 0x7);
		break;
	case 2:
		conf.port_start = (UDP_RANGE2_REG & 0xffff);
		conf.port_end = ((UDP_RANGE2_REG >> 16 )& 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 24) & 0x7);
		break;
	case 3:
		conf.port_start = (UDP_RANGE3_REG & 0xffff);
		conf.port_end = ((UDP_RANGE3_REG >> 16 )& 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 28) & 0x7);
		break;
	}

	if (copy_to_user(ifr->ifr_data, &conf, sizeof(CNS3XXXUdpRangeEtypeControl)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int get_etype(struct ifreq *ifr)
{
	CNS3XXXEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXEtypeControl)) )  
		return -EFAULT; 

	switch (conf.etype_num) {
	case 0:
		conf.val = (ETYPE1_ETYPE0_REG & 0xffff);
		conf.pri = (PRIO_ETYPE_UDP_REG & 0x7);
		break;
	case 1:
		conf.val = ((ETYPE1_ETYPE0_REG >> 16 )& 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 4) & 0x7);
		break;
	case 2:
		conf.val = (ETYPE3_ETYPE2_REG & 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 8) & 0x7);
		break;
	case 3:
		conf.val = ((ETYPE3_ETYPE2_REG >> 16 )& 0xffff);
		conf.pri = ((PRIO_ETYPE_UDP_REG >> 12) & 0x7);
		break;
	}

	if (copy_to_user(ifr->ifr_data, &conf, sizeof(CNS3XXXEtypeControl)) )  
		return -EFAULT; 

	return CAVM_OK;
}

static int set_etype(struct ifreq *ifr)
{
	CNS3XXXEtypeControl conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXEtypeControl)) )  
		return -EFAULT; 

	switch (conf.etype_num) {
	case 0:
		ETYPE1_ETYPE0_REG &= (~0xffff);
		ETYPE1_ETYPE0_REG |= conf.val;

		PRIO_ETYPE_UDP_REG &= (~7);
		PRIO_ETYPE_UDP_REG |= (conf.pri);
		break;
	case 1:
		ETYPE1_ETYPE0_REG &= (~(0xffff << 16));
		ETYPE1_ETYPE0_REG |= (conf.val << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 4));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 4);
		break;
	case 2:
		ETYPE3_ETYPE2_REG &= (~0xffff);
		ETYPE3_ETYPE2_REG |= conf.val;

		PRIO_ETYPE_UDP_REG &= (~(7 << 8));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 8);
		break;
	case 3:
		ETYPE3_ETYPE2_REG &= (~(0xffff << 16));
		ETYPE3_ETYPE2_REG |= (conf.val << 16);

		PRIO_ETYPE_UDP_REG &= (~(7 << 12));
		PRIO_ETYPE_UDP_REG |= (conf.pri << 12);
		break;
	}
	return CAVM_OK;
}

static int get_pri_ip_dscp(struct ifreq *ifr)
{
	CNS3XXXPriIpDscpControl conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXPriIpDscpControl)) )  
		return -EFAULT; 

	if ( 0 <= conf.ip_dscp_num && conf.ip_dscp_num <= 7) {
		conf.pri = ((PRIO_IPDSCP_7_0_REG >> (conf.ip_dscp_num * 4)) & 0x7);
	} else if ( 8 <= conf.ip_dscp_num && conf.ip_dscp_num <= 15) {
			conf.pri = ((PRIO_IPDSCP_15_8_REG >> ((conf.ip_dscp_num-8) * 4)) & 0x7);
		 } else if ( 16 <= conf.ip_dscp_num && conf.ip_dscp_num <= 23) {
				conf.pri = ((PRIO_IPDSCP_23_16_REG >> ((conf.ip_dscp_num-16) * 4)) & 0x7);
		 	} else if ( 24 <= conf.ip_dscp_num && conf.ip_dscp_num <= 31) {
					conf.pri = ((PRIO_IPDSCP_31_24_REG >> ((conf.ip_dscp_num-24) * 4)) & 0x7);
		 		} else if ( 32 <= conf.ip_dscp_num && conf.ip_dscp_num <= 39) {
						conf.pri = ((PRIO_IPDSCP_39_32_REG >> ((conf.ip_dscp_num-32) * 4)) & 0x7);
		 			} else if ( 40 <= conf.ip_dscp_num && conf.ip_dscp_num <= 47) {
							conf.pri = ((PRIO_IPDSCP_47_40_REG >> ((conf.ip_dscp_num-40) * 4)) & 0x7);
		 				} else if ( 48 <= conf.ip_dscp_num && conf.ip_dscp_num <= 55) {
								conf.pri = ((PRIO_IPDSCP_55_48_REG >> ((conf.ip_dscp_num-48) * 4)) & 0x7);
		 					} else if ( 56 <= conf.ip_dscp_num && conf.ip_dscp_num <= 63) {
									conf.pri = ((PRIO_IPDSCP_63_56_REG >> ((conf.ip_dscp_num-56) * 4)) & 0x7);
								} else {
									return CAVM_ERR;
									}


	if (copy_to_user(ifr->ifr_data, &conf, sizeof(CNS3XXXPriIpDscpControl)) )  
		return -EFAULT; 

	return CAVM_OK;
}


static int set_pri_ip_dscp(struct ifreq *ifr)
{
	CNS3XXXPriIpDscpControl conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXPriIpDscpControl)) )  
		return -EFAULT; 

	if ( 0 <= conf.ip_dscp_num && conf.ip_dscp_num <= 7) {
		PRIO_IPDSCP_7_0_REG &=  (~(0x7 << (conf.ip_dscp_num * 4) ) );
		PRIO_IPDSCP_7_0_REG |= (conf.pri << (conf.ip_dscp_num * 4));
	} else if ( 8 <= conf.ip_dscp_num && conf.ip_dscp_num <= 15) {
			PRIO_IPDSCP_15_8_REG &=  (~(0x7 << ((conf.ip_dscp_num-8) * 4) ) );
			PRIO_IPDSCP_15_8_REG |= (conf.pri << ((conf.ip_dscp_num-8) * 4));
		 } else if ( 16 <= conf.ip_dscp_num && conf.ip_dscp_num <= 23) {
				PRIO_IPDSCP_23_16_REG &=  (~(0x7 << ((conf.ip_dscp_num-16) * 4) ) );
				PRIO_IPDSCP_23_16_REG |= (conf.pri << ((conf.ip_dscp_num-16) * 4));

		 	} else if ( 24 <= conf.ip_dscp_num && conf.ip_dscp_num <= 31) {
					PRIO_IPDSCP_31_24_REG &=  (~(0x7 << ((conf.ip_dscp_num-24) * 4) ) );
					PRIO_IPDSCP_31_24_REG |= (conf.pri << ((conf.ip_dscp_num-24) * 4));

		 		} else if ( 32 <= conf.ip_dscp_num && conf.ip_dscp_num <= 39) {
						PRIO_IPDSCP_39_32_REG &=  (~(0x7 << ((conf.ip_dscp_num-32) * 4) ) );
						PRIO_IPDSCP_39_32_REG |= (conf.pri << ((conf.ip_dscp_num-32) * 4));

		 			} else if ( 40 <= conf.ip_dscp_num && conf.ip_dscp_num <= 47) {
							PRIO_IPDSCP_47_40_REG &=  (~(0x7 << ((conf.ip_dscp_num-40) * 4) ) );
							PRIO_IPDSCP_47_40_REG |= (conf.pri << ((conf.ip_dscp_num-40) * 4));
		 				} else if ( 48 <= conf.ip_dscp_num && conf.ip_dscp_num <= 55) {
								PRIO_IPDSCP_55_48_REG &=  (~(0x7 << ((conf.ip_dscp_num-48) * 4) ) );
								PRIO_IPDSCP_55_48_REG |= (conf.pri << ((conf.ip_dscp_num-48) * 4));
		 					} else if ( 56 <= conf.ip_dscp_num && conf.ip_dscp_num <= 63) {
									PRIO_IPDSCP_63_56_REG &=  (~(0x7 << ((conf.ip_dscp_num-56) * 4) ) );
									PRIO_IPDSCP_63_56_REG |= (conf.pri << ((conf.ip_dscp_num-56) * 4));
								} else {
									return CAVM_ERR;
									}
	return CAVM_OK;
}


#if defined(CONFIG_VB)

extern int bcm53115M_reg_read(int page, int offset, u8 *buf, int len);

static int bcm53115M_reg_read_ioctl(struct ifreq *ifr)
{
	CNS3XXXBCM53115M conf;

	int __init_or_module gpio_direction_output(unsigned int pin, unsigned int state);

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXBCM53115M)) )  
		return -EFAULT;
 
	printk("conf.page: %x\n", conf.page);
	printk("conf.offset: %x\n", conf.offset);
	printk("conf.data_len: %x\n", conf.data_len);

	switch (conf.data_len) {
	case 1:
		bcm53115M_reg_read(conf.page, conf.offset, (u8 *)&conf.u8_val, 1);
		printk("conf.u8_val: %x\n", conf.u8_val);
		break;
	case 2:
		bcm53115M_reg_read(conf.page, conf.offset, (u8 *)&conf.u16_val, 2);
		printk("conf.u16_val: %x\n", conf.u16_val);
		break;
	case 4:
		bcm53115M_reg_read(conf.page, conf.offset, (u8 *)&conf.u32_val, 4);
		printk("conf.u32_val: %x\n", conf.u32_val);
		break;
	default:
		printk("[kernel mode]: don't support date length: %d\n", conf.data_len);
		return -EFAULT; 
	}

	if (copy_to_user(ifr->ifr_data, &conf, sizeof(CNS3XXXBCM53115M)) )  
		return -EFAULT;
 
	return CAVM_OK;
}

extern int bcm53115M_reg_write(int page, int offset, u8 *buf, int len);

static int bcm53115M_reg_write_ioctl(struct ifreq *ifr)
{
	CNS3XXXBCM53115M conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXBCM53115M)) )  
		return -EFAULT; 

	switch (conf.data_len) {
	case 1:
		bcm53115M_reg_write(conf.page, conf.offset, (u8 *)&conf.u8_val, 1);
		break;
	case 2:
		bcm53115M_reg_write(conf.page, conf.offset, (u8 *)&conf.u16_val, 2);
		break;
	case 4:
		bcm53115M_reg_write(conf.page, conf.offset, (u8 *)&conf.u32_val, 4);
		break;
	default:
		printk("[kernel mode]: don't support date length: %d\n", conf.data_len);
	}
	return CAVM_OK;
}
#endif

#if 0
int get_rxring(struct ifreq *ifr)
{
	CNS3XXXRingStatus conf;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXRingStatus)) )  
		return -EFAULT; 
	conf.rx_ring=g_ring_info.rx_ring;
	conf.tx_ring=0;
	if (copy_to_user(ifr->ifr_data, &conf, sizeof(CNS3XXXRingStatus)) )  
		return -EFAULT; 
}
#endif

static int dump_mib_counter(struct ifreq *ifr)
{
	CNS3XXXMIBCounter conf;
	int addr=0,i=0;

	if (copy_from_user(&conf, ifr->ifr_data, sizeof(CNS3XXXMIBCounter)) )  
		return -EFAULT; 

	for (addr=0x300; addr <= 0x334 ; addr+=4)
		conf.mib[i++]=SWITCH_REG_VALUE(addr);
	for (addr=0x400; addr <= 0x434 ; addr+=4)
		conf.mib[i++]=SWITCH_REG_VALUE(addr);
	for (addr=0x600; addr <= 0x634 ; addr+=4)
		conf.mib[i++]=SWITCH_REG_VALUE(addr);
	// cpu mib counter
	for (addr=0x500; addr <= 0x528 ; addr+=4)
		conf.mib[i++]=SWITCH_REG_VALUE(addr);
	conf.mib_len=i;
	if (copy_to_user(ifr->ifr_data, &conf, sizeof(CNS3XXXMIBCounter)) )  
		return -EFAULT; 
	return 0;
}

// reference e100.c
int cns3xxx_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	CNS3XXXIoctlCmd ioctl_cmd;

	//printk("cns3xxx_do_ioctl begin\n");

	if (cmd != SIOCDEVPRIVATE) {
		return -EOPNOTSUPP;
	}

	if (copy_from_user(&ioctl_cmd, ifr->ifr_data, sizeof(CNS3XXXIoctlCmd)))
		return -EFAULT;

	//printk("ioctl_cmd: %d\n", ioctl_cmd);
	switch (ioctl_cmd) {
		case CNS3XXX_ARP_REQUEST_SET:
		{
			CNS3XXXArpRequestControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXArpRequestControl)) ) 
				return -EFAULT;

			(ctl.val==0) ? (MAC_GLOB_CFG_REG &= (~(1 << 23)) ): (MAC_GLOB_CFG_REG |= (1 << 23) );

		}

		case CNS3XXX_ARP_REQUEST_GET:
		{
			CNS3XXXArpRequestControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXArpRequestControl)) ) 
				return -EFAULT;

			ctl.val = ((MAC_GLOB_CFG_REG >> 23) & 1);

			if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXArpRequestControl)) )  
				return -EFAULT; 
			return CAVM_OK;
		}
	
		case CNS3XXX_HOL_PREVENT_SET:
		{
			CNS3XXXHOLPreventControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXHOLPreventControl)) ) 
				return -EFAULT;
			(ctl.enable == 1) ? (TC_CTRL_REG |= (1 << 29)) : (TC_CTRL_REG &= (~(1 << 29))) ;

			return CAVM_OK;
		}
		case CNS3XXX_HOL_PREVENT_GET:
		{
			CNS3XXXHOLPreventControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXHOLPreventControl)) ) 
				return -EFAULT;

			ctl.enable = ((TC_CTRL_REG >> 29) & 0x1);

			if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXHOLPreventControl)) )  
				return -EFAULT; 
			return CAVM_OK;
		}

		// for S component or C conponent
		case CNS3XXX_BRIDGE_SET:
		{
			CNS3XXXBridgeControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXBridgeControl)) ) 
				return -EFAULT;
			(ctl.type == 1) ? (VLAN_CFG |= (1 << 1)) : (VLAN_CFG &= (~(1 << 1))) ;


		}
		case CNS3XXX_BRIDGE_GET:
		{
			CNS3XXXBridgeControl ctl;

			ctl.type = ((VLAN_CFG >> 1) & 0x1);
			printk("[kernel mode] ctl.type: %d\n", ctl.type);

			if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXBridgeControl)) )  
				return -EFAULT; 

			return CAVM_OK;
		}

		case CNS3XXX_PORT_NEIGHBOR_SET:
		{
			CNS3XXXPortNeighborControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXPortNeighborControl)) ) 
				return -EFAULT;
			switch (ctl.which_port)
			{
				case 0:
				{
					(ctl.type == 1) ? (VLAN_CFG |= (1 << 4)) : (VLAN_CFG &= (~(1 << 4))) ;
					return 0;
				}
				case 1:
				{
					(ctl.type == 1) ? (VLAN_CFG |= (1 << 5)) : (VLAN_CFG &= (~(1 << 5))) ;
					return 0;
				}
				case 2:
				{
					(ctl.type == 1) ? (VLAN_CFG |= (1 << 7)) : (VLAN_CFG &= (~(1 << 7))) ;
					return 0;
				}
				case 3: // cpu port
				{
					(ctl.type == 1) ? (VLAN_CFG |= (1 << 6)) : (VLAN_CFG &= (~(1 << 6))) ;
					return 0;
				}
				default:
					return -EFAULT;
			}

		}

		case CNS3XXX_PORT_NEIGHBOR_GET:
		{
			CNS3XXXPortNeighborControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXPortNeighborControl)) ) 
				return -EFAULT;
			switch (ctl.which_port)
			{
				case 0:
				{
					ctl.type = ((VLAN_CFG >> 4 ) & 0x1);
					break;
				}
				case 1:
				{
					ctl.type = ((VLAN_CFG >> 5 ) & 0x1);
					break;
				}
				case 2:
				{
					ctl.type = ((VLAN_CFG >> 7 ) & 0x1);
					break;
				}
				case 3: // cpu port
				{
					ctl.type = ((VLAN_CFG >> 6 ) & 0x1);
					break;
				}
			}

			if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXPortNeighborControl)) )  
				return -EFAULT; 

			return CAVM_OK;
		}

		case CNS3XXX_VLAN_TABLE_LOOKUP:
		{
			CNS3XXXVLANTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXVLANTableEntry)) ) 
				return -EFAULT;
			if (cns3xxx_vlan_table_lookup(&ctl.entry) ==  CAVM_NOT_FOUND) {
				return CAVM_NOT_FOUND;
			}

		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXVLANTableEntry)))
		                return -EFAULT;

			return CAVM_FOUND;
		}
		case CNS3XXX_VLAN_TABLE_READ:
		{
			CNS3XXXVLANTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXVLANTableEntry)) ) 
			{
				return -EFAULT;
			}
			cns3xxx_vlan_table_read(&ctl.entry);
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXVLANTableEntry)))
		                return -EFAULT;

			return 0;
		}
		case CNS3XXX_VLAN_TABLE_ADD:
		{	
			CNS3XXXVLANTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXVLANTableEntry)) ) 
				return -EFAULT;
			cns3xxx_vlan_table_add(&ctl.entry);
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXVLANTableEntry)))
		                return -EFAULT;

			return 0;
		}

		case CNS3XXX_ARL_TABLE_ADD:
		{	
			CNS3XXXARLTableEntry ctl;

			printk("[kernel mode] CNS3XXX_ARL_TABLE_ADD\n");
		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;
			cns3xxx_arl_table_add(&ctl.entry);
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return 0;
		}


		case CNS3XXX_ARL_TABLE_DEL:
		{	
			CNS3XXXARLTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;
			cns3xxx_arl_table_invalid(&ctl.entry);
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return 0;
		}
		case CNS3XXX_VLAN_TABLE_DEL:
		{	
			CNS3XXXARLTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;
			cns3xxx_arl_table_invalid(&ctl.entry);

		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return CAVM_FOUND;
		}

		case CNS3XXX_ARL_TABLE_SEARCH:
		{	
			CNS3XXXARLTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;
			if (cns3xxx_arl_table_search(&ctl.entry) == CAVM_NOT_FOUND){
				printk("[kernel mode] not found\n");
				return CAVM_NOT_FOUND;
			}
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return CAVM_FOUND;
		}
		case CNS3XXX_ARL_IS_TABLE_END:
		{	
			CNS3XXXARLTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;
			if (cns3xxx_is_arl_table_end() == CAVM_ERR)
				return CAVM_ERR;
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return CAVM_OK;
		}

		case CNS3XXX_ARL_TABLE_SEARCH_AGAIN:
		{	
			CNS3XXXARLTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;
			if (cns3xxx_arl_table_search_again(&ctl.entry) == CAVM_NOT_FOUND)
				return CAVM_NOT_FOUND;
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return CAVM_FOUND;
		}

		case CNS3XXX_ARL_TABLE_FLUSH:
		{	
			CNS3XXXARLTableEntry ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;

			cns3xxx_arl_table_flush();

		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return CAVM_FOUND;
		}



		case CNS3XXX_ARL_TABLE_LOOKUP:
		{	
			CNS3XXXARLTableEntry ctl;

		
			printk("[kernel mode] in CNS3XXX_ARL_TABLE_LOOKUP\n");
		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXARLTableEntry)) ) 
				return -EFAULT;
			if (cns3xxx_arl_table_lookup(&ctl.entry) == CAVM_NOT_FOUND)
				return CAVM_NOT_FOUND;
		        if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXARLTableEntry)))
		                return -EFAULT;

			return CAVM_FOUND;
		}

		case CNS3XXX_TC_SET:
		{	
			CNS3XXXTrafficClassControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXTrafficClassControl)) ) 
				return -EFAULT;
			TC_CTRL_REG &= (~(0x3 << 30));
			TC_CTRL_REG |= (ctl.tc << 30);
			return CAVM_OK;
		}
		case CNS3XXX_TC_GET:
		{
			CNS3XXXTrafficClassControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXTrafficClassControl)) ) 
				return -EFAULT;

			ctl.tc = ((TC_CTRL_REG >> 30) & 0x3);

			if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXTrafficClassControl)) )  
				return -EFAULT; 

			return CAVM_OK;
		}

		case CNS3XXX_PRI_CTRL_SET:
		{
			CNS3XXXPriCtrlControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXPriCtrlControl)) ) 
				return -EFAULT;

			switch (ctl.which_port)
			{
				case 0:
				{
					MAC0_PRI_CTRL_REG &= (~(0x7 << 24));
					MAC0_PRI_CTRL_REG &= (~(0xf << 18));
					
					MAC0_PRI_CTRL_REG |= (ctl.port_pri << 24);

					MAC0_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
					MAC0_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
					MAC0_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
					MAC0_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
					break;
				}
				case 1:
				{
					MAC1_PRI_CTRL_REG &= (~(0x7 << 24));
					MAC1_PRI_CTRL_REG &= (~(0xf << 18));
					
					MAC1_PRI_CTRL_REG |= (ctl.port_pri << 24);

					MAC1_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
					MAC1_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
					MAC1_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
					MAC1_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
					break;
				}
				case 2:
				{
					MAC2_PRI_CTRL_REG &= (~(0x7 << 24));
					MAC2_PRI_CTRL_REG &= (~(0xf << 18));
					
					MAC2_PRI_CTRL_REG |= (ctl.port_pri << 24);

					MAC2_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
					MAC2_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
					MAC2_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
					MAC2_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
					break;
				}
				case 3: // cpu
				{
					printk("[kernel mode] CPU_PRI_CTRL_REG: %#x\n", CPU_PRI_CTRL_REG);
					CPU_PRI_CTRL_REG &= (~(0x7 << 24));
					CPU_PRI_CTRL_REG &= (~(0xf << 18));
					
					CPU_PRI_CTRL_REG |= (ctl.port_pri << 24);

					CPU_PRI_CTRL_REG |= (ctl.ether_pri_en << 18);
					CPU_PRI_CTRL_REG |= (ctl.vlan_pri_en << 19);
					CPU_PRI_CTRL_REG |= (ctl.dscp_pri_en << 20);
					CPU_PRI_CTRL_REG |= (ctl.udp_pri_en << 21);
					break;
				}
			}

			return CAVM_OK;
		}

		case CNS3XXX_PRI_CTRL_GET:
		{
			CNS3XXXPriCtrlControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXPriCtrlControl)) ) 
				return -EFAULT;

			switch (ctl.which_port)
			{
				case 0:
				{
					ctl.port_pri = (unsigned char)((MAC0_PRI_CTRL_REG >> 24) & 0x7);
					ctl.ether_pri_en = (unsigned char)((MAC0_PRI_CTRL_REG >> 18) & 0x1);
					ctl.vlan_pri_en = (unsigned char)((MAC0_PRI_CTRL_REG >> 19) & 0x1);
					ctl.dscp_pri_en = (unsigned char)((MAC0_PRI_CTRL_REG >> 20) & 0x1);
					ctl.udp_pri_en = (unsigned char)((MAC0_PRI_CTRL_REG >> 21) & 0x1);
					break;
				}
				case 1:
				{
					ctl.port_pri = (unsigned char)((MAC1_PRI_CTRL_REG >> 24) & 0x7);
					ctl.ether_pri_en = (unsigned char)((MAC1_PRI_CTRL_REG >> 18) & 0x1);
					ctl.vlan_pri_en = (unsigned char)((MAC1_PRI_CTRL_REG >> 19) & 0x1);
					ctl.dscp_pri_en = (unsigned char)((MAC1_PRI_CTRL_REG >> 20) & 0x1);
					ctl.udp_pri_en = (unsigned char)((MAC1_PRI_CTRL_REG >> 21) & 0x1);
					break;
				}
				case 2:
				{
					ctl.port_pri = (unsigned char)((MAC2_PRI_CTRL_REG >> 24) & 0x7);
					ctl.ether_pri_en = (unsigned char)((MAC2_PRI_CTRL_REG >> 18) & 0x1);
					ctl.vlan_pri_en = (unsigned char)((MAC2_PRI_CTRL_REG >> 19) & 0x1);
					ctl.dscp_pri_en = (unsigned char)((MAC2_PRI_CTRL_REG >> 20) & 0x1);
					ctl.udp_pri_en = (unsigned char)((MAC2_PRI_CTRL_REG >> 21) & 0x1);
					break;
				}
				case 3:
				{
					ctl.port_pri = (unsigned char)((CPU_PRI_CTRL_REG >> 24) & 0x7);
					ctl.ether_pri_en = (unsigned char)((CPU_PRI_CTRL_REG >> 18) & 0x1);
					ctl.vlan_pri_en = (unsigned char)((CPU_PRI_CTRL_REG >> 19) & 0x1);
					ctl.dscp_pri_en = (unsigned char)((CPU_PRI_CTRL_REG >> 20) & 0x1);
					ctl.udp_pri_en = (unsigned char)((CPU_PRI_CTRL_REG >> 21) & 0x1);
					break;
				}
			}

			if (copy_to_user(ifr->ifr_data, &ctl, sizeof(CNS3XXXPriCtrlControl)) )  
				return -EFAULT; 

			return CAVM_OK;
		}

		case CNS3XXX_DMA_RING_CTRL_SET:
		{
			CNS3XXXDmaRingCtrlControl ctl;

		        if (copy_from_user(&ctl, ifr->ifr_data, sizeof(CNS3XXXDmaRingCtrlControl)) ) 
				return -EFAULT;

			(ctl.ts_double_ring_en == 0) ? DMA_RING_CTRL_REG &= (~(0x1 << 16)) : (DMA_RING_CTRL_REG |= (ctl.ts_double_ring_en << 16));
			(ctl.fs_double_ring_en == 0) ? DMA_RING_CTRL_REG &= (~(0x1 << 0)) : (DMA_RING_CTRL_REG |= (ctl.fs_double_ring_en << 0));
			(ctl.fs_pkt_allocate == 0) ? DMA_RING_CTRL_REG &= (~(0x1 << 1)) : (DMA_RING_CTRL_REG |= (ctl.fs_pkt_allocate << 1));
		}

		case CNS3XXX_PRI_IP_DSCP_SET:
		{
			return set_pri_ip_dscp(ifr);
		}
		case CNS3XXX_PRI_IP_DSCP_GET:
		{
			return get_pri_ip_dscp(ifr);
		}

		case CNS3XXX_ETYPE_SET:
		{
			return set_etype(ifr);
		}
		case CNS3XXX_ETYPE_GET:
		{
			return get_etype(ifr);
		}

		case CNS3XXX_UDP_RANGE_SET:
		{
			return set_udp_range(ifr);
		}
		case CNS3XXX_UDP_RANGE_GET:
		{
			return get_udp_range(ifr);
		}

		case CNS3XXX_RATE_LIMIT_SET:
		{
			return set_rate_limit(ifr);
		}
		case CNS3XXX_RATE_LIMIT_GET:
		{
			return get_rate_limit(ifr);
		}
		case CNS3XXX_QUEUE_WEIGHT_SET:
		{
			return set_queue_weight(ifr);
		}
		case CNS3XXX_QUEUE_WEIGHT_GET:
		{
			return get_queue_weight(ifr);
		}

		case CNS3XXX_FC_RLS_SET:
		{
			return set_fc_rls(ifr);
		}
		case CNS3XXX_FC_RLS_GET:
		{
			return get_fc_rls(ifr);
		}

		case CNS3XXX_FC_SET_SET:
		{
			return set_fc_set(ifr);
		}
		case CNS3XXX_FC_SET_GET:
		{
			return get_fc_set(ifr);
		}

		case CNS3XXX_SARL_RLS_SET:
		{
			return set_sarl_rls(ifr);
		}
		case CNS3XXX_SARL_RLS_GET:
		{
			return get_sarl_rls(ifr);
		}

		case CNS3XXX_SARL_SET_SET:
		{
			return set_sarl_set(ifr);
		}
		case CNS3XXX_SARL_SET_GET:
		{
			return get_sarl_set(ifr);
		}

		case CNS3XXX_SARL_OQ_SET:
		{
			return set_sarl_oq(ifr);
		}
		case CNS3XXX_SARL_OQ_GET:
		{
			return get_sarl_oq(ifr);
		}

		case CNS3XXX_SARL_ENABLE_SET:
		{
			return set_sarl_enable(ifr);
		}
		case CNS3XXX_SARL_ENABLE_GET:
		{
			return get_sarl_enable(ifr);
		}

		case CNS3XXX_FC_SET:
		{
			return set_fc(ifr);
		}
		case CNS3XXX_FC_GET:
		{
			return get_fc(ifr);
		}

		case CNS3XXX_IVL_SET:
		{
			return set_ivl(ifr);
		}
		case CNS3XXX_IVL_GET:
		{
			return get_ivl(ifr);
		}

		case CNS3XXX_WAN_PORT_SET:
		{
			return set_wan_port(ifr);
		}
		case CNS3XXX_WAN_PORT_GET:
		{
			return get_wan_port(ifr);
		}

		case CNS3XXX_PVID_SET:
		{
			return set_pvid(ifr);
		}
		case CNS3XXX_PVID_GET:
		{
			return get_pvid(ifr);
		}

		case CNS3XXX_QA_GET:
		{
			return get_qa(ifr);
		}
		case CNS3XXX_QA_SET:
		{
			return set_qa(ifr);
		}

		case CNS3XXX_PACKET_MAX_LEN_GET:
		{
			return get_packet_max_len(ifr);
		}
		case CNS3XXX_PACKET_MAX_LEN_SET:
		{
			return set_packet_max_len(ifr);
		}

#if defined(CONFIG_VB)
		case CNS3XXX_BCM53115M_REG_READ:
		{
			return bcm53115M_reg_read_ioctl(ifr);
		}
		case CNS3XXX_BCM53115M_REG_WRITE:
		{
			return bcm53115M_reg_write_ioctl(ifr);
		}

#endif

#if 0
		case CNS3XXX_RXRING_STATUS:
		{
			return get_rxring(ifr);
		}
#endif
		case CNS3XXX_DUMP_MIB_COUNTER:
		{
			return dump_mib_counter(ifr);
		}
	

		default:
		{
			printk("[kernel mode] don't match any command\n");
			break;
		}

	} // end switch (ioctl_cmd) 
	return 0;
}

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

#include <linux/init.h>
#include <linux/module.h>
//#include <mach/board.h>
#include <linux/gpio.h>

#include "cns3xxx.h"
#include "cns3xxx_tool.h"
#include "cns3xxx_init_config.h"
#include <linux/cns3xxx/pse_init_common.h>

#ifdef CONFIG_FPGA
#include "fpga.h"
#endif

#ifdef CONFIG_VB
#include "vb.h"
#endif

#ifdef CONFIG_VB_2
#include "vb2.h"
#endif

#ifdef CONFIG_RTL8211
#include "rtl8211.h"
#endif

#ifdef CONFIG_RTL8211_X2
#include "rtl8211_X2.h"
#endif

#ifdef CONFIG_RTL8211_MAC0_PHYADDR1
#include "rtl8211_mac0_phyaddr1.h"
#endif

#ifdef CONFIG_AR8021
#include "ar8021.h"
#endif

#ifdef CONFIG_AR8316
#include "ar8316.h"
#endif

#ifdef CONFIG_AR8327
#include "ar8327.h"
#endif

#ifdef CONFIG_RTL8366RB
#include "rtl8366rb.h"
#endif

#ifdef CONFIG_RTL8367
#include "rtl8367.h"
#endif

#ifdef CONFIG_RTL8367_ONE_LEG
#include "rtl8367_one_leg.h"
#endif

#ifdef CONFIG_IP1001_X2
#include "ip1001_X2.h"
#endif

#ifdef CONFIG_VITESSE7395
#include "vitess7395.h"
#endif

#ifdef CNS3XXX_DELAYED_INTERRUPT
u32 max_pend_int_cnt=MAX_PEND_INT_CNT, max_pend_time=MAX_PEND_TIME;
#endif
int init_port=7; // bit map 7 means port 0, 1 and 2, default is 7.

//module_param(init_port, u8, S_IRUGO);
module_param(init_port, int, 0);

void cns3xxx_gsw_sop_init(void);
void cns3xxx_gsw_up_init(void);

extern int bcm53115M_reg_write(int page, int offset, u8 *buf, int len);
extern int bcm53115M_reg_read(int page, int offset, u8 *buf, int len);
void bcm_disable_learning(void);

#define GPIO_POWER_OFF
#ifdef GPIO_POWER_OFF
extern int (*cns3xxx_gpio_poweroff)(void);
extern void (*disable_rtl_phy_action)(void);
extern void (*enable_rtl_phy_action)(void);

void __disable_rtl_phy_action(void)
{
	u16 phy_data=0;
	u16 phy_addr=0;

	phy_addr = 0x1;
	if (cns3xxx_read_phy(phy_addr, 0, &phy_data) != CAVM_OK)
		return;
	phy_data |= (0x1 << 10);
	if (cns3xxx_write_phy(phy_addr, 0, phy_data) != CAVM_OK)
		return;

	phy_addr = 0x2;
	if (cns3xxx_read_phy(phy_addr, 0, &phy_data) != CAVM_OK)
		return;
	phy_data |= (0x1 << 10);
	if (cns3xxx_write_phy(phy_addr, 0, phy_data) != CAVM_OK)
		return;
}

void __enable_rtl_phy_action(void)
{
	u16 phy_data=0;
	u16 phy_addr=0;

	phy_addr = 0x1;
	if (cns3xxx_read_phy(phy_addr, 0, &phy_data) != CAVM_OK)
		return;
	phy_data &= ~(0x1 << 10);
	if (cns3xxx_write_phy(phy_addr, 0, phy_data) != CAVM_OK)
		return;

	phy_addr = 0x2;
	if (cns3xxx_read_phy(phy_addr, 0, &phy_data) != CAVM_OK)
		return;
	phy_data &= ~(0x1 << 10);
	if (cns3xxx_write_phy(phy_addr, 0, phy_data) != CAVM_OK)
		return;
}

static unsigned char* cns3xxx_get_mac_addr_by_port(u8 port)
{
	struct net_device *dev, *ret;
	u16 vid;
	
	ret = NULL;
	rcu_read_lock();
	for_each_netdev_rcu(&init_net, dev)
	{
		CNS3XXXPrivate *priv = netdev_priv(dev);

		if (priv && priv->net_device_priv && priv->net_device_priv->arl_table_entry)
		{
			vid = priv->net_device_priv->arl_table_entry->vid;
			if((port == 0) && (vid == 1)) {
				ret = dev;
				break;
			}
			else if((port == 1) && (vid == 2)) {
				ret = dev;
				break;
			}
		}
	}
	rcu_read_unlock();
	
	return ((ret) ? ret->dev_addr : NULL);
}

static void rtl8211e_wol_enable(u8 port, u8 phy_addr)
{
	unsigned char* mac_addr = 0;
	unsigned char my_mac_addr[8];
		
	cns3xxx_write_phy(phy_addr, 0x1F, 0x7);
	
	// reset wol
	cns3xxx_write_phy(phy_addr, 0x1E, 0x6d);
	cns3xxx_write_phy(phy_addr, 0x16, 0x9fff);

	// set mac
	if ( (mac_addr = cns3xxx_get_mac_addr_by_port(port)) )
	{
		u16 value = 0;

		printk("%s\n", __func__);
		printk("port(phy_addr):%d(%02x)\n", port, phy_addr);
		printk("mac: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
		cns3xxx_write_phy(phy_addr, 0x1E, 0x6e);
		value = mac_addr[0] | (mac_addr[1] << 8);
		cns3xxx_write_phy(phy_addr, 0x15, value);
		value = mac_addr[2] | (mac_addr[3] << 8);
		cns3xxx_write_phy(phy_addr, 0x16, value);
		value = mac_addr[4] | (mac_addr[5] << 8);
		cns3xxx_write_phy(phy_addr, 0x17, value);

		cns3xxx_read_phy(phy_addr, 0x15, &value);
		my_mac_addr[0] = value & 0xFF;
		my_mac_addr[1] = (value & 0xFF00) >> 8;
		cns3xxx_read_phy(phy_addr, 0x16, &value);
		my_mac_addr[2] = value & 0xFF;
		my_mac_addr[3] = (value & 0xFF00) >> 8;
		cns3xxx_read_phy(phy_addr, 0x17, &value);
		my_mac_addr[4] = value & 0xFF;
		my_mac_addr[5] = (value & 0xFF00) >> 8;
		printk("read back mac: %02X:%02X:%02X:%02X:%02X:%02X\n", my_mac_addr[0], my_mac_addr[1], my_mac_addr[2], my_mac_addr[3], my_mac_addr[4], my_mac_addr[5]);
	}

	// set max packet length
	cns3xxx_write_phy(phy_addr, 0x1E, 0x6d);
	cns3xxx_write_phy(phy_addr, 0x16, 0x1fff);

	// set to pulse low
	cns3xxx_write_phy(phy_addr, 0x1E, 0x6d);
	cns3xxx_write_phy(phy_addr, 0x18, 0x7);

	// enable wol events
	cns3xxx_write_phy(phy_addr, 0x1E, 0x6d);
	cns3xxx_write_phy(phy_addr, 0x15, 0x1000);

	// disable GMII/RGMII pad for power saving
	cns3xxx_write_phy(phy_addr, 0x19, 0x1);

	// back to page 0
	cns3xxx_write_phy(phy_addr, 0x1F, 0x0);
}

static void rtl8211e_setup_wrap(u8 port, u8 phy_addr)
{
	u16 read_data;

	printk(KERN_INFO "phy_addr = %08x\n", phy_addr);

	/* Set isolated, disable AN, set link speed to 10Mb. */
	cns3xxx_read_phy(phy_addr, 0x0, &read_data);
	printk(KERN_INFO "BMCR = %08x\n", read_data);
	read_data &= ~(1 << 12);	/* disable AN */
	read_data &= ~(1 << 13);	/* speed[0] = 0 */
	read_data &= ~(1 << 6);		/* speed[1] = 0 */
	read_data &= ~(1 << 8);		/* half duplex */
	cns3xxx_write_phy(phy_addr, 0x0, read_data);
	read_data = 0x0;
	cns3xxx_read_phy(phy_addr, 0x0, &read_data);
	printk(KERN_INFO "BMCR = %08x\n", read_data);
	read_data = 0x0;

	// enter WOL mode
	rtl8211e_wol_enable(port, phy_addr);
}

int cns3xxx_gpio_power_off(void)
{
	/* Disable cns3xxx MAC0 port clk. */
	printk(KERN_INFO "MAC0_CFG_REG = %08x\n", MAC0_CFG_REG);
	MAC0_CFG_REG |= (0x1 << 18);
	printk(KERN_INFO "MAC0_CFG_REG = %08x\n", MAC0_CFG_REG);
	mdelay(1);
	printk(KERN_INFO "MAC1_CFG_REG = %08x\n", MAC1_CFG_REG);
	MAC1_CFG_REG |= (0x1 << 18);
	printk(KERN_INFO "MAC1_CFG_REG = %08x\n", MAC1_CFG_REG);
	mdelay(1);
	printk(KERN_INFO "PHY_AUTO_ADDR_REG = %08x\n", PHY_AUTO_ADDR_REG);
	PHY_AUTO_ADDR_REG &= ~(0x1 << 7 | 0x1 << 15 | 0x1 << 23 | 0x1 << 5 | 0x1 << 13 | 0x1 << 21);
	printk(KERN_INFO "PHY_AUTO_ADDR_REG = %08x\n", PHY_AUTO_ADDR_REG);

	/* Power on Phy for wol */
#ifdef CONFIG_CNS3XXX_PSE_WOL
	PORT0_LINK_UP
	PORT1_LINK_UP
#endif

	rtl8211e_setup_wrap(0, 0x1); /* phy_addr = 1 */
	
// if required to wrap phy2 define CONFIG_RTL8211_MAC1_PHYADDR2 in kconfig file
#ifdef CONFIG_RTL8211_MAC1_PHYADDR2
	rtl8211e_setup_wrap(1, 0x2); /* phy_addr = 2 */
#endif

	/* Use gpio A pin 7 to turn off the power. */
	if (gpio_request(7, "cns3xxx_gpio")==0) {
		gpio_direction_output(7, 1);
		gpio_free(7);
	}

	return 0;
}
#endif

#ifdef CONFIG_VITESSE7395
int vitesse7395_reg_read_ioctl(struct ifreq *ifr)
{

	return CAVM_OK;
}

int vitesse7395_reg_write_ioctl(struct ifreq *ifr)
{

	return CAVM_OK;
}

#endif

void take_off_vlan_header(struct sk_buff *skb)
{
	// take off VLAN header
	memmove(skb->data + 4, skb->data, 12);
	skb->data += 4;
	skb->len -= 4; // minus 4 byte vlan tag
}

int rx_port_base(struct sk_buff *skb, RXDesc *rx_desc_ptr, const struct CNS3XXXPrivate_ *priv)
{
	if (skb->data[12] == 0x81 && skb->data[13] == 0x00)  // VLAN header
	{
		printk("take_off_vlan_header\n");
		take_off_vlan_header(skb);
//		print_packet(skb->data, skb->len);
	}
	return 0;
}

int rx_vlan_base(struct sk_buff *skb, RXDesc *rx_desc_ptr, const struct CNS3XXXPrivate_ *priv)
{
	return 0;
}

int tx_port_base(TXDesc *tx_desc_ptr, const struct CNS3XXXPrivate_ *priv, struct sk_buff *skb)
{
#if defined(CNS3XXX_VLAN_8021Q) && defined (CNS3XXX_8021Q_HW_TX)
	if (skb && priv->vlgrp != NULL && vlan_tx_tag_present(skb)) 
	{
		tx_desc_ptr->c_vid = cpu_to_le16(vlan_tx_tag_get(skb));
		tx_desc_ptr->ctv=1;
#if 1 //KH: 20100201 add
		tx_desc_ptr->fr = 1;
		tx_desc_ptr->pmap = priv->net_device_priv->pmap;
#else
		tx_desc_ptr->fr = 0;
#endif
	}
	else 
#endif
	{
		tx_desc_ptr->ctv = 0;
		tx_desc_ptr->pmap = priv->net_device_priv->pmap;
		tx_desc_ptr->fr	= 1;
	}

	return 0;
}

int tx_vlan_base(TXDesc *tx_desc_ptr, const struct CNS3XXXPrivate_ *priv, struct sk_buff *skb)
{
#if defined(CNS3XXX_VLAN_8021Q)

        if (skb && priv->vlgrp != NULL && vlan_tx_tag_present(skb)) {
		tx_desc_ptr->c_vid = cpu_to_le16(vlan_tx_tag_get(skb));
        }
#else
	tx_desc_ptr->c_vid = priv->net_device_priv->vlan_tag;

#endif
	tx_desc_ptr->ctv=1;
	tx_desc_ptr->fr	= 0;

	return 0;
}

#if defined (CONFIG_CNS3XXX_SPPE)
int fp_port_base(TXDesc *tx_desc_ptr, const struct CNS3XXXPrivate_ *priv, struct sk_buff *skb)
{
#if 1
	tx_desc_ptr->fr = 1;
	tx_desc_ptr->pmap = 0x8;
#else
	tx_desc_ptr->fr = 0;
	tx_desc_ptr->ctv = 1;
	tx_desc_ptr->c_vid = 80;
#endif
    return 0;
}
#endif

// port: 0, 1, 2 ; port0, port1 and port2
// config general mac port configuration
void cns3xxx_general_mac_cfg(u8 port)
{
	u32 cfg=0;

	switch (port)
	{
		case 0:
		{
			cfg = MAC0_CFG_REG;
			break;
		}
		case 1:
		{
			cfg = MAC1_CFG_REG;
			break;
		}
		case 2:
		{
			cfg = MAC2_CFG_REG;
			break;
		}
	}


	// txc_check_en: 1 
	cfg |= (1 << 13);

	// bp_en: 1
	cfg |= (1 << 17);

#ifdef CNS3XXX_LEARN_ENABLE
	// learn_dis: 0
	cfg &= (~(1 << 19));
#else
	// learn disable
	cfg |= (1 << 19);
#endif

	// blocking_state: 0
	cfg &= (~(1 << 20));

	// block_mode: 0
	cfg &= (~(1 << 21));

#ifdef CNS3XXX_AGE_ENABLE
	// age_en: 1
	cfg |= (1 << 22);

#else
	// age disable
	cfg &= (~(1 << 22));
#endif

	// SA_secured: 0
	cfg &= (~(1 << 23));

	switch (port)
	{
		case 0:
		{
			MAC0_CFG_REG = cfg;
			break;
		}
		case 1:
		{
			MAC1_CFG_REG = cfg;
			break;
		}
		case 2:
		{
			MAC2_CFG_REG = cfg;
			break;
		}
	}

}

void cns3xxx_configu_cpu_port(void)
{
	// Set CPU port to general configuration
	
#ifdef CNS3XXX_LEARN_ENABLE
		CPU_CFG_REG &= (~(1 << 19));
#else
	// learn_dis: 1
		CPU_CFG_REG |= (1 << 19);
#endif

#ifdef CNS3XXX_AGE_ENABLE
	// age_en: 1
		CPU_CFG_REG |= (1 << 22);
#else
	// age disable
		CPU_CFG_REG &= (~(1 << 22));
#endif

	// SA_secured: 0
	CPU_CFG_REG &= (~(1 << 23));

	// go to hnat:1
	CPU_CFG_REG |= (1 << 29);

	//offset 4N +2 
	CPU_CFG_REG &= (~(1 << 30));
#ifdef CNS3XXX_4N
	CPU_CFG_REG |= (1 << 30);
#endif

	// cpu flow control disable
	CPU_CFG_REG &= (~(1 << 31));
#ifdef CNS3XXX_CPU_PORT_FC
	// cpu flow control enable
	CPU_CFG_REG |= (1 << 31);
#endif
	
}

void cns3xxx_gsw_hw_init(void)
{
	//u32 mac_port_config;
	int i;
	//u32 cfg_reg = 0;
	//u32 reg_config = 0;

#ifdef CONFIG_SILICON

	GPIOB_PIN_EN_REG |= (1 << 14); //enable GMII2_CRS
	GPIOB_PIN_EN_REG |= (1 << 15); //enable GMII2_COL
	GPIOB_PIN_EN_REG |= (1 << 20); //enable MDC
	GPIOB_PIN_EN_REG |= (1 << 21); //enable MDIO

#if 0
	// for MII reference
	GPIOB_PIN_EN_REG |= (1 << 16); //enable RGMII1_CRS
	GPIOB_PIN_EN_REG |= (1 << 17); //enable RGMII1_COL

	GPIOB_PIN_EN_REG |= (1 << 18); //enable RGMII0_CRS
	GPIOB_PIN_EN_REG |= (1 << 19); //enable RGMII0_COL
#endif
	
	cns3xxx_gsw_power_enable();
	cns3xxx_gsw_software_reset();
#endif

        // RGMII0 high speed drive strength
        IOCDA_REG &= (~(3 << 2));
        IOCDA_REG |= (2 << 2);

        // RGMII1 high speed drive strength
        IOCDA_REG &= (~(3 << 6));
        IOCDA_REG |= (2 << 6);

#if 1
        // GMII2 high speed drive strength
        IOCDA_REG &= (~(3 << 10));
        IOCDA_REG |= (2 << 10);
#endif

	// RGMII0 no pull
	IOPUD_REG &= (~(3 << 0));

	// RGMII1 no pull
	IOPUD_REG &= (~(3 << 2));

	// GMII2 no pull
	IOPUD_REG &= (~(3 << 4));

        // disable all ports auto polling
#if defined(CONFIG_AR8327) || defined(CONFIG_RTL8211_MAC0_PHYADDR1) || defined(CONFIG_RTL8211_X2)
#else
        cns3xxx_phy_auto_polling_enable(0, 0);
        cns3xxx_phy_auto_polling_enable(1, 0);
        cns3xxx_phy_auto_polling_enable(2, 0);
#endif

#ifdef CONFIG_CNS3XXX_PSE_SW_PHY_AUTO_POLLING
	extern void cns3xxx_sw_auto_polling_init(void);
	cns3xxx_sw_auto_polling_init();
#endif

#ifdef CNS3XXX_CONFIG_SIM_MODE 
        printk("FPGA turns on GSW_SIM_MODE_BIT\n");
        SLK_SKEW_CTRL_REG |= (1 << 31);
#endif


#if 1
	//printk("do cns3xxx gsw memory test\n");
	while (((SRAM_TEST_REG >> 20) & 1) == 0);
	//printk("cns3xxx gsw memory test is complete\n");

#endif

#ifdef CONFIG_FPGA
	clear_fs_dma_state(1);
	
	// disable port mac0, mac1, mac2, cpu port
	enable_port(0, 0);
	enable_port(1, 0);
	enable_port(2, 0);
	enable_port(3, 0);

	// disable RX0/TX0 RX1/TX1 DMA
	enable_tx_dma(0, 0);
	enable_tx_dma(1, 0);
	enable_rx_dma(0, 0);
	enable_rx_dma(1, 0);
#endif

	INTR_STAT_REG = 0xffffffff; // write 1 for clear.

#ifdef CNS3XXX_DELAYED_INTERRUPT
	DELAY_INTR_CFG_REG = (1 << 16) | (max_pend_int_cnt << 8) | (max_pend_time);
#endif

//        reg_config = PHY_AUTO_ADDR_REG;
//        reg_config &= ~(3 << 30);
//#ifdef CONFIG_CNS3XXX_JUMBO_FRAME
//        reg_config |= (3 << 30); // maximum frame length: 9600 bytes
//#else
//        reg_config |= (2 << 30); // maximum frame length: 1536 bytes
//#endif
//
//        PHY_AUTO_ADDR_REG = reg_config;


	// Set general value for MAC_GLOB_CFG_REG
	// age_time: 2 ^(1-1) * 300 sec 
	MAC_GLOB_CFG_REG &= (~0xf);
	MAC_GLOB_CFG_REG |= 1;


	// bkoff_mode: 111 follow standard
	MAC_GLOB_CFG_REG &= (~(0x7 << 9));
	MAC_GLOB_CFG_REG |= (0x7 << 9);

	// jam_no: 1010: 
	MAC_GLOB_CFG_REG &= (~(0xf << 12));
	MAC_GLOB_CFG_REG |= (0xa << 12);

	// bp_mode: 10: 
	MAC_GLOB_CFG_REG &= (~(0x3 << 16));
	MAC_GLOB_CFG_REG |= (0x2 << 16);

	// res_mc_flt: 0
	MAC_GLOB_CFG_REG &= (~(0x1 << 28));

	// col_mode: 11
	MAC_GLOB_CFG_REG &= (~(0x3 << 18));
	MAC_GLOB_CFG_REG |= (0x3 << 18);

	// crc_stripping: 1
	MAC_GLOB_CFG_REG |= (0x1 << 20);


	// ACCEPT_CRC_BAD_PKT : 0
	MAC_GLOB_CFG_REG &= (~(0x1 << 21));

#ifdef ACCEPT_CRC_BAD_PKT
	MAC_GLOB_CFG_REG |= (0x1 << 21);
#endif

	// SVL
	MAC_GLOB_CFG_REG &= (~(0x1 << 7));

#ifdef IVL
	// IVL: 1 (IVL), 0 (SVL)
	MAC_GLOB_CFG_REG |= (0x1 << 7);
#endif


	// HNAT_en: 0
	MAC_GLOB_CFG_REG &= (~(0x1 << 26));

	// Firewall_mode: 0
	MAC_GLOB_CFG_REG &= (~(0x1 << 27));



	cns3xxx_general_mac_cfg(0);
	cns3xxx_general_mac_cfg(1);
	cns3xxx_general_mac_cfg(2);
	cns3xxx_configu_cpu_port();

	// write vlan table
	// set cpu port vlan table
	cns3xxx_vlan_table_add(&cpu_vlan_table_entry);
	for (i=0 ; i < sizeof(vlan_table_entry)/sizeof(VLANTableEntry) ; ++i) 
		cns3xxx_vlan_table_add(&vlan_table_entry[i]);
	
	cns3xxx_set_pvid(0, PORT0_PVID);
	cns3xxx_set_pvid(1, PORT1_PVID);
	cns3xxx_set_pvid(2, PORT2_PVID);
	cns3xxx_set_pvid(3, CPU_PVID);

#ifdef CNS3XXX_SET_ARL_TABLE
	// set arl table
	cns3xxx_arl_table_flush();
#endif
}

int cns3xxx_gsw_config_mac_port0(void)
{
        INIT_PORT0_PHY
	INIT_PORT0_MAC
        PORT0_LINK_DOWN
        return 0;
}

int cns3xxx_gsw_config_mac_port1(void)
{
        INIT_PORT1_PHY
	INIT_PORT1_MAC
        PORT1_LINK_DOWN
        return 0;
}

int cns3xxx_gsw_config_mac_port2(void)
{
        INIT_PORT2_PHY
	INIT_PORT2_MAC
        PORT2_LINK_DOWN
        return 0;
}

void cns3xxx_gsw_hw_sop_init(void)
{
	int i;
        u32 reg_config = 0;

#ifdef CONFIG_SILICON
	GPIOB_PIN_EN_REG |= (1 << 14); //enable GMII2_CRS
	GPIOB_PIN_EN_REG |= (1 << 15); //enable GMII2_COL
	GPIOB_PIN_EN_REG |= (1 << 20); //enable MDC
	GPIOB_PIN_EN_REG |= (1 << 21); //enable MDIO
	printk("GPIOB_PIN_EN_REG: %x\n", GPIOB_PIN_EN_REG);
	
    // New PSE Init Sequence Shared by Taiwan Team 
#if 1
	printk("\nDo power enable\n");                                                                                                             
	cns3xxx_gsw_power_enable();
	
	printk("Disabling MAC Ports\n");
	MAC0_CFG_REG |= (1<<18); 
	MAC1_CFG_REG |= (1<<18); 
	MAC2_CFG_REG |= (1<<18);
	
	printk("Suspending DMA\n");
	DMA_AUTO_POLL_CFG_REG |= (1<<4);
	DMA_AUTO_POLL_CFG_REG |= (1<<0);
	
	printk("Waiting for DMA Status not IDLE");
	printk("\n %x %x\n",TS_DMA_STA_REG,FS_DMA_STA_REG);
	while(TS_DMA_STA_REG );
	//while(FS_DMA_STA_REG );
	while(1) {
	    if(FS_DMA_STA_REG==0) {
	            break;
	    }
	    else if(FS_DMA_STA_REG==2 && FS_DMA_CTRL0_REG==0) {
	        if(FS_DMA_STA_REG==2 && FS_DMA_CTRL0_REG==0) //check twice
	            break;
	    }
	};
	printk(" done\n");
	
	printk("Do power reset\n");
	cns3xxx_gsw_software_reset();
	
	//printk("Disabling Mac clks..\n");
	//PHY_AUTO_ADDR_REG &= ~(1<<23);
	//PHY_AUTO_ADDR_REG &= ~(1<<15);
	//PHY_AUTO_ADDR_REG &= ~(1<<7);
	
	//printk("Disabling DMA \n\n");
	//TS_DMA_CTRL0_REG &= ~(1<<0);
	//FS_DMA_CTRL0_REG &= ~(1<<0);
	//TS_DMA_CTRL1_REG &= ~(1<<0);
	//FS_DMA_CTRL1_REG &= ~(1<<0);
#else                                                                                                                                
    printk("Do power enable\n");                                                                                                             
    cns3xxx_gsw_power_enable();
    
    printk("Disabling MAC Ports\n");
    MAC0_CFG_REG |= (1<<18); 
    MAC1_CFG_REG |= (1<<18); 
    MAC2_CFG_REG |= (1<<18);
    
    printk("Disabling DMA_Auto poll\n");
    DMA_AUTO_POLL_CFG_REG |= (1<<4);
    DMA_AUTO_POLL_CFG_REG |= (1<<0);
    
    printk("Waiting for DMA Status clear...");
    while(TS_DMA_STA_REG );
    while(FS_DMA_STA_REG );
    printk(" done\n");
    
    printk("Do power reset\n");
	cns3xxx_gsw_software_reset();
    
    printk("Disabling Mac clks..\n");
    PHY_AUTO_ADDR_REG &= ~(1<<23);
    PHY_AUTO_ADDR_REG &= ~(1<<15);
    PHY_AUTO_ADDR_REG &= ~(1<<7);
    
    printk("Disabling DMA \n");                                                                                                                
    TS_DMA_CTRL0_REG &= ~(1<<0);
    FS_DMA_CTRL0_REG &= ~(1<<0);
    TS_DMA_CTRL1_REG &= ~(1<<0);
    FS_DMA_CTRL1_REG &= ~(1<<0);
#endif    
#endif

	printk("do cns3xxx gsw memory test\n");
	while (((SRAM_TEST_REG >> 20) & 1) == 0);
	printk("cns3xxx gsw memory test is complete\n");

        // RGMII0 high speed drive strength
        IOCDA_REG &= (~(3 << 2));
        IOCDA_REG |= (2 << 2);

        // RGMII1 high speed drive strength
        IOCDA_REG &= (~(3 << 6));
        IOCDA_REG |= (2 << 6);

        // GMII2 high speed drive strength
        IOCDA_REG &= (~(3 << 10));
        IOCDA_REG |= (2 << 10);

	// RGMII0 no pull
	IOPUD_REG &= (~(3 << 0));

	// RGMII1 no pull
	IOPUD_REG &= (~(3 << 2));

	// GMII2 no pull
	IOPUD_REG &= (~(3 << 4));

#if defined(CONFIG_RTL8211_MAC0_PHYADDR1) || defined(CONFIG_RTL8211_X2)
#else
        // disable all ports auto polling
        cns3xxx_phy_auto_polling_enable(0, 0);
        cns3xxx_phy_auto_polling_enable(1, 0);
        cns3xxx_phy_auto_polling_enable(2, 0);
#endif

#ifdef CONFIG_CNS3XXX_PSE_SW_PHY_AUTO_POLLING
	extern void cns3xxx_sw_auto_polling_init(void);
	cns3xxx_sw_auto_polling_init();
#endif

#ifdef CONFIG_SILICON
	clear_fs_dma_state(1);
#endif
	enable_port(0, 0);
	enable_port(1, 0);
	enable_port(2, 0);
	enable_port(3, 0);

	// disable RX0/TX0 RX1/TX1 DMA
	enable_tx_dma(0, 0);
	enable_tx_dma(1, 0);
	enable_rx_dma(0, 0);
	enable_rx_dma(1, 0);

	INTR_STAT_REG = 0xffffffff; // write 1 for clear.

        reg_config = PHY_AUTO_ADDR_REG;
        reg_config &= ~(3 << 30);

#ifdef CONFIG_PAGE_SIZE_64K
	reg_config |= (3 << 30); // maximum frame length: 9600 bytes
#else 
	reg_config |= (2 << 30); // maximum frame length: 1536 bytes
#endif 

        PHY_AUTO_ADDR_REG = reg_config;

	// Set general value for MAC_GLOB_CFG_REG
	// age_time: 2 ^(1-1) * 300 sec 
	MAC_GLOB_CFG_REG &= (~0xf);
	MAC_GLOB_CFG_REG |= 1;

	// bkoff_mode: 111 follow standard
	MAC_GLOB_CFG_REG &= (~(0x7 << 9));
	MAC_GLOB_CFG_REG |= (0x7 << 9);

	// jam_no: 1010: 
	MAC_GLOB_CFG_REG &= (~(0xf << 12));
	MAC_GLOB_CFG_REG |= (0xa << 12);

	// bp_mode: 10: 
	MAC_GLOB_CFG_REG &= (~(0x3 << 16));
	MAC_GLOB_CFG_REG |= (0x2 << 16);

	// res_mc_flt: 0
	MAC_GLOB_CFG_REG &= (~(0x1 << 28));

	// col_mode: 11
	MAC_GLOB_CFG_REG &= (~(0x3 << 18));
	MAC_GLOB_CFG_REG |= (0x3 << 18);

	// crc_stripping: 1
	MAC_GLOB_CFG_REG |= (0x1 << 20);


	MAC_GLOB_CFG_REG &= (~(0x1 << 21));

	// SVL
	MAC_GLOB_CFG_REG &= (~(0x1 << 7));

	// HNAT_en: 0
	MAC_GLOB_CFG_REG &= (~(0x1 << 26));

	// Firewall_mode: 0
	MAC_GLOB_CFG_REG &= (~(0x1 << 27));

	cns3xxx_general_mac_cfg(0);
	cns3xxx_general_mac_cfg(1);
	cns3xxx_general_mac_cfg(2);
	cns3xxx_configu_cpu_port();

	// write vlan table
	// set cpu port vlan table
	cns3xxx_vlan_table_add(&cpu_vlan_table_entry);
	for (i=0 ; i < sizeof(vlan_table_entry)/sizeof(VLANTableEntry) ; ++i) 
		cns3xxx_vlan_table_add(&vlan_table_entry[i]);
	
	cns3xxx_set_pvid(0, PORT0_PVID);
	cns3xxx_set_pvid(1, PORT1_PVID);
	cns3xxx_set_pvid(2, PORT2_PVID);
	cns3xxx_set_pvid(3, CPU_PVID);

#ifdef CNS3XXX_SET_ARL_TABLE
	// set arl table
	cns3xxx_arl_table_flush();
#endif
}

#if defined(CONFIG_VB)
void bcm_disable_learning(void)
{
	u8 page=0, offset=0;
	u16 u16_val=0;
	u32 u32_val=0;
	
	//disable learning
	page=0x00;
	offset=0x3c;
	u16_val=0x13f;
	bcm53115M_reg_write(page, offset, (u8 *)&u16_val, 2);
	bcm53115M_reg_read(page, offset, (u8 *)&u16_val, 2);
	
	page=0x02;
	offset=0x06;
	u32_val=4;
	bcm53115M_reg_write(page, offset, (u8 *)&u32_val, 4);
}
#endif

void cns3xxx_gsw_up_init(void)
{

	cns3xxx_gsw_hw_init();

#ifdef CONFIG_FPGA
	// GIGA mode disable
	MAC0_CFG_REG &= (~(1<<16));
	MAC1_CFG_REG &= (~(1<<16));
	MAC2_CFG_REG &= (~(1<<16));
#endif

	printk("CNS3XXX PSE: Initialize\n");
	if ((init_port & 1) == 1) {
		printk("MAC 0\n");
		cns3xxx_gsw_config_mac_port0();
	}

	if (((init_port >> 1) & 1) == 1) {
		printk("MAC 1\n");
		cns3xxx_gsw_config_mac_port1();
	}

	if (((init_port >> 2)& 1) == 1) {
		printk("MAC 2\n");
		cns3xxx_gsw_config_mac_port2();
	}
}
EXPORT_SYMBOL_GPL(cns3xxx_gsw_up_init);

void cns3xxx_gsw_sop_init(void)
{
	cns3xxx_gsw_hw_sop_init();

/*	if (init_fail) {
		printk("cns3xxx_init_module: init fail, directly exit\r\n");
		dbg_dumprxdescs();		
		return -1;
	}
*/

	printk("CNS3XXX PSE: Initialize ");
	printk("init_port: %x\n", init_port);

	if ((init_port & 1) == 1) {
		printk("MAC 0\n");
		cns3xxx_gsw_config_mac_port0();
	}

	if (((init_port >> 1) & 1) == 1) {
		printk("MAC 1\n");
		cns3xxx_gsw_config_mac_port1();
	}

	if (((init_port >> 2)& 1) == 1) {
		printk("MAC 2\n");
		cns3xxx_gsw_config_mac_port2();
	}
#if defined(CONFIG_VB)
	bcm_disable_learning();
#endif

}
EXPORT_SYMBOL_GPL(cns3xxx_gsw_sop_init);

static int __init cns3xxx_pse_init_init_module(void)
{
	cns3xxx_gpio_poweroff = cns3xxx_gpio_power_off;
	disable_rtl_phy_action = __disable_rtl_phy_action;
	enable_rtl_phy_action = __enable_rtl_phy_action;
	return 0;
}

static void __exit cns3xxx_pse_init_exit_module(void)
{
#ifdef CONFIG_CNS3XXX_PSE_SW_PHY_AUTO_POLLING
	extern void cns3xxx_sw_auto_polling_uninit(void);
	cns3xxx_sw_auto_polling_uninit();
#endif
	
	cns3xxx_gpio_poweroff = NULL;
	disable_rtl_phy_action = NULL;
	enable_rtl_phy_action = NULL;
	// disable phy auto-poll
	PHY_AUTO_ADDR_REG &= ~((1<<5) | (1<<13) | (1<<21));
	mdelay(1000); // wait state machine idle	
}

MODULE_AUTHOR("Cavium Networks, <tech@XXXX.com>");
MODULE_DESCRIPTION("CNS3XXX PSE Init");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(cns3xxx_pse_init_init_module);
module_exit(cns3xxx_pse_init_exit_module);


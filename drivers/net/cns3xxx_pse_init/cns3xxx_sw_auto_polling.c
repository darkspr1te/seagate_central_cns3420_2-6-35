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
#include <linux/timer.h>

#include "cns3xxx.h"
#include "cns3xxx_tool.h"
#include "cns3xxx_phy.h"

#define PHY_POLLING_TIME	(1*HZ)

#define MAC_SPEED_1000		2
#define MAC_SPEED_100		1
#define MAC_SPEED_10		0

#define MAC_DUPLEX_FULL		1
#define MAC_DUPLEX_HALF		0

typedef struct link_cap_t
{
	u8  autoneg;
	u8  link_st;
	u16	speed;
	u8	duplex;
	u8	fc;
} link_cap;

struct cnx3xxx_sw_phy_auto_polling
{
	struct work_struct	phy_polling_task;
	struct timer_list	phy_polling_timer;

	// private data
	u8	SW_PHY_AUTO_POLL_REG[3];	// bit[0-4]:    phy addr
									// bit[5]:      sw polling enable
									// bit[6]:      AN enable
									// bit[7]:      link status
} cns3xxx_sw_auto_polling;

// rx/tx flow control, duplex, speed
static int cns3xxx_get_phy_link_status(u16 phy_addr, link_cap *status)
{
	u16 data0 = 0, data1 = 0, data2 = 0;;
	
	cns3xxx_read_phy(phy_addr, 0x0, &data0);
	cns3xxx_read_phy(phy_addr, 0x1, &data1);
	
	status->autoneg = ((data0 >> 12) & 0x1);
	status->link_st = ((data1 >> 2) & 0x1);

	// force to 1000BASE-T by advertising 1000BASE-T capability only.
	cns3xxx_read_phy(phy_addr, 0x4, &data2);
	if ( !((data1 >> 11) & (data2 >> 5) & 0x1f) )
	{
		status->autoneg = 0;
		return CAVM_ERR;
	}
	
	// AN disable & power down
	if ( !((data0 >> 12) & 0x1) || ((data0 >> 11) & 0x1) )
		return CAVM_ERR;
	
	// Link up & AN is complete
	if ( (data1 & (0x1 << 2)) && (data1 & (0x1 << 5)) )
	{
		u16 anar = 0, anlpar = 0;	// AN Advertising Register & AN Link Partner Ability Register
		u16 gbcr = 0, gbsr = 0;		// 1000BASE-T Control Register & 1000BASE-T Status Register

		cns3xxx_read_phy(phy_addr, 0x4, &anar);
		cns3xxx_read_phy(phy_addr, 0x5, &anlpar);
		cns3xxx_read_phy(phy_addr, 0x9, &gbcr);
		cns3xxx_read_phy(phy_addr, 0xa, &gbsr);

		// rx/tx flow control
		data0 = 0;
		data0 = ((((anar >> 10 ) & 0x3) << 2) | ((anlpar >> 10 ) & 0x3));
		if ((data0 & 5) == 5)
			status->fc = 3;
		else if (data0 == 0xb)
			status->fc = 2;
		else if (data0 == 0xe)
			status->fc = 1;
		else
			status->fc = 0;

		// speed & duplex
		switch (((gbcr >> 8) & 0x3) & ((gbsr >> 10) & 0x3))
		{
			case 0x3:   // 1000BASE-T half+full-duplex
			case 0x2:   // 1000BASE-T full-duplex
				status->speed = MAC_SPEED_1000;
				status->duplex = MAC_DUPLEX_FULL;
				break;
			
			case 0x1:   // 1000BASE-T half-duplex
				status->speed = MAC_SPEED_1000;
				status->duplex = MAC_DUPLEX_HALF;
				break;

			case 0x0:
				// 100BASE-T4
				if ( ((anar >> 9) & 0x1) & ((anlpar >> 9) & 0x1) )
				{
					return CAVM_ERR;
				}
				// 100BASE-TX full-duplex
				else if ( ((anar >> 8) & 0x1) & ((anlpar >> 8) & 0x1) )
				{
					status->speed = MAC_SPEED_100;
					status->duplex = MAC_DUPLEX_FULL;
				}
				// 100BASE-TX half-duplex
				else if ( ((anar >> 7) & 0x1) & ((anlpar >> 7) & 0x1) )
				{
					status->speed = MAC_SPEED_100;
					status->duplex = MAC_DUPLEX_HALF;
				}
				// 10BASE-T full-duplex
				else if ( ((anar >> 6) & 0x1) & ((anlpar >> 6) & 0x1) )
				{
					status->speed = MAC_SPEED_10;
					status->duplex = MAC_DUPLEX_FULL;
				}
				// 10BASE-T half-duplex
				else if ( ((anar >> 5) & 0x1) & ((anlpar >> 5) & 0x1) )
				{
					status->speed = MAC_SPEED_10;
					status->duplex = MAC_DUPLEX_HALF;
				}
				break;
		} // end of switch

		return CAVM_OK;
	}
	return CAVM_ERR;
}

static void cns3xxx_phy_polling_task(struct work_struct *work)
{
	u8 *sw_auto_polling_reg = cns3xxx_sw_auto_polling.SW_PHY_AUTO_POLL_REG;
	u8	mac_cfg_reg[] = {0x0c, 0x10, 0x18};
	u16 port;
	link_cap status;
	
	for (port=0; port < 3; port++)
	{
		memset(&status, 0x0, sizeof(link_cap));
		
		// s/w auto polling enable
		if ((sw_auto_polling_reg[port] >> 5) & 0x1)
		{
			u32 mac_cfg_old = 0, mac_cfg_new = 0;
			u16 phy_addr = (sw_auto_polling_reg[port] & 0x1f);
			
			if (cns3xxx_get_phy_link_status(phy_addr, &status) == CAVM_OK)
			{
				mac_cfg_old = SWITCH_REG_VALUE(mac_cfg_reg[port]);
				mac_cfg_new = mac_cfg_old;
				
				mac_cfg_new &= (~(3 << 8));
				mac_cfg_new |= (status.speed << 8);
				mac_cfg_new &= (~(1 << 10));
				mac_cfg_new |= (status.duplex << 10);
				mac_cfg_new &= (~(3 << 11));
				mac_cfg_new |= (status.fc << 11);
				
				if (mac_cfg_new != mac_cfg_old)
				{
					u8 *speed_str[3] = { "10Mbps", "100Mbps", "1000Mbps" };
					
					SWITCH_REG_VALUE(mac_cfg_reg[port]) = mac_cfg_new;
					printk("MAC%d: S/W AN link up to %s, %s, FC-Rx=%d, FC-Tx=%d\n",
								port, speed_str[status.speed], (status.duplex == 1) ? "Full-duplex" : "Half-duplex",
								(status.fc & 0x1), ((status.fc >> 1) & 0x1));
				}
			}
			
			// update LINK_ST and AN to sw_auto_polling_reg[port]
			sw_auto_polling_reg[port] &= (~(3 << 6));
			sw_auto_polling_reg[port] |= ((status.link_st << 7) | (status.autoneg << 6));
		}
	} // end of loop

	// restart polling timer
	mod_timer(&cns3xxx_sw_auto_polling.phy_polling_timer, jiffies + PHY_POLLING_TIME);
	return;
}

static void cns3xxx_phy_polling_timer(unsigned long __opaque)
{
	// H/W auto polling must be disabled
	if (PHY_AUTO_ADDR_REG & ((0x1 << 5) | (0x1 << 13) | (0x1 << 21)))
	{
		printk("%s: ERROR !!! Be sure to disable H/W auto-polling first !!!\n", __FUNCTION__);
		return;
	}
	
	schedule_work(&cns3xxx_sw_auto_polling.phy_polling_task);
}

int cns3xxx_sw_auto_polling_conf(u8 mac_port, u16 phy_addr, u8 en)
{
	u8 *sw_auto_polling_reg = cns3xxx_sw_auto_polling.SW_PHY_AUTO_POLL_REG;
		
	if (mac_port < 0 || mac_port > 2)
	{
		printk("%s: error !!!\n", __FUNCTION__);
		return CAVM_ERR;
	}

	if (((sw_auto_polling_reg[0] >> 5) & 0x1) ||
		((sw_auto_polling_reg[1] >> 5) & 0x1) ||
		((sw_auto_polling_reg[2] >> 5) & 0x1))
		del_timer(&cns3xxx_sw_auto_polling.phy_polling_timer);
	
	sw_auto_polling_reg[mac_port] &= (~0x1f);
	sw_auto_polling_reg[mac_port] |= phy_addr;

	sw_auto_polling_reg[mac_port] &= (~(1 << 5));
	if (en)
		sw_auto_polling_reg[mac_port] |= (1 << 5);

	// start polling timer
	if (((sw_auto_polling_reg[0] >> 5) & 0x1) ||
		((sw_auto_polling_reg[1] >> 5) & 0x1) ||
		((sw_auto_polling_reg[2] >> 5) & 0x1))
		add_timer(&cns3xxx_sw_auto_polling.phy_polling_timer);
	
	return CAVM_OK;
}

u8 cns3xxx_sw_auto_polling_reg(u8 mac_port)
{
	u8 *sw_auto_polling_reg = cns3xxx_sw_auto_polling.SW_PHY_AUTO_POLL_REG;

	if (mac_port < 0 || mac_port > 2)
	{
		printk("%s: error !!!\n", __FUNCTION__);
		return 0;
	}
	return sw_auto_polling_reg[mac_port];
}

void cns3xxx_sw_auto_polling_init(void)
{
	printk("Initialize PHY s/w auto-polling\n");
	
	// init all
	memset(&cns3xxx_sw_auto_polling, 0x0, sizeof(struct cnx3xxx_sw_phy_auto_polling));

	// init workqueue task
	INIT_WORK(&cns3xxx_sw_auto_polling.phy_polling_task, cns3xxx_phy_polling_task);
	
	// init timer
	init_timer(&cns3xxx_sw_auto_polling.phy_polling_timer);
	cns3xxx_sw_auto_polling.phy_polling_timer.function   = cns3xxx_phy_polling_timer;
	cns3xxx_sw_auto_polling.phy_polling_timer.data       = 0;
	cns3xxx_sw_auto_polling.phy_polling_timer.expires    = jiffies + PHY_POLLING_TIME;
	return;
}

void cns3xxx_sw_auto_polling_uninit(void)
{
	printk("Uninitialize PHY s/w auto-polling\n");
	flush_scheduled_work();	
	del_timer(&cns3xxx_sw_auto_polling.phy_polling_timer);
}

EXPORT_SYMBOL(cns3xxx_sw_auto_polling_init);
EXPORT_SYMBOL(cns3xxx_sw_auto_polling_uninit);
EXPORT_SYMBOL(cns3xxx_sw_auto_polling_conf);
EXPORT_SYMBOL(cns3xxx_sw_auto_polling_reg);

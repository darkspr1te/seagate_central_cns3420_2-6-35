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
1*   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
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

#include "cns3xxx_phy.h"

#include <linux/gpio.h>


#if defined(LINUX_KERNEL)
#include "cns3xxx_tool.h"
#include <linux/cns3xxx/switch_api.h> // for CAVM_OK ... macro
#include <linux/delay.h>
#include "cns3xxx_config.h"
#define printf printk
#else // u-boot
#include <common.h>
#include "cns3xxx_switch_type.h"
#define printk printf
#endif

int cns3xxx_phy_reset(u8 phy_addr)
{
	u16 phy_data=0;

	if (cns3xxx_read_phy(phy_addr, 0, &phy_data) != CAVM_OK)
		return CAVM_ERR;
	phy_data |= (0x1 << 15);
	if (cns3xxx_write_phy(phy_addr, 0, phy_data) != CAVM_OK)
		return CAVM_ERR;

	return CAVM_OK;
}

// mac_port: 0, 1, 2
int cns3xxx_enable_mac_clock(u8 mac_port, u8 en)
{
	switch (mac_port)
	{
		case 0:
		{
			(en==1)?(PHY_AUTO_ADDR_REG |= 1 << 7) :(PHY_AUTO_ADDR_REG &= (~(1 << 7)) );
			break;
		}
		case 1:
		{
			(en==1)?(PHY_AUTO_ADDR_REG |= (1 << 15)) :(PHY_AUTO_ADDR_REG &= (~(1 << 15)) );
			break;
		}
		case 2:
		{
			(en==1)?(PHY_AUTO_ADDR_REG |= (1 << 23)) :(PHY_AUTO_ADDR_REG &= (~(1 << 23)) );
			break;
		}
	}

	//printk("PHY_AUTO_ADDR_REG: %x\n", PHY_AUTO_ADDR_REG);

	return CAVM_OK;
}

// dis: 1 disable
// dis: 0 enable
int cns3xxx_phy_auto_polling_enable(u8 port, u8 en)
{
	u8 phy_addr[]={5, 13, 21};

	PHY_AUTO_ADDR_REG &= (~(1 << phy_addr[port]));
	if (en) {
		PHY_AUTO_ADDR_REG |= (1 << phy_addr[port]);
	}
	return CAVM_OK;
}

// dis: 1 disable
// dis: 0 enable
int cns3xxx_mdc_mdio_disable(u8 dis)
{

	PHY_CTRL_REG &= (~(1 << 7));
	if (dis) {
		PHY_CTRL_REG |= (1 << 7);
	}
	return CAVM_OK;
}


int cns3xxx_phy_auto_polling_conf(int mac_port, u8 phy_addr)
{
	//printk("mac_port: %d\n", mac_port);

	if ( (mac_port < 0) || (mac_port > 2) ) {
		printk("err\n");
		return CAVM_ERR;
	}
	//printk("ok\n");

	switch (mac_port)
	{
		case 0:
		{
			PHY_AUTO_ADDR_REG &= (~0x1f);
			PHY_AUTO_ADDR_REG |= phy_addr;
			break;
		}
		case 1:
		{
			PHY_AUTO_ADDR_REG &= (~(0x1f << 8));
			PHY_AUTO_ADDR_REG |= (phy_addr << 8);
			break;
		}
		case 2:
		{
			PHY_AUTO_ADDR_REG &= (~(0x1f << 16));
			PHY_AUTO_ADDR_REG |= (phy_addr << 16);
			break;
		}
	}
	cns3xxx_phy_auto_polling_enable(mac_port, 1);
	return CAVM_OK;
}



int cns3xxx_read_phy(u8 phy_addr, u8 phy_reg, u16 *read_data)
{
	int delay=0;
	u32 volatile tmp = PHY_CTRL_REG;

	PHY_CTRL_REG |= (1 << 15); // clear "command completed" bit
	// delay
	for (delay=0; delay<10; delay++);
#if 0


	//printk("clear r/w PHY_CTRL_REG: %x\n", PHY_CTRL_REG);

	PHY_CTRL_REG &= (~0x1f);
	PHY_CTRL_REG |= phy_addr;

	PHY_CTRL_REG &= (~(0x1f << 8));
	PHY_CTRL_REG |= (phy_reg << 8);

	PHY_CTRL_REG |= (1 << 14); // read command

	//printk(" read command PHY_CTRL_REG : %x\n", PHY_CTRL_REG);
#else

	tmp &= (~0x1f);
	tmp |= phy_addr;

	tmp &= (~(0x1f << 8));
	tmp |= (phy_reg << 8);

	tmp |= (1 << 14); // read command
	//printk("read cmd: %x\n", tmp);

	PHY_CTRL_REG = tmp;
#endif

	// wait command complete
	while ( ((PHY_CTRL_REG >> 15) & 1) == 0);

	*read_data = (PHY_CTRL_REG >> 16);

	PHY_CTRL_REG |= (1 << 15); // clear "command completed" bit

	return CAVM_OK;
}

int cns3xxx_write_phy(u8 phy_addr, u8 phy_reg, u16 write_data)
{
	int delay=0;
	u32 tmp = PHY_CTRL_REG;

	PHY_CTRL_REG |= (1 << 15); // clear "command completed" bit
	// delay
	for (delay=0; delay<10; delay++);
#if 0

	PHY_CTRL_REG &= (~(0xffff << 16));
	PHY_CTRL_REG |= (write_data << 16);

	PHY_CTRL_REG &= (~0x1f);
	PHY_CTRL_REG |= phy_addr;

	PHY_CTRL_REG &= (~(0x1f << 8));
	PHY_CTRL_REG |= (phy_reg << 8);

	PHY_CTRL_REG |= (1 << 13); // write command
#else
	//tmp |= (1 << 15); // clear "command completed" bit

	tmp &= (~(0xffff << 16));
	tmp |= (write_data << 16);

	tmp &= (~0x1f);
	tmp |= phy_addr;

	tmp &= (~(0x1f << 8));
	tmp |= (phy_reg << 8);

	tmp |= (1 << 13); // write command

	PHY_CTRL_REG = tmp;
	//printk("write command: %x\n", tmp);
#endif


	// wait command complete
	while ( ((PHY_CTRL_REG >> 15) & 1) == 0);


	return CAVM_OK;
}

// port 0,1,2
void cns3xxx_rxc_dly(u8 port, u8 val)
{
	switch (port)
	{
		case 0:
		{
			SLK_SKEW_CTRL_REG &= (~(0x3 << 4));
			SLK_SKEW_CTRL_REG |= (val << 4);
			break;
		}
		case 1:
		{
			SLK_SKEW_CTRL_REG &= (~(0x3 << 12));
			SLK_SKEW_CTRL_REG |= (val << 12);
			break;
		}
		case 2:
		{
			SLK_SKEW_CTRL_REG &= (~(0x3 << 20));
			SLK_SKEW_CTRL_REG |= (val << 20);
			break;
		}
	}
}

// port 0,1,2
void cns3xxx_txc_dly(u8 port, u8 val)
{
	switch (port)
	{
		case 0:
		{
			SLK_SKEW_CTRL_REG &= (~(0x3 << 6));
			SLK_SKEW_CTRL_REG |= (val << 6);
			break;
		}
		case 1:
		{
			SLK_SKEW_CTRL_REG &= (~(0x3 << 14));
			SLK_SKEW_CTRL_REG |= (val << 14);
			break;
		}
		case 2:
		{
			SLK_SKEW_CTRL_REG &= (~(0x3 << 22));
			SLK_SKEW_CTRL_REG |= (val << 22);
			break;
		}
	}
}

void cns3xxx_mac2_gtxd_dly(u8 val)
{
	SLK_SKEW_CTRL_REG &= (~(0x3 << 24));
	SLK_SKEW_CTRL_REG |= (val << 24);
}

// VITESSE suggest use isolate bit.
int vsc8601_power_down(int phy_addr, int y)
{
        u16 phy_data = 0;
        /* set isolate bit instead of powerdown */
        cns3xxx_read_phy(phy_addr, 0, &phy_data);
        if (y==1) // set isolate
                phy_data |= (0x1 << 10);
        if (y==0) // unset isolate
                phy_data &= (~(0x1 << 10));
        cns3xxx_write_phy(phy_addr, 0, phy_data);

        return 0;
}


// port : 0 => port0 ; port : 1 => port1
// y = 1 ; disable AN
void disable_AN(int port, int y)
{
	u32 mac_port_config=0;

	switch (port)
	{
		case 0:
		{
			mac_port_config = MAC0_CFG_REG;
			break;
		}
		case 1:
		{
			mac_port_config = MAC1_CFG_REG;
			break;
		}
		case 2:
		{
			mac_port_config = MAC2_CFG_REG;
			break;
		}
	}


	// disable PHY's AN
	if (y==1)
	{
	  mac_port_config &= ~(0x1 << 7);
	}

	// enable PHY's AN
	if (y==0)
	{
	  mac_port_config |= (0x1 << 7);
	}

	switch (port)
	{
		case 0:
		{
			MAC0_CFG_REG = mac_port_config;
			break;
		}
		case 1:
		{
			MAC1_CFG_REG = mac_port_config;
			break;
		}
		case 2:
		{
			MAC2_CFG_REG = mac_port_config;
			break;
		}
	}
}

int cns3xxx_std_phy_power_down(int phy_addr, int y)
{
        u16 phy_data = 0;
        // power-down or up the PHY
        cns3xxx_read_phy(phy_addr, 0, &phy_data);
        if (y==1) // down
                phy_data |= (0x1 << 11);
        if (y==0) // up
                phy_data &= (~(0x1 << 11));
        cns3xxx_write_phy(phy_addr, 0, phy_data);

	phy_data=0;
        cns3xxx_read_phy(phy_addr, 0, &phy_data);

        return 0;
}


#if (defined(CONFIG_VB) || defined (CONFIG_VB_2))
#if defined(LINUX_KERNEL)
int cns3xxx_spi_tx_rx_n(u32 tx_data, u32 *rx_data, u32 tx_channel, u32 tx_eof_flag)
{
	u8 cns3xxx_spi_tx_rx(u8 tx_channel, u8 tx_eof, u32 tx_data, u32 * rx_data);

	return cns3xxx_spi_tx_rx(tx_channel, tx_eof_flag, tx_data, rx_data);
}

int bcm53115M_reg_read(int page, int offset, u8 *buf, int len)
{
	u32 ch = BCM53115_SPI_CHANNEL;
	u8 cmd_byte;
	u32	dumy_word;
	u32 spi_status;
	int i;

	/*
	 * Normal SPI Mode (Command Byte)
	 * Bit7		Bit6		Bit5		Bit4		Bit3		Bit2		Bit1		Bit0
	 * 0		1			1			Mode=0		CHIP_ID2	ID1			ID0(lsb)	Rd/Wr(0/1)
	 *
	 */

	/* Normal Read Operation */
	/* 1. Issue a normal read command(0x60) to poll the SPIF bit in the
	      SPI status register(0XFE) to determine the operation can start */
	do
        {
		cmd_byte = 0x60;
		cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0xFE, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0x00, &spi_status, ch, 1);
		udelay(100);
	}while ((spi_status >> ROBO_SPIF_BIT) & 1) ; // wait SPI bit to 0

	/* 2. Issue a normal write command(0x61) to write the register page value
		  into the SPI page register(0xFF) 	 */
	cmd_byte = 0x61;
	cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0xFF, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(page, &dumy_word, ch, 1);

	/* 3. Issue a normal read command(0x60) to setup the required RobiSwitch register
		  address 	 */
	cmd_byte = 0x60;
	cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(offset, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0x00, &dumy_word, ch, 1);

	/* 4. Issue a normal read command(0x60) to poll the RACK bit in the
	      SPI status register(0XFE) to determine the completion of read 	 */
	do
	{
		cmd_byte = 0x60;
		cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0xFE, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0x00, &spi_status, ch, 1);
		udelay(100);
	}while (((spi_status >> ROBO_RACK_BIT) & 1) == 0); // wait RACK bit to 1

	/* 5. Issue a normal read command(0x60) to read the specific register's conternt
		  placed in the SPI data I/O register(0xF0) 	 */
	cmd_byte = 0x60;
	cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0xF0, &dumy_word, ch, 0);
	// read content
	for (i=0; i<len; i++) {
		cns3xxx_spi_tx_rx_n(0x00, &dumy_word, ch, ((i==(len-1)) ? 1 : 0));
		buf[i] = (u8)dumy_word;
	}

	return 0;

}

int bcm53115M_reg_write(int page, int offset, u8 *buf, int len)
{
	u32 ch = BCM53115_SPI_CHANNEL;
	u8 cmd_byte;
	u32 dumy_word;
	u32 spi_status;
	int i;

	/*
	 * Normal SPI Mode (Command Byte)
	 * Bit7		Bit6		Bit5		Bit4		Bit3		Bit2		Bit1		Bit0
	 * 0		1			1			Mode=0		CHIP_ID2	ID1			ID0(lsb)	Rd/Wr(0/1)
	 *
	 */

	/* Normal Write Operation */
	/* 1. Issue a normal read command(0x60) to poll the SPIF bit in the
	      SPI status register(0XFE) to determine the operation can start */

	do
        {
		cmd_byte = 0x60;
		cns3xxx_spi_tx_rx_n(cmd_byte, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0xFE, &dumy_word, ch, 0);
		cns3xxx_spi_tx_rx_n(0x00, &spi_status, ch, 1);
	}while ((spi_status >> ROBO_SPIF_BIT) & 1) ; // wait SPI bit to 0

	/* 2. Issue a normal write command(0x61) to write the register page value
		  into the SPI page register(0xFF) 	 */
	cmd_byte = 0x61;
	cns3xxx_spi_tx_rx_n((u32)cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(0xFF, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(page, &dumy_word, ch, 1);

	/* 3. Issue a normal write command(0x61) and write the address of the accessed
		  register followed by the write content starting from a lower byte */
	cmd_byte = 0x61;
	cns3xxx_spi_tx_rx_n((u32)cmd_byte, &dumy_word, ch, 0);
	cns3xxx_spi_tx_rx_n(offset, &dumy_word, ch, 0);
	// write content
	for (i=0; i<len; i++) {
		cns3xxx_spi_tx_rx_n((u32)buf[i], &dumy_word, ch, ((i==(len-1)) ? 1 : 0));
	}

	return 0;
}

void bcm53115M_init_mac(u8 mac_port, u16 phy_addr)
{
	u32 mac_port_config = 0;
	u8 mac_addr[]={0x0c, 0x10, 0x18};

	cns3xxx_enable_mac_clock(mac_port, 1);
	cns3xxx_phy_auto_polling_enable(mac_port, 0);

	mac_port_config = SWITCH_REG_VALUE(mac_addr[mac_port]);

	// enable GMII, MII, reverse MII
	mac_port_config &= (~(1 << 15));

	// enable RGMII
	mac_port_config |= (1 << 15);

        // disable GIGA mode
        mac_port_config &= (~(1<<16));

        // enable GIGA mode
        mac_port_config |= (1<<16);

        // disable PHY's AN
	mac_port_config &= (~(0x1 << 7));

	// force 1000Mbps
	mac_port_config &= (~(0x3 << 8));
	mac_port_config |= (0x2 << 8);

	// force duplex
	mac_port_config |= (0x1 << 10);

	// TX flow control on
	mac_port_config |= (0x1 << 12);

	// RX flow control on
	mac_port_config |= (0x1 << 11);

	// Turn off GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config &= (~(0x1 << 13));

	// Turn on GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config |= (0x1 << 13);

	SWITCH_REG_VALUE(mac_addr[mac_port]) = mac_port_config;
}

typedef struct bcm53115M_vlan_entry_t
{
	u16 vid;
	u16 forward_map;
	u16 untag_map;
}bcm53115M_vlan_entry;



int bcm53115M_write_vlan(bcm53115M_vlan_entry *v)
{
	u8 bval;
	u16 wval;
	u32 dwval;

	// fill vid
	wval = (u16)v->vid;
	bcm53115M_reg_write(0x05, 0x81, (u8*)&wval, 2);

	// fill table content
	dwval = 0;
	dwval |= (v->forward_map & 0x1FF);
	dwval |= ((v->untag_map& 0x1FF) << 9);
	bcm53115M_reg_write(0x05, 0x83, (u8*)&wval, 4);

	// write cmd
	bval = VLAN_WRITE_CMD;
	bval |= (1 << VLAN_START_BIT);
	bcm53115M_reg_write(0x05, 0x80, (u8*)&bval, 1);

	// wait cmd complete
	while(1) {
		bcm53115M_reg_read(0x05, 0x80, (u8*)&bval, 1);
		if (((bval >> VLAN_START_BIT) & 1) == 0) break;
	}

	return CAVM_OK;
}

typedef struct bcm_port_cfg_t
{
	u8 link;
	u8 fdx;
	BCM_PORT_SPEED speed;
	u8 rx_flw_ctrl;
	u8 tx_flw_ctrl;
	u8 ow;
}bcm_port_cfg;



int bcm53115M_mac_port_config(int port, bcm_port_cfg *cfg)
{
	u8 bval = 0;
	int page, addr;

	if (cfg->link) bval |= (1<<0);
	if (cfg->fdx) bval |= (1<<1);
	bval |= ((cfg->speed&0x3) << 2);
	if (cfg->rx_flw_ctrl) bval |= (1<<4);
	if (cfg->tx_flw_ctrl) bval |= (1<<5);

	if (port == BCM_PORT_IMP) {
		bval |= (1<<7); // Use content of this register
		page = 0x00;
		addr = 0x0E;
	}else {
		page = 0x00;
		addr = 0x58+port;
	}

	bcm53115M_reg_write(page, addr, &bval, 1);

	return 0;
}

int bcm53115M_init_internal_phy(void)
{
	int p, page;
	u16 wval;

	for (p=BCM_PORT_0; p<=BCM_PORT_4; p++) {
		page = 0x10+p;

		// reset phy
		bcm53115M_reg_read(page, 0x00, (u8*)&wval, 2);
		wval |= 0x8000;
		bcm53115M_reg_write(page, 0x00, (u8*)&wval, 2);

		// config auto-nego & all advertisement
		bcm53115M_reg_read(page, 0x00, (u8*)&wval, 2);
		wval |= (1<<12); // auto-nego
		bcm53115M_reg_write(page, 0x00, (u8*)&wval, 2);

		bcm53115M_reg_read(page, 0x08, (u8*)&wval, 2);
		wval |= 0x1E0; // advertisement all
		bcm53115M_reg_write(page, 0x08, (u8*)&wval, 2);

		// 1000BASE-T
		bcm53115M_reg_read(page, 0x12, (u8*)&wval, 2);
		wval &= ~(1<<12); // automatic master/slave configuration
		wval |= 0x300; // 1000-base full/half advertisements
		bcm53115M_reg_write(page, 0x12, (u8*)&wval, 2);
	}

	return 0;
}

int bcm53115M_led_init(void)
{
	u16 led_func, bval, wval;

	/* LED function 1G/ACT, 100M/ACT, 10M/ACT, not used */
	led_func = 0x2C00;
	bcm53115M_reg_write(0x00, 0x10, (u8*)&led_func, 2);
	bcm53115M_reg_write(0x00, 0x12, (u8*)&led_func, 2);

	/* LED map enable */
	wval = 0x1F; // port0~4
	bcm53115M_reg_write(0x00, 0x16, (u8*)&wval, 2);

	/* LED mode map */
	wval = 0x1F; // led auto mode
	bcm53115M_reg_write(0x00, 0x18, (u8*)&wval, 2);
	bcm53115M_reg_write(0x00, 0x1A, (u8*)&wval, 2);

	/* LED enable */
	bcm53115M_reg_read(0x00, 0x0F, (u8*)&bval, 1);
	bval |= 0x80;
	bcm53115M_reg_write(0x00, 0x0F, (u8*)&bval, 1);

	return 0;
}

//#define BCM53115M_DUMMY_SWITCH
//#define BCM53115M_DISABLE_LEARNING

int bcm53115M_init(u8 mac_port, u16 phy_addr)
{
	u32 u32_val=0;
	u16 u16_val=0;
	u8 bval=0;
	int i=0;
	bcm53115M_vlan_entry v_ent;
	bcm_port_cfg pc;
	u8 page=0, offset=0;
	u8 gpio_pin=50; // for control bcm53115, gpio B pin 18

	printk("Initialize BCM53115M\n");

	memset(&v_ent, 0, sizeof(bcm53115M_vlan_entry));

	// gpio B pin 18
	if (gpio_request(gpio_pin, "cns3xxx_gpio")==0) {
		gpio_direction_output(gpio_pin, 0);
		gpio_free(gpio_pin);
	}

	bcm53115M_init_mac(0, 0);
	bcm53115M_init_mac(1, 1);

	// read device id
	bcm53115M_reg_read(0x02, 0x30, (u8*)&u32_val, 4);
	printk("bcm53115M device id:(0x%x)\r\n", u32_val);

	if (u32_val != 0x53115) {
		printk("bad device id(0x%x)\r\n", u32_val);
		return -1;
	}

	u16_val=0;
	// read phy id
	bcm53115M_reg_read(0x10, 0x04, (u8 *)&u16_val, 2);
	printk("bcm53115M phy id_1:(0x%x)\r\n", u16_val);

	if (u16_val != 0x143) {
		printk("bad phy id1(0x%x)\r\n", u16_val);
		return CAVM_ERR;
	}

	u16_val=0;
	// read phy id2
	bcm53115M_reg_read(0x10, 0x06, (u8 *)&u16_val, 2);
	printk("bcm53115M phy id_2:(0x%x)\r\n", u16_val);

#ifdef BCM53115M_DUMMY_SWITCH
	bval=0;
	bcm53115M_reg_read(0x00, 0x0e, (u8 *)&bval, 1);
	printk("bcm53115M page:0 addr:0x0e ## %x\n", bval);
	bval |= (1 << 7);
	bval |= 1;

	bval = 0x8b;
	bval |= (1 << 5);
	bval |= (1 << 4);
	printk("bval : %x\n", bval);
	bcm53115M_reg_write(0x00, 0x0e, (u8 *)&bval, 1);
	bcm53115M_reg_read(0x00, 0x0e, (u8 *)&bval, 1);
	printk("bcm53115M page:0 addr:0x0e ## %x\n", bval);

	/* Unmanagement mode */
	// Switch Mode. Page 00h,Address 0Bh
	bval = 0x06; // forward enable, unmanaged mode
	//bval = 0x3; // forward enable, managed mode
	bcm53115M_reg_write(0x0, 0xb, &bval, 1);
	bcm53115M_reg_read(0x0, 0xb, (u8 *)&bval, 1);
	printk("bcm53115M page:0 addr:0xb ## %x\n", bval);

	page=0x0;
	offset=0x5d; // port 5
	bval=0x7b;
	bcm53115M_reg_write(page, offset, (u8 *)&bval, 1);
	bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);

	printk("bcm53115M page:%x addr:%x ## %x\n", page, offset, bval);

	page=0x0;
	offset=0x58; // port 0
	bval=0x7b;
	bcm53115M_reg_write(page, offset, (u8 *)&bval, 1);
	bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);
	printk("bcm53115M page:%x addr:%x ## %x\n", page, offset, bval);

#ifdef CONFIG_CNS3XXX_JUMBO_FRAME
	printk("enable BCM53115 jumbo frame\n");

	page=0x40;
	offset=0x01;
	u32_val=0x013f; // enable 0-5 port and IMP port jumbo frame. MAX frame: 9720 bytes.
	bcm53115M_reg_write(page, offset, (u8 *)&u32_val, 4);
	bcm53115M_reg_read(page, offset, (u8 *)&u32_val, 4);
	printk("bcm53115M page:%x addr:%x ## %x\n", page, offset, u32_val);

#if 0
	page=0x40;
	offset=0x05;
	u16_val=0;
	bcm53115M_reg_write(page, offset, (u8 *)&u16_val, 2);
#endif

#endif

#else // !BCM53115M_DUMMY_SWITCH
	/* Loop detection disable */
	bcm53115M_reg_read(0x72, 0x00, (u8 *)&u16_val, 2);
	u16_val &= ~(0x3<<11);
	bcm53115M_reg_write(0x72, 0x00, (u8 *)&u16_val, 2);


	/* VLAN forwarding mask */
	// Bit8 IMP port, Bits[5:0] correspond to ports[5:0]
	// port 0 <-> port IMP
	u16_val = 0x103;
	bcm53115M_reg_write(0x31, 0x0, (u8 *)&u16_val, 2); // port 0
	u16_val = 0x103;
	bcm53115M_reg_write(0x31, 0x10, (u8 *)&u16_val, 2); // IMP


	// port 4 <-> port 5
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x08, (u8 *)&u16_val, 2); // port 4
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x0A, (u8 *)&u16_val, 2); // port 5


	// others <-> none
	u16_val = 0x00;
	bcm53115M_reg_write(0x31, 0x02, (u8 *)&u16_val, 2); // port 1
	bcm53115M_reg_write(0x31, 0x04, (u8 *)&u16_val, 2); // port 2
	bcm53115M_reg_write(0x31, 0x06, (u8 *)&u16_val, 2); // port 3

	// port 1 <-> port IMP
	u16_val = 0x103;
	bcm53115M_reg_write(0x31, 0x2, (u8 *)&u16_val, 2); // port 1

	// port 2 <-> port 5
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x4, (u8 *)&u16_val, 2); // port 2

	// port 3 <-> port 5
	u16_val = 0x3c;
	bcm53115M_reg_write(0x31, 0x6, (u8 *)&u16_val, 2); // port 3

	/* Create VLAN1 for default port pvid */
#if 0
	v_ent.vid = 1;
	v_ent.forward_map = 0x13F; // all ports
	robo_write_vlan(&v_ent);
#endif

	/* Unmanagement mode */
	// Switch Mode. Page 00h,Address 0Bh
	bval = 0x02; // forward enable, unmanaged mode
	bcm53115M_reg_write(0x0, 0xb, &bval, 1);

	/* Init port5 & IMP  (test giga mode first) */
	// IMP port control. Page 00h,Address 08h
	bval = 0x1C; // RX UCST/MCST/BCST enable
	bcm53115M_reg_write(0x0, 0x8, &bval, 1);

	offset=0x5d; // port 5
	bval=0x7b;
	bcm53115M_reg_write(page, offset, (u8 *)&bval, 1);
	bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);

	// Speed, dulplex......etc
	// setting in Gsw_Configure_Gsw_Hardware()

	// Mgmt configuration, Page 02h, Address 00h
	bval = 0;
	bcm53115M_reg_write(0x02, 0x00, &bval, 1);
	// BRCM header, Page 02h, Address 03h
	bval = 0; // without additional header information
	bcm53115M_reg_write(0x02, 0x03, &bval, 1);

	/* Init front ports, port0-4 */
	// MAC
	pc.speed = BCM_PORT_1G;
	pc.link = 0; // link detect by robo_port_update()
	pc.ow = 0;
	for (i=BCM_PORT_0; i<=BCM_PORT_4; i++)
		bcm53115M_mac_port_config(i, &pc);
	// Internal Phy
	bcm53115M_init_internal_phy();

	/* Enable all port, STP_STATE=No spanning tree, TX/RX enable */
	// Page 00h, Address 00h-05h
	bval = 0x0;
	for (i=0; i<=5; i++)
		bcm53115M_reg_write(0x0, i, &bval, 1);

	// Disable broadcast storm control due to h/w strap pin BC_SUPP_EN
	// Page 41h, Address 10h-13h, bit28&22

	// for port 0 ~ 5
	for (i=0 ; i <= 0x14; i+=4) {
		bcm53115M_reg_read(0x41, 0x10+i, (u8 *)&u32_val, 4);
		u32_val &= ~((1<<28) | (1<<22));
		bcm53115M_reg_write(0x41, 0x10+i, (u8 *)&u32_val, 4);
	}

	// for IMP port
	bcm53115M_reg_read(0x41, 0x30, (u8 *)&u32_val, 4);
	u32_val &= ~((1<<28) | (1<<22));
	bcm53115M_reg_write(0x41, 0x30, (u8 *)&u32_val, 4);

	/* Misc */
	// led
	bcm53115M_led_init();
	// multicast fwd rule, Page 00h, Address 2Fh
	bval = 0;
	bcm53115M_reg_write(0x00, 0x2F, &bval, 1);

#ifdef BCM53115M_DISABLE_LEARNING
	// disable learning
	page=0x00;
	offset=0x3c;
	u16_val=0x13f;
	bcm53115M_reg_write(page, offset, (u8 *)&u16_val, 2);
	bcm53115M_reg_read(page, offset, (u8 *)&u16_val, 2);

	page=0x02;
	offset=0x06;
	u32_val=4;
	bcm53115M_reg_write(page, offset, (u8 *)&u32_val, 4);
#endif
#endif

#if 1
        // enable IMF port flow control function.
        page=0x00;
        offset=0x0e;
        bval=0;
        bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);
        bval |= (1<<4);
        bval |= (1<<5);
        bcm53115M_reg_write(page, offset, (u8 *)&bval, 1);
        bcm53115M_reg_read(page, offset, (u8 *)&bval, 1);
#endif


	return CAVM_OK;
}
#endif // defined(LINUX_KERNEL)
#endif // ONFIG_VB




#if defined(CONFIG_VB_2) || defined(CONFIG_IP1001_X2)
#define MAC2_RGMII
#endif
#define CNS3XXX_MAC2_IP1001_GIGA_MODE
//#define MAC2_RGMII

#if defined(CONFIG_VB_2)
void rtk_rtl8211_init_mac(u8 mac_port, u16 phy_addr)
#else
void icp_ip1001_init_mac(u8 mac_port, u16 phy_addr)
#endif
{
	u32 mac_port_config = 0;
	u8 mac_addr[]={0x0c, 0x10, 0x18};

	cns3xxx_enable_mac_clock(mac_port, 1);

	mac_port_config = SWITCH_REG_VALUE(mac_addr[mac_port]);

	//cns3xxx_txc_dly(mac_port, 2);
	//cns3xxx_rxc_dly(mac_port, 2);
	//SLK_SKEW_CTRL_REG
#if 1

	// enable GMII, MII, reverse MII
	mac_port_config &= (~(1 << 15));

#ifdef MAC2_RGMII
	mac_port_config |= (1 << 15);
#endif

	// TXC check disable
	//mac_port_config &= (~(1 << 13));

        // disable GIGA mode
        mac_port_config &= (~(1<<16));

#ifdef CNS3XXX_MAC2_IP1001_GIGA_MODE
        // enable GIGA mode
        mac_port_config |= (1<<16);

        //mac_port_config |= (1<<19);
#endif

        // disable PHY's AN
	mac_port_config &= (~(0x1 << 7));

        // enable PHY's AN
	mac_port_config |= (0x1 << 7);
#else
        // disable PHY's AN
	mac_port_config &= (~(0x1 << 7));
        // disable GIGA mode
        mac_port_config &= (~(1<<16));

	// force 100Mbps
	mac_port_config &= (~(0x3 << 8));
	mac_port_config |= (0x1 << 8);

	// force duplex
	mac_port_config |= (0x1 << 10);

	// TX flow control off
	mac_port_config &= (~(0x1 << 12));

	// RX flow control off
	mac_port_config &= (~(0x1 << 11));

#if 0
	// TX flow control on
	mac_port_config |= (0x1 << 12);

	// RX flow control on
	mac_port_config |= (0x1 << 11);
#endif

	// enable GMII, MII, reverse MII
	mac_port_config &= (~(1 << 15));
#endif
	SWITCH_REG_VALUE(mac_addr[mac_port]) = mac_port_config;
}

#if defined(CONFIG_VB_2)
int rtk_rtl8211_init(u8 mac_port, u8 phy_addr)
#define PROBE_PHY_ID 0x001C
#else
int icp_ip1001_init(u8 mac_port, u8 phy_addr)
#define PROBE_PHY_ID 0x0243
#endif
{
        u16 phy_data = 0;

#if defined(CONFIG_VB_2)
	printk("Initialize RTL8211\n");
#else
	printk("Initialize ICPLUS IP1001\n");
#endif
	//printk("mac_port: %d ## phy_addr: %d\n", mac_port, phy_addr);
	cns3xxx_mdc_mdio_disable(0);

	phy_data = get_phy_id(phy_addr); // should be 0x243

	//printk("ICPLUS IP 1001 phy id : %x\n", phy_data);

	if (phy_data != PROBE_PHY_ID) {
		printk("wrong phy id: %x!! Should be %x\n", phy_data, PROBE_PHY_ID);
		return CAVM_ERR;
	}


	cns3xxx_phy_reset(phy_addr);

#if defined(CONFIG_VB_2)
	rtk_rtl8211_init_mac(mac_port, phy_addr);
#else
	icp_ip1001_init_mac(mac_port, phy_addr);
#endif

	// read advertisement register
	cns3xxx_read_phy(phy_addr, 0x4, &phy_data);

	// enable PAUSE frame capability
	phy_data |= (0x1 << 10);

	phy_data &= (~(0x1 << 5));
	phy_data &= (~(0x1 << 6));
	phy_data &= (~(0x1 << 7));
	phy_data &= (~(0x1 << 8));

#if 1
	phy_data |= (0x1 << 5);
	phy_data |= (0x1 << 6);
	phy_data |= (0x1 << 7);
	phy_data |= (0x1 << 8);
#endif

	cns3xxx_write_phy(phy_addr, 0x4, phy_data);

	cns3xxx_read_phy(phy_addr, 9, &phy_data);

	phy_data &= (~(1<<8)); // remove advertise 1000 half duples
	phy_data &= (~(1<<9)); // remove advertise 1000 full duples
#ifdef CNS3XXX_MAC2_IP1001_GIGA_MODE
	//phy_data |= (1<<8); // add advertise 1000 half duples
	phy_data |= (1<<9); // add advertise 1000 full duples
#endif
	cns3xxx_write_phy(phy_addr, 9, phy_data);

	cns3xxx_read_phy(phy_addr, 9, &phy_data);

	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	// AN enable
	phy_data |= (0x1 << 12);
	cns3xxx_write_phy(phy_addr, 0, phy_data);

	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	// restart AN
	phy_data |= (0x1 << 9);
	cns3xxx_write_phy(phy_addr, 0, phy_data);

	// If mac port AN turns on, auto polling needs to turn on.
	cns3xxx_phy_auto_polling_conf(mac_port, phy_addr);
        return 0;
}

#define PHY_CONTROL_REG_ADDR 0x00
#define PHY_AN_ADVERTISEMENT_REG_ADDR 0x04

int icp_101a_init_mac(u8 port, u8 phy_addr)
{
	u32 mac_port_config = 0;

	cns3xxx_enable_mac_clock(port, 1);

	switch (port)
	{
		case 0:
		{
			mac_port_config = MAC0_CFG_REG;
			break;
		}
		case 1:
		{
			mac_port_config = MAC1_CFG_REG;
			break;
		}
		case 2:
		{
			mac_port_config = MAC2_CFG_REG;
			break;
		}
	}

	// enable GMII, MII, reverse MII
	mac_port_config &= (~(1 << 15));

        // disable PHY's AN, use force mode
	mac_port_config &= (~(0x1 << 7));
#ifdef CONFIG_FPGA_FORCE

	// force 100Mbps
	mac_port_config &= (~(0x3 << 8));
	mac_port_config |= (0x1 << 8);

	// force duplex
	mac_port_config |= (0x1 << 10);

	// TX flow control on
	mac_port_config |= (0x1 << 12);

	// RX flow control on
	mac_port_config |= (0x1 << 11);

	// Turn off GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config &= (~(0x1 << 13));
#else
        // enable PHY's AN
	mac_port_config |= (0x1 << 7);
	// If mac port AN turns on, auto polling needs to turn on.
	cns3xxx_phy_auto_polling_conf(port, phy_addr);
#endif
	// normal MII
	mac_port_config &= (~(1 << 14));


	switch (port)
	{
		case 0:
		{
			MAC0_CFG_REG = mac_port_config;
			break;
		}
		case 1:
		{
			MAC1_CFG_REG = mac_port_config;
			break;
		}
		case 2:
		{
			MAC2_CFG_REG = mac_port_config;
			break;
		}
	}


	return CAVM_OK;
}

int icp_101a_init(u8 mac_port, u8 phy_addr)
{
	u32 mac_port_config=0;
        u16 phy_data = 0;

	cns3xxx_mdc_mdio_disable(0);
	cns3xxx_phy_reset(phy_addr);

	phy_data = get_phy_id(mac_port);
	if (phy_data != 0x0243) {
		printk("ICPLUS 101A phy id should be 0x243, but the phy id is : %x\n", phy_data);
		return CAVM_ERR;
	}
	printk("phy id : %x\n", phy_data);
	printk("init IC+101A\n");

	icp_101a_init_mac(mac_port, phy_addr);

	// read advertisement register
	cns3xxx_read_phy(phy_addr, 0x4, &phy_data);

	// enable PAUSE frame capability
	phy_data |= (0x1 << 10);

	cns3xxx_write_phy(phy_addr, 0x4, phy_data);

#ifndef CONFIG_FPGA_FORCE

	switch (mac_port)
	{
		case 0:
		{
			mac_port_config = MAC0_CFG_REG;
			break;
		}
		case 1:
		{
			mac_port_config = MAC1_CFG_REG;
			break;
		}
		case 2:
		{
			mac_port_config = MAC2_CFG_REG;
			break;
		}
	}

#if 0
	if (!(mac_port_config & (0x1 << 5))) {
		if (cns3xxx_read_phy (port, PHY_AN_ADVERTISEMENT_REG_ADDR, &phy_data) == CAVM_ERR)
		{
			//PDEBUG("\n PORT%d, enable local flow control capability Fail\n", port);
			return CAVM_ERR;
		}
		else
		{
			// enable PAUSE frame capability
			phy_data |= (0x1 << 10);

			if (cns3xxx_write_phy (port, PHY_AN_ADVERTISEMENT_REG_ADDR, phy_data) == CAVM_ERR)
			{
				//PDEBUG("\nPORT%d, enable PAUSE frame capability Fail\n", port);
				return CAVM_ERR;
			}
		}
	}
#endif

	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	// an enable
	phy_data |= (0x1 << 12);

	// restart AN
	phy_data |= (0x1 << 9);
	cns3xxx_write_phy(phy_addr, 0, phy_data);

	while (1)
	{
		//PDEBUG ("\n Polling  PHY%d AN \n", port);
		cns3xxx_read_phy (phy_data, 0, &phy_data);

		if (phy_data & (0x1 << 9)) {
			continue;
		} else {
			//PDEBUG ("\n PHY%d AN restart is complete \n", port);
			break;
		}
	}

#endif

	return CAVM_OK;
}

int cns3xxx_config_VSC8601_mac(u8 port)
{
	u32 mac_port_config = 0;

        printk("INIT VSC8601 mac\n");

	switch (port)
	{
		case 0:
		{
			mac_port_config = MAC0_CFG_REG;
			break;
		}
		case 1:
		{
			mac_port_config = MAC1_CFG_REG;
			break;
		}
		case 2:
		{
			mac_port_config = MAC2_CFG_REG;
			break;
		}
	}

	switch (port)
	{
		case 0:
		{
			MAC0_CFG_REG = mac_port_config;
			break;
		}
		case 1:
		{
			MAC1_CFG_REG = mac_port_config;
			break;
		}
		case 2:
		{
			MAC2_CFG_REG = mac_port_config;
			break;
		}
	}
	return CAVM_OK;
}

u16 get_phy_id(u8 phy_addr)
{
	u16 read_data;

	cns3xxx_read_phy(phy_addr, 2, &read_data);

	return read_data;
}

u32 get_vsc8601_recv_err_counter(u8 phy_addr)
{
	u16 read_data=0;
	cns3xxx_read_phy(phy_addr, 19, &read_data);
	//printk("recv_err_counter: %d\n", read_data);
	return read_data;
}

u32 get_crc_good_counter(u8 phy_addr)
{
	u16 read_data=0;


#if 0
	printk("in get crc phy_addr: %d\n", phy_addr);
	cns3xxx_read_phy(phy_addr, 3, &read_data);
	printk("8601 phy id2: %d\n", read_data);

	cns3xxx_read_phy(phy_addr, 2, &read_data);
	printk("8601 phy id1: %d\n", read_data);
#endif

	// enter extended register mode
	cns3xxx_write_phy(phy_addr, 31, 0x0001);

#if 0
	cns3xxx_read_phy(phy_addr, 31, &read_data);
	printk("01 31 reg data: %x\n", read_data);
#endif

	cns3xxx_read_phy(phy_addr, 18, &read_data);

	// back to normal register mode
	cns3xxx_write_phy(phy_addr, 31, 0x0000);

#if 0
	cns3xxx_read_phy(phy_addr, 31, &read_data);
	printk("00 31 reg data: %x\n", read_data);
#endif

	//printk("good crc: %d\n", read_data);
	return read_data;
}

int cns3xxx_config_VSC8601(u8 mac_port, u8 phy_addr)
{
        u16 phy_data=0;
	u32 mac_port_config=0;
	//u8 tx_skew=1, rx_skew=1;
	u16 phy_id=0;

	cns3xxx_mdc_mdio_disable(0);

	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	// software reset
	phy_data |= (0x1 << 15);
	cns3xxx_write_phy(phy_addr, 0, phy_data);
	udelay(10);

	phy_id = get_phy_id(phy_addr);
	printk("found VSC8601,  phy id is : %x\n", phy_id);

	if (phy_id != 7) {
		printk("VSC8601 phy id %x is not correct.\n", phy_id);
		return CAVM_ERR;
	}
        printk("begin to initialize VSC8601\n");

#if 1
	switch (mac_port)
	{
		case 0:
		{
			mac_port_config = MAC0_CFG_REG;
			break;
		}
		case 1:
		{
			mac_port_config = MAC1_CFG_REG;
			break;
		}
		case 2:
		{
			mac_port_config = MAC2_CFG_REG;
			break;
		}
	}

	cns3xxx_enable_mac_clock(mac_port, 1);
        //phy_auto_polling(mac_port, phy_addr);

	// enable RGMII-PHY mode
	mac_port_config |= (0x1 << 15);
#ifdef CONFIG_FPGA_FORCE
        // disable PHY's AN, use force mode
	mac_port_config &= (~(0x1 << 7));

	#ifdef CONFIG_FPGA_10

	// force 10Mbps
	mac_port_config &= (~(0x3 << 8));

	#else

	// force 100Mbps
	mac_port_config &= (~(0x3 << 8));
	mac_port_config |= (0x1 << 8);


	#endif

	// force duplex
	mac_port_config |= (0x1 << 10);

	// TX flow control on
	mac_port_config |= (0x1 << 12);

	// RX flow control on
	mac_port_config |= (0x1 << 11);

	// Turn off GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config &= (~(0x1 << 13));



#else // non FPGA

	// If mac AN turns on, auto polling needs to turn on.
        // enable PHY's AN
	mac_port_config |= (0x1 << 7);
	cns3xxx_phy_auto_polling_conf(mac_port, phy_addr);

	// enable GSW MAC port 0
	//mac_port_config &= ~(0x1 << 18);
#endif

	// normal MII
	mac_port_config &= (~(1 << 14));

	switch (mac_port)
	{
		case 0:
		{
			MAC0_CFG_REG = mac_port_config;
			printk("8601 MAC0_CFG_REG: %x\n", MAC0_CFG_REG);
			break;
		}
		case 1:
		{
			MAC1_CFG_REG = mac_port_config;
			printk("8601 MAC1_CFG_REG: %x\n", MAC1_CFG_REG);
			break;
		}
		case 2:
		{
			MAC2_CFG_REG = mac_port_config;
			break;
		}
	}

        cns3xxx_read_phy(phy_addr, 3, &phy_data);
	printk("id2 id: %d\n", phy_data);
	if ((phy_data & 0x000f) == 0x0000) { // type A chip
                u16 tmp16;

                printk("VSC8601 Type A Chip\n");
                cns3xxx_write_phy(phy_addr, 31, 0x52B5);
                cns3xxx_write_phy(phy_addr, 16, 0xAF8A);

                phy_data = 0x0;
                cns3xxx_read_phy(phy_addr, 18, &tmp16);
                phy_data |= (tmp16 & ~0x0);
                cns3xxx_write_phy(phy_addr, 18, phy_data);

                phy_data = 0x0008;
                cns3xxx_read_phy(phy_addr, 17, &tmp16);
                phy_data |= (tmp16 & ~0x000C);
                cns3xxx_write_phy(phy_addr, 17, phy_data);

                cns3xxx_write_phy(phy_addr, 16, 0x8F8A);

                cns3xxx_write_phy(phy_addr, 16, 0xAF86);
                phy_data = 0x0008;
                cns3xxx_read_phy(phy_addr, 18, &tmp16);
                phy_data |= (tmp16 & ~0x000C);
                cns3xxx_write_phy(phy_addr, 18, phy_data);

                phy_data = 0x0;
                cns3xxx_read_phy(phy_addr, 17, &tmp16);
                phy_data |= (tmp16 & ~0x0);
                cns3xxx_write_phy(phy_addr, 17, phy_data);
                cns3xxx_write_phy(phy_addr, 16, 0x8F8A);

                cns3xxx_write_phy(phy_addr, 16, 0xAF82);

                phy_data = 0x0;
                cns3xxx_read_phy(phy_addr, 18, &tmp16);
                phy_data |= (tmp16 & ~0x0);
                cns3xxx_write_phy(phy_addr, 18, phy_data);

                phy_data = 0x0100;
                cns3xxx_read_phy(phy_addr, 17, &tmp16);
                phy_data |= (tmp16 & ~0x0180);
                cns3xxx_write_phy(phy_addr, 17, phy_data);

                cns3xxx_write_phy(phy_addr, 16, 0x8F82);

                cns3xxx_write_phy(phy_addr, 31, 0x0);
                //Set port type: single port
		cns3xxx_read_phy(phy_addr, 9, &phy_data);
		phy_data &= ~( 0x1 << 10);
		cns3xxx_write_phy(phy_addr, 9, phy_data);
        } else if ((phy_data & 0x000f) == 0x0001) { // type B chip
                printk("VSC8601 Type B Chip\n");
                cns3xxx_read_phy(phy_addr, 23, &phy_data);
                phy_data |= ( 0x1 << 8); //set RGMII timing skew
                cns3xxx_write_phy(phy_addr, 23, phy_data);
        }
    /*
     * Enable full-duplex mode
     */
#if 0 //ref: non-os
	cns3xxx_read_phy(phy_addr, 0, &phy_data);
	phy_data |= (0x1 << 8);
	cns3xxx_write_phy(phy_addr, 0, phy_data);
#endif

#ifdef CONFIG_FPGA
{
	u16 d=5;
	cns3xxx_read_phy(phy_addr, 9, &d);
	d &= (~(1 << 9));
	d &= (~(1 << 8));
	cns3xxx_write_phy(phy_addr, 9, d);

#if 0
        if (phy_addr == 1) {
        cns3xxx_read_phy(phy_addr, 4, &d);
#if 1
        // remove 100 Mbps capability
        d &= (~(1 << 9));
        d &= (~(1 << 8));
        d &= (~(1 << 7));
#else
        // remove 10 Mbps capability
        d &= (~(1 << 6));
        d &= (~(1 << 5));
#endif
        cns3xxx_write_phy(phy_addr, 4, d);
        }
#endif

}
#endif


	cns3xxx_read_phy(phy_addr, 0, &phy_data);
#ifdef CONFIG_FPGA_FORCE
	// full duplex
	phy_data |= (0x1 << 8);

	#ifdef CONFIG_FPGA_10
	// 10 Mbps
	phy_data &= (~(0x1 << 6));
	phy_data &= (~(0x1 << 13));

	#else
	// 100 Mbps
	phy_data |= (0x1 << 13);
	phy_data &= (~(0x1 << 6));
	#endif

	// an disable
	phy_data &= (~(0x1 << 12));
#else // CONFIG_FPGA


	// an enable
	phy_data |= (0x1 << 12);

	// restart AN
	phy_data |= (0x1 << 9);

#endif // CONFIG_FPGA
	cns3xxx_write_phy(phy_addr, 0, phy_data);



        // change to extended registers
	cns3xxx_write_phy(phy_addr, 31, 0x0001);

	cns3xxx_read_phy(phy_addr, 28, &phy_data);
	phy_data &= ~(0x3 << 14); // set RGMII TX timing skew
	phy_data |= (0x3 << 14); // 2.0ns
	phy_data &= ~(0x3 << 12); // set RGMII RX timing skew
	phy_data |= (0x3 << 12); // 2.0ns

#ifdef CONFIG_CNS3XXX_JUMBO_FRAME
        phy_data &= ~(0x3 << 10); //set Jumbo frame mode
        phy_data |=  (0x3 << 10); //set 16KB
#endif

	cns3xxx_write_phy(phy_addr, 28, phy_data);


	// change to normal registers
	cns3xxx_write_phy(phy_addr, 31, 0x0000);

#if 0
	tx_skew=3;
	cns3xxx_txc_dly(mac_port, tx_skew);
	rx_skew=3;
	cns3xxx_rxc_dly(mac_port, rx_skew);
#endif


#endif
	return CAVM_OK;
}

#ifdef CONFIG_AR8316

#define AR8316_MAC0_GIGA_MODE

// atheros 8316
void ar8316_init_mac(u8 mac_port, u16 phy_addr)
{
	u32 mac_port_config = 0;
	u8 mac_addr[]={0x0c, 0x10, 0x18};

#if defined(AR8316_MAC0_GIGA_MODE) || defined(AR8316_MAC1_GIGA_MODE)
        IOCDA_REG = 0x55555800;
#endif
	//cns3xxx_mdc_mdio_disable(0);
	printf("ar8316_init_mac : %d\n", mac_port);
	cns3xxx_enable_mac_clock(mac_port, 1);
	cns3xxx_phy_auto_polling_enable(mac_port, 0);

	mac_port_config = SWITCH_REG_VALUE(mac_addr[mac_port]);

	// enable GMII, MII, reverse MII
	mac_port_config &= (~(1 << 15));

	// enable RGMII
	mac_port_config |= (1 << 15);

        // disable GIGA mode
        mac_port_config &= (~(1<<16));

#ifdef AR8316_MAC0_GIGA_MODE
        // enable GIGA mode
        mac_port_config |= (1<<16);
#endif

        // disable PHY's AN
	mac_port_config &= (~(0x1 << 7));

	// force speed
	mac_port_config &= (~(0x3 << 8));

#ifdef AR8316_MAC0_GIGA_MODE
	// force 1000Mbps
	mac_port_config |= (0x2 << 8);
#else
	// force 100Mbps
	mac_port_config |= (0x1 << 8);
#endif

	// force duplex
	mac_port_config |= (0x1 << 10);

	// TX flow control on
	mac_port_config |= (0x1 << 12);

	// RX flow control on
	mac_port_config |= (0x1 << 11);

	// Turn off GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config &= (~(0x1 << 13));

	// Turn on GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config |= (0x1 << 13);

	SWITCH_REG_VALUE(mac_addr[mac_port]) = mac_port_config;
}


#define phy_reg_read        cns3xxx_ar8316_phy_read
#define phy_reg_write       cns3xxx_ar8316_phy_write
#define uint32_t u32
#define uint16_t u16
#define uint8_t u8

void cns3xxx_ar8316_phy_write(int unit, uint32_t phy_addr, uint8_t reg, uint16_t data)
{
	cns3xxx_write_phy(phy_addr, reg, data);
}

uint16_t cns3xxx_ar8316_phy_read(int unit, uint32_t phy_addr, uint8_t reg)
{
	u16 read_data=0;

	cns3xxx_read_phy(phy_addr, reg, &read_data);
	return read_data;
}

static uint32_t athrs16_reg_read(uint32_t reg_addr)
{
    uint32_t reg_word_addr;
    uint32_t phy_addr, tmp_val, reg_val;
    uint16_t phy_val;
    uint8_t phy_reg;

    /* change reg_addr to 16-bit word address, 32-bit aligned */
    reg_word_addr = (reg_addr & 0xfffffffc) >> 1;

    /* configure register high address */
    phy_addr = 0x18;
    phy_reg = 0x0;
    phy_val = (uint16_t) ((reg_word_addr >> 8) & 0x1ff);  /* bit16-8 of reg address */
    phy_reg_write(0, phy_addr, phy_reg, phy_val);

    /* For some registers such as MIBs, since it is read/clear, we should */
    /* read the lower 16-bit register then the higher one */

    /* read register in lower address */
    phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
    phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
    reg_val = (uint32_t) phy_reg_read(0, phy_addr, phy_reg);

    /* read register in higher address */
    reg_word_addr++;
    phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
    phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
    tmp_val = (uint32_t) phy_reg_read(0, phy_addr, phy_reg);
    reg_val |= (tmp_val << 16);

    return reg_val;
}

static void athrs16_reg_write(uint32_t reg_addr, uint32_t reg_val)
{
    uint32_t reg_word_addr;
    uint32_t phy_addr;
    uint16_t phy_val;
    uint8_t phy_reg;

    /* change reg_addr to 16-bit word address, 32-bit aligned */
    reg_word_addr = (reg_addr & 0xfffffffc) >> 1;

    /* configure register high address */
    phy_addr = 0x18;
    phy_reg = 0x0;
    phy_val = (uint16_t) ((reg_word_addr >> 8) & 0x1ff);  /* bit16-8 of reg address */
    phy_reg_write(0, phy_addr, phy_reg, phy_val);

    /* For some registers such as ARL and VLAN, since they include BUSY bit */
    /* in lower address, we should write the higher 16-bit register then the */
    /* lower one */

    /* read register in higher address */
    reg_word_addr++;
    phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
    phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
    phy_val = (uint16_t) ((reg_val >> 16) & 0xffff);
    phy_reg_write(0, phy_addr, phy_reg, phy_val);

    /* write register in lower address */
    reg_word_addr--;
    phy_addr = 0x10 | ((reg_word_addr >> 5) & 0x7); /* bit7-5 of reg address */
    phy_reg = (uint8_t) (reg_word_addr & 0x1f);   /* bit4-0 of reg address */
    phy_val = (uint16_t) (reg_val & 0xffff);
    phy_reg_write(0, phy_addr, phy_reg, phy_val);
}

static inline void cns3xxx_write_phy_wrap(u8 phy_addr, u16 phy_reg, u16 phy_data)
{
	cns3xxx_write_phy(phy_addr, phy_reg, phy_data);
	//cns3xxx_read_phy(phy_addr, phy_reg, &phy_data);
	//printf("phy_addr : %#x ## phy_reg : %#x ## phy_data : %#x\n", phy_addr, phy_reg, phy_data);
}

// phy 4
int ar8316_init_port5(u8 mac_port, u16 phy_addr)
{
	u32 mac_port_config = 0;
	u8 mac_addr[]={0x0c, 0x10, 0x18};
	u16 data=0;
	u32 ar8316_reg_data=0;
	u16 ar8316_reg_offset=0;

	cns3xxx_phy_auto_polling_enable(mac_port, 0);
	printf("initialize mac port : %d\n", mac_port);

	//setting PHY4 RGMII mode
	cns3xxx_write_phy_wrap(0x4, 0x1d, 0x12);
	cns3xxx_write_phy_wrap(0x4, 0x1e, 0x480c);

	// debug register 0x00 bit15:RXCLK(output) delay control set 1 to add 2ns delay
	cns3xxx_write_phy_wrap(0x4, 0x1d, 0x0);
	cns3xxx_write_phy_wrap(0x4, 0x1e, 0x824e);

	// debug register 0x05 bit8:GTXCLK(input) delay control set 1 to enable GTXCLK delay
	cns3xxx_write_phy_wrap(0x4, 0x1d, 0x5);
	cns3xxx_write_phy_wrap(0x4, 0x1e, 0x3d47);

	// debug register 0xb bit[6:5]:GTXCLK delay selection,00:0.2ns,01:1.2ns,10:2.1ns,11:3ns
	cns3xxx_write_phy_wrap(0x4, 0x1d, 0xb);
	cns3xxx_write_phy_wrap(0x4, 0x1e, 0xbc40);

	ar8316_reg_offset=0x2c;
	ar8316_reg_data=0x043f003f;
	athrs16_reg_write(ar8316_reg_offset, ar8316_reg_data);
	ar8316_reg_data=athrs16_reg_read(ar8316_reg_offset);


	cns3xxx_enable_mac_clock(mac_port, 1);
	mac_port_config = SWITCH_REG_VALUE(mac_addr[mac_port]);
	// enable GMII, MII, reverse MII
	mac_port_config &= (~(1 << 15));

	// enable RGMII
	mac_port_config |= (1 << 15);

        // disable GIGA mode
        mac_port_config &= (~(1<<16));

#if 1
        // enable GIGA mode
        mac_port_config |= (1<<16);
#endif

        // disable PHY's AN
	mac_port_config &= (~(0x1 << 7));

        // enable PHY's AN
	mac_port_config |= (0x1 << 7);

	SWITCH_REG_VALUE(mac_addr[mac_port]) = mac_port_config;

	// If mac port AN turns on, auto polling needs to turn on.
	cns3xxx_phy_auto_polling_conf(mac_port, phy_addr);

	return 0;
}


int ar8316_init(u8 mac_port, u16 phy_address)
{
	u8 phy_addr=0;
	u16 phy_reg=0;
	u16 phy_data=0;
	u16 data=0;
	u32 offset=0x0d02;
	u32 d=0;
	u32 ar8316_reg_data=0;
	u16 ar8316_reg_offset=0;

	printf("initialize ar8316\n");
	ar8316_init_mac(0, 0); // init mac0

	cns3xxx_mdc_mdio_disable(0);

	// do software reset
	ar8316_reg_offset=0x08;
	ar8316_reg_data=0x01061b60;
	athrs16_reg_write(ar8316_reg_offset, ar8316_reg_data);

	// do software reset
	ar8316_reg_offset=0x0;
	ar8316_reg_data=0x80000000;
	athrs16_reg_write(ar8316_reg_offset, ar8316_reg_data);

	ar8316_reg_offset=0;
	ar8316_reg_data=athrs16_reg_read(ar8316_reg_offset);
	printf("ar8316_reg_offset: %#x ## ar8316_reg_data: %#x\n", ar8316_reg_offset, ar8316_reg_data);

	for (phy_addr=0; phy_addr < 5; ++phy_addr){
		cns3xxx_read_phy(phy_addr, offset, &data);
		printf("phy_addr: %d ## offset: %x ## data: %x\n", phy_addr, offset, data);
		data=0;
	}

	offset=0x02;
	for (phy_addr=0; phy_addr < 5; ++phy_addr){
		cns3xxx_read_phy(phy_addr, offset, &data);
		printf("phy_addr: %d ## offset: %x ## data: %x\n", phy_addr, offset, data);
	}

	ar8316_reg_offset=0x100;
	ar8316_reg_data=athrs16_reg_read(ar8316_reg_offset);
	printf("ar8316_reg_offset: %#x ## ar8316_reg_data: %#x\n", ar8316_reg_offset, ar8316_reg_data);
#ifdef AR8316_MAC0_GIGA_MODE
	ar8316_reg_data=0x7e; // 1000 mode
#else
	ar8316_reg_data=0x7d; // 100 M mode
#endif
	athrs16_reg_write(ar8316_reg_offset, ar8316_reg_data);
	ar8316_reg_data=athrs16_reg_read(ar8316_reg_offset);

	ar8316_reg_offset=0x600;
	ar8316_reg_data=0x0;
	athrs16_reg_write(ar8316_reg_offset, ar8316_reg_data);
	ar8316_reg_data=athrs16_reg_read(ar8316_reg_offset);

	ar8316_reg_offset=0x08;
	ar8316_reg_data=0x01261bea;

	// ar8316 tx delay
#if 1
	ar8316_reg_data &= (~(3 << 21));
#ifdef AR8316_MAC0_GIGA_MODE
	ar8316_reg_data |= (2 << 21);
#endif
#endif

#if 1
	// set ar8316 rx delay
	ar8316_reg_data &= (~(1 << 23));
	ar8316_reg_data &= (~(1 << 26));

	ar8316_reg_data |= (1 << 23);
	ar8316_reg_data |= (0 << 26);
#endif

	athrs16_reg_write(ar8316_reg_offset, ar8316_reg_data);
	ar8316_reg_data=athrs16_reg_read(ar8316_reg_offset);

	return 0;
}
#endif // CONFIG_AR8316


#ifdef CONFIG_LIBRA
void icp_175c_all_phy_power_down(int y)
{
        int i=0;

        for (i=0 ; i < 5 ; ++i)
                std_phy_power_down(i, y);

}

static int star_gsw_config_icplus_175c_phy4(void)
{
    u16			phy_data = 0, phy_data2 = 0;
    u32 volatile	ii, jj;
    u8			phy_speed_dup = 0, phy_flowctrl = 0;
    u32 volatile	reg;
	u8 gsw_mac_0_phy_addr = 0;
	u8 gsw_mac_1_phy_addr = 1;


	printk("config IC+175C\n");
    /*
     * Configure MAC port 0
     * For IP175C Switch setting
     * Force 100Mbps, and full-duplex, and flow control on
     */
    reg = GSW_MAC_PORT_0_CONFIG_REG;

    // disable PHY's AN
    reg &= ~(0x1 << 7);

    // disable RGMII-PHY mode
    reg &= ~(0x1 << 15);

    // force speed = 100Mbps
    reg &= ~(0x3 << 8);
    reg |= (0x1 << 8);

    // force full-duplex
    reg |= (0x1 << 10);

    // force Tx/Rx flow-control on
    reg |= (0x1 << 11) | (0x1 << 12);

    GSW_MAC_PORT_0_CONFIG_REG = reg;


    for (ii = 0; ii < 0x2000; ii++)
    {
	reg = GSW_MAC_PORT_0_CONFIG_REG;

        if ((reg & 0x1) && !(reg & 0x2))
        {
            /*
             * enable MAC port 0
             */
            reg &= ~(0x1 << 18);


            /*
             * enable the forwarding of unknown, multicast and broadcast packets to CPU
             */
            reg &= ~((0x1 << 25) | (0x1 << 26) | (0x1 << 27));

            /*
             * include unknown, multicast and broadcast packets into broadcast storm
             */
            reg |= ((0x1 << 29) | (0x1 << 30) | ((u32)0x1 << 31));

		GSW_MAC_PORT_0_CONFIG_REG = reg;

            break;
        }
        else
        {
            for (jj = 0; jj < 0x1000; jj++);


            if ((ii % 4) == 0)
                printk("\rCheck MAC/PHY 0 Link Status : |");
            else if ((ii % 4) == 1)
                printk("\rCheck MAC/PHY 0 Link Status : /");
            else if ((ii % 4) == 2)
                printk("\rCheck MAC/PHY 0 Link Status : -");
            else if ((ii % 4) == 3)
                printk("\rCheck MAC/PHY 0 Link Status : \\");
        }
    }


    if (!(reg & 0x1) || (reg & 0x2))
    {
        /*
         * Port 0 PHY link down or no TXC in Port 0
         */
        printk("\rCheck MAC/PHY 0 Link Status : DOWN!\n");

        return -1;
    }
    else
    {
        printk("\rCheck MAC/PHY 0 Link Status : UP!\n");
    }



    /*
     * Configure MAC port 1
     */
	reg = GSW_MAC_PORT_0_CONFIG_REG;

    // disable MAC's AN
    reg &= ~(0x1 << 7);

	GSW_MAC_PORT_0_CONFIG_REG = reg;


    /* enable flow control on (PAUSE frame) */
    star_gsw_read_phy(gsw_mac_1_phy_addr, 0x4, &phy_data);

    phy_data |= (0x1 << 10);

    star_gsw_write_phy(gsw_mac_1_phy_addr, 0x4, phy_data);

#if 1
	/* 2007/12/18 Jerry
		The software reset of IC+ 175C won't reset MII register 29, 30, 31.
		Router Control Register: bit 7 (TAG_VLAN_EN) is a VLAN related filed which affect vlan setting.
		Router Control Register: bit 3 (ROUTER_EN) enable router function at MII port.
		We set them to default to let U-boot properly work.
	*/
	phy_data = 0x1001;
    star_gsw_write_phy(30, 9, phy_data);
#endif
    /* restart PHY auto neg. */
    star_gsw_read_phy(gsw_mac_1_phy_addr, 0x0, &phy_data);

    phy_data |= (0x1 << 9) | (0x1 << 12);

    star_gsw_write_phy(gsw_mac_1_phy_addr, 0x0, phy_data);



    /* wait for PHY auto neg. complete */
    for (ii = 0; ii < 0x20; ii++)
    {
        star_gsw_read_phy(gsw_mac_1_phy_addr, 0x1, &phy_data);

        if ((phy_data & (0x1 << 2)) && (phy_data & (0x1 << 5)))
        {
            break;
        }
        else
        {
            if ((ii % 4) == 0)
                printk("\rCheck MAC/PHY 1 Link Status : |");
            else if ((ii % 4) == 1)
                printk("\rCheck MAC/PHY 1 Link Status : /");
            else if ((ii % 4) == 2)
                printk("\rCheck MAC/PHY 1 Link Status : -");
            else if ((ii % 4) == 3)
                printk("\rCheck MAC/PHY 1 Link Status : \\");
        }
    }


    if (ii >= 0x20)
    {
        printk("\rCheck MAC/PHY 1 Link Status : DOWN!\n");

        return -1;
    }
    else
    {
        printk("\rCheck MAC/PHY 1 Link Status : UP!\n");
    }


    star_gsw_read_phy(gsw_mac_1_phy_addr, 0x4, &phy_data);

    star_gsw_read_phy(gsw_mac_1_phy_addr, 0x5, &phy_data2);


    if (phy_data & 0x0400)	//FC on
    {
        //printk("<FC ON>");
        phy_flowctrl = 1;
    }
    else
    {
        // printk("<FC OFF>");
        phy_flowctrl = 0;
    }


    phy_speed_dup = 0;

    if ((phy_data & 0x0100) && (phy_data2 & 0x0100)) //100F
    {
        // printk("<100F>");
        phy_speed_dup |= (0x1 << 3); //set bit3 for 100F
    }
    else if ((phy_data & 0x0080) && (phy_data2 & 0x0080)) //100F
    {
        // printk("<100H>");
        phy_speed_dup |= (0x1 << 2);
    }
    else if ((phy_data & 0x0040) && (phy_data2 & 0x0040)) //100F
    {
        // printk("<10F>");
        phy_speed_dup |= (0x1 << 1);
    }
    else if ((phy_data & 0x0020) && (phy_data2 & 0x0020)) //100F
    {
        // printk("<10H>");
        phy_speed_dup |= 0x1;
    }


    /*
     * Configure MAC port 1 in forced setting subject to the current PHY status
     */
	reg = GSW_MAC_PORT_1_CONFIG_REG;

    reg &= ~(0x1 << 7);	//AN off

    reg &= ~(0x3 << 8);

    if (phy_speed_dup & 0x0C)     //100
    {
        //printk("<set 100>");
        reg |= (0x01 << 8);
    }
    else if (phy_speed_dup & 0x03)     //10
    {
        //printk("<set 10>");
        reg |= (0x00 << 8);
    }

    reg &= ~(0x1 << 11);

    if (phy_flowctrl)	//FC on
    {
        //printk("<set FC on>");
        reg |= (0x1 << 11);
    }
    else
    {
        //printk("<set FC off>");
        reg |= (0x0 << 11);
    }

    reg &= ~(0x1 << 10);

    if ((phy_speed_dup & 0x2) || (phy_speed_dup & 0x8))	//FullDup
    {
        //printk("<set full>");
        reg |= (0x1 << 10);
    }
    else	//HalfDup
    {
        //printk("<set half>");
        reg |= (0x0 << 10); //Half
    }

	GSW_MAC_PORT_1_CONFIG_REG = reg;


    /*
     * Check MAC port 1 link status
     */
    for (ii = 0; ii < 0x1000; ii++)
    {
	reg = GSW_MAC_PORT_1_CONFIG_REG;

        if ((reg & 0x1) && !(reg & 0x2))
        {
            /*
             * enable MAC port 1
             */
            reg &= ~(0x1 << 18);

            /*
             * enable the forwarding of unknown, multicast and broadcast packets to CPU
             */
            reg &= ~((0x1 << 25) | (0x1 << 26) | (0x1 << 27));

            /*
             * include unknown, multicast and broadcast packets into broadcast storm
             */
            reg |= ((0x1 << 29) | (0x1 << 30) | ((u32)0x1 << 31));

	GSW_MAC_PORT_1_CONFIG_REG = reg;

            return 0;
        }
    }


    if (ii > 0x1000)
    {
        /*
         * Port 1 PHY link down or no TXC in Port 1
         */
        printk("\rCheck MAC/PHY 1 Link Status : DOWN!\n");

        return -1;
    }
	return 0;
}
#endif

#if 0
static int star_gsw_config_VSC8201(u8 mac_port, u8 phy_addr)	// include cicada 8201
{
	//u32 mac_port_base = 0;
	u32 mac_port_config=0;
	u16 phy_reg;
	int i;

	printk("\nconfigure VSC8201\n");
	//PDEBUG("mac port : %d phy addr : %d\n", mac_port, phy_addr);
	/*
	 * Configure MAC port 0
	 * For Cicada CIS8201 single PHY
	 */
	if (mac_port == 0) {
		//PDEBUG("port 0\n");
		mac_port_config = GSW_MAC_PORT_0_CONFIG_REG;
	}
	if (mac_port == 1) {
		//PDEBUG("port 1\n");
		mac_port_config = GSW_MAC_PORT_1_CONFIG_REG;
	}

	star_gsw_set_phy_addr(mac_port, phy_addr);
	//star_gsw_set_phy_addr(1, 1);

	//mac_port_config = __REG(mac_port_base);

	// enable PHY's AN
	mac_port_config |= (0x1 << 7);

	// enable RGMII-PHY mode
	mac_port_config |= (0x1 << 15);

	// enable GSW MAC port 0
	mac_port_config &= ~(0x1 << 18);

	if (mac_port == 0) {
		//PDEBUG("port 0\n");
		GSW_MAC_PORT_0_CONFIG_REG = mac_port_config;
	}
	if (mac_port == 1) {
		//PDEBUG("port 1\n");
		GSW_MAC_PORT_1_CONFIG_REG = mac_port_config;
	}

	/*
	 * Configure Cicada's CIS8201 single PHY
	 */
#ifdef CONFIG_STAR9100_SHNAT_PCI_FASTPATH
	/* near-end loopback mode */
	star_gsw_read_phy(phy_addr, 0x0, &phy_reg);
	phy_reg |= (0x1 << 14);
	star_gsw_write_phy(phy_addr, 0x0, phy_reg);
#endif

	star_gsw_read_phy(phy_addr, 0x1C, &phy_reg);

	// configure SMI registers have higher priority over MODE/FRC_DPLX, and ANEG_DIS pins
	phy_reg |= (0x1 << 2);

	star_gsw_write_phy(phy_addr, 0x1C, phy_reg);

	star_gsw_read_phy(phy_addr, 0x17, &phy_reg);

	// enable RGMII MAC interface mode
	phy_reg &= ~(0xF << 12);
	phy_reg |= (0x1 << 12);

	// enable RGMII I/O pins operating from 2.5V supply
	phy_reg &= ~(0x7 << 9);
	phy_reg |= (0x1 << 9);

	star_gsw_write_phy(phy_addr, 0x17, phy_reg);

	star_gsw_read_phy(phy_addr, 0x4, &phy_reg);

	// Enable symmetric Pause capable
	phy_reg |= (0x1 << 10);

	star_gsw_write_phy(phy_addr, 0x4, phy_reg);



	if (mac_port == 0) {
		//PDEBUG("port 0\n");
		mac_port_config = GSW_MAC_PORT_0_CONFIG_REG;
	}
	if (mac_port == 1) {
		//PDEBUG("port 1\n");
		mac_port_config = GSW_MAC_PORT_1_CONFIG_REG;
	}







	// enable PHY's AN
	mac_port_config |= (0x1 << 7);

	if (mac_port == 0) {
		//PDEBUG("port 0\n");
		GSW_MAC_PORT_0_CONFIG_REG = mac_port_config;
	}
	if (mac_port == 1) {
		//PDEBUG("port 1\n");
		GSW_MAC_PORT_1_CONFIG_REG = mac_port_config;
	}

	/*
	 * Enable PHY1 AN restart bit to restart PHY1 AN
	 */
	star_gsw_read_phy(phy_addr, 0x0, &phy_reg);

	phy_reg |= (0x1 << 9) | (0x1 << 12);

	star_gsw_write_phy(phy_addr, 0x0, phy_reg);

	/*
	 * Polling until PHY0 AN restart is complete
	 */
	for (i = 0; i < 0x1000; i++) {
		star_gsw_read_phy(phy_addr, 0x1, &phy_reg);

		if ((phy_reg & (0x1 << 5)) && (phy_reg & (0x1 << 2))) {
			printk("0x1 phy reg: %x\n", phy_reg);
			break;
		} else {
			udelay(100);
		}
	}

	if (mac_port == 0) {
		//PDEBUG("port 0\n");
		mac_port_config = GSW_MAC_PORT_0_CONFIG_REG;
	}
	if (mac_port == 1) {
		//PDEBUG("port 1\n");
		mac_port_config = GSW_MAC_PORT_1_CONFIG_REG;
	}

	if (((mac_port_config & 0x1) == 0) || (mac_port_config & 0x2)) {
		printk("Check MAC/PHY%s Link Status : DOWN!\n", (mac_port == 0 ? "0" : "1"));
	} else {
		printk("Check MAC/PHY%s Link Status : UP!\n", (mac_port == 0 ? "0" : "1"));
		/*
		 * There is a bug for CIS8201 PHY operating at 10H mode, and we use the following
		 * code segment to work-around
		 */
		star_gsw_read_phy(phy_addr, 0x05, &phy_reg);

		if ((phy_reg & (0x1 << 5)) && (!(phy_reg & (0x1 << 6))) && (!(phy_reg & (0x1 << 7))) && (!(phy_reg & (0x1 << 8)))) {	/* 10H,10F/100F/100H off */
			star_gsw_read_phy(phy_addr, 0x0a, &phy_reg);

			if ((!(phy_reg & (0x1 << 10))) && (!(phy_reg & (0x1 << 11)))) {	/* 1000F/1000H off */
				star_gsw_read_phy(phy_addr, 0x16, &phy_reg);

				phy_reg |= (0x1 << 13) | (0x1 << 15);	// disable "Link integrity check(B13)" & "Echo mode(B15)"

				star_gsw_write_phy(phy_addr, 0x16, phy_reg);
			}
		}
	}

	if (mac_port == 0) {
		// adjust MAC port 0 RX/TX clock skew
		GSW_BIST_RESULT_TEST_0_REG &= ~((0x3 << 24) | (0x3 << 26));
		GSW_BIST_RESULT_TEST_0_REG |= ((0x2 << 24) | (0x2 << 26));
	}

	if (mac_port == 1) {
		// adjust MAC port 1 RX/TX clock skew
		GSW_BIST_RESULT_TEST_0_REG &= ~((0x3 << 28) | (0x3 << 30));
		GSW_BIST_RESULT_TEST_0_REG |= ((0x2 << 28) | (0x2 << 30));
	}

	return 0;
}




static void star_gsw_config_VSC8X01()
{
   u16	phy_id = 0;

#ifdef CONFIG_DORADO2
   star_gsw_set_phy_addr(1,1);
   star_gsw_read_phy(1, 0x02, &phy_id);
 //  printk("phy id = %X\n", phy_id);
   if (phy_id == 0x000F) //VSC8201
	star_gsw_config_VSC8201(1,1);
   else
	star_gsw_config_VSC8601(1,1);
#else
#ifdef CONFIG_LEO
   star_gsw_set_phy_addr(0,0);
   star_gsw_read_phy(0, 0x02, &phy_id);
 //  printk("phy id = %X\n", phy_id);
   if (phy_id == 0x000F) //VSC8201
	star_gsw_config_VSC8201(0,0);
   else
	star_gsw_config_VSC8601(0,0);
#endif
#endif
}
#endif

#if defined(CONFIG_DORADO) || defined(CONFIG_DORADO2)
static int star_gsw_config_port0_VSC7385(void)
{
	u32 mac_port_config=0;
	int i;

	printk("config VSC7385\n");

	mac_port_config = GSW_MAC_PORT_0_CONFIG_REG;

	// disable PHY's AN
	mac_port_config &= ~(0x1 << 7);

	// enable RGMII-PHY mode
	mac_port_config |= (0x1 << 15);

	// force speed = 1000Mbps
	mac_port_config &= ~(0x3 << 8);
	mac_port_config |= (0x2 << 8);

	// force full-duplex
	mac_port_config |= (0x1 << 10);

	// force Tx/Rx flow-control on
	mac_port_config |= (0x1 << 11) | (0x1 << 12);

	GSW_MAC_PORT_0_CONFIG_REG = mac_port_config;

	udelay(1000);

	for (i = 0; i < 50000; i++) {
		mac_port_config = GSW_MAC_PORT_0_CONFIG_REG;
		if ((mac_port_config & 0x1) && !(mac_port_config & 0x2)) {
			break;
		} else {
			udelay(100);
		}
	}

	if (!(mac_port_config & 0x1) || (mac_port_config & 0x2)) {
		printk("MAC0 PHY Link Status : DOWN!\n");
		return -1;
	} else {
		printk("MAC0 PHY Link Status : UP!\n");
	}

	// enable MAC port 0
	mac_port_config &= ~(0x1 << 18);

	// disable SA learning
	mac_port_config |= (0x1 << 19);

	// forward unknown, multicast and broadcast packets to CPU
	mac_port_config &= ~((0x1 << 25) | (0x1 << 26) | (0x1 << 27));

	// storm rate control for unknown, multicast and broadcast packets
	mac_port_config |= (0x1 << 29) | (0x1 << 30) | ((u32)0x1 << 31);

	GSW_MAC_PORT_0_CONFIG_REG = mac_port_config;

	// disable MAC port 1
	mac_port_config = GSW_MAC_PORT_1_CONFIG_REG;
	mac_port_config |= (0x1 << 18);
	GSW_MAC_PORT_1_CONFIG_REG = mac_port_config;

	// adjust MAC port 0 /RX/TX clock skew
	GSW_BIST_RESULT_TEST_0_REG &= ~((0x3 << 24) | (0x3 << 26));
	GSW_BIST_RESULT_TEST_0_REG |= ((0x2 << 24) | (0x2 << 26));

	return 0;
}
#endif

#ifdef CONFIG_RTL8367


#include "rtl8367/rtk_api.h"
rtk_api_ret_t rtk_switch_init(void);
rtk_api_ret_t rtk_led_enable_set(rtk_led_group_t group, rtk_portmask_t portmask);
rtk_api_ret_t rtk_cpu_enable_set(rtk_enable_t enable);
rtk_api_ret_t rtk_cpu_tagPort_set(rtk_port_t port, rtk_cpu_insert_t mode);
rtk_api_ret_t rtk_vlan_init(void);
rtk_api_ret_t rtk_port_macForceLinkExt1_set(rtk_mode_ext_t mode, rtk_port_mac_ability_t *pPortability);
rtk_api_ret_t rtk_port_macForceLinkExt0_set(rtk_mode_ext_t mode, rtk_port_mac_ability_t *pPortability);
rtk_api_ret_t rtk_port_macForceLinkExt1_get(rtk_mode_ext_t *pMode, rtk_port_mac_ability_t *pPortability);
int32 smi_read(uint32 mAddrs, uint32 *rData);
rtk_api_ret_t rtk_vlan_set(rtk_vlan_t vid, rtk_portmask_t mbrmsk, rtk_portmask_t untagmsk, rtk_fid_t fid);
rtk_api_ret_t rtk_vlan_portPvid_set(rtk_port_t port, rtk_vlan_t pvid, rtk_pri_t priority);
void rtl8367_init_phy(void)
{

	cns3xxx_mdc_mdio_disable(1);
	PMU_REG_VALUE(0x0) |= (0x1<<10);    /* enable GPIO */
	PMU_REG_VALUE(0x4) &= ~(0x1<<10);   /* reset GPIO */
	PMU_REG_VALUE(0x4) |= (0x1<<10);
	MISC_REG_VALUE(0x18) &= ~(0x3<<20); /* configure GPIOB*/

	if (0 != rtk_switch_init())  {
		printk("rtk_switch_init fail\n");
		return;
	} else {
		rtk_port_mac_ability_t ability;
		rtk_data_t mode;
		rtk_portmask_t portmask;
		portmask.bits[0] = 0x1F;
		rtk_led_enable_set(LED_GROUP_0, portmask);
		rtk_led_enable_set(LED_GROUP_1, portmask);
		rtk_cpu_enable_set(ENABLE);
#if 0
		rtk_cpu_tagPort_set(RTK_EXT_1_MAC, CPU_INSERT_TO_TRAPPING);
		rtk_cpu_tagPort_set(RTK_EXT_0_MAC, CPU_INSERT_TO_TRAPPING);
#else
		rtk_cpu_tagPort_set(RTK_EXT_1_MAC, CPU_INSERT_TO_ALL);
		rtk_cpu_tagPort_set(RTK_EXT_0_MAC, CPU_INSERT_TO_TRAPPING);
#endif
		rtk_vlan_init();
		ability.forcemode = 1;
		ability.speed = 2;
		ability.duplex = 1;
		ability.link = 1;
		ability.nway = 0;
		ability.txpause = 1;
		ability.rxpause = 1;
		rtk_port_macForceLinkExt0_set(MODE_EXT_RGMII, &ability);
		rtk_port_macForceLinkExt1_set(MODE_EXT_RGMII, &ability);

	{
		rtk_portmask_t mbrmsk, untagmsk;

		mbrmsk.bits[0] = 0x010F;
		untagmsk.bits[0] = 0x01FF;
		rtk_vlan_set(50, mbrmsk, untagmsk, 50);

		mbrmsk.bits[0] = 0x0210;
		untagmsk.bits[0] = 0x02FF;
		rtk_vlan_set(60, mbrmsk, untagmsk, 60);
#if 0
		rtk_vlan_portPvid_set(0, 50, 0);
		rtk_vlan_portPvid_set(1, 50, 0);
		rtk_vlan_portPvid_set(2, 50, 0);
		rtk_vlan_portPvid_set(3, 50, 0);
		rtk_vlan_portPvid_set(8, 50, 0);
		rtk_vlan_portPvid_set(4, 60, 0);
		rtk_vlan_portPvid_set(9, 60, 0);
#endif
	}

	#if 0
	{
        u32 rData, mAddr;
        mAddr = 0x1305;
        if (0 == smi_read(mAddr, &rData)) {
            printk("[0x%.8x] => 0x%.8x\n", mAddr, rData);
        } else {
            printk("[0x%.8x] => fail\n", mAddr);
        }


        mAddr = 0x1310;
        if (0 == smi_read(mAddr, &rData)) {
            printk("[0x%.8x] => 0x%.8x\n", mAddr, rData);
        } else {
            printk("[0x%.8x] => fail\n", mAddr);
        }

        mAddr = 0x1311;
        if (0 == smi_read(mAddr, &rData)) {
            printk("[0x%.8x] => 0x%.8x\n", mAddr, rData);
        } else {
            printk("[0x%.8x] => fail\n", mAddr);
        }


        if (0 == rtk_port_macForceLinkExt1_get(&mode, &ability)) {
            printk("mode %d\n", mode);
            printk("forcemode %d spped %d duplex %d link %d nway %d txpause %d rxpause %d\n",
                ability.forcemode, ability.speed, ability.duplex,
                ability.link, ability.nway, ability.txpause, ability.rxpause);
        };
	}
	#endif

	}
}

void rtl8367_init_mac(u8 mac_port)
{
	u32 mac_port_config = 0;
	u8 mac_addr[]={0x0c, 0x10, 0x18};
	u8 tx = 0, rx = 0;

	cns3xxx_enable_mac_clock(mac_port, 1);
	cns3xxx_phy_auto_polling_enable(mac_port, 0);

	mac_port_config = SWITCH_REG_VALUE(mac_addr[mac_port]);

	// enable GMII, MII, reverse MII
	mac_port_config &= (~(1 << 15));

	// enable RGMII
	mac_port_config |= (1 << 15);

	// disable GIGA mode
	mac_port_config &= (~(1<<16));

	// enable GIGA mode
	mac_port_config |= (1<<16);

	// disable PHY's AN
	mac_port_config &= (~(0x1 << 7));

	// force 1000Mbps
	mac_port_config &= (~(0x3 << 8));
	mac_port_config |= (0x2 << 8);

	// force duplex
	mac_port_config |= (0x1 << 10);

	// TX flow control on
	mac_port_config |= (0x1 << 12);

	// RX flow control on
	mac_port_config |= (0x1 << 11);

	// Turn off GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config &= (~(0x1 << 13));

	// Turn on GSW_PORT_TX_CHECK_EN_BIT
	mac_port_config |= (0x1 << 13);

	SWITCH_REG_VALUE(mac_addr[mac_port]) = mac_port_config;

	SWITCH_REG_VALUE(0xf0) &= ~(0xf0);
	SWITCH_REG_VALUE(0xf0) |= (0x1<<4); /* rx */
	SWITCH_REG_VALUE(0xf0) |= (0x2<<6); /* tx*/

	rx = 2;
	tx = 2;
	SWITCH_REG_VALUE(0xf0) &= ~(0xf000);
	SWITCH_REG_VALUE(0xf0) |= (rx<<12); /* rx */
	SWITCH_REG_VALUE(0xf0) |= (tx<<14); /* tx*/
}
#endif

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

#include "../cns3xxx_pse_init/cns3xxx.h"
#include "cns3xxx_tool.h"
#include "cns3xxx_config.h"
#include <linux/cns3xxx/pse_init_common.h>

#ifdef CONFIG_SUSPEND
#include <linux/suspend.h>
#endif

#if defined (CONFIG_CNS3XXX_SPPE)
#include <linux/cns3xxx/sppe.h>
#define PACKET_REASON_TO_CPU (0x2C)
#endif

#if defined (CONFIG_CNS3XXX_ETHADDR_IN_FLASH)
#ifdef CONFIG_CIRRUS_DUAL_MTD_ENV
#include "../../rtc/rtc-nvedit.h"
#else /*CONFIG_CIRRUS_DUAL_MTD_ENV*/
#include <linux/mtd/mtd.h>
#endif /*CONFIG_CIRRUS_DUAL_MTD_ENV*/
#endif

#ifdef CNS3XXX_SG_DEBUG
static u32 sg_time = 0;
#endif

extern NetDevicePriv net_device_prive[10];

#define RX_SDP_ALIGN 64

static ushort MAX_PACKET_LEN = 1536;
#define CPU_CACHE_BYTES         64
#define CPU_CACHE_ALIGN(X)      (((X) + (CPU_CACHE_BYTES-1)) & ~(CPU_CACHE_BYTES-1))

#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
atomic_t cpufreq_pse_flag;
#endif

#if defined (CONFIG_CNS3XXX_ETHADDR_IN_FLASH)

#ifdef CONFIG_CIRRUS_DUAL_MTD_ENV

static char ethaddr[12]={0};

static int init_mtd_env(void) {
    rtcnvet_init();
    return 0;
}

int fmg_get(const char *name, int *ret_len) {
    char *buf;
    char *nbuf;
    int ret;
    int i,j;
    /* sanity */

    if (NULL == name || NULL == ret_len) {
        printk(KERN_ERR\
            "%s: Called with invalid data!\n",\
            __func__);
        return -1;
    }

    ret = strlen(name);

    if (0 == ret) {
        printk(KERN_ERR\
            "%s: Called with invalid data!\n",\
            __func__);
        return -1;
    }

    nbuf = kmalloc(ret + 1, GFP_KERNEL);

    if (NULL == nbuf) {
        printk(KERN_ERR\
            "%s: Failed to allocate kernel memory!\n",\
            __func__);
        return -1;
    }

    memset(nbuf, 0x0, ret);

    /* name is 'key=', we need 'key' */

    memmove(nbuf, name, ret-1);

    buf = NULL;

    ret = rtcnvet_get_env(nbuf, &buf);

    kfree(nbuf);
    
    if (0 == ret) {
        if (NULL != buf) {
            ret = strlen(buf);
        	memset(ethaddr,0x0,12);
            j=0;
            for (i=0; i<ret; i++) {
                if (':' != buf[i]) {
                    ethaddr[j] = buf[i];
                    j++;
                }
            }             
            kfree(buf);
            *ret_len = strlen(ethaddr);
            return 0;
        }
    }
    printk(KERN_ERR\
        "%s: Cannot retrive value from ENV for \"%s\"!\n",\
        __func__, name);    
    return -1;
}

#else /*CONFIG_CIRRUS_DUAL_MTD_ENV*/
#define MTD_READ(mtd, args...) (*(mtd->read))(mtd, args)

#ifdef CONFIG_CNS3XXX_MAC_IN_SPI_FLASH
#define ENV_OFFSET 0x30000
#define PARTITION_NAME "SPI-UBoot"	/* refer to cns3xxx_spi_flash_partitions */
#else
#define ENV_OFFSET 0x0
#define PARTITION_NAME "UBootEnv"
#endif

#define MTD_READ_LEN 1024

static char mtd_str[MTD_READ_LEN], ethaddr[12]={0};

static int init_mtd_env(void)
{
	struct mtd_info *mtd;
	size_t retlen=0;

	mtd = get_mtd_device_nm(PARTITION_NAME);

	if (IS_ERR(mtd)) { return -ENODEV; }
	
	MTD_READ(mtd, ENV_OFFSET, MTD_READ_LEN, &retlen, mtd_str);

	return 0;
}

int fmg_get(const char *name, int *ret_len)
{
	int i,j,x,z;
	int nlen = strlen(name);
	char tmp_str[20];

	memset(ethaddr,0x0,12);

	for(i=0;i<MTD_READ_LEN-nlen;i++) {
		z=0;
		for(x=0;x<nlen;x++){if(mtd_str[i+x]==name[x]){z++;}}
		/* printk("z = %d , nlen = %d\n",z,nlen); */
		if(z==nlen) {
			memcpy(tmp_str,mtd_str+i+nlen,17);
			tmp_str[17] = '\0';
			/* printk("tmp_str = [%s]\n",tmp_str); */
			for(j=0;j<17;j++){if(tmp_str[j]!=':'){sprintf(ethaddr,"%s%c",ethaddr,tmp_str[j]);}}
			*ret_len = strlen(ethaddr);
			return 0;
		}
	}
	return -1;
}

#endif /*CONFIG_CIRRUS_DUAL_MTD_ENV*/

int mac_str_to_int(const char *mac_str, int mac_str_len, u8 *mac_int, int mac_len)
{
	int i=0,j=0;
	char mac_s[3]={0,0,0};

	for (i=0 ; i < mac_str_len ; i+=2) {
		mac_s[0] = mac_str[i];
		mac_s[1] = mac_str[i+1];
		mac_int[j++] = simple_strtol(mac_s, NULL, 16);
	}
	return 0;
}
#endif /* CONFIG_CNS3XXX_ETHADDR_IN_FLASH */

int cns3xxx_send_packet(struct sk_buff *skb, struct net_device *netdev);

static int install_isr_rc = 0;
static int rc_setup_rx_tx = 0; // rc means reference counting.
static struct net_device *intr_netdev;
struct net_device *net_dev_array[NETDEV_SIZE]; 
spinlock_t tx_lock;
spinlock_t rx_lock;

#ifdef CONFIG_FAST_BRIDGE
static u8 fast_bridge_en = 0;
#endif

const int MAX_RX_DESC_SIZE = 512;
const int MAX_TX_DESC_SIZE = 512;
int RX_DESC_SIZE = 128;
int TX_DESC_SIZE = 120;
module_param(RX_DESC_SIZE, int, 0);
module_param(TX_DESC_SIZE, int, 0);

static u8 tx_ring_index = 0; // 0 or 1

#ifdef CNS3XXX_DELAYED_INTERRUPT
static u32 max_pend_int_cnt=MAX_PEND_INT_CNT, max_pend_time=MAX_PEND_TIME;
#endif

#ifdef CONFIG_CNS3XXX_NAPI
struct net_device *napi_dev;
	#ifdef CNS3XXX_DOUBLE_RX_RING
	struct net_device *r1_napi_dev; // ring1 napi dev
	#endif
#endif

// only for debug (proc)
RingInfo g_ring_info; 

int MSG_LEVEL = NORMAL_MSG;

#ifdef CNS3XXX_STATUS_ISR
const char *cns3xxx_gsw_status_tbl[] = {
	"\nMAC0_Q_FULL\n",
	"\nMAC1_Q_FULL\n",
	"\nCPU_Q_FULL\n",
	"\nHNAT_Q_FULL\n",
	"\nMAC2_Q_FULL\n",
	"\nMAC0_Q_EXT_FULL\n",
	"\nGLOBAL_Q_FULL\n",
	"\nBUFFER_FULL\n",
	"\nMIB_COUNTER_TH\n",
	"\n", // 9
	"\nMAC0_INTRUDER\n",
	"\nMAC1_INTRUDER\n",
	"\nCPU_INTRUDER\n",
	"\nMAC2_INTRUDER\n",
	"\nMAC0_STATUS_CHG\n",
	"\nMAC1_STATUS_CHG\n",
	"\nMAC2_STATUS_CHG\n",
	"\nMAC0_NO_LINK_DROP\n",
	"\nMAC1_NO_LINK_DROP\n",
	"\nMAC2_NO_LINK_DROP\n",
	"\nMAC0_RX_ERROR_DROP\n",
	"\nMAC1_RX_ERROR_DROP\n",
	"\nMAC2_RX_ERROR_DROP\n",
	"\nMAC0_NO_DESTINATION_DROP\n",
	"\nMAC1_NO_DESTINATION_DROP\n",
	"\nMAC2_NO_DESTINATION_DROP\n",
	"\nMAC0_RMC_PAUSE_DROP\n",
	"\nMAC1_RMC_PAUSE_DROP\n",
	"\nMAC2_RMC_PAUSE_DROP\n",
	"\nMAC0_LOCAL_DROP\n",
	"\nMAC1_LOCAL_DROP\n",
	"\nMAC2_LOCAL_DROP\n",
};
#endif

#define MIN_PACKET_LEN 14

void cns3xxx_write_pri_mask(u8 pri_mask);

static int cns3xxx_notify_reboot(struct notifier_block *nb, unsigned long event, void *ptr);

static struct notifier_block cns3xxx_notifier_reboot = {
	.notifier_call	= cns3xxx_notify_reboot,
	.next		= NULL,
	.priority	= 0
};

#if defined(CNS3XXX_VLAN_8021Q)
void cns3xxx_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid);
void cns3xxx_vlan_rx_register(struct net_device *dev, struct vlan_group *grp);
#endif

void __take_off_vlan_header(struct sk_buff *skb)
{
	// take off VLAN header
	memmove(skb->data + 4, skb->data, 12);
	skb->data += 4;
	skb->len -= 4; // minus 4 byte vlan tag
}

static struct sk_buff *cns3xxx_alloc_skb(void)
{
	struct sk_buff *skb;
	u32 align_64;

	skb = dev_alloc_skb(MAX_PACKET_LEN + 2 + RX_SDP_ALIGN);

	if (unlikely(!skb)) {
		/* don't show this message for to ease off system loading */
#if 0 
		printk("\n dev_alloc_skb fail!! while allocate RFD ring !!\n");
#endif
		return NULL;
	}
//	pci_dma_sync_single_for_device(NULL, virt_to_phys(skb->data), MAX_PACKET_LEN+2+RX_SDP_ALIGN, PCI_DMA_FROMDEVICE);

	//printk("skb->data: %x\n", skb->data);
	align_64=CPU_CACHE_ALIGN((u32)skb->data);
	//printk("align_64: %x\n", align_64);
	//printk("align_64-(u32)skb->data = %x\n", align_64-(u32)skb->data);
	skb_reserve(skb, align_64-(u32)skb->data);	/* 16 bytes alignment */

#ifndef CNS3XXX_4N
	skb_reserve(skb, NET_IP_ALIGN);	/* 16 bytes alignment */
#endif
	//printk("xxx skb->data: %x\n", skb->data);

	return skb;
}

static int free_rx_skb(RXRing *rx_ring)
{
	int i=0;
	RXBuffer *rx_buffer = rx_ring->head;
        //RXDesc *rx_desc = rx_ring.rx_desc_head_vir_addr;

	for (i=0 ; i < get_rx_ring_size(rx_ring) ; ++i) {
		if (rx_buffer->skb) {
			dma_unmap_single(NULL,
					 rx_buffer->rx_desc->sdp,
					 rx_buffer->rx_desc->sdl,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(rx_buffer->skb);
			rx_buffer->skb=0;
		}
		++rx_buffer;
	}
	return 0;
}

int cns3xxx_setup_all_rx_resources(RXRing *rx_ring, u8 ring_num)
{
	int i=0;
	RXBuffer *rx_buffer = 0;
	RXDesc *rx_desc = 0;
	dma_addr_t dmap;

	/* allocate RXDesc array */
	rx_ring->rx_desc_head_vir_addr = dma_alloc_coherent(NULL, sizeof(RXDesc) * (get_rx_ring_size(rx_ring)), &rx_ring->rx_desc_head_phy_addr, GFP_KERNEL);
	if (!rx_ring->rx_desc_head_vir_addr) {
		printk("rx_ring->rx_desc_head_vir_addr alloc memory fail!\n");
		return -ENOMEM;
	}

	memset(rx_ring->rx_desc_head_vir_addr, 0, sizeof(RXDesc) * get_rx_ring_size(rx_ring));

	/* allocate RXBuffer array */
	rx_ring->head = kmalloc(sizeof(RXBuffer) * get_rx_ring_size(rx_ring), GFP_KERNEL);
	
	if (!rx_ring->head) {
		printk("rx_ring.head alloc memory fail!\n");
		return -ENOMEM;
	}

	rx_buffer = rx_ring->head;
	for (i=0 ; i < get_rx_ring_size(rx_ring) ; ++i) {
		rx_buffer->skb=0;
		++rx_buffer;
	}

	rx_buffer = rx_ring->head;
	rx_desc = rx_ring->rx_desc_head_vir_addr;

	for (i=0 ; i < get_rx_ring_size(rx_ring) ; ++i, ++rx_buffer, ++rx_desc) {
		rx_buffer->rx_desc = rx_desc;
		rx_buffer->skb = cns3xxx_alloc_skb();

		if (!rx_buffer->skb) {
			free_rx_skb(rx_ring);
			kfree(rx_ring->head);
			dma_free_coherent(NULL, sizeof(RXDesc) * get_rx_ring_size(rx_ring), rx_ring->rx_desc_head_vir_addr, rx_ring->rx_desc_head_phy_addr);
			printk("cannot alloc rx skb!!");
			return -ENOMEM;
		}

#ifdef CONFIG_SWITCH_BIG_ENDIAN
	{
		RXDesc tmp_rx_desc;

                memset(&tmp_rx_desc, 0, sizeof(RXDesc));
//                tmp_rx_desc.sdp = (u32)virt_to_phys(rx_buffer->skb->data);
                tmp_rx_desc.sdl = MAX_PACKET_LEN;
                if (i == (get_rx_ring_size(rx_ring)-1) ){
                        tmp_rx_desc.eor = 1;
                }
                tmp_rx_desc.fsd = 1;
                tmp_rx_desc.lsd = 1;
                swap_rx_desc(&tmp_rx_desc, rx_buffer->rx_desc);
	}

#else
//		rx_buffer->rx_desc->sdp = (u32)virt_to_phys(rx_buffer->skb->data);
		rx_buffer->rx_desc->sdl = MAX_PACKET_LEN;
		if (i == (get_rx_ring_size(rx_ring)-1) ){
			rx_buffer->rx_desc->eor = 1;
		}
		rx_buffer->rx_desc->fsd = 1;
		rx_buffer->rx_desc->lsd = 1;
#endif

		dmap = dma_map_single(NULL, rx_buffer->skb->data, MAX_PACKET_LEN, DMA_FROM_DEVICE);
		rx_buffer->rx_desc->sdp = dmap;
	}

	rx_ring->cur_index = 0 ;

	if (ring_num == 0){
		FS_DESC_PTR0_REG = rx_ring->rx_desc_head_phy_addr;
		FS_DESC_BASE_ADDR0_REG = rx_ring->rx_desc_head_phy_addr;
	} else if (ring_num == 1){
		FS_DESC_PTR1_REG = rx_ring->rx_desc_head_phy_addr;
		FS_DESC_BASE_ADDR1_REG = rx_ring->rx_desc_head_phy_addr;
	}

	return CAVM_OK;
}

static int cns3xxx_setup_all_tx_resources(TXRing *tx_ring, u8 ring_num)
{
	int i=0;
	TXBuffer *tx_buffer = 0;
	TXDesc *tx_desc = 0;

	spin_lock_init(&(tx_ring->tx_lock));

	tx_ring->tx_desc_head_vir_addr = dma_alloc_coherent(NULL, sizeof(TXDesc) * get_tx_ring_size(tx_ring), &tx_ring->tx_desc_head_phy_addr, GFP_KERNEL);
	if (!tx_ring->tx_desc_head_vir_addr) {
		printk("tx_ring->tx_desc_head_vir_addr alloc memory fail!\n");
		return -ENOMEM;
	}

	memset(tx_ring->tx_desc_head_vir_addr, 0, sizeof(TXDesc) * get_tx_ring_size(tx_ring));
	tx_ring->head = kmalloc(sizeof(TXBuffer) * get_tx_ring_size(tx_ring), GFP_KERNEL);

	tx_buffer = tx_ring->head;
	tx_desc = tx_ring->tx_desc_head_vir_addr;

	for (i=0 ; i < get_tx_ring_size(tx_ring) ; ++i, ++tx_buffer, ++tx_desc) {
		//printk("%d : ## tx_buffer %p ## tx_desc : %p\n", i, tx_buffer, tx_desc);
		tx_buffer->tx_desc = tx_desc;

		tx_buffer->tx_desc->cown = 1;
		tx_buffer->skb = 0;
		if (i == (get_tx_ring_size(tx_ring)-1) ){
			tx_buffer->tx_desc->eor = 1;
		}
#ifdef CONFIG_SWITCH_BIG_ENDIAN
		swap_tx_desc(tx_buffer->tx_desc, tx_buffer->tx_desc);
#endif

	}
	tx_ring->cur_index = 0 ;

	if (ring_num == 0){
		TS_DESC_PTR0_REG = tx_ring->tx_desc_head_phy_addr;
		TS_DESC_BASE_ADDR0_REG = tx_ring->tx_desc_head_phy_addr;
	} else if (ring_num == 1){
		TS_DESC_PTR1_REG = tx_ring->tx_desc_head_phy_addr;
		TS_DESC_BASE_ADDR1_REG = tx_ring->tx_desc_head_phy_addr;
	}
	return CAVM_OK;
}

int cns3xxx_free_all_rx_resources(RXRing *rx_ring)
{
	free_rx_skb(rx_ring);
	kfree(rx_ring->head);
	dma_free_coherent(NULL, sizeof(RXDesc) * get_rx_ring_size(rx_ring), rx_ring->rx_desc_head_vir_addr, rx_ring->rx_desc_head_phy_addr);
	return 0;
}

static int free_tx_skb(TXRing *tx_ring)
{
	int i=0;
	TXBuffer *tx_buffer = tx_ring->head;

	for (i=0 ; i < get_tx_ring_size(tx_ring) ; ++i) {
		if (tx_buffer->skb) {
			dma_unmap_single(NULL,
					 tx_buffer->tx_desc->sdp,
					 tx_buffer->tx_desc->sdl,
					 DMA_TO_DEVICE);

			dev_kfree_skb(tx_buffer->skb);
			tx_buffer->skb = 0;
		}
		++tx_buffer;
	}
	return 0;
}

int cns3xxx_free_all_tx_resources(TXRing *tx_ring)
{
	free_tx_skb(tx_ring);
	kfree(tx_ring->head);
	dma_free_coherent(NULL, sizeof(TXDesc) * get_tx_ring_size(tx_ring), tx_ring->tx_desc_head_vir_addr, tx_ring->tx_desc_head_phy_addr);
	return 0;
}

#define UDP_OVERRUN_FIXUP

#ifdef UDP_OVERRUN_FIXUP
static struct timer_list fs_dma_timer;
static void fs_dma_enable(unsigned long data)
{
	enable_rx_dma(0, 1);
	mod_timer(&fs_dma_timer, jiffies +1*HZ);
}

static void pse_timer_init(void)
{
	init_timer(&fs_dma_timer);
	fs_dma_timer.function = &fs_dma_enable;
	mod_timer(&fs_dma_timer, jiffies +2*HZ);
}

static void pse_timer_fini(void)
{
	del_timer_sync(&fs_dma_timer);
}
#endif

static void pse_dma_idle(void);
static int cns3xxx_free_rx_tx_res(CNS3XXXPrivate *priv)
{
	--rc_setup_rx_tx;
	if (rc_setup_rx_tx == 0) {
		int i;

		printk("free tx/rx resource\n");
#ifdef UDP_OVERRUN_FIXUP
		pse_timer_fini();
#endif
		enable_port(3, 0); // disable cpu port
		pse_dma_idle();

		for (i=0 ; i < priv->num_rx_queues ; ++i) {
			// stop RX dma
			enable_rx_dma(i, 0);
			cns3xxx_free_all_rx_resources(priv->rx_ring+i);
			memset(priv->rx_ring + i, 0, sizeof(RXRing));
		}

		for (i=0 ; i < priv->num_tx_queues ; ++i) {
			// stop TX dma
			enable_tx_dma(i, 0);
			cns3xxx_free_all_tx_resources(priv->tx_ring+i);
			memset(priv->tx_ring + i, 0, sizeof(TXRing));
		}
		
	}
	return 0;
}

static int cns3xxx_setup_rx_tx_res(CNS3XXXPrivate *priv)
{
	if (rc_setup_rx_tx == 0) {
		int i;

		printk("alloc tx/rx resource\n");
		clear_fs_dma_state(1);
		FS_DESC_PTR0_REG = 0;
		FS_DESC_BASE_ADDR0_REG = 0;
		FS_DESC_PTR1_REG = 0;
		FS_DESC_BASE_ADDR1_REG = 0;
		TS_DESC_PTR0_REG = 0;
		TS_DESC_BASE_ADDR0_REG = 0;
		TS_DESC_PTR1_REG = 0;
		TS_DESC_BASE_ADDR1_REG = 0;

		for (i=0 ; i < priv->num_tx_queues ; ++i) {
			spin_lock_init(&((priv->tx_ring+i)->tx_lock));
			(priv->tx_ring+i)->max_ring_size = MAX_TX_DESC_SIZE;
			(priv->tx_ring+i)->ring_size = TX_DESC_SIZE;
			if (cns3xxx_setup_all_tx_resources(priv->tx_ring+i, i) != CAVM_OK)
				return CAVM_ERR;
		}

		for (i=0 ; i < priv->num_rx_queues ; ++i) {
			(priv->rx_ring+i)->max_ring_size = MAX_RX_DESC_SIZE;
			(priv->rx_ring+i)->ring_size = RX_DESC_SIZE;
			if (cns3xxx_setup_all_rx_resources(priv->rx_ring+i, i) != CAVM_OK)
				return CAVM_ERR;
		
		}
		clear_fs_dma_state(0);
#if 1
		//printk("unsuspending DMA\n");
		DMA_AUTO_POLL_CFG_REG &= (~(1<<4));
		DMA_AUTO_POLL_CFG_REG &= (~(1<<0));
#endif

#ifdef UDP_OVERRUN_FIXUP
		pse_timer_init();
#endif
	}
	++rc_setup_rx_tx;
	return CAVM_OK;
}

static int free_tx_desc_skb(TXRing *tx_ring, u8 ring_num)
{
	int i;
	//u32 tssd_current=0;
	TXBuffer *tx_buffer;
	u32 tx_ring_size = get_tx_ring_size(tx_ring);
	// check curent hw index previous tx descriptor
	u32 cur_index = cns3xxx_get_tx_hw_index(ring_num) - 1; 
	TXDesc *tx_desc_ptr;
#ifdef CONFIG_SWTICH_BIG_ENDIAN
	TXDesc tmp_tx_desc;
#endif
	
	tx_buffer = get_tx_buffer_by_index(tx_ring, cur_index);

	for (i=0 ; i < tx_ring_size ; ++i) {
		//printk("cur_index : %d\n", cur_index);
		if (tx_buffer->tx_desc->cown == 1 && tx_buffer->skb) {
			//printk("tx_buffer: %p ## free skb\n", tx_buffer);
#ifdef CONFIG_SWTICH_BIG_ENDIAN
			tx_desc_ptr = &tmp_tx_desc;
			swap_tx_desc(tx_buffer->tx_desc, tx_desc_ptr);
#else
			tx_desc_ptr = tx_buffer->tx_desc;
#endif
			dma_unmap_single(NULL,
					 tx_desc_ptr->sdp,
					 tx_desc_ptr->sdl,
					 DMA_TO_DEVICE);
			dev_kfree_skb_any(tx_buffer->skb);
			tx_buffer->skb=0;
			//tx_buffer->tx_desc->cown == 1;
		} else {
			//printk("tx_buffer: %p ## don't free skb\n", tx_buffer);
			break;
		}
		//printk("tssd_index: %d\n", tssd_index);
		// --tx_desc_pair_ptr
		--cur_index;
		tx_buffer = get_tx_buffer_by_index(tx_ring, cur_index);
	}
	return 0;
}

void do_arl_lookup(void)
{
}

static void assign_netdev(RXBuffer volatile *rx_buffer)
{
	RXDesc *rx_desc=0;
#ifdef CONFIG_SWITCH_BIG_ENDIAN
	RXDesc tmp_rx_desc;

	rx_desc = &tmp_rx_desc;
	swap_rx_desc(rx_buffer->rx_desc, rx_desc);
#else
	rx_desc = rx_buffer->rx_desc;
#endif

#ifndef CNS3XXX_VLAN_8021Q
	if (is_config_cns3xxx_port_base()) {
#endif
		switch (rx_desc->sp) {
		/* 
		 * sp: the source port of the extracted packet,
		 *	0 - mac port 0
		 *	1 - mac port 1
		 *	4 - mac port 2
		 */
		case 0:
			rx_buffer->skb->dev = PORT0_NETDEV;
			break;
		case 1:
			rx_buffer->skb->dev = PORT1_NETDEV;
			break;
		case 4:
			rx_buffer->skb->dev = PORT2_NETDEV;
			break;
		/*FIXME: need to add exception handler*/

		}
#ifndef CNS3XXX_VLAN_8021Q
	}
#endif
	if (is_config_cns3xxx_vlan_base()) {
		u16 vlan_tag;

		vlan_tag = rx_desc->c_vid;
		rx_buffer->skb->dev = net_dev_array[vlan_tag];
	}

}

#if 0
#if defined(CNS3XXX_VLAN_8021Q)
static int cns3xxx_vlan_rx(CNS3XXXPrivate *priv, struct sk_buff *skb, u16 vlan_tag)
{                       
        return vlan_hwaccel_receive_skb(skb, priv->vlgrp, vlan_tag);
}
#endif
#endif


#if defined (CONFIG_CNS3XXX_SPPE)
#define CNS3XXX_SPPE_PCI_FP_DEBUG
static int sppe_pcie_to_wan_fastpath(struct sk_buff *skb)
{
	SPPE_PARAM param;
	int pci_dev_index;
	struct iphdr *iph;
#if defined (CONFIG_PPPOE)
	struct ethhdr *ethh;

	ethh = (struct ethhdr *)(skb->data);
	if (ntohs(ETH_P_PPP_SES) == ethh->h_proto) {
		/* Remove PPPoE Header */
		memmove(skb->data+8, skb->data, 12); 
		skb->data+=8; 
		skb->len-=8;
		ethh = (struct ethhdr *)(skb->data);
		ethh->h_proto = htons(ETH_P_IP);
	}
#endif

	iph = (struct iphdr *)(skb->data + sizeof(struct ethhdr));

	memset(&param, 0, sizeof(SPPE_PARAM));
	param.cmd = SPPE_CMD_ARP;
	param.op = SPPE_OP_GET;
	param.data.sppe_arp.ip[0] = iph->daddr;

	if (SPPE_RESULT_SUCCESS != sppe_func_hook(&param)) {
#ifdef CNS3XXX_SPPE_PCI_FP_DEBUG
		printk("<%s>read arp fail\n", __FUNCTION__);
#endif
		return (-1);
	} else {
		pci_dev_index = param.data.sppe_arp.unused_1;
	}

	param.cmd = SPPE_CMD_PCI_FP_DEV;
	param.op = SPPE_OP_GET;
	param.data.sppe_pci_fp_dev.dev = NULL;
	param.data.sppe_pci_fp_dev.index = pci_dev_index;

	if (SPPE_RESULT_SUCCESS != sppe_pci_fp_hook(&param)) {
#ifdef CNS3XXX_SPPE_PCI_FP_DEBUG
		printk("<%s>get dev fail\n", __FUNCTION__);
#endif
		return (-1);
	} else {
		skb->dev = param.data.sppe_pci_fp_dev.dev;
	}

	dev_queue_xmit(skb);

return 0;
}
#endif

// old_priv has ring index information, current version only uses the information.
static int cns3xxx_get_rfd_buff(RXDesc *rx_desc, RXBuffer volatile *rx_buffer, CNS3XXXPrivate *old_priv)
{
	CNS3XXXPrivate *priv;
	struct sk_buff *skb;
	u32 len;

	skb = rx_buffer->skb;
	len = rx_desc->sdl;

#ifdef DEBUG_RX
	if (MSG_LEVEL == DUMP_RX_PKT_INFO) {
		printk("rx\n");
		print_packet(skb->data, len);	
	}
#endif

	dma_unmap_single(NULL,
		 rx_desc->sdp,
		 rx_desc->sdl,
		 DMA_FROM_DEVICE);

//	pci_dma_sync_single_for_device(NULL, virt_to_phys(skb->data), len, PCI_DMA_FROMDEVICE);

#if defined (CONFIG_CNS3XXX_SPPE)
	if (PACKET_REASON_TO_CPU == rx_desc->hr) {
		if (sppe_pci_fp_ready) {
			skb_put(skb, len);
			if (0 == sppe_pcie_to_wan_fastpath(skb)) {
				return 0;
			}
		}
		goto freepacket; /* We drop this packet */
	}
#endif

#if defined (CONFIG_CPU_FREQ)
	if (unlikely(atomic_read(&cpufreq_pse_flag)))
		goto freepacket;	
#endif

	if (is_cns3xxx_non_nic_mode_8021q()) {
		if (cns3xxx_is_untag_packet(rx_desc) == 1)
			__take_off_vlan_header(skb);
	}
	
	if (is_config_cns3xxx_port_base()) {
		assign_netdev(rx_buffer);

		if (rx_buffer->skb->dev) // if skb->dev is 0, means VLAN base
			goto determine_dev_ok;
	}

	if (is_config_cns3xxx_vlan_base()) {

		if (is_config_have_vlan_tag()) {

#if defined(CNS3XXX_VLAN_8021Q)
	//printk("rx CNS3XXX_VLAN_8021Q\n");
	// some funcion need netdev like eth_type_trans(), so need to assign it.
			skb->dev = intr_netdev;
	// 8021Q module will determine right netdev by vlan tag.
#else  // defined(CNS3XXX_VLAN_8021Q)
			assign_netdev(rx_buffer);

			__take_off_vlan_header(skb);

#ifdef DEBUG_RX
			if (MSG_LEVEL == 5)
				print_packet(skb->data, 32);
#endif

			if (rx_buffer->skb->dev == 0){
				goto freepacket;
			}
#endif // CNS3XXX_VLAN_8021Q
		}
	}

determine_dev_ok:
	skb_put(skb, len);

	if (skb->dev) {
		priv = netdev_priv(skb->dev);
	} else {
		DEBUG_MSG(WARNING_MSG, "skb_ptr->dev==NULL\n");
		goto freepacket;
	}

#ifdef CNS3XXX_RX_HW_CHECKSUM
	if (priv->rx_en==1) {
	
		switch (rx_desc->prot) {
		case 1:	/* IPv4H5NF & UDP */
		case 2: /* IPv4H5NF & TCP */
		case 5: /* IPv6NENF & UDP */
		case 6:	/* IPv6NENF & TCP */
			if ( rx_desc->l4f == 0 && rx_desc->ipf == 0) { // tcp/udp checksum is correct
				skb->ip_summed = CHECKSUM_UNNECESSARY;
				//printk("CHECKSUM_UNNECESSARY\n");
			} else {
				skb->ip_summed = CHECKSUM_NONE; 
				//printk("CHECKSUM_NONE\n");
			}
			break;
		default:
			skb->ip_summed = CHECKSUM_NONE; 
			//printk("rx_desc->prot: %x\n", rx_desc->prot);
			break;
		}
	} else {
		skb->ip_summed = CHECKSUM_NONE; 
	}
#else
	skb->ip_summed = CHECKSUM_NONE; 
#endif // CNS3XXX_RX_HW_CHECKSUM
	
	skb->dev->last_rx = jiffies;
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += len;

#ifdef CONFIG_FAST_BRIDGE
    /* fast-bridge is for debug purpose
	 * it just receive a packet and send it out here.
	 */
	if (1 == fast_bridge_en) { 
		skb->ip_summed = CHECKSUM_NONE;
		if (skb->dev == PORT0_NETDEV) {
			skb->dev = PORT2_NETDEV;
		} else if (skb->dev == PORT2_NETDEV) {
			skb->dev = PORT0_NETDEV;
		} else {
			goto not_fast_bridge;
		}    
		cns3xxx_send_packet(skb, skb->dev);
		return 0;
	}    
not_fast_bridge:
#endif /* CONFIG_FAST_BRIDGE */

    skb->protocol = eth_type_trans(skb, skb->dev);

#ifdef CONFIG_CNS3XXX_NAPI
	netif_receive_skb(skb);
#else
	netif_rx(skb);
#endif

	//vlan_hwaccel_receive_skb(skb, priv->vlgrp, 1);

	return 0;

freepacket:
	//DEBUG_MSG(NORMAL_MSG, "freepacket\n");
	dev_kfree_skb_any(skb);
	return 0;
}

// index from 1
u32 get_rx_hw_index(CNS3XXXPrivate *priv)
{
	printk("get_rx_head_phy_addr(&RX_RING0(priv): %x\n", get_rx_head_phy_addr(&RX_RING0(priv)) );
	printk("FS_DESC_PTR0_REG: %x\n", FS_DESC_PTR0_REG);

	return ((FS_DESC_PTR0_REG - get_rx_head_phy_addr(&RX_RING0(priv))) / sizeof(RXDesc) );
}

int get_rx_hw_index_by_reg(u8 ring_num)
{
	if (ring_num == 0 ) {
		return ((FS_DESC_PTR0_REG - FS_DESC_BASE_ADDR0_REG) / sizeof(RXDesc) );
	} else if (ring_num == 1 ) {
		return ((FS_DESC_PTR1_REG - FS_DESC_BASE_ADDR1_REG) / sizeof(RXDesc) );
	}

	return CAVM_FAIL;
}

void dump_rxring(void)
{
	int j=0;
	RXBuffer *rx_buffer = 0;

	rx_buffer = get_rx_ring_head(g_ring_info.rx_ring+0);
	for (j=0 ; j < get_rx_ring_size(g_ring_info.rx_ring+0); ++j, ++rx_buffer) {
		printk("[%d] ## rx_buffer->rx_desc->cown: %d\n", j, rx_buffer->rx_desc->cown);
	}
}

#ifdef CONFIG_CNS3XXX_NAPI
void cns3xxx_receive_packet(CNS3XXXPrivate *priv, int mode, int *work_done, int work_to_do)
#else
void cns3xxx_receive_packet(CNS3XXXPrivate *priv, int mode)
#endif
{
	int fssd_index;
	RXBuffer volatile *rx_buffer;
	RXDesc volatile *rx_desc;
	struct sk_buff *skb;
#ifndef CONFIG_CNS3XXX_NAPI
	int fsqf = 0; // Queue Full Mode =0
#endif
	int i, rxcount = 0;
	u8 queue_index = priv->ring_index;
	dma_addr_t dmap;

	RXDesc volatile *rx_desc_ori_ptr;
	volatile RXDesc rx_desc_tmp;

#ifdef CONFIG_SWITCH_BIG_ENDIAN
	RXDesc tmp_rx_desc;
#endif

	rx_buffer = get_cur_rx_buffer(&(priv->rx_ring[queue_index]));

#ifdef CONFIG_SWITCH_BIG_ENDIAN
	rx_desc = &tmp_rx_desc;
	swap_rx_desc(rx_buffer->rx_desc, rx_desc);
#else
	rx_desc = rx_buffer->rx_desc;
#endif

	fssd_index = get_rx_hw_index_by_reg(queue_index);

	if (fssd_index > get_rx_cur_index(&priv->rx_ring[queue_index]) ) {
		rxcount = fssd_index - get_rx_cur_index(&priv->rx_ring[queue_index]);
	} else if (fssd_index < get_rx_cur_index(&priv->rx_ring[queue_index])) {
		rxcount = (get_rx_ring_size(&priv->rx_ring[queue_index]) - get_rx_cur_index(&priv->rx_ring[queue_index]) ) + fssd_index;
	} else { // fssd_index == rxring.cur_index
		if (rx_desc->cown == 0) { // if rx_desc->cown is 1, we can receive the RX descriptor.
			enable_rx_dma(0, 1);

			goto receive_packet_exit;
		} else {
			// Queue Full
#ifndef CONFIG_CNS3XXX_NAPI
			fsqf = 1;
#endif
			rxcount = get_rx_ring_size(&priv->rx_ring[queue_index]);
		}
	}

#ifndef CONFIG_CNS3XXX_NAPI
	if (mode == 1) {
		fsqf = 1;
		rxcount = get_rx_ring_size(&priv->rx_ring[queue_index]);
	}
#endif
	
#ifdef CNS3XXX_FREE_TX_IN_RX_PATH
	free_tx_desc_skb(priv->tx_ring + 0, 0);
#ifdef CNS3XXX_DOUBLE_TX_RING
	free_tx_desc_skb(priv->tx_ring + 1, 1);
#endif
#endif

	for (i = 0; i < rxcount; i++) {
		if (rx_desc->cown != 0) { // start to get packet
#ifdef UDP_OVERRUN_FIXUP
			/* copy descriptor from NCNB to CB  */
			rx_desc_ori_ptr = rx_desc;
			rx_desc = &rx_desc_tmp;
			rx_desc_tmp = *rx_desc_ori_ptr;

			if (rx_buffer->skb) { /* receive packet when rx_buffer->skb is available */
				cns3xxx_get_rfd_buff((RXDesc *)rx_desc, rx_buffer, priv);
				++(*work_done);
			}
	
			/* Alloc New skb_buff */
			skb = cns3xxx_alloc_skb();
			rx_buffer->skb = skb;
			if (likely(skb)) { /* update rx desc if we can get new skb */
				dmap = dma_map_single(NULL, skb->data, MAX_PACKET_LEN, DMA_FROM_DEVICE);
				rx_desc->sdp = dmap;
				rx_desc->sdl = MAX_PACKET_LEN;
				rx_desc->fsd = 1;
				rx_desc->lsd = 1;
				rx_desc->cown = 0; 
			}

			/* write descriptor back to NCNB */
			*rx_desc_ori_ptr = rx_desc_tmp;
			rx_desc = rx_desc_ori_ptr;
#ifdef CONFIG_SWITCH_BIG_ENDIAN
			swap_rx_desc(rx_desc, rx_buffer->rx_desc);
#endif

#ifdef CONFIG_CNS3XXX_NAPI
			if (*work_done >= work_to_do) {
				rx_index_next(&priv->rx_ring[queue_index]); // rx_ring.cur_index points to next
				rx_buffer = get_cur_rx_buffer(&priv->rx_ring[queue_index]);
				rx_desc = rx_buffer->rx_desc;
				break;
			}
#endif
#else /* !UDP_OVERRUN_FIXUP */
			// Alloc New skb_buff 
			skb = cns3xxx_alloc_skb();
			// Check skb_buff
			if (skb) {
				rx_desc_ori_ptr = rx_desc;
				rx_desc = &rx_desc_tmp;
				rx_desc_tmp = *rx_desc_ori_ptr;

				cns3xxx_get_rfd_buff((RXDesc *)rx_desc, rx_buffer, priv);

				rx_buffer->skb = skb;
#ifndef NCNB_TEST
//				rx_desc->sdp = (u32)virt_to_phys(skb->data);
#endif
				dmap = dma_map_single(NULL, skb->data, MAX_PACKET_LEN, DMA_FROM_DEVICE);
				
				rx_desc->sdp = dmap;
				rx_desc->sdl = MAX_PACKET_LEN;
				rx_desc->fsd = 1;
				rx_desc->lsd = 1;

				*rx_desc_ori_ptr = rx_desc_tmp;
				rx_desc = rx_desc_ori_ptr;

				rx_desc->cown = 0; // set cbit to 0 

#ifdef CONFIG_SWITCH_BIG_ENDIAN
				swap_rx_desc(rx_desc, rx_buffer->rx_desc);
#endif

#ifdef CONFIG_CNS3XXX_NAPI
				++(*work_done);
				if (*work_done >= work_to_do) {
					rx_index_next(&priv->rx_ring[queue_index]); // rx_ring.cur_index points to next
					rx_buffer = get_cur_rx_buffer(&priv->rx_ring[queue_index]);
					rx_desc = rx_buffer->rx_desc;
					break;
				}
#endif
			} else {
				// I will add dev->lp.stats->rx_dropped, it will effect the performance
				//PDEBUG("%s: Alloc sk_buff fail, reuse the buffer\n", __FUNCTION__);
				printk("%s: Alloc sk_buff fail, reuse the buffer\n", __FUNCTION__);
				rx_desc->cown = 0; // set cbit to 0 

#ifdef CONFIG_SWITCH_BIG_ENDIAN
				swap_rx_desc(rx_desc, rx_buffer->rx_desc);
#endif
				return;
			}
#endif /* #ifdef UDP_OVERRUN_FIXUP */
		} else { // cown is 0, no packets
			//*work_done = 0;
#ifdef CONFIG_SWITCH_BIG_ENDIAN
			swap_rx_desc(rx_desc, rx_buffer->rx_desc);
#endif
			return;
		}

		rx_index_next(&priv->rx_ring[queue_index]); // rx_ring.cur_index points to next
		rx_buffer = get_cur_rx_buffer(&priv->rx_ring[queue_index]);
		rx_desc = rx_buffer->rx_desc;
	} // end for (i = 0; i < rxcount; i++) 

#ifndef CONFIG_CNS3XXX_NAPI
	if (fsqf) {
		priv->rx_ring[queue_index].cur_index = fssd_index;
		mb();
		enable_rx_dma(0, 1);
	}
#endif

	//spin_unlock(&rx_lock);
receive_packet_exit:
	return;
}

irqreturn_t cns3xxx_fsrc_ring0_isr(int irq, void *dev_id)
{
#ifdef CONFIG_CNS3XXX_NAPI
	CNS3XXXPrivate *priv = netdev_priv(napi_dev);
	priv->ring_index = 0;

#ifdef CNS3XXX_USE_MASK
	cns3xxx_write_pri_mask(0xb0);
#else
	cns3xxx_disable_irq(FSRC_RING0_INTERRUPT_ID);
#endif

	if (likely(napi_schedule_prep(&priv->napi))) {
		__napi_schedule(&priv->napi);
	} else {
#ifdef CNS3XXX_USE_MASK
		cns3xxx_write_pri_mask(0xf0);
#else
		cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);
#endif
	}

#else /* !CONFIG_CNS3XXX_NAPI */
	struct net_device *netdev = dev_id;
	CNS3XXXPrivate *priv = netdev_priv(netdev);

	priv->ring_index = 0;

#ifdef CNS3XXX_USE_MASK
	cns3xxx_write_pri_mask(0xb0);
#else
	cns3xxx_disable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_disable_irq(FSQF_RING0_INTERRUPT_ID);
#endif

	cns3xxx_receive_packet(priv, 0); // Receive Once

#ifdef CNS3XXX_USE_MASK
	cns3xxx_write_pri_mask(0xf0);
#else
	cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_enable_irq(FSQF_RING0_INTERRUPT_ID);
#endif
	enable_rx_dma(0, 1);
#endif /* #ifdef CONFIG_CNS3XXX_NAPI */

	return IRQ_HANDLED;
}


#if defined(CNS3XXX_DOUBLE_RX_RING)
irqreturn_t cns3xxx_fsrc_ring1_isr(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	CNS3XXXPrivate *priv = netdev_priv(netdev);
	priv->ring_index=1;

#if defined(CONFIG_CNS3XXX_NAPI) && defined(CNS3XXX_DOUBLE_RX_RING)
	priv = netdev_priv(r1_napi_dev);
	priv->ring_index = 1;

	cns3xxx_disable_irq(FSRC_RING1_INTERRUPT_ID);

	if (likely(napi_schedule_prep(&priv->napi))) {
		__napi_schedule(&priv->napi);
	} else {
		cns3xxx_enable_irq(FSRC_RING1_INTERRUPT_ID);
	}
#else

	cns3xxx_disable_irq(CNS3XXX_FSRC_RING1_INTERRUPT_ID);
	cns3xxx_disable_irq(CNS3XXX_FSQF_RING1_INTERRUPT_ID);
	cns3xxx_receive_packet(priv, 0); // Receive Once
	enable_rx_dma(1, 1);

	cns3xxx_enable_irq(CNS3XXX_FSRC_RING1_INTERRUPT_ID);
	cns3xxx_enable_irq(CNS3XXX_FSQF_RING1_INTERRUPT_ID);
#endif

	return IRQ_HANDLED;
}
#endif

#ifndef CNS3XXX_SG_DISABLE
static int cns3xxx_check_enough_tx_descriptor(TXRing *tx_ring, int need_free_tx_desc)
{
	int i=0;
	TXDesc *tx_desc=0;
	u32 cur_index = get_tx_cur_index(tx_ring);
	TXBuffer *tx_buffer = get_tx_buffer_by_index(tx_ring, cur_index);

#ifdef CONFIG_SWITCH_BIG_ENDIAN
	TXDesc tmp_tx_desc;
	tx_desc = &tmp_tx_desc;
	swap_tx_desc(tx_buffer->tx_desc, tx_desc);
#else
	tx_desc = tx_buffer->tx_desc;
#endif
     
	for (i=0 ; i < need_free_tx_desc ; ++i) {
		if ( tx_desc->cown == 0 ) {
			return 0; // no free TX descriptor
		}
		tx_buffer = get_tx_buffer_by_index(tx_ring, ++cur_index);
#ifdef CONFIG_SWITCH_BIG_ENDIAN
		tx_desc = &tmp_tx_desc;
		swap_tx_desc(tx_buffer->tx_desc, tx_desc);
#else
		tx_desc = tx_buffer->tx_desc;
#endif
	}
	return 1;
}
#endif

// if return CAVM_ERR, means pad is fail, the packet cannot send by switch.
static int fill_a_skb_to_tx_desc(TXBuffer *tx_buffer, u8 *data, int len, struct sk_buff *skb, const struct CNS3XXXPrivate_ *priv, int sg, int fsd, int lsd)
{
	TXDesc *tx_desc_ptr = 0;
	TXDesc *tx_desc_ori_ptr = 0;
	TXDesc tx_desc_tmp;
	
	dma_addr_t dmap;
  
#ifdef CONFIG_SWTICH_BIG_ENDIAN
	TXDesc tmp_tx_desc;
	tx_desc_ptr = &tmp_tx_desc;
	swap_tx_desc(tx_buffer->tx_desc, tx_desc_ptr);
#else
	tx_desc_ptr = tx_buffer->tx_desc;
#endif

	tx_desc_ori_ptr = tx_desc_ptr;
	tx_desc_ptr = &tx_desc_tmp;
	tx_desc_tmp = *tx_desc_ori_ptr;
	
	if (tx_buffer->skb) {	 
		dma_unmap_single(NULL,
				tx_desc_ptr->sdp,
				tx_desc_ptr->sdl,
				DMA_TO_DEVICE);
		dev_kfree_skb_any(tx_buffer->skb); 
	} 

	tx_buffer->skb = skb;  /* for free skb */ 
	//tx_desc_ptr->sdp = virt_to_phys(data);
	/* FIXME:*/ 
#if 0
	tx_buffer->tx_index = cns3xxx_get_tx_hw_index(0);
#endif

#ifdef CNS3XXX_TX_HW_CHECKSUM 
	tx_desc_ptr->ico = 1; 
	tx_desc_ptr->uco = 1; 
	tx_desc_ptr->tco = 1; 
#else 
	tx_desc_ptr->ico = 0; 
	tx_desc_ptr->uco = 0; 
	tx_desc_ptr->tco = 0; 
#endif 
		// Wake interrupt
#ifdef CNS3XXX_TSTC_RING0_ISR
	tx_desc_ptr->interrupt = 1;
#else
	tx_desc_ptr->interrupt = 0;
#endif

	/* move packet length check to outside of this */
#if 0
	/* fill 0 to MIN_PACKET_LEN size */ 
	// can change MIN_PACKET_LEN to 14
	
	/* FIXME: is this kind of packet possible in real world?
  	 *  perhaps we can drop it directly.
 	 */
	if (sg==0 && len < MIN_PACKET_LEN) { 
		if (skb_padto(skb, MIN_PACKET_LEN)) {
			printk("padding skb error.\n");
			return CAVM_ERR;
		}

		//memset(skb->data + len, 0, MIN_PACKET_LEN - len); 
		//skb->len = MIN_PACKET_LEN;
		tx_desc_ptr->sdl = MIN_PACKET_LEN; 
	} else { 
		tx_desc_ptr->sdl = len; 
	} 
#else
	tx_desc_ptr->sdl = len; 
#endif

	//dma_cache_maint(data, tx_desc_ptr->sdl, PCI_DMA_TODEVICE); 
	dmap = dma_map_single(NULL, data, len, DMA_TO_DEVICE);
	//should check return value.
	//if (unlikely(dma_mapping_error(NULL, dmap))) {
	//	/* Failed to DMA map this SKB, give it back for now */
	//	return NETDEV_TX_BUSY;
	//}
	tx_desc_ptr->sdp = dmap;

	/* VLAN base or port base function to set TX descriptor */ 
	/* reference: tx_//port_base(), tx_vlan_base() */ 
	priv->net_device_priv->tx_func(tx_desc_ptr, priv, skb); 
	tx_desc_ptr->fsd = fsd;
	tx_desc_ptr->lsd = lsd;

	*tx_desc_ori_ptr = tx_desc_tmp;
	tx_desc_ptr = tx_desc_ori_ptr;

#ifndef CNS3XXX_SG_DISABLE
	/* NOT SG packet */
	if (!sg) 
#endif
		tx_desc_ptr->cown = 0;

#ifdef CONFIG_SWITCH_BIG_ENDIAN
	swap_tx_desc(tx_desc_ptr, tx_buffer->tx_desc);
#endif

	return CAVM_OK;
} 

int cns3xxx_send_packet(struct sk_buff *skb, struct net_device *netdev)
{
	CNS3XXXPrivate *priv = netdev_priv(netdev);
	TXBuffer *tx_buffer;
	unsigned long flags;
#ifndef CNS3XXX_SG_DISABLE
	int nr_frags = skb_shinfo(skb)->nr_frags;
#endif
	TXRing *tx_ring;

	spin_lock_irqsave(&tx_lock, flags);

	/* 
	 * 1. PSE min packet length is MIN_PACKET_LEN (14)
	 * 2. packet length < 14 byte is impossible in real world,
	 * following check is just for hardware protection 
 	 */
	if (unlikely(MIN_PACKET_LEN > skb->len)) {
		dev_kfree_skb_any(skb); 
		spin_unlock_irqrestore(&tx_lock, flags);
		return NETDEV_TX_OK;
	}

#if defined (CONFIG_CPU_FREQ)
	if (unlikely(atomic_read(&cpufreq_pse_flag))) {
		spin_unlock_irqrestore(&tx_lock, flags);
		return NETDEV_TX_OK;
	}
#endif

#ifdef DEBUG_TX
	if (MSG_LEVEL == DUMP_TX_PKT_INFO) {
		printk("tx\n");
		print_packet(tx_buffer->skb->data, tx_buffer->tx_desc->sdl);
		//dump_tx_desc(tx_buffer->tx_desc);
	}
#endif

	tx_ring = priv->tx_ring + tx_ring_index;

#ifndef CNS3XXX_SG_DISABLE
	if (cns3xxx_check_enough_tx_descriptor(tx_ring, nr_frags+1) == 0) { 
		// no enough tx descriptor
		spin_unlock_irqrestore(&tx_lock, flags);
		// re-queue the skb
		return NETDEV_TX_BUSY;
	} 
	tx_buffer = get_cur_tx_buffer(tx_ring);
#else
	tx_buffer = get_cur_tx_buffer(tx_ring);

	if (0 == tx_buffer->tx_desc->cown)
		return NETDEV_TX_BUSY;
#endif

#ifndef CNS3XXX_SG_DISABLE
	if (nr_frags == 0) { // non scatter/gather I/O
#endif
		fill_a_skb_to_tx_desc(tx_buffer, skb->data, skb->len, skb, priv, 0, 1, 1);
		tx_index_next(tx_ring);
#ifndef CNS3XXX_SG_DISABLE
	} else { // scatter/gather I/O
		struct skb_frag_struct *frag = 0;
		TXDesc *tx_desc[MAX_SKB_FRAGS+1];
		int tx_desc_count=0;
		int i=0;
	
#ifdef CNS3XXX_SG_DEBUG
		++sg_time;
#endif

		fill_a_skb_to_tx_desc(tx_buffer, skb->data, skb->len - skb->data_len, 0, priv, 1, 1, 0);
		tx_desc[tx_desc_count++] = tx_buffer->tx_desc;
		tx_index_next(tx_ring);
		tx_buffer = get_cur_tx_buffer(tx_ring);

		for (i=0 ; i < nr_frags-1 ; ++i) {
			frag = &skb_shinfo(skb)->frags[i];

			fill_a_skb_to_tx_desc(tx_buffer, page_address(frag->page) + frag->page_offset, frag->size, 0, priv, 1, 0, 0);
			tx_desc[tx_desc_count++] = tx_buffer->tx_desc;

			tx_index_next(tx_ring);
			tx_buffer = get_cur_tx_buffer(tx_ring);
		}
		frag = &skb_shinfo(skb)->frags[nr_frags-1];

		// last fragment
		fill_a_skb_to_tx_desc(tx_buffer, page_address(frag->page) + frag->page_offset, frag->size, skb, priv, 1, 0, 1);
		tx_desc[tx_desc_count++] = tx_buffer->tx_desc;

		tx_index_next(tx_ring);
		tx_buffer = get_cur_tx_buffer(tx_ring);
		
		for (i = 0; i < tx_desc_count ; i++ )
			tx_desc[i]->cown = 0 ; 
	}
#endif
	mb();

	enable_tx_dma(tx_ring_index, 1);

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;
	netdev->trans_start = jiffies;

	spin_unlock_irqrestore(&tx_lock, flags);

	return NETDEV_TX_OK;
}


#ifdef CNS3XXX_FSQF_RING0_ISR
irqreturn_t cns3xxx_fsqf_ring0_isr(int irq, void *dev_id)
{
#ifdef CONFIG_CNS3XXX_NAPI
	CNS3XXXPrivate *priv = netdev_priv(napi_dev);

#ifndef UDP_OVERRUN_FIXUP 
	// because in normal state, fsql only invoke once and set_bit is atomic function.
	// so I don't mask it.
	set_bit(0, &priv->is_qf);
#else
	priv->ring_index=0;

	if (likely(napi_schedule_prep(&priv->napi))) {
		__napi_schedule(&priv->napi);
	}
#endif

#else /* ! CONFIG_CNS3XXX_NAPI */
	struct net_device *netdev = dev_id;
	CNS3XXXPrivate *priv = netdev_priv(netdev);

	/* disable interrupt */
#ifdef CNS3XXX_USE_MASK
	cns3xxx_write_pri_mask(0xb0);
#else
	cns3xxx_disable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_disable_irq(FSQF_RING0_INTERRUPT_ID);
#endif

	cns3xxx_receive_packet(priv, 1); // Receive at Queue Full Mode

	/* enable interrupt */
#ifdef CNS3XXX_USE_MASK
	cns3xxx_write_pri_mask(0xf0);
#else
	cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);
	cns3xxx_enable_irq(FSQF_RING0_INTERRUPT_ID);
#endif

	enable_rx_dma(0, 1);
#endif // CONFIG_CNS3XXX_NAPI

	return IRQ_HANDLED;
}
#endif


#if defined(CNS3XXX_DOUBLE_RX_RING)
#ifdef CNS3XXX_FSQF_RING1_ISR
irqreturn_t cns3xxx_fsqf_ring1_isr(int irq, void *dev_id)
{
#ifdef CONFIG_CNS3XXX_NAPI
	CNS3XXXPrivate *priv = netdev_priv(r1_napi_dev);
	// because in normal state, fsqf only invoke once and set_bit is atomic function.
	// so don't mask it.
	set_bit(0, &priv->is_qf);
#else
	struct net_device *netdev = dev_id;
	CNS3XXXPrivate *priv = netdev_priv(netdev);

	cns3xxx_disable_irq(FSRC_RING1_INTERRUPT_ID);
	cns3xxx_disable_irq(FSQF_RING1_INTERRUPT_ID);

	cns3xxx_receive_packet(priv, 1); // Receive at Queue Full Mode
	enable_rx_dma(1, 1);

	cns3xxx_enable_irq(FSRC_RING1_INTERRUPT_ID);
	cns3xxx_enable_irq(FSQF_RING1_INTERRUPT_ID);
#endif
	return IRQ_HANDLED;
}
#endif
#endif //#if defined(CNS3XXX_DOUBLE_RX_RING)

static void cns3xxx_print_link_info(struct net_device *netdev, u32 regVal)
{
	unsigned int link_speed = 10*((regVal>>2) & 0x3);

	switch (((regVal>>2) & 0x3)){
	case 0:
		link_speed = 10;
		break;
	case 1:
		link_speed = 100;
		break;
	case 2:
		link_speed = 1000;
		break;
	case 3:
	default:
		link_speed = 0; /* Reserved */
	}

	if (regVal & 0x1)
		printk(KERN_INFO "cns3xxx: %s NIC Link is Up %d Mbps %s, "
		       "Flow Control: %s\n",
		       netdev->name,
		       link_speed,
		       ((regVal>>4) & 0x1) ?
		                        "Full Duplex" : "Half Duplex",
		       ((link_speed < 1000) && ((regVal>>5) & 0x1)) ?
		                        "RX/TX" :
		       (((regVal>>5) & 0x1)  ? "RX" :
		       (((regVal>>6) & 0x1)  ? "TX" : "None" )));
	else
		printk(KERN_INFO "cns3xxx: %s NIC Link is Down\n", netdev->name);
}

#ifdef CNS3XXX_STATUS_ISR
irqreturn_t cns3xxx_status_isr(int irq, void *dev_id)
{
	u32 int_status = INTR_STAT_REG;
	u32 i=0;
	CNS3XXXPrivate *priv = 0;
	NetDevicePriv *devPriv = 0;
	struct net_device *netdev = (struct net_device *)dev_id;

	/* In current design, we always get the first defined net device.
	 */
	priv = netdev_priv(netdev);
	devPriv = priv->net_device_priv;

	cns3xxx_disable_irq(STATUS_INTERRUPT_ID);
	//printk("INTR_STAT_REG : %#x\n", INTR_STAT_REG);
#ifdef CNS3XXX_SHOW_LINK_STATUS
	for (i=14 ; i <= 16 ; ++i) {
		u8 mac_cfg[]={0xc, 0x10, 0x18};

		if (((int_status >> i) & 1)) {
			unsigned int devIndex = i - 14;

			netdev = devPriv[devIndex].netdev;
			//printk(KERN_INFO " cns3000: mac%d(%s): Link %s\n\r", i-14, netdev->name, (SWITCH_REG_VALUE(mac_cfg[i-14]) & 1)? "UP" :"DOWN");
			cns3xxx_print_link_info(netdev, SWITCH_REG_VALUE(mac_cfg[i-14]));

			if ((SWITCH_REG_VALUE(mac_cfg[i-14]) & 1)){ // link up

      				// GMII2 high speed drive strength
       				IOCDA_REG &= (~(3 << 10));
				if (((SWITCH_REG_VALUE(mac_cfg[i-14]) >> 2) &0x3) == 2) { // 1000 Mbps
				        IOCDA_REG |= (2 << 10);
				}
				netif_carrier_on(netdev);
			} else {
				netif_carrier_off(netdev);
			}

			//printk("%x", SWITCH_REG_VALUE(mac_cfg[i-14]));

		}
	}
#else
	for (i = 0; i < 32; i++) {
		if (int_status & (1 << i)) {
			PRINT_INFO(cns3xxx_gsw_status_tbl[i]);
		}
	}
#endif
	INTR_STAT_REG = 0xffffffff; // write 1 for clear.
	cns3xxx_enable_irq(STATUS_INTERRUPT_ID);
	return IRQ_HANDLED;
}
#endif


#ifdef CNS3XXX_TSTC_RING0_ISR
irqreturn_t cns3xxx_tstc_ring0_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
#endif


static int cns3xxx_install_isr(struct net_device *dev)
{
	int retval;
	CNS3XXXPrivate *priv = netdev_priv(dev);
	
	if (install_isr_rc == 0) {

		retval = request_irq(FSRC_RING0_INTERRUPT_ID, cns3xxx_fsrc_ring0_isr, IRQF_SHARED, "FSRC_RING0", intr_netdev);

		if (retval) {
			printk("%s: unable to get IRQ %d (irqval=%d).\n", "FSRC_RING0",FSRC_RING0_INTERRUPT_ID, retval);
			return 1;
		}

#ifdef CNS3XXX_FSQF_RING0_ISR
		retval = request_irq(FSQF_RING0_INTERRUPT_ID, cns3xxx_fsqf_ring0_isr, IRQF_SHARED, "FSQF_RING0", intr_netdev);

		if (retval) {
			PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n", "FSQF_RING0", FSQF_RING0_INTERRUPT_ID, retval);
			return 2;
		}
#endif	

#ifdef CNS3XXX_TSTC_RING0_ISR
		retval = request_irq(TSTC_RING0_INTERRUPT_ID, cns3xxx_tstc_ring0_isr, IRQF_SHARED, "TSTC_RING0", intr_netdev);

		if (retval) {
			PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n", "TSTC_RING0", FSQF_RING0_INTERRUPT_ID, retval);
			return 3;
		}
#endif

		if (priv->num_rx_queues == 2) {
#if defined(CNS3XXX_DOUBLE_RX_RING)
			retval = request_irq(FSRC_RING1_INTERRUPT_ID, cns3xxx_fsrc_ring1_isr, IRQF_SHARED, "FSRC_RING1", intr_netdev);
			printk("xx intr_netdev->name: %s\n", intr_netdev->name);

			if (retval) {
				printk("%s: unable to get IRQ %d (irqval=%d).\n", "FSRC_RING1",FSRC_RING1_INTERRUPT_ID, retval);
				return 1;
			}

#ifdef CNS3XXX_FSQF_RING1_ISR
			retval = request_irq(FSQF_RING1_INTERRUPT_ID, cns3xxx_fsqf_ring1_isr, IRQF_SHARED, "FSQF_RING1", intr_netdev);

			if (retval) {
				PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n", "FSQF_RING1", FSQF_RING1_INTERRUPT_ID, retval);
				return 2;
			}
#endif	
#endif
		}

#ifdef CNS3XXX_STATUS_ISR
#ifdef CNS3XXX_SHOW_LINK_STATUS
		INTR_MASK_REG=0xffffffff;
		// only enable mac link status
		INTR_MASK_REG &= (~(1 << 14));
		INTR_MASK_REG &= (~(1 << 15));
		INTR_MASK_REG &= (~(1 << 16));
#endif
		retval = request_irq(STATUS_INTERRUPT_ID, cns3xxx_status_isr, IRQF_SHARED, "GSW_STATUS", intr_netdev);

		if (retval) {
			PRINT_INFO("%s: unable to get IRQ %d (irqval=%d).\n", "GSW STATUS INT", STATUS_INTERRUPT_ID, retval);
			return 3;
		}
		INTR_MASK_REG = 0;
#endif

#ifdef CONFIG_CNS3XXX_NAPI
		priv = netdev_priv(napi_dev);
		napi_enable(&priv->napi);
		netif_start_queue(napi_dev);
#ifdef CNS3XXX_DOUBLE_RX_RING
		priv = netdev_priv(r1_napi_dev);    
		napi_enable(&priv->napi);
		netif_start_queue(r1_napi_dev);
#endif
#endif
		// enable cpu port
		enable_port(3, 1);

	} // end if (install_isr_rc == 0) 

	++install_isr_rc;

	return 0;
}


int cns3xxx_open(struct net_device *dev)
{
	CNS3XXXPrivate *priv = netdev_priv(dev);
	//static int init_state=0;
	
	if (cns3xxx_setup_rx_tx_res(priv) != CAVM_OK) {
		printk("alloc rx/tx ring fail!!\n");	
		return -1;
	}

	netif_start_queue(dev);

	cns3xxx_install_isr(dev);

	enable_rx_dma(0, 1);

	if (priv->num_rx_queues == 2) 
		enable_rx_dma(1, 1);

#if defined(CNS3XXX_STATUS_ISR)&&defined(CNS3XXX_SHOW_LINK_STATUS)
	/* defered the operation for mac2 until link is up
	 * bad look but quick fix
	 */
	if (priv->net_device_priv != &net_device_prive[2])
		netif_carrier_on(dev);

	priv->net_device_priv->open();
#else
	netif_carrier_on(dev);
	priv->net_device_priv->open();
#endif
	return 0;
}

static int cns3xxx_uninstall_isr(struct net_device *dev)
{
	CNS3XXXPrivate *priv = netdev_priv(dev);
	--install_isr_rc;
	if (install_isr_rc == 0) {
		enable_port(3, 0);
		free_irq(FSRC_RING0_INTERRUPT_ID, intr_netdev);
#ifdef CNS3XXX_STATUS_ISR
		free_irq(STATUS_INTERRUPT_ID, intr_netdev);
#endif

#ifdef CNS3XXX_FSQF_RING0_ISR
		free_irq(FSQF_RING0_INTERRUPT_ID, intr_netdev);
#endif

#ifdef CNS3XXX_TSTC_RING0_ISR
		free_irq(TSTC_RING0_INTERRUPT_ID, intr_netdev);
#endif

		if (priv->num_rx_queues == 2) {
			free_irq(FSRC_RING1_INTERRUPT_ID, intr_netdev);
#ifdef CNS3XXX_FSQF_RING1_ISR
			free_irq(FSQF_RING1_INTERRUPT_ID, intr_netdev);
#endif
		}

#ifdef CONFIG_CNS3XXX_NAPI
		priv = netdev_priv(napi_dev);

		napi_disable(&priv->napi);
		netif_stop_queue(napi_dev);
#ifdef CNS3XXX_DOUBLE_RX_RING
		priv = netdev_priv(r1_napi_dev);

		napi_disable(&priv->napi);
		netif_stop_queue(r1_napi_dev);
#endif
#endif
	}

	return 0;
}

int cns3xxx_close(struct net_device *dev)
{
	CNS3XXXPrivate *priv = netdev_priv(dev);

	enable_rx_dma(0, 0);
	enable_tx_dma(0, 0);

	if (priv->num_rx_queues == 2) 
		enable_rx_dma(1, 0);

	if (priv->num_tx_queues == 2) 
		enable_tx_dma(1, 0);

	netif_stop_queue(dev);

	priv->net_device_priv->close();
	cns3xxx_uninstall_isr(dev);
	cns3xxx_free_rx_tx_res(priv);
	netif_carrier_off(dev);

	return 0;
}

//#define MAC_PORT(p) MAC##p##_CFG_REG

void broadcast_storm_cfg(u8 port, u8 boradcast, u8 multicast, u8 unknown)
{
	switch (port)
	{
		case 0:
		{
			(boradcast == 1) ? (MAC0_CFG_REG |= (1 << 30)) : (MAC0_CFG_REG &= (~(1 << 30))) ;
			(multicast == 1) ? (MAC0_CFG_REG |= (1 << 29)) : (MAC0_CFG_REG &= (~(1 << 29))) ;
			(unknown == 1) ? (MAC0_CFG_REG |= (1 << 28)) : (MAC0_CFG_REG &= (~(1 << 28))) ;
			break;
		}
		case 1:
		{
			(boradcast == 1) ? (MAC1_CFG_REG |= (1 << 30)) : (MAC1_CFG_REG &= (~(1 << 30))) ;
			(multicast == 1) ? (MAC1_CFG_REG |= (1 << 29)) : (MAC1_CFG_REG &= (~(1 << 29))) ;
			(unknown == 1) ? (MAC1_CFG_REG |= (1 << 28)) : (MAC1_CFG_REG &= (~(1 << 28))) ;
			break;
		}
		case 2:
		{
			(boradcast == 1) ? (MAC2_CFG_REG |= (1 << 30)) : (MAC2_CFG_REG &= (~(1 << 30))) ;
			(multicast == 1) ? (MAC2_CFG_REG |= (1 << 29)) : (MAC2_CFG_REG &= (~(1 << 29))) ;
			(unknown == 1) ? (MAC2_CFG_REG |= (1 << 28)) : (MAC2_CFG_REG &= (~(1 << 28))) ;
			break;
		}
	}
}

void broadcast_storm_rate(u8 rate)
{
	TC_CTRL_REG &= (~(0xf << 24));
	TC_CTRL_REG |= (rate << 24);
}

static int cns3xxx_set_mac_addr(struct net_device *dev, void *p)
{
	//struct sockaddr *sock_addr = addr;
	CNS3XXXPrivate *priv = netdev_priv(dev);

        struct sockaddr *addr= p;


	spin_lock_irq(&priv->lock);


        if (!is_valid_ether_addr(addr->sa_data))
                return -EADDRNOTAVAIL;

	// 1. delete old arl mac entry
	// 2. add new arl mac entry
	// 3. copy new mac to netdev field

	if (priv->net_device_priv->arl_table_entry) {
		cns3xxx_arl_table_invalid(priv->net_device_priv->arl_table_entry);
		memcpy(priv->net_device_priv->arl_table_entry->mac, addr->sa_data, dev->addr_len);
		//print_arl_table_entry(priv->net_device_priv->arl_table_entry);
		cns3xxx_arl_table_add(priv->net_device_priv->arl_table_entry);
#if defined (CONFIG_CNS3XXX_SPPE)
		/* If IVL, we need additional ARLs for CPU to PPE */
		if (MAC_GLOB_CFG_REG & (0x1 << 7)) {
			ARLTableEntry arl;
			arl = *(priv->net_device_priv->arl_table_entry);
			arl.vid = (MAC2_CPU_PVID_REG & 0xFFF); 
			cns3xxx_arl_table_add(&arl);
		}
#endif

	}
	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	spin_unlock_irq(&priv->lock);
	return 0;
}

#ifdef CONFIG_CNS3XXX_NAPI
static int cns3xxx_poll(struct napi_struct *napi, int budget)
{
	CNS3XXXPrivate *sp = container_of(napi, CNS3XXXPrivate, napi);
	int work_done = 0;
	int work_to_do = budget; // define minima value

	//printk("sp->ring_index: %d\n", sp->ring_index);
	cns3xxx_receive_packet(sp, 0, &work_done, work_to_do);

	budget -= work_done;
	//printk("xx budget: %d ## work_done: %d\n", budget, work_done);
	//printk("=================\n");

    if (work_done) {
		if (test_bit(0, (unsigned long *)&sp->is_qf) == 1){
			//printk("qf\n");
			clear_bit(0, (unsigned long *)&sp->is_qf);
			//printk("sp->ring_index: %d\n", sp->ring_index);
			enable_rx_dma(sp->ring_index, 1);
       	}
    } else {
		//netif_rx_complete(napi_dev, &sp->napi)
		napi_complete(napi);
#ifdef CNS3XXX_USE_MASK
		cns3xxx_write_pri_mask(0xf0);
#else
		if (sp->ring_index == 0)
			cns3xxx_enable_irq(FSRC_RING0_INTERRUPT_ID);
		else
			cns3xxx_enable_irq(FSRC_RING1_INTERRUPT_ID);
#endif
			//printk("exit poll\n");
	}

	return work_done;
}
#endif

static struct net_device_stats *cns3xxx_get_stats(struct net_device *dev)
{
	CNS3XXXPrivate *priv = netdev_priv(dev);

	return &priv->stats;
}

static int cns3xxx_change_mtu(struct net_device *dev, int new_mtu)
{
	if (new_mtu < cns3xxx_min_mtu() || new_mtu > cns3xxx_max_mtu())
		return -EINVAL;

	dev->mtu = new_mtu;

	return 0;
}

static void cns3xxx_timeout(struct net_device *dev)
{
	printk("%s:cns3xxx gsw timeout\n", dev->name);
	//star_gsw_enable(dev);
	netif_wake_queue(dev);
	dev->trans_start = jiffies;
}

int cns3xxx_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);

#ifdef LINUX2631
static const struct net_device_ops cns3xxx_netdev_ops = {
	.ndo_open               = cns3xxx_open,
	.ndo_stop               = cns3xxx_close,
	.ndo_start_xmit         = cns3xxx_send_packet,
	//.ndo_validate_addr      = eth_validate_addr,
	//.ndo_set_multicast_list = cns3xxx_set_multicast_list,
	.ndo_set_mac_address    = cns3xxx_set_mac_addr,
	.ndo_change_mtu         = cns3xxx_change_mtu,
	.ndo_do_ioctl           = cns3xxx_do_ioctl,
	.ndo_tx_timeout         = cns3xxx_timeout,
	.ndo_get_stats		= cns3xxx_get_stats,

#if defined(CNS3XXX_VLAN_8021Q)
	.ndo_vlan_rx_register   = cns3xxx_vlan_rx_register,
	//.ndo_vlan_rx_add_vid    = e1000_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid   = cns3xxx_vlan_rx_kill_vid,
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller    = cns3xxx_netpoll,
#endif
};
#endif // LINUX2631

static int __init cns3xxx_probe(RingInfo ring_info)
{
	void cns3xxx_set_ethtool_ops(struct net_device *netdev);

//	int netdev_size = sizeof(net_device_prive)/sizeof(NetDevicePriv);
	int netdev_size = num_net_dev_priv;
	int i=0, err=0;
	struct net_device *netdev=0;
	CNS3XXXPrivate *priv=0;
	struct sockaddr sock_addr;
#ifdef CONFIG_CNS3XXX_ETHADDR_IN_FLASH
    u8 mac_int[6];
    int val_len;
    int do_new_mac = 0;
    if (0 == init_mtd_env()) {
        do_new_mac = 1;
    }
#endif

	for (i=0 ; i < netdev_size ; ++i) {

		netdev = alloc_etherdev(sizeof(CNS3XXXPrivate));
		if (!netdev) {
			err = -ENOMEM;
			goto err_alloc_etherdev;
		}
		if (net_device_prive[i].name)
			strcpy(netdev->name, net_device_prive[i].name);

		net_device_prive[i].netdev = netdev;
		net_dev_array[net_device_prive[i].vlan_tag] = netdev;
		//printk("\tnet_dev_array[%d]: %x\n", net_device_prive[i].vlan_tag , netdev);
		if (intr_netdev==0)
			intr_netdev = netdev;

		SET_NETDEV_DEV(netdev, NULL);
		priv = netdev_priv(netdev);
		spin_lock_init(&priv->lock);
		memset(priv, 0, sizeof(CNS3XXXPrivate));

		priv->num_rx_queues = ring_info.num_rx_queues;
		priv->num_tx_queues = ring_info.num_tx_queues;
		priv->rx_ring = ring_info.rx_ring;
		priv->tx_ring = ring_info.tx_ring;
		priv->rx_en=1;

		priv->net_device_priv = &net_device_prive[i];
		//printk("net_device_prive[i].pmap : %x\n", net_device_prive[i].pmap);

		// set netdev MAC address
#if defined (CONFIG_CNS3XXX_ETHADDR_IN_FLASH)
		if (do_new_mac) {
			char name[20];
			sprintf(name, "ethaddr%d=", i);
			if (0 == fmg_get(name,&val_len)) {
				mac_str_to_int(ethaddr, val_len,mac_int, 6);
				memcpy(sock_addr.sa_data, mac_int, 6);
			} else {
				memcpy(sock_addr.sa_data, net_device_prive[i].mac, 6);
			}
		} else {
			memcpy(sock_addr.sa_data, net_device_prive[i].mac, 6);
		}
#else
		memcpy(sock_addr.sa_data, net_device_prive[i].mac, 6);
#endif
		cns3xxx_set_mac_addr(netdev, &sock_addr);

#ifdef LINUX2631
		netdev->netdev_ops = &cns3xxx_netdev_ops;
#endif

		cns3xxx_set_ethtool_ops(netdev);
#ifdef LINUX2627
		//netdev->base_addr		= IO_ADDRESS(GSW_BASE_ADDR);
		netdev->base_addr = 0;
		netdev->open = cns3xxx_open;
		netdev->stop = cns3xxx_close;
		netdev->hard_start_xmit	= cns3xxx_send_packet;
		//netdev->hard_start_xmit	= 0;
		netdev->do_ioctl = cns3xxx_do_ioctl;
		netdev->change_mtu = cns3xxx_change_mtu;

		//netdev->get_stats = cns3xxx_get_stats;
		netdev->watchdog_timeo = 5 * HZ; // ref e1000_main.c
		netdev->tx_timeout = cns3xxx_timeout;
		netdev->set_mac_address	= cns3xxx_set_mac_addr;
#endif

#if defined(CNS3XXX_TX_HW_CHECKSUM)
		netdev->features |= NETIF_F_IP_CSUM;
	#ifndef CNS3XXX_SG_DISABLE
		netdev->features |= NETIF_F_SG;
	#endif
#endif

#ifdef CONFIG_CNS3XXX_NAPI
		//printk("napi netdev: %s\n", netdev->name);
		//printk("netdev addr: %p\n", netdev);
		//netif_napi_add(netdev, &priv->napi, cns3xxx_poll, CNS3XXX_NAPI_WEIGHT);
#endif

#if defined(CNS3XXX_VLAN_8021Q)
		// do not let 8021Q module insert vlan tag
		// can use the snippet code to get vlan tage
		// if (priv->vlgrp && vlan_tx_tag_present(skb)) 
		//   vlan_tag = cpu_to_be16(vlan_tx_tag_get(skb));
#ifdef CNS3XXX_8021Q_HW_TX
		// hardware support insert VLAN tag on TX path
		netdev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
#else
		netdev->features |= NETIF_F_HW_VLAN_RX; // remove NETIF_F_HW_VLAN_TX flag that 8021Q module to insert vlan tag.
#endif

	        //netdev->vlan_rx_register = cns3xxx_vlan_rx_register;
		//netdev->vlan_rx_kill_vid = cns3xxx_vlan_rx_kill_vid;
#endif


		err = register_netdev(netdev);
		if (err) {
			printk("Register network dev :%s failed \n", netdev->name);
			goto err_register_netdev;
		}

		netif_carrier_off(netdev);
		netdev = 0;
	} // for (i=0 ; i < netdev_size ; ++i) 

	return 0;


err_register_netdev:
	free_netdev(netdev);

err_alloc_etherdev:
	return err;
}

#if 0
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
#endif

static int cns3xxx_pse_regs_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int num = 0;
	int i=0;

	for (i=0x00 ; i <= 0x84 ; i+=4)
		num += sprintf(page + num, "offset %#x: %#x\n", i, SWITCH_REG_VALUE(i) );
	for (i=0xd0 ; i <= 0xf8 ; i+=4)
		num += sprintf(page + num, "offset %#x: %#x\n", i, SWITCH_REG_VALUE(i) );
	for (i=0x100 ; i <= 0x164 ; i+=4)
		num += sprintf(page + num, "offset %#x: %#x\n", i, SWITCH_REG_VALUE(i) );
	num += sprintf(page + num, "IOCDA_REG:  %#x\n", IOCDA_REG);

	return num;
}

static int cns3xxx_pse_counter_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int num = 0;
#ifdef CNS3XXX_MAC0_MIB_COUNTER
	num += sprintf(page + num, "C_RXOKPKT_MAC0_REG: %d\n", C_RXOKPKT_MAC0_REG);
	num += sprintf(page + num, "C_RXOKBYTE_MAC0_REG  : %d\n", C_RXOKBYTE_MAC0_REG  );
	num += sprintf(page + num, "C_RXRUNT_MAC0_REG  : %d\n", C_RXRUNT_MAC0_REG  );
	num += sprintf(page + num, "C_RXLONG_MAC0_REG  : %d\n", C_RXLONG_MAC0_REG  );
	num += sprintf(page + num, "C_RXDROP_MAC0_REG  : %d\n", C_RXDROP_MAC0_REG  );
	num += sprintf(page + num, "C_RXCRC_MAC0_REG  : %d\n", C_RXCRC_MAC0_REG  );
	num += sprintf(page + num, "C_RXARLDROP_MAC0_REG  : %d\n", C_RXARLDROP_MAC0_REG  );
	num += sprintf(page + num, "C_VIDROP_MAC0_REG  : %d\n", C_VIDROP_MAC0_REG  );
	num += sprintf(page + num, "C_VEDROP_MAC0_REG  : %d\n", C_VEDROP_MAC0_REG  );
	num += sprintf(page + num, "C_RXRL_MAC0_REG  : %d\n", C_RXRL_MAC0_REG  );
	num += sprintf(page + num, "C_RXPAUSE_MAC0_REG  : %d\n", C_RXPAUSE_MAC0_REG  );

	num += sprintf(page + num, "C_TXOKPKT_MAC0_REG  : %d\n", C_TXOKPKT_MAC0_REG  );
	num += sprintf(page + num, "C_TXOKBYTE_MAC0_REG  : %d\n", C_TXOKBYTE_MAC0_REG  );
	num += sprintf(page + num, "C_TXPAUSECOL_MAC0_REG  : %d\n", C_TXPAUSECOL_MAC0_REG  );
#endif

#ifdef CNS3XXX_MAC1_MIB_COUNTER
	num += sprintf(page + num, "C_RXOKPKT_MAC1_REG  : %d\n", C_RXOKPKT_MAC1_REG  );
	num += sprintf(page + num, "C_RXOKBYTE_MAC1_REG  : %d\n", C_RXOKBYTE_MAC1_REG  );
	num += sprintf(page + num, "C_RXRUNT_MAC1_REG  : %d\n", C_RXRUNT_MAC1_REG  );
	num += sprintf(page + num, "C_RXLONG_MAC1_REG  : %d\n", C_RXLONG_MAC1_REG  );
	num += sprintf(page + num, "C_RXDROP_MAC1_REG  : %d\n", C_RXDROP_MAC1_REG  );
	num += sprintf(page + num, "C_RXCRC_MAC1_REG  : %d\n", C_RXCRC_MAC1_REG  );
	num += sprintf(page + num, "C_RXARLDROP_MAC1_REG  : %d\n", C_RXARLDROP_MAC1_REG  );
	num += sprintf(page + num, "C_VIDROP_MAC1_REG  : %d\n", C_VIDROP_MAC1_REG  );
	num += sprintf(page + num, "C_VEDROP_MAC1_REG  : %d\n", C_VEDROP_MAC1_REG  );
	num += sprintf(page + num, "C_RXRL_MAC1_REG  : %d\n", C_RXRL_MAC1_REG  );
	num += sprintf(page + num, "C_RXPAUSE_MAC1_REG  : %d\n", C_RXPAUSE_MAC1_REG  );

	num += sprintf(page + num, "C_TXOKPKT_MAC1_REG  : %d\n", C_TXOKPKT_MAC1_REG  );
	num += sprintf(page + num, "C_TXOKBYTE_MAC1_REG  : %d\n", C_TXOKBYTE_MAC1_REG  );
	num += sprintf(page + num, "C_TXPAUSECOL_MAC1_REG  : %d\n", C_TXPAUSECOL_MAC1_REG  );
#endif

#ifdef CNS3XXX_CPU_MIB_COUNTER
	num += sprintf(page + num, "C_TSOKPKT_CPU_REG  : %d\n", C_TSOKPKT_CPU_REG  );
	num += sprintf(page + num, "C_TSOKBYTE_CPU_REG  : %d\n", C_TSOKBYTE_CPU_REG  );
	num += sprintf(page + num, "C_TSRUNT_CPU_REG  : %d\n", C_TSRUNT_CPU_REG  );
	num += sprintf(page + num, "C_TSLONG_CPU_REG  : %d\n", C_TSLONG_CPU_REG  );
	num += sprintf(page + num, "C_TSNODSTDROP_CPU_REG  : %d\n", C_TSNODSTDROP_CPU_REG  );
	num += sprintf(page + num, "C_TSARLDROP_CPU_REG  : %d\n", C_TSARLDROP_CPU_REG  );
	num += sprintf(page + num, "C_TSVIDROP_CPU_REG  : %d\n", C_TSVIDROP_CPU_REG  );
	num += sprintf(page + num, "C_TSVEDROP_CPU_REG  : %d\n", C_TSVEDROP_CPU_REG  );
	num += sprintf(page + num, "C_TSRL_CPU_REG  : %d\n", C_TSRL_CPU_REG  );

	num += sprintf(page + num, "C_FSOKPKT_CPU_REG  : %d\n", C_FSOKPKT_CPU_REG  );
	num += sprintf(page + num, "C_FSOKBYTE_CPU_REG  : %d\n", C_FSOKBYTE_CPU_REG  );
#endif

#ifdef CNS3XXX_MAC2_MIB_COUNTER
	num += sprintf(page + num, "C_RXOKPKT_MAC2_REG  : %d\n", C_RXOKPKT_MAC2_REG  );
	num += sprintf(page + num, "C_RXOKBYTE_MAC2_REG  : %d\n", C_RXOKBYTE_MAC2_REG  );
	num += sprintf(page + num, "C_RXRUNT_MAC2_REG  : %d\n", C_RXRUNT_MAC2_REG  );
	num += sprintf(page + num, "C_RXLONG_MAC2_REG  : %d\n", C_RXLONG_MAC2_REG  );
	num += sprintf(page + num, "C_RXDROP_MAC2_REG  : %d\n", C_RXDROP_MAC2_REG  );
	num += sprintf(page + num, "C_RXCRC_MAC2_REG  : %d\n", C_RXCRC_MAC2_REG  );
	num += sprintf(page + num, "C_RXARLDROP_MAC2_REG  : %d\n", C_RXARLDROP_MAC2_REG  );
	num += sprintf(page + num, "C_VIDROP_MAC2_REG  : %d\n", C_VIDROP_MAC2_REG  );
	num += sprintf(page + num, "C_VEDROP_MAC2_REG  : %d\n", C_VEDROP_MAC2_REG  );
	num += sprintf(page + num, "C_RXRL_MAC2_REG  : %d\n", C_RXRL_MAC2_REG  );
	num += sprintf(page + num, "C_RXPAUSE_MAC2_REG  : %d\n", C_RXPAUSE_MAC2_REG  );

	num += sprintf(page + num, "C_TXOKPKT_MAC2_REG  : %d\n", C_TXOKPKT_MAC2_REG  );
	num += sprintf(page + num, "C_TXOKBYTE_MAC2_REG  : %d\n", C_TXOKBYTE_MAC2_REG  );
	num += sprintf(page + num, "C_TXPAUSECOL_MAC2_REG  : %d\n", C_TXPAUSECOL_MAC2_REG  );
#endif
	return num;
}

static int cns3xxx_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int num = 0;
	//int i=0;

#if defined(CNS3XXX_VLAN_8021Q)
	num += sprintf(page + num, "CNS3XXX_VLAN_8021Q Enable\n");
#else
	num += sprintf(page + num, "CNS3XXX_VLAN_8021Q Disable\n");
#endif

#ifdef CONFIG_CNS3XXX_NAPI
	num += sprintf(page + num, "NAPI Enable\n");
#else
	num += sprintf(page + num, "NAPI Disable\n");
#endif
	num += sprintf(page + num, "PHY_CTRL_REG: %x \n", PHY_CTRL_REG);
	num += sprintf(page + num, "PHY_AUTO_ADDR_REG: %x \n", PHY_AUTO_ADDR_REG);
	num += sprintf(page + num, "MAC_GLOB_CFG_REG: %x \n", MAC_GLOB_CFG_REG);
	num += sprintf(page + num, "MAC0_CFG_REG: %x\n", MAC0_CFG_REG);
	num += sprintf(page + num, "MAC1_CFG_REG: %x\n", MAC1_CFG_REG);
	num += sprintf(page + num, "MAC2_CFG_REG: %x\n", MAC2_CFG_REG);
	num += sprintf(page + num, "CPU_CFG_REG: %x\n", CPU_CFG_REG);

	num += sprintf(page + num, "FS_DMA_CTRL0_REG : %#x\n", FS_DMA_CTRL0_REG); 
	num += sprintf(page + num, "FS_DESC_PTR0_REG : %#x\n", FS_DESC_PTR0_REG); 
	num += sprintf(page + num, "FS_DESC_BASE_ADDR0_REG : %#x\n", FS_DESC_BASE_ADDR0_REG); 
	num += sprintf(page + num, "TS_DMA_CTRL0_REG : %#x\n", TS_DMA_CTRL0_REG); 
	num += sprintf(page + num, "TS_DESC_PTR0_REG : %#x\n", TS_DESC_PTR0_REG); 
	num += sprintf(page + num, "TS_DESC_BASE_ADDR0_REG : %#x\n", TS_DESC_BASE_ADDR0_REG); 

	//num += sprintf(page + num, "sizeof(RXDesc) : %d\n", sizeof(RXDesc) );
	num += sprintf(page + num, "DELAY_INTR_CFG_REG: %x \n", DELAY_INTR_CFG_REG);
	num += sprintf(page + num, "VLAN_CFG: %x \n", VLAN_CFG);

#ifdef CNS3XXX_PVID_PROC
	num += sprintf(page + num, "MAC1_MAC0_PVID_REG: %x \n", MAC1_MAC0_PVID_REG);
	num += sprintf(page + num, "MAC2_CPU_PVID_REG: %x \n", MAC2_CPU_PVID_REG);
#endif

	num += sprintf(page + num, "FC_GLOB_THRS_REG: %x \n", FC_GLOB_THRS_REG);
	num += sprintf(page + num, "MC_GLOB_THRS_REG: %x \n", MC_GLOB_THRS_REG);

#ifdef CNS3XXX_DOUBLE_RX_RING
	num += sprintf(page + num, "FS_DMA_CTRL1_REG : %#x\n", FS_DMA_CTRL1_REG); 
	num += sprintf(page + num, "FS_DESC_PTR1_REG : %#x\n", FS_DESC_PTR1_REG); 
	num += sprintf(page + num, "FS_DESC_BASE_ADDR1_REG : %#x\n", FS_DESC_BASE_ADDR1_REG); 
#endif

#ifdef CNS3XXX_DOUBLE_TX_RING
	num += sprintf(page + num, "TS_DMA_CTRL1_REG : %#x\n", TS_DMA_CTRL1_REG); 
	num += sprintf(page + num, "TS_DESC_PTR1_REG : %#x\n", TS_DESC_PTR1_REG); 
	num += sprintf(page + num, "TS_DESC_BASE_ADDR1_REG : %#x\n", TS_DESC_BASE_ADDR1_REG); 
#endif

	num += sprintf(page + num, "SRAM_TEST_REG : %#x\n", SRAM_TEST_REG); 
	num += sprintf(page + num, "DMA_RING_CTRL_REG: %x \n", DMA_RING_CTRL_REG);
	num += sprintf(page + num, "TC_CTRL_REG : %#x\n", TC_CTRL_REG); 
	num += sprintf(page + num, "RATE_CTRL_REG : %#x\n", RATE_CTRL_REG); 
	num += sprintf(page + num, "MAC0_PRI_CTRL_REG : %#x\n", MAC0_PRI_CTRL_REG); 
	num += sprintf(page + num, "MAC1_PRI_CTRL_REG : %#x\n", MAC1_PRI_CTRL_REG); 
	num += sprintf(page + num, "MAC2_PRI_CTRL_REG : %#x\n", MAC2_PRI_CTRL_REG); 
	num += sprintf(page + num, "CPU_PRI_CTRL_REG : %#x\n", CPU_PRI_CTRL_REG); 
	num += sprintf(page + num, "HNAT_PRI_CTRL_REG : %#x\n", HNAT_PRI_CTRL_REG); 
	num += sprintf(page + num, "DMA_AUTO_POLL_CFG_REG : %#x\n", DMA_AUTO_POLL_CFG_REG); 

#ifdef CNS3XXX_SARL_PROC
	num += sprintf(page + num, "SARL_CTRL_REG : %#x\n", SARL_CTRL_REG); 
	num += sprintf(page + num, "SARL_OQ_GTH_REG : %#x\n", SARL_OQ_GTH_REG); 
	num += sprintf(page + num, "SARL_OQ_YTH_REG : %#x\n", SARL_OQ_YTH_REG); 
	num += sprintf(page + num, "SARL_OQ_RTH_REG : %#x\n", SARL_OQ_RTH_REG); 
#endif

	num += sprintf(page + num, "g_ring_info.num_rx_queues: %d\n", g_ring_info.num_rx_queues);
	num += sprintf(page + num, "g_ring_info.num_tx_queues: %d\n", g_ring_info.num_tx_queues);
	num += sprintf(page + num, "g_ring_info.rx_ring: %p\n", g_ring_info.rx_ring);
	num += sprintf(page + num, "g_ring_info.tx_ring: %p\n", g_ring_info.tx_ring);

	num += sprintf(page + num, "tx_ring_index: %d\n", tx_ring_index);


//#ifdef CONFIG_CNS3XXX_PORT_BASE
	if(is_config_cns3xxx_port_base()) {
		num += sprintf(page + num, "CONFIG_PORT_BASE\n");
	}
//#endif

//#ifdef  CONFIG_CNS3XXX_VLAN_BASE
	if(is_config_cns3xxx_vlan_base()) {
		num += sprintf(page + num, "CONFIG_VLAN_BASE\n");
	}
//#endif

//#ifdef CONFIG_HAVE_VLAN_TAG
	if(is_config_have_vlan_tag()) {
		num += sprintf(page + num, "CONFIG_HAVE_VLAN_TAG\n");
	}
//#endif

#ifdef DEBUG_PRIO_IPDSCR
{
	u32 addr=0;

	for (addr=0x50 ; addr <= 0x6c; addr+=4)
		num += sprintf(page + num, "addr %#x: %#x\n", addr, SWITCH_REG_VALUE(addr));
}
#endif

#ifdef CNS3XXX_TX_HW_CHECKSUM
	num += sprintf(page + num, "TX HW CHECKSUM Enable\n");
#else
	num += sprintf(page + num, "TX HW CHECKSUM Disable\n");
#endif

#ifdef CNS3XXX_RX_HW_CHECKSUM
	num += sprintf(page + num, "RX HW CHECKSUM Enable\n");
#else
	num += sprintf(page + num, "RX HW CHECKSUM Disable\n");
#endif
	num += sprintf(page + num, "MSG_LEVEL: %#x\n", MSG_LEVEL);
	num += sprintf(page + num, "C_TXOKPKT_MAC0_EXT_REG: %x\n", C_TXOKPKT_MAC0_EXT_REG);
	num += sprintf(page + num, "C_TXOKBYTE_MAC0_EXT_REG: %x\n", C_TXOKBYTE_MAC0_EXT_REG);





#ifdef CONFIG_SWITCH_BIG_ENDIAN
	num += sprintf(page + num, "switch big endian\n");
#else
	num += sprintf(page + num, "switch little endian\n");
#endif

	num += sprintf(page + num, "MAC_GLOB_CFG_EXT_REG: %x\n", MAC_GLOB_CFG_EXT_REG);
	num += sprintf(page + num, "FC_PORT_THRS_REG: %x\n", FC_PORT_THRS_REG);

#if 0
	num += sprintf(page + num, "GPIOB_PIN_EN_REG: %x\n", GPIOB_PIN_EN_REG);
	num += sprintf(page + num, "PLL_HM_PD_CTRL_REG: %x\n", PLL_HM_PD_CTRL_REG);
	num += sprintf(page + num, "CLK_GATE_REG: %x\n", CLK_GATE_REG);
#endif
	num += sprintf(page + num, "SLK_SKEW_CTRL_REG: %x\n", SLK_SKEW_CTRL_REG);
	num += sprintf(page + num, "IOCDA_REG: %x\n", IOCDA_REG);

#if 0	
	{
	u16 phy_data=0;
	u16 phy_addr=0;

	phy_addr=2;
        cns3xxx_read_phy(phy_addr, 0 , &phy_data);
	num += sprintf(page + num, "phy %d ## register 0 : %x\n", phy_addr, phy_data);
        cns3xxx_read_phy(phy_addr, 1 , &phy_data);
	num += sprintf(page + num, "phy %d ## register 1 : %x\n", phy_addr, phy_data);

        cns3xxx_read_phy(phy_addr, 4, &phy_data);
	num += sprintf(page + num, "phy %d ## register 4 : %x\n", phy_addr, phy_data);
        cns3xxx_read_phy(phy_addr, 5 , &phy_data);
	num += sprintf(page + num, "phy %d ## register 5 : %x\n", phy_addr, phy_data);

        cns3xxx_read_phy(phy_addr, 9 , &phy_data);
	num += sprintf(page + num, "phy %d ## register 9 : %x\n", phy_addr, phy_data);

        cns3xxx_read_phy(phy_addr, 10 , &phy_data);
	num += sprintf(page + num, "phy %d ## register 10 : %x\n", phy_addr, phy_data);

        cns3xxx_read_phy(phy_addr, 15 , &phy_data);
	num += sprintf(page + num, "phy %d ## register 15 : %x\n", phy_addr, phy_data);

        cns3xxx_read_phy(phy_addr, 17 , &phy_data);
	num += sprintf(page + num, "phy %d ## register 17 : %x\n", phy_addr, phy_data);
	}
#endif	
	
	if(jumbo_frame) {
		num += sprintf(page + num, "CNS3XXX JUMBO FRAME Support\n");
	} else {
		num += sprintf(page + num, "CNS3XXX JUMBO FRAME Not Support\n");
	}
#ifdef CNS3XXX_SG_DEBUG
	num += sprintf(page + num, "sg_time: %d\n", sg_time);
#endif
	return num;
}

int cns3xxx_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	if (count) {
		if (buffer[0] == 'r' && buffer[1] == 'g') {
			u8 rgmii = simple_strtol(buffer+3, 0, 0);

		        // enable GMII, MII, reverse MII
		        MAC2_CFG_REG &= (~(1 << 15));

			if (rgmii==1)
		        	MAC2_CFG_REG |= (1 << 15);
		}

#ifdef CNS3XXX_CONFIG_CHANGE_TX_RING
		if (buffer[0] == 't') {
			tx_ring_index = buffer[2] - '0';
			printk("tx_ring_index: %d\n", tx_ring_index);
		}
#endif

		if (buffer[0] == 'm') {
			MSG_LEVEL = simple_strtol(buffer+2, 0, 0);
			//dscp = simple_strtol("0x4", 0, 0);
			printk("MSG_LEVEL: %#x\n", MSG_LEVEL);
			//printk("buffer+2: %s\n", buffer+2);
		}
#ifdef CONFIG_FAST_BRIDGE
		if (buffer[0] == 'f') {
			fast_bridge_en = simple_strtol(buffer+2, 0, 0);
			printk("fast_bridge_en: %d\n", fast_bridge_en);
		}
#endif
		if (buffer[0] == 'u') {
			u8 e=0;
			e = simple_strtol(buffer+2, 0, 0);
			MAC_GLOB_CFG_REG &= (~(1 << 25));
			if (e == 1)
				MAC_GLOB_CFG_REG |= (1 << 25);
			
		}
		if (buffer[0] == 'n') {
			cns3xxx_nic_mode(0);

			// NIC mode on
			if (buffer[2] == '1') 
				cns3xxx_nic_mode(1);
			
		}
		if (buffer[0] == 'i') {
			u32 intr_mask=0;

			intr_mask = simple_strtol(buffer+2, 0, 0);
			INTR_MASK_REG = intr_mask;
		}
		if (buffer[0] == 'p') {
			CPU_CFG_REG &= (~(1 << 30));

			// 4N mode
			if (buffer[2] == '1') 
				CPU_CFG_REG |= (1 << 30);
		}

		if (buffer[0] == 'x') { 
			if (buffer[2] == '1') 
				enable_tx_dma(0, 1);
			if (buffer[2] == '0') 
				enable_tx_dma(0, 0);
		}
		if (buffer[0] == 'g') { 
			MAC2_CFG_REG &= (~(1<<16));
			if (buffer[2] == '1') 
				MAC2_CFG_REG |= (1<<16);
		}

	}
	return count;
}

static void *dumpTXstart(struct seq_file *m, loff_t *pos);
static void *dumpTXnext(struct seq_file *m, void *v, loff_t *pos);
static void dumpTXstop(struct seq_file *m, void *v);
static int dumpTXshow(struct seq_file *m, void *v);

static void *dumpTXstart(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *dumpTXnext(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void dumpTXstop(struct seq_file *m, void *v)
{
}

static int dumpTXshow(struct seq_file *m, void *v)
{
	int i, j;
	TXBuffer *tx_buffer;
	TXDesc *tx_desc;

	seq_printf(m, "g_ring_info.num_tx_queues: %d\n", g_ring_info.num_tx_queues);
	if (g_ring_info.tx_ring) {
		for (i=0 ; i < g_ring_info.num_tx_queues ; ++i) {
			for (j=0 ; j < get_tx_ring_size(g_ring_info.tx_ring+i); ++j) {
				tx_buffer = get_tx_ring_head(g_ring_info.tx_ring+i);
				tx_buffer += j;
				tx_desc = tx_buffer->tx_desc;
				seq_printf(m,"idx: %d ", j);
				if (tx_desc) {
					seq_printf(m,"tx_desc: %p ", tx_desc);
					seq_printf(m,"sdp: %x ", tx_desc->sdp);
					seq_printf(m,"sdl: %d ", tx_desc->sdl);
					seq_printf(m,"tco: %d ", tx_desc->tco);
					seq_printf(m,"uco: %d ", tx_desc->uco);
					seq_printf(m,"ico: %d ", tx_desc->ico);
					seq_printf(m,"pri: %d ", tx_desc->pri);
					seq_printf(m,"fp: %d ", tx_desc->fp);
					seq_printf(m,"fr: %d ", tx_desc->fr);
					seq_printf(m,"intr: %d ", tx_desc->interrupt);
					seq_printf(m,"lsd: %d ", tx_desc->lsd);
					seq_printf(m,"fsd: %d ", tx_desc->fsd);
					seq_printf(m,"eor: %d ", tx_desc->eor);
					seq_printf(m,"cown: %d\n", tx_desc->cown);

					seq_printf(m,"ctv: %d ", tx_desc->ctv);
					seq_printf(m,"stv: %d ", tx_desc->stv);
					seq_printf(m,"sid: %d ", tx_desc->sid);
					seq_printf(m,"inss: %d ", tx_desc->inss);
					seq_printf(m,"dels: %d ", tx_desc->dels);
					seq_printf(m,"pmap: %d ", tx_desc->pmap);
					seq_printf(m,"mark: %d ", tx_desc->mark);
					seq_printf(m,"ewan: %d ", tx_desc->ewan);
					seq_printf(m,"fewan: %d\n", tx_desc->fewan);
					
					seq_printf(m,"c_vid: %d ", tx_desc->c_vid);
					seq_printf(m,"c_cfs: %d ", tx_desc->c_cfs);
					seq_printf(m,"c_pri: %d ", tx_desc->c_pri);
					seq_printf(m,"s_vid: %d ", tx_desc->s_vid);
					seq_printf(m,"s_dei: %d ", tx_desc->s_dei);
					seq_printf(m,"s_pri: %d\n", tx_desc->s_pri);
				}
			} // for (j=0 ; j < get_tx_ring_size(g_ring_info.tx_ring+i); ++j) 
		} // for (i=0 ; i < g_ring_info.num_tx_queues ; ++i) 
	} // if (g_ring_info.tx_ring) 
	
	return 0;
}

const struct seq_operations dumpTX_ops = {
	.start  = dumpTXstart,
	.next = dumpTXnext,
	.stop = dumpTXstop,
	.show = dumpTXshow
};

static int dumpTX_ops_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &dumpTX_ops);
}


static const struct file_operations dumpTX_fs_ops= { 
	.open           = dumpTX_ops_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};


static void *dumpRXstart(struct seq_file *m, loff_t *pos);
static void *dumpRXnext(struct seq_file *m, void *v, loff_t *pos);
static void dumpRXstop(struct seq_file *m, void *v);
static int dumpRXshow(struct seq_file *m, void *v);

const struct seq_operations dumpRX_ops = {
	.start  = dumpRXstart,
	.next = dumpRXnext,
	.stop = dumpRXstop,
	.show = dumpRXshow
};

static int dumpRX_ops_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &dumpRX_ops);
}

static const struct file_operations dumpRX_fs_ops= {
	.open           = dumpRX_ops_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};


static void *dumpRXstart(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *dumpRXnext(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void dumpRXstop(struct seq_file *m, void *v)
{
}

static int dumpRXshow(struct seq_file *m, void *v)
{
	int i, j;
	RXBuffer *rx_buffer;
	RXDesc *rx_desc;

	seq_printf(m, "g_ring_info.num_rx_queues: %d\n", g_ring_info.num_rx_queues);
	if (g_ring_info.rx_ring) {
		for (i=0 ; i < g_ring_info.num_rx_queues ; ++i) {
			for (j=0 ; j < get_rx_ring_size(g_ring_info.rx_ring+i); ++j) {
				rx_buffer = get_rx_ring_head(g_ring_info.rx_ring+i);
				rx_buffer += j;
				rx_desc = rx_buffer->rx_desc;
				seq_printf(m,"idx: %d ", j);
				if (rx_desc) {
					seq_printf(m,"rx_desc: %p ", rx_desc);
					seq_printf(m,"sdp: %x ", rx_desc->sdp);
					seq_printf(m,"sdl: %d ", rx_desc->sdl);
					seq_printf(m,"l4f: %d ", rx_desc->l4f);
					seq_printf(m,"ipf: %d ", rx_desc->ipf);
					seq_printf(m,"prot: %d ", rx_desc->prot);
					seq_printf(m,"hr: %d ", rx_desc->hr);
					seq_printf(m,"lsd: %d ", rx_desc->lsd);
					seq_printf(m,"fsd: %d ", rx_desc->fsd);
					seq_printf(m,"eor: %d ", rx_desc->eor);
					seq_printf(m,"cown: %d\n", rx_desc->cown);

					seq_printf(m,"ctv: %d ", rx_desc->ctv);
					seq_printf(m,"stv: %d ", rx_desc->stv);
					seq_printf(m,"unv: %d ", rx_desc->unv);
					seq_printf(m,"iwan: %d ", rx_desc->iwan);
					seq_printf(m,"exdv: %d ", rx_desc->exdv);
					seq_printf(m,"sp: %d ", rx_desc->sp);
					seq_printf(m,"crc_err: %d ", rx_desc->crc_err);
					seq_printf(m,"un_eth: %d ", rx_desc->un_eth);
					seq_printf(m,"tc: %d ", rx_desc->tc);
					seq_printf(m,"ip_offset: %d\n", rx_desc->ip_offset);

					seq_printf(m,"c_vid: %d ", rx_desc->c_vid);
					seq_printf(m,"c_cfs: %d ", rx_desc->c_cfs);
					seq_printf(m,"c_pri: %d ", rx_desc->c_pri);
					seq_printf(m,"s_vid: %d ", rx_desc->s_vid);
					seq_printf(m,"s_dei: %d ", rx_desc->s_dei);
					seq_printf(m,"s_pri: %d\n", rx_desc->s_pri);
				}
			} // for (j=0 ; j < get_rx_ring_size(g_ring_info.rx_ring+i); ++j) 
		} // for (i=0 ; i < g_ring_info.num_rx_queues ; ++i) 
	} // if (g_ring_info.rx_ring) 
	
	return 0;
}

static int cns3xxx_proc_remove(void)
{
	remove_proc_entry("pse", cns3xxx_proc_dir);
	remove_proc_entry("pse_regs", cns3xxx_proc_dir);
	remove_proc_entry("pse_counter", cns3xxx_proc_dir);
	remove_proc_entry("pse_tx_desc", cns3xxx_proc_dir);
	remove_proc_entry("pse_rx_desc", cns3xxx_proc_dir);

	return 0;
}

static int __init cns3xxx_proc_init(void)
{
	struct proc_dir_entry *switch_proc_entry=0;
		
	switch_proc_entry = create_proc_entry("pse", S_IFREG | S_IRUGO, cns3xxx_proc_dir);
	if (switch_proc_entry) {
		switch_proc_entry->read_proc = cns3xxx_read_proc;
		switch_proc_entry->write_proc = cns3xxx_write_proc;
	}

	switch_proc_entry = create_proc_entry("pse_counter", S_IFREG | S_IRUGO, cns3xxx_proc_dir);
	if (switch_proc_entry) {
		switch_proc_entry->read_proc = cns3xxx_pse_counter_read_proc;
	}

	switch_proc_entry = create_proc_entry("pse_regs", S_IFREG | S_IRUGO, cns3xxx_proc_dir);
	if (switch_proc_entry) {
		switch_proc_entry->read_proc = cns3xxx_pse_regs_read_proc;
	}

	proc_create("cns3xxx/pse_tx_desc", 0, NULL, &dumpTX_fs_ops);
	proc_create("cns3xxx/pse_rx_desc", 0, NULL, &dumpRX_fs_ops);

	return 1;
}

static int cns3xxx_notify_reboot(struct notifier_block *nb, unsigned long event, void *ptr)
{
	// stop the DMA
	enable_rx_dma(0, 0);
	enable_tx_dma(0, 0);
	enable_rx_dma(1, 0);
	enable_tx_dma(1, 0);

	// disable Port 0
	enable_port(0, 0); 
	enable_port(1, 0); 
	enable_port(2, 0); 
	enable_port(3, 0); 

	// disable phy auto-poll
	PHY_AUTO_ADDR_REG &= ~((1<<5) | (1<<13) | (1<<21));
	//printk("disable phy auto-poll\r\n");
	// wait state machine idle
	mdelay(1000);
	//printk("wait 1 sec end\r\n");
	
	return NOTIFY_DONE;
}

#ifdef CONFIG_CNS3XXX_NAPI
static struct net_device *init_napi_dev(struct net_device *ndev, const RingInfo *ring_info)
{
	CNS3XXXPrivate *priv;

	ndev = alloc_etherdev(sizeof(CNS3XXXPrivate));
	if (!ndev) {
		printk("Cannot allocate NAPI virtual device \n");
		BUG();
	}
	priv = netdev_priv(ndev);
	memset(priv, 0, sizeof(CNS3XXXPrivate));

	//priv = netdev_priv(napi_dev);
	priv->num_rx_queues = ring_info->num_rx_queues;
	priv->num_tx_queues = ring_info->num_tx_queues;
	priv->rx_ring = ring_info->rx_ring;
	priv->tx_ring = ring_info->tx_ring;
	//priv->is_qf=0; // because of memset, so need not the line

	//printk("ndev netdev: %s\n", ndev->name);
	//printk("ndev addr: %p\n", ndev);
	netif_napi_add(ndev, &priv->napi , cns3xxx_poll, CNS3XXX_NAPI_WEIGHT);
	dev_hold(ndev);
	set_bit(__LINK_STATE_START, &ndev->state);

	return ndev;
}
#endif

void cns3xxx_config_intr(void)
{
	void set_interrupt_type(u32 id, u32 type);
	void get_interrupt_type(u32 id, u32 *type);

	void set_interrupt_pri(u32 id, u32 pri);
	void get_interrupt_pri(u32 id, u32 *pri);

	u32 v=0xffffffff;

	get_interrupt_type(FSRC_RING0_INTERRUPT_ID, &v);
	set_interrupt_type(FSRC_RING0_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSRC_RING0_INTERRUPT_ID, &v);

	get_interrupt_type(FSRC_RING1_INTERRUPT_ID, &v);
	set_interrupt_type(FSRC_RING1_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSRC_RING1_INTERRUPT_ID, &v);

	get_interrupt_type(FSQF_RING0_INTERRUPT_ID, &v);
	set_interrupt_type(FSQF_RING0_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSQF_RING0_INTERRUPT_ID, &v);

	get_interrupt_type(FSQF_RING1_INTERRUPT_ID, &v);
	set_interrupt_type(FSQF_RING1_INTERRUPT_ID, RISING_EDGE);
	get_interrupt_type(FSQF_RING1_INTERRUPT_ID, &v);

#ifdef CNS3XXX_USE_MASK
	get_interrupt_pri(FSRC_RING0_INTERRUPT_ID, &v);
	set_interrupt_pri(FSRC_RING0_INTERRUPT_ID, 0xc);
	get_interrupt_pri(FSRC_RING0_INTERRUPT_ID, &v);

	get_interrupt_pri(FSRC_RING1_INTERRUPT_ID, &v);
	set_interrupt_pri(FSRC_RING1_INTERRUPT_ID, 0xc);
	get_interrupt_pri(FSRC_RING1_INTERRUPT_ID, &v);

	get_interrupt_pri(FSQF_RING1_INTERRUPT_ID, &v);
	set_interrupt_pri(FSQF_RING1_INTERRUPT_ID, 0xc);
	get_interrupt_pri(FSQF_RING1_INTERRUPT_ID, &v);

	#ifndef CONFIG_CNS3XXX_NAPI
	set_interrupt_pri(FSQF_RING0_INTERRUPT_ID, 0xc);
	#endif
#endif // CNS3XXX_USE_MASK

}

#define PSE_DEFAULT_FREE_PAGE_COUNT	0x2F6
static inline void empty_rx(void)
{
	/* FIXME: 
	 * this function recevie all packets (in PSE memory) and drop them.
	 * If SRAM_TEST_REG[9:0] is 0x2F6, we think that PSE is idle
	 */
    struct net_device *netdev = PORT0_NETDEV;
    int work_done;
#ifdef DEBUG
	int cnt = 0;
#endif
	
    CNS3XXXPrivate *priv = netdev_priv(netdev);
	while ((SRAM_TEST_REG &0x3FF) < PSE_DEFAULT_FREE_PAGE_COUNT) {
		work_done = 0;
#ifdef CONFIG_CNS3XXX_NAPI
	    cns3xxx_receive_packet(priv, 0, &work_done, 16);
#else
	    cns3xxx_receive_packet(priv, 0);
#endif
	/*XXX: for debug */
	#if 0 
		cnt++;
		if (cnt>100) {
			printk("%s: count %d, SRAM_TEST_REG: 0x%.8x\n", __FUNCTION__, cnt, SRAM_TEST_REG);
			printk("MAC0_CFG_REG 0x%.8x MAC1_CFG_REG 0x%.8x MAC2_CFG_REG 0x%.8x\n", MAC0_CFG_REG, MAC1_CFG_REG, MAC2_CFG_REG);
			break;
		}
	#endif
	}
}

// Waiting for DMA Status to IDLE.
static void pse_dma_idle(void)
{
	//printk("Suspending DMA\n");
	DMA_AUTO_POLL_CFG_REG |= (1<<4);
	DMA_AUTO_POLL_CFG_REG |= (1<<0);
	// receive all fs ring packets
	empty_rx();
	
	//printk("Waiting for DMA Status not IDLE\n");
	//printk("\n %x %x\n",TS_DMA_STA_REG,FS_DMA_STA_REG);
	while(TS_DMA_STA_REG );
	while(1) {
	    if(FS_DMA_STA_REG==0) {
	            break;
	    }
	    else if(FS_DMA_STA_REG==2 && FS_DMA_CTRL0_REG==0) {
	        if(FS_DMA_STA_REG==2 && FS_DMA_CTRL0_REG==0) //check twice
	            break;
	    }
	};
	
	//clear fs dma
	DMA_RING_CTRL_REG |= (1 << 31);
	while(((DMA_RING_CTRL_REG >> 30) & 1) == 0);
	DMA_RING_CTRL_REG &= (~(1 << 31));
	//printk("DMA Status is IDLE\n");
}

static int is_tx_empty(TXRing *tx_ring)
{
	u32 cur_index, ring_size = get_tx_ring_size(tx_ring);
	TXDesc *tx_desc;

	for (cur_index=0; cur_index<ring_size; cur_index++) {
		tx_desc = (get_tx_buffer_by_index(tx_ring, cur_index))->tx_desc;

		if (0 == tx_desc->cown) {
			return 0;
		}
	}
	return 1;	
}

static inline int pse_suspend(void)
{
    struct net_device *netdev = PORT0_NETDEV;
    CNS3XXXPrivate *priv = netdev_priv(netdev);

#ifdef CONFIG_CPU_FREQ
	atomic_set(&cpufreq_pse_flag, 1);
#endif
	/* 1. suspend tx dma and disable TX DMA */
	while(!is_tx_empty(priv->tx_ring));
	while (TS_DMA_STA_REG); /* wait for TX DMA complete */
	DMA_AUTO_POLL_CFG_REG |= (0x1<<4);
	enable_tx_dma(0, 0);
	enable_tx_dma(1, 0);
	/* 2. disable mac port */
	enable_port(0, 0);
	enable_port(1, 0);
	enable_port(2, 0);
	enable_port(3, 0);
 	/* 3. empty PSE memory */
	empty_rx();
 	/* 4. suspend rx dma and disable RX DMA */
	while (FS_DMA_STA_REG); /* wait for RX DMA complete */
	DMA_AUTO_POLL_CFG_REG |= 0x1;
	enable_rx_dma(0, 0);
	enable_rx_dma(1, 0);
	INTR_MASK_REG=0xffffffff;

    return NOTIFY_DONE;
}

static inline int pse_resume(void)
{
	/*  2. resume tx dma */
	while (TS_DMA_STA_REG);
	DMA_AUTO_POLL_CFG_REG &= ~(0x1<<4);
	enable_tx_dma(0, 1);
	enable_tx_dma(1, 1);
	/*  3. resume rx dma */
	while (FS_DMA_STA_REG);
	DMA_AUTO_POLL_CFG_REG &= ~ 0x1;
	enable_rx_dma(0, 1);
	enable_rx_dma(1, 1);
	/*  1. enable mac port */
	enable_port(0, 1);
	enable_port(1, 1);
	enable_port(2, 1);
	enable_port(3, 1);
	INTR_MASK_REG=0x0;
#ifdef CONFIG_CPU_FREQ
	atomic_set(&cpufreq_pse_flag, 0);
#endif
    return NOTIFY_DONE;
}

#ifdef CONFIG_CNS3XXX_PSE_WOL
static int pse_wol_suspend(void)
{
	INTR_STAT_REG = INTR_STAT_REG; /* clear interrupt status */
	INTR_MASK_REG = 0xE3FFFFFF; /* Only enable pause frame drop interrupt */
	enable_port(3, 0); /* disable CPU port */
    return NOTIFY_DONE;
}

static int pse_wol_resume(void)
{
	INTR_STAT_REG = INTR_STAT_REG; /* clear interrupt status*/
	INTR_MASK_REG = 0x0; /* */
	enable_port(3, 1); /* enable CPU port */
	
    return NOTIFY_DONE;
}
#endif

#ifdef CONFIG_CPU_FREQ
static int pse_cpufreq_notifier(struct notifier_block *nb, unsigned long phase, void *p)
{
	if (unlikely(!rc_setup_rx_tx)) {
		return NOTIFY_DONE;
	}

	switch (phase) {
	case CPUFREQ_POSTCHANGE:
		pse_resume();
		break;
	case CPUFREQ_PRECHANGE:
		pse_suspend();
		break;
	default:
		break; 
	}
	return NOTIFY_DONE;
}

static struct notifier_block pse_cpufreq_notifier_block = {
	.notifier_call = pse_cpufreq_notifier
};
#endif /* #ifdef CONFIG_CPU_FREQ */

#ifdef CONFIG_SUSPEND
/*
 * This function is called for suspend and resume.
 */
static int pse_power_event(struct notifier_block *this, unsigned long event,
                 void *ptr)
{
	if (unlikely(!rc_setup_rx_tx)) {
		return NOTIFY_DONE;
	}

    switch (event) {
    case PM_POST_SUSPEND:
#ifdef CONFIG_CNS3XXX_PSE_WOL
		return pse_wol_resume();
#endif
    case PM_POST_HIBERNATION:
        return pse_resume();
    case PM_SUSPEND_PREPARE:
#ifdef CONFIG_CNS3XXX_PSE_WOL
		return pse_wol_suspend();
#endif
    case PM_HIBERNATION_PREPARE:
        return pse_suspend();
    default:
        return NOTIFY_DONE;
    }
}

static struct notifier_block pse_power_notifier = {
    .notifier_call = pse_power_event,
};
#endif

static int __init cns3xxx_init_module(void)
{
	// when tx_ring/rx_ring alloc memory, 
	// don't free them until cns3xxx_exit_module
	
	RingInfo ring_info; 
	int i=0;
	u32 reg_config = 0;

#ifdef CNS3XXX_DOUBLE_RX_RING
	ring_info.num_rx_queues = 2;
#else
	ring_info.num_rx_queues = 1;
#endif

#ifdef CNS3XXX_DOUBLE_TX_RING
	ring_info.num_tx_queues = 2;
#else
	ring_info.num_tx_queues = 1;
#endif

	ring_info.rx_ring = kcalloc(ring_info.num_rx_queues, sizeof(RXRing), GFP_KERNEL);
	if (!ring_info.rx_ring)
		return -ENOMEM;
	
	//printk("ring_info.rx_ring: %p\n", ring_info.rx_ring);
	for (i=0 ; i < ring_info.num_rx_queues ; ++i) {
		memset(ring_info.rx_ring + i, 0, sizeof(RXRing));
	}

	ring_info.tx_ring = kcalloc(ring_info.num_tx_queues, sizeof(TXRing), GFP_KERNEL);

	if (!ring_info.tx_ring)
		return -ENOMEM;
	//printk("ring_info.tx_ring: %p\n", ring_info.tx_ring);

	for (i=0 ; i < ring_info.num_tx_queues ; ++i) {
		memset(ring_info.tx_ring + i, 0, sizeof(TXRing));
	}

	g_ring_info = ring_info;

	cns3xxx_gsw_up_init();

	/* jumbo frame configuration */
	reg_config = PHY_AUTO_ADDR_REG;
	reg_config &= ~(3 << 30);

	if (jumbo_frame) {
		reg_config |= (3 << 30); // maximum frame length: 9600 bytes
		MAX_PACKET_LEN = 9600;
	} else {
		reg_config |= (2 << 30); // maximum frame length: 1536 bytes
		MAX_PACKET_LEN = 1536;
	}

	PHY_AUTO_ADDR_REG = reg_config;
  
	printk("jumbo_frame %s\n", jumbo_frame?"on":"off");
	printk("MAX_PACKET_LEN:%d\n", MAX_PACKET_LEN);

	cns3xxx_probe(ring_info);
	cns3xxx_config_intr();

#ifdef CNS3XXX_VLAN_8021Q
	if (is_cns3xxx_nic_mode_8021q()) {
		cns3xxx_nic_mode(1);
	}
#endif
	spin_lock_init(&tx_lock);
	spin_lock_init(&rx_lock);

#ifdef CONFIG_CNS3XXX_NAPI
	napi_dev = init_napi_dev(napi_dev, &ring_info);
	#ifdef CNS3XXX_DOUBLE_RX_RING
	r1_napi_dev = init_napi_dev(r1_napi_dev, &ring_info);
	#endif
#endif

	cns3xxx_proc_init();
	register_reboot_notifier(&cns3xxx_notifier_reboot);
	clear_fs_dma_state(0);

	if (ring_info.num_rx_queues == 2) {
		// enable RX dobule ring
		DMA_RING_CTRL_REG |= 1;
	}

	if (ring_info.num_tx_queues == 2 ) {
		// enable TX dobule ring
		DMA_RING_CTRL_REG |= (1 << 16);
	}
#ifdef CONFIG_SUSPEND
    register_pm_notifier(&pse_power_notifier);
#endif

#ifdef CONFIG_CPU_FREQ
	atomic_set(&cpufreq_pse_flag, 0);
	if (cpufreq_register_notifier(&pse_cpufreq_notifier_block, 
						CPUFREQ_TRANSITION_NOTIFIER)) {
		printk("%s: Failed to setup cpufreq notifier\n", __FUNCTION__);
	}
#endif

	return 0;
}

static void __exit cns3xxx_exit_module(void)
{
	int i=0;

	//printk("g_ring_info.rx_ring: %p\n", g_ring_info.rx_ring);
	//printk("g_ring_info.tx_ring: %p\n", g_ring_info.tx_ring);
	kfree(g_ring_info.rx_ring);
	kfree(g_ring_info.tx_ring);

	cns3xxx_proc_remove();

	for (i=0 ; i < NETDEV_SIZE ; ++i) {
		CNS3XXXPrivate *priv = 0;

		if (net_dev_array[i]){
			priv = netdev_priv(net_dev_array[i]);

			unregister_netdev(net_dev_array[i]);
			free_netdev(net_dev_array[i]);
		}
	}

#ifdef CONFIG_CNS3XXX_NAPI
	free_netdev(napi_dev);
	#ifdef CNS3XXX_DOUBLE_RX_RING
	free_netdev(r1_napi_dev);
	#endif
#endif

#ifdef CONFIG_CPU_FREQ
	cpufreq_unregister_notifier(&pse_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
#endif

	unregister_reboot_notifier(&cns3xxx_notifier_reboot);
	printk("remove cns3xxx pse module\n");

#if 0
	// disable phy auto-poll
	PHY_AUTO_ADDR_REG &= ~((1<<5) | (1<<13) | (1<<21));
	mdelay(1000); // wait state machine idle
#endif
}

// this snippet code ref 8139cp.c
#if defined(CNS3XXX_VLAN_8021Q)
void cns3xxx_vlan_rx_register(struct net_device *dev, struct vlan_group *grp)
{
	CNS3XXXPrivate *priv = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	printk("gsw_vlan_rx_register\n");
	priv->vlgrp = grp;
	spin_unlock_irqrestore(&priv->lock, flags);
}

void cns3xxx_vlan_rx_kill_vid(struct net_device *dev, unsigned short vid)
{
	CNS3XXXPrivate *priv = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	printk("gsw_vlan_rx_kill_vid\n");
	// reference: linux-2.6.24-current/drivers/netvia-velocity.c
	vlan_group_set_device(priv->vlgrp, vid, NULL);
	//priv->vlgrp->vlan_devices[vid] = NULL;
	spin_unlock_irqrestore(&priv->lock, flags);
}
#endif

u32 cns3xxx_main;
EXPORT_SYMBOL(cns3xxx_main);

MODULE_AUTHOR("Cavium Networks, <tech@XXXX.com>");
MODULE_DESCRIPTION("CNS3XXX Switch Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(cns3xxx_init_module);
module_exit(cns3xxx_exit_module);

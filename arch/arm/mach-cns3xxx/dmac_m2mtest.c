/*******************************************************************************
 *
 *  Copyright (c) 2011 Cavium
 *
 *  This file is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, Version 2, as
 *  published by the Free Software Foundation.
 *
 *  This file is distributed in the hope that it will be useful,
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 *  NONINFRINGEMENT.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this file; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or
 *  visit http://www.gnu.org/licenses/.
 *
 *  This file may also be available under a different license from Cavium.
 *  Contact Cavium for more information
 *
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/memory.h>
#include <asm/dma.h>
#include <mach/cns3xxx.h>

#include <mach/dmac.h>

#define BUFFER_SIZE		0x1000
#define CNS3XXX_DMAC_REG(x)  (*((u32 volatile *)(CNS3XXX_DMAC_BASE_VIRT + x)))
#define DMAC_GET_PERIPH_ID()	CNS3XXX_DMAC_REG(0xFE0)
#define VALID_CHANNEL(x)	(!((x < 0) || (x > 7)))

static int s;
module_param(s, int, 0444);
MODULE_PARM_DESC(s, "To enable Simple M2M Test");
static int l;
module_param(l, int, 0444);
MODULE_PARM_DESC(l, "To enable Large M2M Test");
static int es;
module_param(es, int, 0444);
MODULE_PARM_DESC(es, "To enable 2 ch sync transfer test");
static int itest_ch;
static int itest;
module_param(itest, int, 0444);
MODULE_PARM_DESC(itest, "To test interrupts");


/* Global variables for simple m2m test */
void *buf_src, *buf_dst;
dma_addr_t buf_src_paddr, buf_dst_paddr;
int ch, lc0;
static int bpb = 4; /* default : 4 bytes per burst */
static int dt = 2; /* default : 2 data transfer */
module_param(bpb, int, 0444);
MODULE_PARM_DESC(bpb, "bytes per burst for simp m2m test");
module_param(dt, int, 0444);
MODULE_PARM_DESC(dt, "Number of data transfers for simple m2m test");
#define	 SIMP_MAX_BYTES_TO_TRANSFER	  (2 * 1024)
int bytes_to_transfer = SIMP_MAX_BYTES_TO_TRANSFER;
module_param(bytes_to_transfer, int, 0444);
MODULE_PARM_DESC(bytes_to_transfer, "Total bytes to transfer");

/*Global variables for large m2m test */
void *lm2m_src, *lm2m_dst;
dma_addr_t lm2m_src_paddr, lm2m_dst_paddr;
int lm2m_ch, lm2m_lc0, lm2m_lc1, lm2m_bytes_to_transfer;
int lm2m_bpb = 8; /* default : 8 bytes per burst */
int lm2m_dt = 2; /* default : 2 data transfers */
module_param(lm2m_bpb, int, 0444);
MODULE_PARM_DESC(lm2m_bpb, "bytes per burst for large m2m test");
module_param(lm2m_dt, int, 0444);
MODULE_PARM_DESC(lm2m_dt, "Number of data transfers for large m2m test");
/* MIN is 128 bytes */
/* MAX bytes to transfer for lm2m */
#define LM2M_BUFFER_SIZE	(256 * 1024)
#define LM2M_BYTES_TO_TRANSFER  (256 * 1024)

static void dmac_dump_regs(void)
{
	printk(KERN_INFO "Read Periph Id 0 for GDMAC is %x\n",
			DMAC_GET_PERIPH_ID());
	printk(KERN_INFO "DS Register: %x\n",	   CNS3XXX_DMAC_REG(0x0));
	printk(KERN_INFO "Conf Reg 0 : %x\n",	   CNS3XXX_DMAC_REG(0xE00));
	printk(KERN_INFO "Conf Reg 1 : %x\n",	   CNS3XXX_DMAC_REG(0xE04));
	printk(KERN_INFO "Conf Reg 2 : %x\n",	   CNS3XXX_DMAC_REG(0xE08));
	printk(KERN_INFO "Conf Reg 3 : %x\n",	   CNS3XXX_DMAC_REG(0xE0C));
	printk(KERN_INFO "Conf Reg 4 : %x\n",	   CNS3XXX_DMAC_REG(0xE10));
	printk(KERN_INFO "Conf Reg d : %x\n",	   CNS3XXX_DMAC_REG(0xE14));

	printk(KERN_INFO "Dumping the status registers\n");
	printk(KERN_INFO "INTEN Register: %x\n",   CNS3XXX_DMAC_REG(0x20));
	printk(KERN_INFO "ES Register: %x\n",	   CNS3XXX_DMAC_REG(0x24));
	printk(KERN_INFO "INTSTAT Register: %x\n", CNS3XXX_DMAC_REG(0x28));
	printk(KERN_INFO "FSDM Register: %x\n",    CNS3XXX_DMAC_REG(0x30));
	printk(KERN_INFO "FSC Register: %x\n",	   CNS3XXX_DMAC_REG(0x34));
	printk(KERN_INFO "FTM Register: %x\n",	   CNS3XXX_DMAC_REG(0x38));
	printk(KERN_INFO "FTC0 Register: %x\n",    CNS3XXX_DMAC_REG(0x40));
	printk(KERN_INFO "FTC1 Register: %x\n",    CNS3XXX_DMAC_REG(0x44));
	printk(KERN_INFO "CS0 Register: %x\n",	   CNS3XXX_DMAC_REG(0x100));
	printk(KERN_INFO "CPC0 Register: %x\n",    CNS3XXX_DMAC_REG(0x104));
	printk(KERN_INFO "CS1 Register: %x\n",	   CNS3XXX_DMAC_REG(0x108));
	printk(KERN_INFO "CPC1 Register: %x\n",    CNS3XXX_DMAC_REG(0x10C));
}

static int mem_verify(uint8_t *p, char c, int sz)
{
	int i;
	for (i = 0; i < sz; i++)
		if ((*(p + i)) != c)
			break;
	if (i == sz)
		return 1;
	printk(KERN_INFO "Broken at byte %d\n", i);
	return 0;
}

static int get_bpb_val(int bpb)
{
	int i = bpb;
	int retval = -1;
	while (i) {
		retval += 0x1;
		i /= 2;
	}
	return retval;
}

/* Double loop test with transfer size of 256KB */
static int do_large_m2m_test(void)
{
	uint32_t	ctrl_value;
	uint8_t		bpb_val;
	printk(KERN_INFO "\n**\n%s : Start\n**\n", __func__);
	if (!((lm2m_bpb == 1)  || (lm2m_bpb == 2) || (lm2m_bpb == 4) ||
		(lm2m_bpb == 8) || (lm2m_bpb == 16) || (lm2m_bpb == 32) ||
		(lm2m_bpb == 64) || (lm2m_bpb == 128))) {
		printk(KERN_INFO "Invalid BPB Param .. setting defaults and "
				"proceeding\n");
		lm2m_bpb = 8;
	}
	if ((lm2m_dt < 1) || (lm2m_dt > 16)) {
		printk(KERN_INFO "Invalid DT Param .. setting defaults and "
				"proceeding\n");
		lm2m_dt = 2;
	}
	bpb_val = get_bpb_val(lm2m_bpb);

	lm2m_src = dma_alloc_coherent(NULL, LM2M_BUFFER_SIZE,
			&lm2m_src_paddr, GFP_KERNEL);
	lm2m_dst = dma_alloc_coherent(NULL, LM2M_BUFFER_SIZE,
			&lm2m_dst_paddr, GFP_KERNEL);

	if (!lm2m_src || !lm2m_dst) {
		printk(KERN_INFO "%s:%d - Memory Allocation Failed\n",
				__func__, __LINE__);
		return -1;
	} else {
		printk(KERN_INFO "Got the physical addresses - lm2m_src: %x, "
				"lm2m_dst: %x\n", lm2m_src_paddr,
				lm2m_dst_paddr);
	}

	ctrl_value =	(0x1 << 0) |		/* src_increment */
			((bpb_val&0x7) << 1) |  /* bytes per burst */
			((lm2m_dt-1) << 4) |	/* data transfers */
			(0x2 << 8) |		/* src_prot_ctrl */
			(0x0 << 11) |		/* src_cache_ctrl */
			(0x1 << 14) |		/* dst_increment */
			((bpb_val&0x7) << 15) |	/* bytes per burst */
			((lm2m_dt-1) << 18) |	/* data transfers */
			(0x2 << 22) |		/* src_prot_ctrl */
			(0x0 << 25);		/* dst_prot_ctrl */

	printk(KERN_INFO "Filling the lm2m_src address %p [%d] with character "
			"'c' for size %d ..", lm2m_src, lm2m_src_paddr,
			LM2M_BUFFER_SIZE);
	memset(lm2m_src, 'c', LM2M_BUFFER_SIZE);
	if (mem_verify((uint8_t *)lm2m_src, 'c', LM2M_BUFFER_SIZE))
		printk(KERN_INFO " Verified Memory fill\n");

	printk(KERN_INFO "Filling the lm2m_dst address %p [%d] with character "
			"'a' for size %d ..", lm2m_dst, lm2m_dst_paddr,
			LM2M_BUFFER_SIZE);
	memset(lm2m_dst, 'a', LM2M_BUFFER_SIZE);
	if (mem_verify((uint8_t *)lm2m_dst, 'a', LM2M_BUFFER_SIZE))
		printk(KERN_INFO " Verified Memory fill\n");

	wmb();
	lm2m_bytes_to_transfer = LM2M_BYTES_TO_TRANSFER;
	if (lm2m_bpb == 128)
		lm2m_lc0 = 0x100/2;
	else
		lm2m_lc0 = 0x100; /* 256 times */

	lm2m_lc1 = lm2m_bytes_to_transfer / (lm2m_lc0 * lm2m_bpb * lm2m_dt);
	if (lm2m_lc1  > 0x100) {
		printk(KERN_WARNING "%s:%d > Warning : uncertain bpb setting "
				"... test may fail\n" , __func__, __LINE__);
		lm2m_lc1 &= 0xff;
	}

	lm2m_ch = dmac_get_channel(NULL, NULL);
	printk(KERN_INFO "Got the Free channel : %d. Proceeding with ctrl: %x\n"
			, lm2m_ch, ctrl_value);
	if ((lm2m_ch < 0) || (lm2m_ch > 7)) {
		printk(KERN_INFO "Got an invalid channel no\n");
		return -1;
	}

	{
		DMAC_DMAMOV(lm2m_ch, SAR, lm2m_src_paddr); /* SAR */
		DMAC_DMAMOV(lm2m_ch, DAR, lm2m_dst_paddr); /* DAR */
		DMAC_DMAMOV(lm2m_ch, CCR, ctrl_value);	    /* CCR */

		DMAC_DMALP(lm2m_ch, 1, lm2m_lc1);
		DMAC_DMALP(lm2m_ch, 0, lm2m_lc0);
		DMAC_DMALD(lm2m_ch);
		DMAC_DMAST(lm2m_ch);
		DMAC_DMALPEND(lm2m_ch, 0, DMAST_INSTR_SIZE + DMALD_INSTR_SIZE,
				1);
		DMAC_DMALPEND(lm2m_ch, 1, DMALPEND_INSTR_SIZE + DMAST_INSTR_SIZE
				+ DMALD_INSTR_SIZE + DMALP_INSTR_SIZE, 1);

		DMAC_DMAEND(lm2m_ch);
	}

	DMAC_DMAGO(lm2m_ch);

	return 0;
}

/* Single loop Test with transfer size of 1KB */
static int do_simple_m2m_test(void)
{
	uint32_t ctrl_value;
	int bpb_val;
	int residue;
	printk(KERN_INFO "\n**\n%s : Start\n**\n", __func__);
	if (!((bpb == 1) || (bpb == 2) || (bpb == 4) || (bpb == 8) ||
		(bpb == 16) || (bpb == 32) || (bpb == 64) || (bpb == 128))) {
		printk(KERN_INFO "Invalid BPB Param .. setting defaults and "
				"proceeding\n");
		bpb = 4;
	}
	if ((dt < 1) || (dt > 16)) {
		printk(KERN_INFO "Invalid DT Param .. setting defaults and "
				"proceeding\n");
		dt = 2;
	}

	if ((bytes_to_transfer < 4) ||
			(bytes_to_transfer > SIMP_MAX_BYTES_TO_TRANSFER)) {
		printk(KERN_INFO "Invalid bytestotransfer param. Limit b/w 4 "
				"and 2048\n");
		bytes_to_transfer = SIMP_MAX_BYTES_TO_TRANSFER;
	}

	buf_src = dma_alloc_coherent(NULL, BUFFER_SIZE*4, &buf_src_paddr,
			GFP_KERNEL|GFP_DMA);
	buf_dst = dma_alloc_coherent(NULL, BUFFER_SIZE*4, &buf_dst_paddr,
			GFP_KERNEL|GFP_DMA);

	if (!buf_src || !buf_dst) {
		printk(KERN_INFO "%s:%d - Memory Allocation Failed\n", __func__,
				__LINE__);
		return -1;
	} else
		printk(KERN_INFO "Got the physical addresses - src: %x, dst: "
				"%x\n", buf_src_paddr, buf_dst_paddr);

	bpb_val = get_bpb_val(bpb);
	ctrl_value =	(0x1 << 0) |		/* src_increment */
			((bpb_val&0x7) << 1) |	/* bytes per burst */
			((dt-1) << 4) |		/* data transfers */
			(0x2 << 8) |		/* src_prot_ctrl */
			(0x0 << 11) |		/* src_cache_ctrl */
			(0x1 << 14) |		/* dst_increment */
			((bpb_val&0x7) << 15) |	/* bytes per burst */
			((dt-1) << 18) |	/* data transfers */
			(0x2 << 22) |		/* dst_prot_ctrl */
			(0x0 << 25);		/* dst_cache_ctrl */

	printk(KERN_INFO "Filling the src address %p [%d] with character 'c' "
		"for size %d ..", buf_src, buf_src_paddr, BUFFER_SIZE * 4);
	memset(buf_src, 'c', BUFFER_SIZE * 4);
	if (mem_verify((uint8_t *)buf_src, 'c', BUFFER_SIZE * 4))
		printk(KERN_INFO " Verified Memory fill\n");

	printk(KERN_INFO "Filling the dst address %p [%d] with character 'a' "
		"for size %d ..", buf_dst, buf_dst_paddr, BUFFER_SIZE * 4);
	memset(buf_dst, 'a', BUFFER_SIZE * 4);
	if (mem_verify((uint8_t *)buf_dst, 'a', BUFFER_SIZE * 4))
		printk(KERN_INFO " Verified Memory fill\n");

	wmb();

	if (bytes_to_transfer < (bpb * dt)) {
		printk(KERN_INFO "bpb(%d)*dt(%d) is greater than "
				"bytes_to_transfer(%d) .. fine tune your "
				"options\n", bpb, dt, bytes_to_transfer);
		bytes_to_transfer = 2048;
	}
	lc0 = bytes_to_transfer / (bpb * dt);
	residue = bytes_to_transfer % (bpb * dt);
	if (lc0 > 0x100) {
		printk(KERN_INFO "%s:%d > Invalid bpb value [%d].. transfer "
				"size is %d, use lm2m instead\n", __func__,
				__LINE__, bpb, bytes_to_transfer);
	}

	ch = dmac_get_channel(NULL, NULL);
	printk(KERN_INFO "Got the Free channel : %d. Proceeding with ctrl: "
			"%x\n", ch, ctrl_value);
	if ((ch < 0) || (ch > 7)) {
		printk(KERN_INFO "Got an invalid channel no\n");
		return -1;
	}

	{
		DMAC_DMAMOV(ch, SAR, buf_src_paddr); /* SAR */
		DMAC_DMAMOV(ch, DAR, buf_dst_paddr); /* DAR */
		DMAC_DMAMOV(ch, CCR, ctrl_value);    /* CCR */

		DMAC_DMALP(ch, 0, lc0);
		DMAC_DMALD(ch);
		DMAC_DMAST(ch);
		DMAC_DMALPEND(ch, 0, DMAST_INSTR_SIZE + DMALD_INSTR_SIZE, 1);
		while (residue--) {
			DMAC_DMALD(ch);
			DMAC_DMAST(ch);
		}

		DMAC_DMAEND(ch);
	}

	DMAC_DMAGO(ch);

	return 0;
}

static int itest_handler(void *p)
{
	printk(KERN_INFO "OK, interrupt generated with ev no %d\n", itest_ch);
	return 0;
}

static int do_itest(void)
{
	itest_ch = dmac_get_channel(itest_handler, NULL);
	if (!VALID_CHANNEL(itest_ch)) {
		printk(KERN_INFO "Debug this !! got invalid channel [%d]\n",
				itest_ch);
		return -1;
	}

	printk(KERN_INFO "Testing interrupt with channel no %d\n", itest_ch);
	DMAC_DMASEV(itest_ch, itest_ch);
	DMAC_DMAEND(itest_ch);
	DMAC_DMAGO(itest_ch);

	return 0;
}

static void do_verify_itest(void)
{
	dmac_release_channel(itest_ch);
}

#define EVENTSYNC_2CH_BUFFER_SIZE   8192
void *es_buf_src, *es_buf_via, *es_buf_dst;
dma_addr_t  es_buf_src_paddr, es_buf_via_paddr, es_buf_dst_paddr;
int es_ch1, es_ch2;
int es_dt;
int es_bytes_to_transfer = EVENTSYNC_2CH_BUFFER_SIZE;
int es_lc0, es_lc1;
int es_ch1_evt = 9;
int es_ch2_evt = 10;
int es_bpb = 4;

int es_m2m_inthandler(void *chval)
{
	if (!chval) {
		printk(KERN_INFO "BUG Here [%s:%d] .. debug\n",
				__func__, __LINE__);
		return -1;
	}
	printk(KERN_INFO "Channel %d generated an interrupt\n",
			*((int *)chval));
	return 0;
}

static int do_eventsync_2ch_m2m_test(void)
{
	int ctrl_value;
	int es_bpb_val;

	es_buf_src = dma_alloc_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE,
			&es_buf_src_paddr, GFP_KERNEL|GFP_DMA);
	es_buf_via = dma_alloc_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE,
			&es_buf_via_paddr, GFP_KERNEL|GFP_DMA);
	es_buf_dst = dma_alloc_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE,
			&es_buf_dst_paddr, GFP_KERNEL|GFP_DMA);

	if (!es_buf_src || !es_buf_via || !es_buf_dst) {
		if (es_buf_src)
			dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE,
					es_buf_src, es_buf_src_paddr);
		if (es_buf_via)
			dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE,
					es_buf_via, es_buf_via_paddr);
		if (es_buf_dst)
			dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE,
					es_buf_dst, es_buf_dst_paddr);

		printk(KERN_INFO "%s:%d > Memory allocation failed.fix this "
				"first\n", __func__, __LINE__);
		return -1;
	}

	memset(es_buf_src, 'c', EVENTSYNC_2CH_BUFFER_SIZE);
	memset(es_buf_dst, 'a', EVENTSYNC_2CH_BUFFER_SIZE);
	if (mem_verify((uint8_t *)es_buf_src, 'c', EVENTSYNC_2CH_BUFFER_SIZE))
		printk(KERN_INFO "Filled Src Buffer (va:%p, pa:%x) with char "
				"'c'\n", es_buf_src, es_buf_src_paddr);

	if (mem_verify((uint8_t *)es_buf_dst, 'a', EVENTSYNC_2CH_BUFFER_SIZE))
		printk(KERN_INFO "Filled Dst Buffer (va:%p, pa:%x) with char "
				"'a'\n", es_buf_dst, es_buf_dst_paddr);


	es_ch1 = dmac_get_channel(es_m2m_inthandler, (void *)&es_ch1);
	if (!VALID_CHANNEL(es_ch1)) {
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_src,
				es_buf_src_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_via,
				es_buf_via_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_dst,
				es_buf_dst_paddr);
		printk(KERN_INFO "%s:%d: Got Invalid channel. Debug this\n",
				__func__, __LINE__);
		return -1;
	}
	printk(KERN_INFO "Got Channel es_ch1 = %d\n", es_ch1);

	if (dmac_get_event(es_ch1, es_ch1_evt) == -1) {
		printk(KERN_INFO "%s:%d > get_event failed\n",
				__func__, __LINE__);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_src,
				es_buf_src_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_via,
				es_buf_via_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_dst,
				es_buf_dst_paddr);
		dmac_release_channel(es_ch1);
	}
	printk(KERN_INFO "Got event no %d for Channel %d\n",
			es_ch1_evt, es_ch1);

	es_ch2 = dmac_get_channel(es_m2m_inthandler, (void *)&es_ch2);
	if (!VALID_CHANNEL(es_ch2)) {
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_src,
				es_buf_src_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_via,
				es_buf_via_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_dst,
				es_buf_dst_paddr);
		dmac_release_channel(es_ch1);
		dmac_release_event(es_ch1, es_ch1_evt);
		printk(KERN_INFO "%s:%d: Got Invalid channel. Debug this\n",
				__func__, __LINE__);
		return -1;
	}
	printk(KERN_INFO "Got Channel es_ch2 = %d\n", es_ch2);
	if (dmac_get_event(es_ch2, es_ch2_evt) == -1) {
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_src,
				es_buf_src_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_via,
				es_buf_via_paddr);
		dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_dst,
				es_buf_dst_paddr);
		dmac_release_channel(es_ch1);
		dmac_release_event(es_ch1, es_ch1_evt);
		dmac_release_channel(es_ch2);
		return -1;
	}
	printk(KERN_INFO "Got event no %d for Channel %d\n", es_ch2_evt,
			es_ch2);

	es_bpb		= 4;		/* 4 bytes per burst */
	es_bpb_val	= get_bpb_val(es_bpb);
	es_dt		= 1;		/* 1 data transfer */
	es_lc0		= 0x100;	/* 256 bytes */
	es_lc1		= es_bytes_to_transfer/(es_bpb * es_dt * es_lc0);
	ctrl_value =    (0x1 << 0) |		     /* src_increment */
			((es_bpb_val & 0x7) << 1) |  /* bytes per burst */
			((es_dt - 1) << 4) |	     /* data transfers */
			(0x2 << 8) |		     /* src_prot_ctrl */
			(0x0 << 11) |		     /* src_cache_ctrl */
			(0x1 << 14) |		     /* dst_increment */
			((es_bpb_val & 0x7) << 15) | /* bytes per burst */
			((es_dt - 1) << 18) |	     /* data transfers */
			(0x2 << 22) |		     /* dst_prot_ctrl */
			(0x0 << 25);		     /* dst_cache_ctrl */
	printk(KERN_INFO "Got the bpb_val as %x and ctrl is %x\n", es_bpb_val,
			ctrl_value);

	{
		DMAC_DMAMOV(es_ch1, SAR, es_buf_src_paddr);
		DMAC_DMAMOV(es_ch1, DAR, es_buf_via_paddr);
		DMAC_DMAMOV(es_ch1, CCR, ctrl_value);

		DMAC_DMALP(es_ch1, 1, es_lc1);
		DMAC_DMALP(es_ch1, 0, es_lc0);
		DMAC_DMALD(es_ch1);
		DMAC_DMAST(es_ch1);
		DMAC_DMAWMB(es_ch1);
		DMAC_DMASEV(es_ch1, es_ch1_evt);
		DMAC_WFE(es_ch1, es_ch2_evt);
		DMAC_DMALPEND(es_ch1, 0, DMAWFE_INSTR_SIZE + DMASEV_INSTR_SIZE
			+ DMAST_INSTR_SIZE + DMALD_INSTR_SIZE
			+ DMAWMB_INSTR_SIZE, 1);
		DMAC_DMALPEND(es_ch1, 1, DMALPEND_INSTR_SIZE + DMAWFE_INSTR_SIZE
			+ DMASEV_INSTR_SIZE + DMAST_INSTR_SIZE
			+ DMALD_INSTR_SIZE + DMAWMB_INSTR_SIZE
			+ DMALP_INSTR_SIZE, 1);
		DMAC_DMAEND(es_ch1);
	}

	{
		DMAC_DMAMOV(es_ch2, SAR, es_buf_via_paddr);
		DMAC_DMAMOV(es_ch2, DAR, es_buf_dst_paddr);
		DMAC_DMAMOV(es_ch2, CCR, ctrl_value);

		DMAC_DMALP(es_ch2, 1, es_lc1);
		DMAC_DMALP(es_ch2, 0, es_lc0);
		DMAC_DMASEV(es_ch2, es_ch2_evt);
		DMAC_WFE(es_ch2, es_ch1_evt);
		DMAC_DMALD(es_ch2);
		DMAC_DMAST(es_ch2);
		DMAC_DMAWMB(es_ch2);
		DMAC_DMALPEND(es_ch2, 0, DMAWFE_INSTR_SIZE + DMASEV_INSTR_SIZE
			+ DMAST_INSTR_SIZE + DMALD_INSTR_SIZE
			+ DMAWMB_INSTR_SIZE, 1);
		DMAC_DMALPEND(es_ch2, 1, DMAWFE_INSTR_SIZE + DMASEV_INSTR_SIZE
			+ DMAST_INSTR_SIZE + DMALD_INSTR_SIZE
			+ DMAWMB_INSTR_SIZE
			+ DMALP_INSTR_SIZE + DMALPEND_INSTR_SIZE, 1);
		/*DMAC_DMASEV(es_ch2, es_ch2_evt); */

		DMAC_DMAEND(es_ch2);
	}
	DMAC_DMAGO(es_ch2);
	DMAC_DMAGO(es_ch1);

	return 0;

}

static void do_verify_es_m2m_test(void)
{
	if (mem_verify((uint8_t *)es_buf_dst, 'c', es_bytes_to_transfer))
		printk(KERN_INFO "\n**ESM2M Result : Verified bytes [%d].. "
				"OK\n", es_bytes_to_transfer);
	else {
		int i = 0;
		printk(KERN_INFO "Dumping dst buf\n");
		for (i = 0; i < 13; i++)
			printk(KERN_INFO "%x ", *((uint8_t *)es_buf_dst + i));
		printk(KERN_INFO "\n**ESM2M Result : Verification of bytes "
				"[%d].. FAILED [%c %c]\n", es_bytes_to_transfer,
				*((uint8_t *)es_buf_dst),
				*((uint8_t *)es_buf_dst + 1));
	}
	dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_src,
			es_buf_src_paddr);
	dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_via,
			es_buf_via_paddr);
	dma_free_coherent(NULL, EVENTSYNC_2CH_BUFFER_SIZE, es_buf_dst,
			es_buf_dst_paddr);
	dmac_release_channel(es_ch1);
	dmac_release_event(es_ch1, es_ch1_evt);
	dmac_release_channel(es_ch2);
	dmac_release_event(es_ch2, es_ch2_evt);
}

static void do_verify_lm2m_test(void)
{
	if (mem_verify((uint8_t *)lm2m_dst, 'c', lm2m_bytes_to_transfer))
		printk(KERN_INFO "\n**LM2M Result : Verified bytes [%d].. OK\n",
				lm2m_bytes_to_transfer);
	else
		printk(KERN_INFO "\n**LM2M Result : Verification of bytes "
				"[%d].. FAILED\n", lm2m_bytes_to_transfer);

	dma_free_coherent(NULL, LM2M_BUFFER_SIZE, lm2m_src, lm2m_src_paddr);
	dma_free_coherent(NULL, LM2M_BUFFER_SIZE, lm2m_dst, lm2m_dst_paddr);
	dmac_release_channel(lm2m_ch);
}

static void do_verify_simple_m2m_test(void)
{
	if (mem_verify((uint8_t *)buf_dst, 'c', bytes_to_transfer))
		printk(KERN_INFO "\n**Result : Verified bytes [%d] .. OK\n",
				bytes_to_transfer);
	else
		printk(KERN_INFO "\n**Result : Verification of bytes [%d].. "
				"FAILED\n", bytes_to_transfer);

	if (*((uint8_t *)buf_dst+bytes_to_transfer) == 'c')
		printk(KERN_WARNING "One byte after the limit is also written "
				".. check this\n");

	dma_free_coherent(NULL, BUFFER_SIZE * 4, buf_src, buf_src_paddr);
	dma_free_coherent(NULL, BUFFER_SIZE * 4, buf_dst, buf_dst_paddr);
	dmac_release_channel(ch);
}

static int __init dmac_dmatest_init(void)
{
	uint32_t	reg_val;

	reg_val = DMAC_GET_PERIPH_ID();
	if (reg_val == 0x30)
		dmac_dump_regs();
	else
		return -1;

	if ((s == 0) && (l == 0) && (es == 0) && (itest == 0)) {
		printk(KERN_INFO "usage : insmod dma_m2m_test.ko <options>\n");
		printk(KERN_INFO "options:\n");
		printk(KERN_INFO "s=1\t\t To enable Simple M2M Test\n");
		printk(KERN_INFO "l=1\t\t To enable Large M2M Test\n");
		printk(KERN_INFO "es=1\t\t To enable 2 chan synch M2M Test\n");
		return -1;
	}

	if (s == 1) {
		if (do_simple_m2m_test()) {
			printk(KERN_INFO "Mem Test Setup Failure for simple m2m"
					" ... bailing out\n");
			return -1;
		}
		dmac_dump_regs();
	}

	if (l == 1) {
		if (do_large_m2m_test()) {
			printk(KERN_INFO "Mem Test Setup Failure for large m2m "
					"... bailing out\n");
			if (s == 1)
				do_verify_simple_m2m_test();
			return -1;
		}
		dmac_dump_regs();
	}

	if (es == 1) {
		if (do_eventsync_2ch_m2m_test()) {
			printk(KERN_INFO "Setup Failure for eventsync 2ch m2m "
					"test .. bailing\n");
			if (s == 1)
				do_verify_simple_m2m_test();
			if (l == 1)
				do_verify_lm2m_test();
			return -1;
		}
		dmac_dump_regs();
	}

	if (itest == 1) {
		if (do_itest()) {
			printk(KERN_INFO "Setup Failure for eventsync 2ch m2m "
					"test .. bailing\n");
			if (s == 1)
				do_verify_simple_m2m_test();
			if (l == 1)
				do_verify_lm2m_test();
			if (es == 1)
				do_verify_es_m2m_test();
			return -1;
		}
		dmac_dump_regs();
	}

	return 0;

}

static void __exit dmac_dmatest_exit(void)
{

	dmac_dump_regs();
	if (s == 1)
		do_verify_simple_m2m_test();
	if (l == 1)
		do_verify_lm2m_test();
	if (es == 1)
		do_verify_es_m2m_test();
	if (itest == 1)
		do_verify_itest();
}



module_init(dmac_dmatest_init);
module_exit(dmac_dmatest_exit);

MODULE_AUTHOR("Cavium");
MODULE_DESCRIPTION("Cavium CNS3XXX DMAC M2M Test Module");
MODULE_LICENSE("GPL");

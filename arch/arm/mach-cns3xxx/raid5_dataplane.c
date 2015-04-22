#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#ifdef AMP_RAID5
#include "rdma.h"
#include <mach/rdma_raid5.h>

#define XOR_QUEUE_SIZE   32

/* Debug Printk */
#define dprintk(x...)   ((void)(rdma_verbose && printk(KERN_WARNING x)))
#define dump_regs(x)    \
do {    \
	dprintk("pa:%08x sg:%08x bp:%08x fp:%08x st:%08x qp:%08x sz:%08x\n", \
		*((x)->cregs->para),    \
		*((x)->cregs->sgad),    \
		*((x)->cregs->back),    \
		*((x)->cregs->frnt),    \
		*((x)->cregs->stat),    \
		*((x)->cregs->qpar),    \
		*((x)->cregs->blsz));   \
} while (0)

#define rdma_dmac_flush_range(start, bytes)     \
	do { \
		dma_cache_maint(start, bytes, DMA_BIDIRECTIONAL);       \
	} while (0);

#define rdma_dmac_inv_range(start, bytes)       \
	do { \
		dma_cache_maint(start, bytes, DMA_FROM_DEVICE); \
	} while (0);

#define rdma_dmac_clean_range(start, bytes)     \
	do { \
		dma_cache_maint(start, bytes, DMA_TO_DEVICE);   \
	} while (0);

#define rdma_dmac_clean_range_local(start, bytes)     \
	do { \
		dma_cache_maint_local(start, bytes, DMA_TO_DEVICE);   \
	} while (0);

#define XOR_SUCCESS 0
#define XOR_FAILURE -1
#define XOR_IN_PROGRESS -2

#define cvm_dump_regs(x)    \
do {    \
	printk(KERN_INFO "pa:%08x\n sg:%08x\n bp:%08x\n fp:%08x\n st:%08x\n " \
			"qp:%08x\n sz:%08x\n", \
		*((x)->cregs->para),    \
		*((x)->cregs->sgad),    \
		*((x)->cregs->back),    \
		*((x)->cregs->frnt),    \
		*((x)->cregs->stat),    \
		*((x)->cregs->qpar),    \
		*((x)->cregs->blsz));   \
} while (0)


int check_xor_status(void)
{
	if (*(dma->cregs->stat) & (REG_STAT_SLAVE_ERROR)) {
		printk(KERN_INFO "Slave error\n");
		*(dma->cregs->stat) = REG_STAT_SLAVE_ERROR;
		return XOR_FAILURE;
	}

	if ((*(dma->cregs->stat) & (REG_STAT_XFER_COMPLETE |
				   REG_STAT_INTERRUPT_FLAG))) {
		/*
		printk(KERN_INFO "%s: pstat=0x%08x\n", __func__,
				*(dma->cregs->stat));
		*/
		*(dma->cregs->stat) = (REG_STAT_XFER_COMPLETE |
				       REG_STAT_INTERRUPT_FLAG);
		return XOR_SUCCESS;
	}

	else {
		/*
		printk(KERN_INFO "%s: pstat=0x%08x\n", __func__,
				*(dma->cregs->stat));
		cvm_dump_regs(dma);
		*/
		return XOR_IN_PROGRESS;
	}
}
EXPORT_SYMBOL(check_xor_status);



/**
 *  * rdma_queue_sg_raid5 - queue an SG, wait done and put it.
 *   * @dma: dma chan
 *    * @sg: sg
 *     * @q_para: parameter
 *      * @q_blsz: block size
 *       * @q_sgad: SG Addr
 *	* @sg_cnt: count of (src_cnt + dst_cnt)
 *	 */
#define	 QUEUE_MODE
void rdma_queue_sg_raid5(rdma_chan_t *rdma, sg_t *sg,
			 u32 q_para, u32 q_blsz, u32 q_sgad, int sg_cnt)
{
	cmdq_t *this_virt = NULL;
	if (rdma_irq_enabled) { /* based on polling, so disable interrupt */
		disable_irq(dma->irq);
		rdma_irq_enabled = 0;
	}

	dump_regs(rdma);

#ifdef QUEUE_MODE
	/* Setup BP */
	this_virt = (cmdq_t *)(phys_to_virt(*(rdma->cregs->frnt)));
	this_virt->parameter = q_para;
	this_virt->block_size = q_blsz;
	this_virt->sg_addr = q_sgad;
	this_virt->reserved = 0;
	dump_regs(rdma);

	/* FIXME */
	{
		void *sgp = (void *)sg;
		void *cqp = (void *)this_virt;

		rdma_dmac_flush_range(sgp, (sg_cnt * sizeof(u64)));
		rdma_dmac_flush_range(cqp, sizeof(cmdq_t));
	}
	if (!(*(rdma->cregs->frnt) == rdma->q_last_phys))
		*(rdma->cregs->frnt) = *(rdma->cregs->frnt) + 16;
	else
		*(rdma->cregs->frnt) = rdma->q_first_phys;
	/* Queue Enable */
	*(rdma->cregs->stat) = REG_STAT_CMD_QUEUE_ENABLE;
	dump_regs(rdma);

#else
	*(dma->cregs->blsz) = q_blsz;
	*(rdma->cregs->sgad) = q_sgad;
	*(rdma->cregs->para) = q_para;
	dump_regs(rdma);
#endif  /* QUEUE_MODE */
	dump_regs(rdma);
	return;
}
EXPORT_SYMBOL(rdma_queue_sg_raid5);

/**
 *  * @src_no: source count
 *   * @bytes: len in bytes
 *    * @bh_ptr: srcs PA
 *     * @dst_ptr: dest PA
 *      *
 *       * Desc:
 *	*     dst_ptr = XOR(bh_ptr[0 ... src_no-1]), in Page Addr
 *	 */
int do_cns_rdma_xorgen_raid5(unsigned int src_no, unsigned int bytes, void **bh_ptr,
				void *dst_ptr, sg_t *sg, enum async_tx_flags flags)
{
	int i, j;
	u32 q_sgad, q_blsz, q_para;

	if (unlikely(!dst_ptr)) {
		printk(KERN_INFO "dest page null\n");
		return -1;
	}

	if (flags & ASYNC_TX_XOR_ZERO_DST) {
		memset(dst_ptr, 0, bytes);
		rdma_dmac_clean_range_local(dst_ptr, bytes);
	}

	/* Setup SG::Read::SRC */
	for (i = 0, j = 0; i < src_no; i++) {
		if (likely(bh_ptr[i])) {
			sg->entry[j] = (SG_ADDR_MASK &
				((u64)virt_to_phys(bh_ptr[i]))) |
				(SG_READ_IDX_MASK & ((u64)j + 1) <<
				 SG_IDX_SHIFT) | (RWI_RD_D);
			j++;
		}
	}
	if (j == 0) {
		printk(KERN_INFO "All src pages are null\n");
		return -1;
	}

	/* Setup SG::Write::P1 */
	sg->entry[j] = (SG_ADDR_MASK & ((u64)virt_to_phys(dst_ptr)))
					   | (RWI_W_P1);

	/* Setup SGAD, BLSZ, PARAMETER */
	q_sgad = virt_to_phys(&(sg->entry[0]));
	q_blsz = bytes & REG_BLSZ_MASK;
	q_para = REG_PARA_ENABLE
			| REG_PARA_XFER_END
			| REG_PARA_CALC_P
			| (REG_PARA_FAULTY_DISKS_CNT * 1);

	if (unlikely(rdma_verbose)) {
		for (i = 0; i < src_no; i++)
			printk(KERN_INFO "set-SG::SRC[%d] = 0x%016llx\n", i,
					sg->entry[i]);
		printk(KERN_INFO "set-SG::DST1ptr= 0x%016llx\n",
				sg->entry[src_no]);
	}
	/* Invalidate dst */
	/* Queue SG */
	rdma_queue_sg_raid5(dma, sg, q_para, q_blsz, q_sgad, (j + 1));
	/* Invalidate dst */
	rdma_dmac_inv_range(dst_ptr, bytes);

	return 0;
}
EXPORT_SYMBOL(do_cns_rdma_xorgen_raid5);
#endif /* AMP_RAID5 */

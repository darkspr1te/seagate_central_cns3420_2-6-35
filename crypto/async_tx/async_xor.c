/*
 * xor offload engine api
 *
 * Copyright © 2006, Intel Corporation.
 *
 *      Dan Williams <dan.j.williams@intel.com>
 *
 *      with architecture considerations by:
 *      Neil Brown <neilb@suse.de>
 *      Jeff Garzik <jeff@garzik.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/raid/xor.h>
#ifndef AMP_RAID5
#include <linux/async_tx.h>
#endif
#ifdef AMP_RAID5
#include <linux/mempool.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <mach/rdma_raid5.h>
#include <linux/proc_fs.h>

#ifdef CONFIG_AMP
#define rdma_raid5_dmac_clean_range(start, bytes)     \
        do { \
                dma_cache_maint_local(start, bytes, DMA_TO_DEVICE);	\
        } while (0);
#endif

void (*fp_xor_offload) (void *) = NULL;
EXPORT_SYMBOL(fp_xor_offload);
void (*fp_raid5_callback) (void) = NULL;
EXPORT_SYMBOL(fp_raid5_callback);
unsigned long timestamp_before, timestamp_after, timestamp_off, timestamp_off_end, timestamp_cb;
unsigned long ts_submission_interval, ts_completion_interval, raid5_wrong_wkup;
unsigned long raid5_offload_submit, raid5_offload_wkup, raid5_offload_success, raid5_ipi_missed, raid5_wkup_rcvd; 


EXPORT_SYMBOL(timestamp_before);
EXPORT_SYMBOL(timestamp_after);
EXPORT_SYMBOL(timestamp_off);
EXPORT_SYMBOL(timestamp_cb);
EXPORT_SYMBOL(timestamp_off_end);

EXPORT_SYMBOL(ts_submission_interval);
EXPORT_SYMBOL(ts_completion_interval);
EXPORT_SYMBOL(raid5_wrong_wkup);
EXPORT_SYMBOL(raid5_ipi_missed);
EXPORT_SYMBOL(raid5_wkup_rcvd);
EXPORT_SYMBOL(raid5_offload_submit);


EXPORT_SYMBOL(raid5_offload_success);

unsigned long  xor_offload_submitted;
EXPORT_SYMBOL(xor_offload_submitted);

unsigned long xor_offload_completed;
EXPORT_SYMBOL(xor_offload_completed);

unsigned long xor_offload_noop;
EXPORT_SYMBOL(xor_offload_noop);

unsigned long xor_offload_success;
EXPORT_SYMBOL(xor_offload_success);


unsigned long xor_offload_cb;
EXPORT_SYMBOL(xor_offload_cb);


static __async_inline struct dma_async_tx_descriptor *
do_cvm_async_xor(struct dma_chan *chan, struct page *dest, struct page **src_list,
             unsigned int offset, int src_cnt, size_t len,
		struct async_submit_ctl *submit)
{
        int i;
        int xor_src_cnt;
        void *_cb_param;
        enum async_tx_flags async_flags;
        struct dma_async_tx_descriptor *tx = NULL;
	struct xor_info xor_info;
        dma_async_tx_callback _cb_fn;
	void *dest_buf;
        void **srcs = (void **) src_list;
	static unsigned long exit_time_stamp=0;


	if(exit_time_stamp)	
		ts_submission_interval += jiffies - exit_time_stamp;


	tx = kmalloc(sizeof(struct dma_async_tx_descriptor), GFP_KERNEL);
	if(tx == NULL) {
		printk(KERN_INFO "Memory allocation for tx failed\n");
		return NULL;
	}
	
	tx->flags = submit->flags;


        /* reuse the 'src_list' array to convert to buffer pointers */
       	for (i = 0; i < src_cnt; i++)
                srcs[i] = page_address(src_list[i]) + offset;

        /* set destination address */
        dest_buf = page_address(dest) + offset;
/*
        if(dest_buf != NULL)
                if (submit->flags & ASYNC_TX_XOR_ZERO_DST)
                        memset(dest_buf, 0, len);
*/
        xor_info.src_list = srcs;
        xor_info.dest = dest_buf;
        xor_info.len = len;
        xor_info.src_cnt = src_cnt;
        xor_info.offset = offset;
	xor_info.tx = (void *)tx;

	/* clean src/dst */
	for (i=0; i<xor_info.src_cnt; i++)
	{
		if (likely(xor_info.src_list[i])) {
			rdma_raid5_dmac_clean_range(xor_info.src_list[i], xor_info.len);
		} else {
			BUG();
		}
	}
/*
	rdma_raid5_dmac_clean_range(xor_info.dest, xor_info.len);
*/
        tx->priv = (void *) &xor_info;
        tx->cookie = -EBUSY;

	while (src_cnt) {
                async_flags = submit->flags;
                xor_src_cnt = min(src_cnt, 32);

                /* if we are submitting additional xors, leave the chain open,
                 * clear the callback parameters, and leave the destination
                 * buffer mapped
                 */
                if (src_cnt > xor_src_cnt) {
                        async_flags &= ~ASYNC_TX_ACK;
			async_flags |= ASYNC_TX_DEP_ACK;
                        _cb_fn = NULL;
                        _cb_param = NULL;
                } else {
			/*async_flags &= ~ASYNC_TX_ACK;
			async_flags |= ASYNC_TX_DEP_ACK;*/
                        _cb_fn = submit->cb_fn;
                        _cb_param = submit->cb_param;
                }
                async_tx_submit(chan, tx, submit);
		submit->depend_tx = tx;
                submit->flags |= ASYNC_TX_DEP_ACK;
                if (src_cnt > xor_src_cnt) {
                        /* drop completed sources */
                        src_cnt -= xor_src_cnt;
                } else
                        break;
        }

	exit_time_stamp = jiffies;

	return NULL;
}

#else		
/* do_async_xor - dma map the pages and perform the xor with an engine */
static __async_inline struct dma_async_tx_descriptor *
do_async_xor(struct dma_chan *chan, struct page *dest, struct page **src_list,
	     unsigned int offset, int src_cnt, size_t len, dma_addr_t *dma_src,
	     struct async_submit_ctl *submit)
{
	struct dma_device *dma = chan->device;
	struct dma_async_tx_descriptor *tx = NULL;
	int src_off = 0;
	int i;
	dma_async_tx_callback cb_fn_orig = submit->cb_fn;
	void *cb_param_orig = submit->cb_param;
	enum async_tx_flags flags_orig = submit->flags;
	enum dma_ctrl_flags dma_flags;
	int xor_src_cnt = 0;
	dma_addr_t dma_dest;

	/* map the dest bidrectional in case it is re-used as a source */
	dma_dest = dma_map_page(dma->dev, dest, offset, len, DMA_BIDIRECTIONAL);
	for (i = 0; i < src_cnt; i++) {
		/* only map the dest once */
		if (!src_list[i])
			continue;
		if (unlikely(src_list[i] == dest)) {
			dma_src[xor_src_cnt++] = dma_dest;
			continue;
		}
		dma_src[xor_src_cnt++] = dma_map_page(dma->dev, src_list[i], offset,
						      len, DMA_TO_DEVICE);
	}
	src_cnt = xor_src_cnt;

	while (src_cnt) {
		submit->flags = flags_orig;
		dma_flags = 0;
		xor_src_cnt = min(src_cnt, (int)dma->max_xor);
		/* if we are submitting additional xors, leave the chain open,
		 * clear the callback parameters, and leave the destination
		 * buffer mapped
		 */
		if (src_cnt > xor_src_cnt) {
			submit->flags &= ~ASYNC_TX_ACK;
			submit->flags |= ASYNC_TX_FENCE;
			dma_flags = DMA_COMPL_SKIP_DEST_UNMAP;
			submit->cb_fn = NULL;
			submit->cb_param = NULL;
		} else {
			submit->cb_fn = cb_fn_orig;
			submit->cb_param = cb_param_orig;
		}
		if (submit->cb_fn)
			dma_flags |= DMA_PREP_INTERRUPT;
		if (submit->flags & ASYNC_TX_FENCE)
			dma_flags |= DMA_PREP_FENCE;
		/* Since we have clobbered the src_list we are committed
		 * to doing this asynchronously.  Drivers force forward progress
		 * in case they can not provide a descriptor
		 */
		tx = dma->device_prep_dma_xor(chan, dma_dest, &dma_src[src_off],
					      xor_src_cnt, len, dma_flags);

		if (unlikely(!tx))
			async_tx_quiesce(&submit->depend_tx);

		/* spin wait for the preceeding transactions to complete */
		while (unlikely(!tx)) {
			dma_async_issue_pending(chan);
			tx = dma->device_prep_dma_xor(chan, dma_dest,
						      &dma_src[src_off],
						      xor_src_cnt, len,
						      dma_flags);
		}

		async_tx_submit(chan, tx, submit);
		submit->depend_tx = tx;

		if (src_cnt > xor_src_cnt) {
			/* drop completed sources */
			src_cnt -= xor_src_cnt;
			src_off += xor_src_cnt;

			/* use the intermediate result a source */
			dma_src[--src_off] = dma_dest;
			src_cnt++;
		} else
			break;
	}

	return tx;
}
#endif /* AMP_RAID5 */

static void
do_sync_xor(struct page *dest, struct page **src_list, unsigned int offset,
	    int src_cnt, size_t len, struct async_submit_ctl *submit)
{
	int i;
	int xor_src_cnt = 0;
	int src_off = 0;
	void *dest_buf;
	void **srcs;

	if (submit->scribble)
		srcs = submit->scribble;
	else
		srcs = (void **) src_list;

	/* convert to buffer pointers */
	for (i = 0; i < src_cnt; i++)
		if (src_list[i])
			srcs[xor_src_cnt++] = page_address(src_list[i]) + offset;
	src_cnt = xor_src_cnt;
	/* set destination address */
	dest_buf = page_address(dest) + offset;

	if (submit->flags & ASYNC_TX_XOR_ZERO_DST)
		memset(dest_buf, 0, len);

	while (src_cnt > 0) {
		/* process up to 'MAX_XOR_BLOCKS' sources */
		xor_src_cnt = min(src_cnt, MAX_XOR_BLOCKS);
		xor_blocks(xor_src_cnt, len, dest_buf, &srcs[src_off]);

		/* drop completed sources */
		src_cnt -= xor_src_cnt;
		src_off += xor_src_cnt;
	}

	async_tx_sync_epilog(submit);
}

/**
 * async_xor - attempt to xor a set of blocks with a dma engine.
 * @dest: destination page
 * @src_list: array of source pages
 * @offset: common src/dst offset to start transaction
 * @src_cnt: number of source pages
 * @len: length in bytes
 * @submit: submission / completion modifiers
 *
 * honored flags: ASYNC_TX_ACK, ASYNC_TX_XOR_ZERO_DST, ASYNC_TX_XOR_DROP_DST
 *
 * xor_blocks always uses the dest as a source so the
 * ASYNC_TX_XOR_ZERO_DST flag must be set to not include dest data in
 * the calculation.  The assumption with dma eninges is that they only
 * use the destination buffer as a source when it is explicity specified
 * in the source list.
 *
 * src_list note: if the dest is also a source it must be at index zero.
 * The contents of this array will be overwritten if a scribble region
 * is not specified.
 */
struct dma_async_tx_descriptor *
async_xor(struct page *dest, struct page **src_list, unsigned int offset,
	  int src_cnt, size_t len, struct async_submit_ctl *submit)
{
	struct dma_chan *chan = async_tx_find_channel(submit, DMA_XOR,
						      &dest, 1, src_list,
						      src_cnt, len);
	dma_addr_t *dma_src = NULL;

	BUG_ON(src_cnt <= 1);

	if (submit->scribble)
		dma_src = submit->scribble;
	else if (sizeof(dma_addr_t) <= sizeof(struct page *))
		dma_src = (dma_addr_t *) src_list;
#ifdef AMP_RAID5
	if (!(dma_src && chan && is_dma_xor_aligned(chan->device, offset, 0, len))) {
#else
	if (dma_src && chan && is_dma_xor_aligned(chan->device, offset, 0, len)) {
#endif
		/* run the xor asynchronously */
		pr_debug("%s (async): len: %zu\n", __func__, len);
#ifndef AMP_RAID5
		return do_async_xor(chan, dest, src_list, offset, src_cnt, len,
				    dma_src, submit);
#else
		return do_cvm_async_xor(chan, dest, src_list, offset, src_cnt, len,
                                   submit );
#endif /* AMP_RAID5 */
	} else {
		/* run the xor synchronously */
		pr_debug("%s (sync): len: %zu\n", __func__, len);
		WARN_ONCE(chan, "%s: no space for dma address conversion\n",
			  __func__);

		/* in the sync case the dest is an implied source
		 * (assumes the dest is the first source)
		 */
		if (submit->flags & ASYNC_TX_XOR_DROP_DST) {
			src_cnt--;
			src_list++;
		}

		/* wait for any prerequisite operations */
		async_tx_quiesce(&submit->depend_tx);

		do_sync_xor(dest, src_list, offset, src_cnt, len, submit);

		return NULL;
	}
}
EXPORT_SYMBOL_GPL(async_xor);

static int page_is_zero(struct page *p, unsigned int offset, size_t len)
{
	char *a = page_address(p) + offset;
	return ((*(u32 *) a) == 0 &&
		memcmp(a, a + 4, len - 4) == 0);
}

static inline struct dma_chan *
xor_val_chan(struct async_submit_ctl *submit, struct page *dest,
		 struct page **src_list, int src_cnt, size_t len)
{
	#ifdef CONFIG_ASYNC_TX_DISABLE_XOR_VAL_DMA
	return NULL;
	#endif
	return async_tx_find_channel(submit, DMA_XOR_VAL, &dest, 1, src_list,
				     src_cnt, len);
}

/**
 * async_xor_val - attempt a xor parity check with a dma engine.
 * @dest: destination page used if the xor is performed synchronously
 * @src_list: array of source pages
 * @offset: offset in pages to start transaction
 * @src_cnt: number of source pages
 * @len: length in bytes
 * @result: 0 if sum == 0 else non-zero
 * @submit: submission / completion modifiers
 *
 * honored flags: ASYNC_TX_ACK
 *
 * src_list note: if the dest is also a source it must be at index zero.
 * The contents of this array will be overwritten if a scribble region
 * is not specified.
 */
struct dma_async_tx_descriptor *
async_xor_val(struct page *dest, struct page **src_list, unsigned int offset,
	      int src_cnt, size_t len, enum sum_check_flags *result,
	      struct async_submit_ctl *submit)
{
	struct dma_chan *chan = xor_val_chan(submit, dest, src_list, src_cnt, len);
	struct dma_device *device = chan ? chan->device : NULL;
	struct dma_async_tx_descriptor *tx = NULL;
	dma_addr_t *dma_src = NULL;

	BUG_ON(src_cnt <= 1);

	if (submit->scribble)
		dma_src = submit->scribble;
	else if (sizeof(dma_addr_t) <= sizeof(struct page *))
		dma_src = (dma_addr_t *) src_list;

	if (dma_src && device && src_cnt <= device->max_xor &&
	    is_dma_xor_aligned(device, offset, 0, len)) {
		unsigned long dma_prep_flags = 0;
		int i;

		pr_debug("%s: (async) len: %zu\n", __func__, len);

		if (submit->cb_fn)
			dma_prep_flags |= DMA_PREP_INTERRUPT;
		if (submit->flags & ASYNC_TX_FENCE)
			dma_prep_flags |= DMA_PREP_FENCE;
		for (i = 0; i < src_cnt; i++)
			dma_src[i] = dma_map_page(device->dev, src_list[i],
						  offset, len, DMA_TO_DEVICE);

		tx = device->device_prep_dma_xor_val(chan, dma_src, src_cnt,
						     len, result,
						     dma_prep_flags);
		if (unlikely(!tx)) {
			async_tx_quiesce(&submit->depend_tx);

			while (!tx) {
				dma_async_issue_pending(chan);
				tx = device->device_prep_dma_xor_val(chan,
					dma_src, src_cnt, len, result,
					dma_prep_flags);
			}
		}

		async_tx_submit(chan, tx, submit);
	} else {
		enum async_tx_flags flags_orig = submit->flags;

		pr_debug("%s: (sync) len: %zu\n", __func__, len);
		WARN_ONCE(device && src_cnt <= device->max_xor,
			  "%s: no space for dma address conversion\n",
			  __func__);

		submit->flags |= ASYNC_TX_XOR_DROP_DST;
		submit->flags &= ~ASYNC_TX_ACK;

		tx = async_xor(dest, src_list, offset, src_cnt, len, submit);

		async_tx_quiesce(&tx);

		*result = !page_is_zero(dest, offset, len) << SUM_CHECK_P;

		async_tx_sync_epilog(submit);
		submit->flags = flags_orig;
	}

	return tx;
}
EXPORT_SYMBOL_GPL(async_xor_val);

#ifdef CONFIG_AMP
#ifdef AMP_RAID5
static int
raid5_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
       int num = 0;
       num += sprintf(page + num , "r5_sub:%lu, r5_pro:%lu, r5_wkup_recvd: %lu, r5_wkup:%lu, r5_ipi: %lu\n",
                       raid5_offload_submit, raid5_offload_success, raid5_wkup_rcvd,
                       raid5_offload_wkup, raid5_ipi_missed);
       num += sprintf(page + num , "xor_offload_submitted:%lu,xor_offload_completed :%lu,"
                                   "xor_offload_noop : %lu,xor_offload_success :%lu,\n"
                                   "xor_offload_cb : %lu\n",
                       xor_offload_submitted, xor_offload_completed, xor_offload_noop,
                       xor_offload_success, xor_offload_cb);
       return num;
}
#endif

static int __init async_xor_init(void)
{
#if AMP_RAID5
	static struct proc_dir_entry *raid5_proc_entry;
#endif
	#ifdef CONFIG_ASYNC_TX_DMA
	/* To conserve stack space the input src_list (array of page pointers)
	 * is reused to hold the array of dma addresses passed to the driver.
	 * This conversion is only possible when dma_addr_t is less than the
	 * the size of a pointer.  HIGHMEM64G is known to violate this
	 * assumption.
	 */
	BUILD_BUG_ON(sizeof(dma_addr_t) > sizeof(struct page *));
	#endif
#ifdef AMP_RAID5
	if(NULL == raid5_proc_entry) {
                raid5_proc_entry = create_proc_entry("raid5",
                                                   S_IFREG | S_IRUGO, NULL);
                if (raid5_proc_entry) {
                        raid5_proc_entry->read_proc = raid5_read_proc;
                }
	}
#endif

	return 0;
}

static void __exit async_xor_exit(void)
{
	do { } while (0);
}

module_init(async_xor_init);
module_exit(async_xor_exit);
#endif //#ifdef CONFIG_AMP

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("asynchronous xor/xor-zero-sum api");
MODULE_LICENSE("GPL");

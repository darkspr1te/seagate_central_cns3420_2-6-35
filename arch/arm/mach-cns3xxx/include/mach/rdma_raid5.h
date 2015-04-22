#ifndef _RDMA_RAID5_H_
#define _RDMA_RAID5_H_

#ifdef AMP_RAID5

#include <linux/async_tx.h>

#define MAX_ENTRIES_PER_SG	32
#define SG_QUEUE_SIZE		1024
#define SG_STATUS_DONE		0x00000008UL
#define QUEUE_SIZE		1024

struct xor_info {
	void **src_list;
	void *dest;
	int src_cnt;
	size_t len;
	unsigned int offset;
	int index;
	struct dma_async_tx_descriptor *tx;
	enum async_tx_flags flags;
};

typedef struct rdma_sgtable sg_t;
struct rdma_sgtable {
	u64     entry[MAX_ENTRIES_PER_SG];

	/* list_add_tail/list_del to/from process_q when schedule or isr */
	struct list_head lru;
	wait_queue_head_t wait;

	u32 status;
};

void call_raid5_callback(void *);
sg_t *alloc_sg_mem(void);
void free_sg_mem(sg_t *);
void prepare_xor_op(void *priv);
void xor_offload(int src_cnt, size_t len, void **src, void *dst, void *tx);

#endif /* AMP_RAID5 */
#endif /* _RDMA_RAID5_H_ */

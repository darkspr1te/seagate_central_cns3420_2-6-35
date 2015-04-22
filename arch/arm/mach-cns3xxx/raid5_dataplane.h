#ifdef AMP_RAID5
#include "rdma.h"

/* Used to transfer arguments to the dp core */
#define SG_QUEUE_SIZE 1024

struct xor_args_list {
	void *srcs[10];
	void *dst;
	size_t len;
	int src_cnt;
	sg_t *sg;
	struct dma_async_tx_descriptor *tx;
	int cb_ack;
};

int do_cns_rdma_xorgen(unsigned int src_no, unsigned int bytes,
			       void **bih_ptr, void *dst_ptr, sg_t *sg,
				enum async_tx_flags flags);
int check_xor_status(int);
void xor_offload(int src_cnt, size_t len, void **src, void *dst);
int cnsx_send_xor_req(void *);
void call_raid5_callback(void);
sg_t *alloc_sg_mem(void);
void free_sg_mem(sg_t *);
void rdma_timeout_handle(void);
#endif /* AMP_RAID5 */

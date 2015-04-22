#ifndef __RAID5_MEMCPY_OFFLOAD_H
#define __RAID5_MEMCPY_OFFLOAD_H

extern wait_queue_head_t raid5_memcpy_offload_waitq;
extern int raid5_memcpy_done;
//extern void (*fp_raid5_memcpy_offload)(char *, char *, int, dma_async_tx_callback, void *);
extern void (*fp_raid5_memcpy_offload)(char *, char *, int);
extern struct dma_async_tx_descriptor *
cavm_async_memcpy(struct page *dest, struct page *src, unsigned int dest_offset,
        unsigned int src_offset, size_t len,
        struct async_submit_ctl *submit);

typedef struct raid5_memcpy_args_list {
        char *src;
        char *dst;
        int len;
       dma_async_tx_callback cb_fn;
       void *cb_param;
} raid5_memcpy_args_list_t;


typedef struct raid5_memcpy_ret_args {
       dma_async_tx_callback cb_fn;
        void *cb_param;
} raid5_memcpy_ret_args_t;


#endif /* __RAID5_MEMCPY_OFFLOAD_H */







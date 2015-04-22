#ifndef __CVM_SPLICE_H
#define __CVM_SPLICE_H

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/pagemap.h>
#include <linux/splice.h>
#include <linux/memcontrol.h>
#include <linux/mm_inline.h>
#include <linux/swap.h>
#include <linux/writeback.h>
#include <linux/buffer_head.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uio.h>
#include <linux/security.h>
#include "cvm_nas_debug.h"
#include <asm/pmon_counters.h>

#include <linux/proc_fs.h>
#include <mach/dmac.h>
#include <linux/page-flags.h>
#include <linux/init.h>


#define TRUE 1
#define FALSE 0
#define MAX_PIPE_PAGES    20

//#define DISABLE_MEMCPY_OFFLOAD
#define CNSX_IPC_PTF_RING_SIZE 256

typedef struct ptf_args_list {
        char *src;
        char *dst;
        int len;
        int page_index;
#ifdef CONFIG_AMP_ALL
        int __padding[4];
#endif
} ptf_args_list_t;

typedef struct __page_arg
{
	struct page *page;
	void *fsdata;
	loff_t pos;
	unsigned int len;
	int last_req;
}page_arg;

#if defined(SUPPORT_SIM_IO) 

/* Last 16 bits of this magic must be Zero */
#define SIO_MAGIC	(0x0EAD0000)
#define SIO_SET_FD_ERR	(0xD0000000)
#define MAX_SIM_FD	(4096) /* Refer Trac #405 */


#define SIM_IO_DIR_SENDFILE	(0)
#define SIM_IO_DIR_RECVFILE (1)


#define SIM_SET_FD_INFO(filp, fd)           		\
do {                                        		\
    BUG_ON(filp->cvm_private_data != NULL);         	\
    filp->cvm_private_data = (void*)((SIO_MAGIC) | fd) ;\
}while(0);

#define SIM_SET_FD_INFO_ERR(filp)			\
do {                                        		\
    BUG_ON(filp->cvm_private_data == NULL);         	\
    filp->cvm_private_data = (void*)((SIO_SET_FD_ERR) | (unsigned int)filp->cvm_private_data) ;\
}while(0);

#define SIM_GET_FD_INFO(filp)               \
    (((((unsigned int)filp->cvm_private_data) & SIO_MAGIC) == SIO_MAGIC) \
        ? (((unsigned int)filp->cvm_private_data) & 0x0FFFF) : -1 );

#define SIM_IS_NEW_SESSION(filp)		\
	((((unsigned int)filp->cvm_private_data) & SIO_MAGIC) != SIO_MAGIC)

/* SIM_GET_FD_ERR_INFO returns -1 if SIO_SET_FD_ERR is set, else returns 0 */
#define SIM_GET_FD_ERR_INFO(filp)               \
    (((((unsigned int)filp->cvm_private_data) & SIO_SET_FD_ERR) == SIO_SET_FD_ERR) \
        ? (-1) : (0));

typedef struct _session_pipe_info_ {
	struct pipe_inode_info *splice_pipe[2][MAX_SIM_FD];
	uint32_t in_use[2][MAX_SIM_FD];
	int err_code[2][MAX_SIM_FD];
	/* If splice_write returns any error, err_code will be marked with
 	 * appropriate error return value */
} session_pipe_info_t;

int sim_io_init(void);
int sim_session_level_init(struct file *filp, int io_fd, int ai_dir);
void sim_session_level_cleanup(struct file *filp, int io_fd, int ai_dir);
struct pipe_inode_info *sim_get_internal_pipe(struct file *filp,
											int ai_dir);

#else /* SUPPORT_SIM_IO */

#define SIM_SET_FD_INFO(filp, fd) 
#define SIM_GET_FD_INFO(filp)

#endif /* SUPPORT_SIM_IO */


extern atomic_t outstanding_job_count ;
extern void wakeup_pipe_writers(struct pipe_inode_info *pipe);
extern int generic_write_sync(struct file *file, loff_t pos, loff_t count);
extern ssize_t default_file_splice_write(struct pipe_inode_info *pipe,
					 struct file *out, loff_t *ppos,
					 size_t len, unsigned int flags);

extern long cvm_pipe_to_file(struct pipe_inode_info *pipe, struct file *out, 
                              loff_t *ppos, size_t len, unsigned int flags);


extern struct pipe_inode_info *alloc_pipe_from_slab(struct kmem_cache* p_cache);
extern void free_pipe_to_slab(struct kmem_cache *p_cache,
                           struct pipe_inode_info *pipe);

#endif /* CVM_SPLICE_H */

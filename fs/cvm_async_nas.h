#ifndef __CVM_ASYNC_NAS_H
#define __CVM_ASYNC_NAS_H

#if defined(CONFIG_AMP)
#include <linux/pipe_fs_i.h>	
#include <linux/splice.h>
#include <linux/list.h>
#include "cvm_splice.h"

#define FAILURE -1
#define SUCCESS 0



#define MAX_BG_NAS_JOB_COUNT 25 
#define MAX_JOBS_TO_PROCESS 25 
#define NAS_SAFETY_TIMEOUT 10	

extern spinlock_t bg_nas_queue_lock;
extern struct list_head bg_nas_queue;
extern int bg_job_done;
extern wait_queue_head_t samba_waitq;
extern int cvm_async_write;

/*
 * The arguments that will be passed from SAMBA process to NAS process.
 * */
typedef struct _bg_nas_args_t {
	struct list_head bg_nas_arg_list;
        struct splice_desc *nas_sd;
        struct pipe_inode_info *nas_pipe;
	session_pipe_info_t *spi;
}bg_nas_args_t;



extern void 
init_bg_nas_task(void);

extern int
cvm_bg_nas_task(void *args);

extern void
cvm_submit_bg_job(session_pipe_info_t *spi, struct splice_desc *nas_sd, 
			struct pipe_inode_info *nas_pipe);

extern struct splice_desc *
cvm_clone_sd(struct splice_desc *samba_sd);

extern struct pipe_inode_info *
cvm_clone_pipe(struct pipe_inode_info *samba_pipe, int len, int flags);

extern int 
ipipe_prep(struct pipe_inode_info *pipe, unsigned int flags);

extern int 
opipe_prep(struct pipe_inode_info *pipe, unsigned int flags);

extern void
release_pipe_bufs(struct pipe_inode_info *pipe);


int cvm_link_pipe(struct pipe_inode_info *ipipe,
                     struct pipe_inode_info *opipe,
                     size_t len, unsigned int flags);


int cvm_splice_pipe_to_pipe(struct pipe_inode_info *ipipe,
                               struct pipe_inode_info *opipe,
                               size_t len, unsigned int flags);


void wait_for_cvm_aio_complete(void);
void wait_for_cvm_file_aio_complete(unsigned int fd, struct file *file);
void wait_for_cvm_session_aio_complete(session_pipe_info_t *spi_ptr,
                int io_fd, int ai_dir);

#endif
#endif /* CVM_ASYNC_NAS_H */

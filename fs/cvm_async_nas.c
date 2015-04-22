#ifdef CONFIG_AMP
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
#include <linux/kthread.h>

#include <linux/proc_fs.h>
#include "cvm_async_nas.h"
#include <linux/delay.h>
#include "cvm_nas_debug.h"
#include "cvm_splice.h"

int bg_nas_task_submitted=0;
int max_jobs_to_process = MAX_JOBS_TO_PROCESS;
int max_jobs_to_submit = MAX_BG_NAS_JOB_COUNT;
wait_queue_head_t bg_nas_task_waitq;
int cvm_async_write = 1 ; 
EXPORT_SYMBOL(cvm_async_write);

struct proc_dir_entry *bg_nas_proc_entry;
spinlock_t bg_nas_queue_lock;
struct list_head bg_nas_queue;
atomic_t outstanding_job_count;
extern atomic_t is_sync_fs;

void wait_for_cvm_aio_complete(void)
{
        while ((atomic_read(&outstanding_job_count)) != 0)
        {
                bg_nas_task_submitted=1;
                wake_up(&bg_nas_task_waitq);
		msleep(1);
        }
}
EXPORT_SYMBOL(wait_for_cvm_aio_complete);

void wait_for_cvm_file_aio_complete(unsigned int fd, struct file *file)
{
        int simfd;
        session_pipe_info_t *spi_ptr;
        simfd = SIM_GET_FD_INFO(file);

        if ((simfd != -1) && (simfd == fd))
        {
                spi_ptr = current->session_pipe_info;
                BUG_ON(spi_ptr == NULL);
                while (spi_ptr->in_use[SIM_IO_DIR_RECVFILE][fd] != 0)
                {
                        bg_nas_task_submitted=1;
                        wake_up(&bg_nas_task_waitq);
			msleep(1);
                }
        }
}
EXPORT_SYMBOL(wait_for_cvm_file_aio_complete);

void wait_for_cvm_session_aio_complete(session_pipe_info_t *spi_ptr,
                int io_fd, int ai_dir)
{
        while (spi_ptr->in_use[ai_dir][io_fd] != 0)
        {
                bg_nas_task_submitted=1;
                wake_up(&bg_nas_task_waitq);
		msleep(1);
        }
}
EXPORT_SYMBOL(wait_for_cvm_session_aio_complete);

#if 0
static void
dump_pipe(struct pipe_inode_info *pipe)
{
	int i;
	struct pipe_buffer *tmp_buf;

	for(i=0; i < pipe->nrbufs; i++)
	{
		tmp_buf = &pipe->bufs[i];
		printk(KERN_CRIT"%s:pg: 0x%x pg_cnt: %d, offset: %d, len: %d\n",
                                                    __FUNCTION__, tmp_buf->page,
                                page_count(tmp_buf->page), tmp_buf->offset, 
				tmp_buf->len);
	}
}
#endif


/*
 * Clone the SAMBA pipe. This function creates a new pipe and links the SAMBA
 * pipe to this new pipe. "Linking" means that pages that are part of the 
 * original pipe become a part of the new pipe. The page counts are incremented
 * so that the original pipe can be freed by SAMBA. 
 */

struct pipe_inode_info *
cvm_clone_pipe(struct pipe_inode_info *samba_pipe, int len, int flags) 
{
	struct pipe_inode_info *nas_pipe;

	nas_pipe = alloc_pipe_info(NULL);
	if(!nas_pipe)
	{
		printk(KERN_CRIT"Allocation of NAS pipe failed\n");
		return NULL;
	}
	nas_pipe->readers = 1;

	cvm_splice_pipe_to_pipe(samba_pipe, nas_pipe, len, flags);	
	
	return nas_pipe;
}

/*
 * Clone the splice descriptor. The splice descriptor stores the disk file 
 * related information that will be needed to transfer pipe data to disk.
 */
struct splice_desc *
cvm_clone_sd(struct splice_desc *samba_sd) 
{
	struct splice_desc *nas_sd;

	nas_sd = kmalloc(sizeof(*nas_sd), GFP_KERNEL);
	if(!nas_sd)
	{
		printk(KERN_CRIT"Allocation of SD failed\n");
		return NULL;
	}


	/*
 	 * Increment the file count, this is done here to ensure that even if 
 	 * SAMBA process closes/delets the file we are fine.
 	 */
	
	memcpy(nas_sd, samba_sd, sizeof(*nas_sd));
	atomic_inc(&(nas_sd->u.file->f_count));

	return nas_sd;
}

static void
add_job(bg_nas_args_t *bg_nas_args)
{

	spin_lock(&bg_nas_queue_lock);

	list_add_tail(&(bg_nas_args->bg_nas_arg_list), &bg_nas_queue);
	atomic_inc(&outstanding_job_count);

	spin_unlock(&bg_nas_queue_lock);
	
	return;
}


/*
 * SAMBA process uses this to submit the job to NAS background task.
 */
void
cvm_submit_bg_job(session_pipe_info_t *spi, struct splice_desc *nas_sd, 
			struct pipe_inode_info *nas_pipe)
{
	bg_nas_args_t *bg_nas_args;	
        int nas_fd = SIM_GET_FD_INFO(nas_sd->u.file);
		
	bg_nas_args = kmalloc(sizeof(*bg_nas_args), GFP_KERNEL);
	if(NULL == bg_nas_args)
	{
		//Handle error.
	}
	
	bg_nas_args->nas_sd = nas_sd;
	bg_nas_args->nas_pipe = nas_pipe;	
	bg_nas_args->spi = spi;

	NAS_INC_CNTR(NAS_CVM_SUBMIT_BG_JOB);
        BG_INC_CNTR(BG_NAS_JOB_CNT);

	/*
	 * Slow down the job submission rate in case we have too many
	 * outstanding jobs.
	 */
	while ((atomic_read(&outstanding_job_count) > max_jobs_to_submit) ||
                                             (atomic_read(&is_sync_fs) > 0))
	{
		//printk(KERN_CRIT"%s: Sleeping due to too many jobs: %d\n", 
		//        __FUNCTION__, atomic_read(&outstanding_job_count));
		BG_INC_CNTR(BG_NAS_SLEEP_CNT);
		bg_nas_task_submitted=1;
		wake_up(&bg_nas_task_waitq);
		msleep(1);
	}
	add_job(bg_nas_args);
	/*
 	 * Test in_use counter for non-negative and Increment it
 	 * whenever a job is added
 	 */
	BUG_ON((spi->in_use[SIM_IO_DIR_RECVFILE][nas_fd]) < 0);
	spi->in_use[SIM_IO_DIR_RECVFILE][nas_fd]++;
	
	bg_nas_task_submitted=1;
	wake_up(&bg_nas_task_waitq);
}


static bg_nas_args_t *
get_job(void)
{
	bg_nas_args_t *temp_arg;

	spin_lock(&bg_nas_queue_lock);
	if(list_empty(&bg_nas_queue)) 
	{
		temp_arg = NULL;
	}
	else
	{
		temp_arg = list_first_entry(&bg_nas_queue, bg_nas_args_t, 
                                                        bg_nas_arg_list);
		list_del(&(temp_arg->bg_nas_arg_list));
	}
	spin_unlock(&bg_nas_queue_lock);

	return temp_arg;
}


static void 
cvm_free_pipe_info(struct pipe_inode_info *pipe)
{
        int i;

        for (i = 0; i < pipe->buffers; i++) {
                struct pipe_buffer *buf = pipe->bufs + i;
                if (buf->ops)
                        buf->ops->release(pipe, buf);
        }
        if (pipe->tmp_page)
                __free_page(pipe->tmp_page);
        kfree(pipe->bufs);
        kfree(pipe);
}

int
handle_nas_job(void)
{
	struct splice_desc *nas_sd;
	struct pipe_inode_info *nas_pipe;
	bg_nas_args_t *temp_arg;
	int processed_job_count=0;
	int bytes_written = 0;
	session_pipe_info_t *spi;
        int nas_fd;
        int fd_err = 0;


	/*
 	 * Process the jobs in batches so that performance is improved.
 	 */
	while (processed_job_count < max_jobs_to_process)
	{

		temp_arg = get_job();
		if(NULL == temp_arg)
		{
			break;
		}
	
		nas_sd = temp_arg->nas_sd;
		nas_pipe = temp_arg->nas_pipe;	
		spi = temp_arg->spi;

		kfree(temp_arg);	

                /* 
 		 * Get file pointer and see if its already marked with ERROR 
 		 */
                nas_fd = SIM_GET_FD_INFO(nas_sd->u.file);
                fd_err = SIM_GET_FD_ERR_INFO(nas_sd->u.file);
                if (fd_err == -1)
                {
                        bytes_written = 
                                     spi->err_code[SIM_IO_DIR_RECVFILE][nas_fd];

                        /* 
 			 * SAMBA wouldn't release these queued pipe buffers 
 			 * during error case, so release pipe and its pages 
                         */
                        release_pipe_bufs(nas_pipe);

                        /* 
 			 * As do_splice_from has returned error for this 
 			 * session already, clean-up this pipe and process 
 			 * next queued job 
                         */
                        goto skip_splice;
                }

		
		/*
		 * Use generic_file_splice_write to transfer data from pipe
		 * to file. This code path follows the same path as the original
		 * recvfile i.e this will make use of 
		 * cvm_splice_from_pipe_feed(), memcpy offload etc.
		 */ 


		NAS_INC_CNTR(NAS_PROCESS_BG_JOB);
		bytes_written =  cvm_pipe_to_file (nas_pipe, nas_sd->u.file, 
		//bytes_written =  do_splice_from(nas_pipe, nas_sd->u.file, 
							  &nas_sd->pos, 
							  nas_sd->total_len,
							  nas_sd->flags);	


                /* 
 		 * Handle the error scenario, 
 		 * Note: Check the case of bytes_written = 0, right now
 		 * it is considered as a success.
 		 */
                if (unlikely(bytes_written<= 0))
                {
                        /* 
 			 * Mark this session-file pointer with ERROR 
                         * Samba while submitting a job in splice, will check 
                         * for the ERROR and if it's set, will not submit a 
                         * new job and simply return error to samba
                         */
                        SIM_SET_FD_INFO_ERR(nas_sd->u.file);
                        spi->err_code[SIM_IO_DIR_RECVFILE][nas_fd] = 
                                                       bytes_written;

                        /* 
 			 * SAMBA wouldn't release these queued pipe buffers 
 			 * during error case, so release pipe and its pages 
                         */
                        release_pipe_bufs(nas_pipe);

                        /* 
 			 * skip_splice will clean-up this pipe and start 
 			 * processing next job 
 			 */
                }

skip_splice:
		/*
		 * Test in_use counter for non-negative and decrement it
		 * whenever a job is processed
		 */
                BUG_ON((spi->in_use[SIM_IO_DIR_RECVFILE][nas_fd]) < 0);
                spi->in_use[SIM_IO_DIR_RECVFILE][nas_fd]--;

		/*
		 * Manage the job counters.
		 * Note:  These counters should/can be used to tune 
		 * performance at a later stage. 
		 */
		processed_job_count++;
		atomic_dec(&outstanding_job_count);

		/*
		 * Release the NAS task pipe/pages. The original SAMBA pipe wo
		 * would be managed by SAMBA process.
		 */
		cvm_free_pipe_info(nas_pipe);	

		/*
		 * Decrement the file count to notify the kernel that we are not
		 * using the file now and then free the splice descriptor.
		 */
		atomic_dec(&(nas_sd->u.file->f_count));
		kfree(nas_sd);
	}
	
	return 0;
}



/*
 * NAS background task, this process transfers data from the pipe to disk file.
 * The basic criteria of the design is to enable recvfile() to return ASAP. 
 * Code is written in this manner so that we don't touch cvm_splice_* function 
 * again, idea is to reuse as much of the existing code.
 */
int
cvm_bg_nas_task(void *args)
{
	int ret;
	
	while(1)
	{

		/*
 		 * Wait for a Job to be submitted by SAMBA process.
 		 * Note: The reason for the timeout is that we can get a 
 		 * state where SAMBA submits a job and issues a wakeup but NAS
 		 * job misses that because it is processing something. If this
 		 * happens for the last packet, then NAS job will end up in a 
 		 * wait state till the next SAMBA job request comes. The current
 		 * solution is not elegant, but should work. 
 		 */
		wait_event_timeout (bg_nas_task_waitq, bg_nas_task_submitted,
                                                       NAS_SAFETY_TIMEOUT);
		bg_nas_task_submitted = 0;
		
		ret = handle_nas_job();


		//This is used in case we are trying to debug things, not 
		//useful in real code.	
		bg_job_done = 1;		
		wake_up(&samba_waitq);

	}
	
	return SUCCESS;
}

void
init_bg_nas_task(void)
{
	atomic_set(&outstanding_job_count, 0);
	init_waitqueue_head(&bg_nas_task_waitq);
	spin_lock_init(&bg_nas_queue_lock);
	INIT_LIST_HEAD(&bg_nas_queue);

	//Dummy waitqueue
	init_waitqueue_head(&samba_waitq);

	kthread_run(cvm_bg_nas_task, NULL, "cvm_nas_task");
}

EXPORT_SYMBOL(init_bg_nas_task);
EXPORT_SYMBOL(cvm_bg_nas_task);
EXPORT_SYMBOL(cvm_submit_bg_job);
EXPORT_SYMBOL(cvm_clone_sd);
EXPORT_SYMBOL(cvm_clone_pipe);

#endif

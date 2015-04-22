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

#include "cvm_async_nas.h"
#include "cvm_splice.h"
#include "cvm_nas_debug.h"

wait_queue_head_t ptf_waitq;
EXPORT_SYMBOL(ptf_waitq);

static int recvfile_initialized=0;
unsigned long total_data_transmit=0;
/* static int pipe_to_file_cnt=0; */
wait_queue_head_t samba_waitq;
int bg_job_done=0;

static struct ptf_args_list ptf_args[CNSX_IPC_PTF_RING_SIZE];
static int ptf_idx = 0;

int (*ptf_offload_req) (void *) = NULL;

volatile uint32_t ptf_tot_req, ptf_tot_resp;
EXPORT_SYMBOL(ptf_tot_req);
EXPORT_SYMBOL(ptf_tot_resp);


unsigned long memcpy_cnt=0;
unsigned long page_alloc_cnt=0;
unsigned long page_write_cnt=0;


static struct kmem_cache *fs_pipe_cache = NULL;
struct mutex fs_slab_lock;

#define BUGG_ON(x)	\
{					\
	if(x) {			\
		printk("BUG_ON : File: %s Func : %s Line %d\n", __FILE__, \
					__func__, __LINE__); \
		BUG(); \
		while(1); \
	} \
}

extern ssize_t splice_direct_from_actor(struct file *in, struct splice_desc *sd,
                                              splice_direct_actor *actor);

extern int direct_splice_actor(struct pipe_inode_info *pipe,
			            struct splice_desc *sd);

/*
 * Attempt to steal a page from a pipe buffer. This should perhaps go into
 * a vm helper function, it's already simplified quite a bit by the
 * addition of remove_mapping(). If success is returned, the caller may
 * attempt to reuse this page for another destination.
 */
static int 
page_cache_pipe_buf_steal(struct pipe_inode_info *pipe,
				     struct pipe_buffer *buf)
{
	struct page *page = buf->page;
	struct address_space *mapping;

	lock_page(page);

	mapping = page_mapping(page);
	if (mapping) {
		WARN_ON(!PageUptodate(page));

		/*
		 * At least for ext2 with nobh option, we need to wait on
		 * writeback completing on this page, since we'll remove it
		 * from the pagecache.  Otherwise truncate wont wait on the
		 * page, allowing the disk blocks to be reused by someone else
		 * before we actually wrote our data to them. fs corruption
		 * ensues.
		 */
		wait_on_page_writeback(page);

		if (page_has_private(page) &&
		    !try_to_release_page(page, GFP_KERNEL))
			goto out_unlock;

		/*
		 * If we succeeded in removing the mapping, set LRU flag
		 * and return good.
		 */
		if (remove_mapping(mapping, page)) {
			buf->flags |= PIPE_BUF_FLAG_LRU;
			return 0;
		}
	}

	/*
	 * Raced with truncate or failed to remove page from current
	 * address space, unlock and return failure.
	 */
out_unlock:
	unlock_page(page);
	return 1;
}

static void 
page_cache_pipe_buf_release(struct pipe_inode_info *pipe,
					struct pipe_buffer *buf)
{
	page_cache_release(buf->page);
	buf->flags &= ~PIPE_BUF_FLAG_LRU;
}

/*
 * Check whether the contents of buf is OK to access. Since the content
 * is a page cache page, IO may be in flight.
 */
static int 
page_cache_pipe_buf_confirm(struct pipe_inode_info *pipe,
				       struct pipe_buffer *buf)
{
	struct page *page = buf->page;
	int err;

	if (!PageUptodate(page)) {
		lock_page(page);

		/*
		 * Page got truncated/unhashed. This will cause a 0-byte
		 * splice, if this is the first page.
		 */
		if (!page->mapping) {
			err = -ENODATA;
			goto error;
		}

		/*
		 * Uh oh, read-error from disk.
		 */
		if (!PageUptodate(page)) {
			err = -EIO;
			goto error;
		}

		/*
		 * Page is ok afterall, we are done.
		 */
		unlock_page(page);
	}

	return 0;
error:
	unlock_page(page);
	return err;
}

static const struct pipe_buf_operations page_cache_pipe_buf_ops = {
        .can_merge = 0,
        .map = generic_pipe_buf_map,
        .unmap = generic_pipe_buf_unmap,
        .confirm = page_cache_pipe_buf_confirm,
        .release = page_cache_pipe_buf_release,
        .steal = page_cache_pipe_buf_steal,
        .get = generic_pipe_buf_get,
};   

static void cvm_spd_release_page(struct splice_pipe_desc *spd, unsigned int i)
{
	page_cache_release(spd->pages[i]);
}


/* 
 * Per process inits, called for each SAMBA process. 
 */
int 
sim_io_init(void)
{

	session_pipe_info_t *spi_ptr;
	BUGG_ON(current->session_pipe_info != NULL);
	
	spi_ptr = kzalloc(sizeof(session_pipe_info_t), GFP_KERNEL);
	if(unlikely(!spi_ptr)) {
		printk("%s(): %d - Memory Exhausted during session pipe info" 
					  " allocation", __func__, __LINE__);
		BUG();
		return -1;
	}

	if(unlikely(!fs_pipe_cache)) {
		fs_pipe_cache = kmem_cache_create("pipe_cache", 
				   sizeof(struct pipe_inode_info), 0, 0, NULL);
		
		if(unlikely(!fs_pipe_cache)) {
			printk("%s() : %d - Memory exhausted during pipe "
                                        "allocation", __func__, __LINE__);
			BUG();
			return -1;
		}
		
		/* Initialize the mutex controlling slab allocation */
		mutex_init(&fs_slab_lock);
	}
	current->session_pipe_info = spi_ptr;
	return 0;
}

/* 
 * This routine is called, before each session starts for allocating a pipe
 * to that session.
 * A session is a 'Read' or a 'Write' transaction, triggered by sendfile or 
 * recvfile respectively. Either call this in open, or let the first 
 * recvfile per transacation have this routine called. 
 */
int 
sim_session_level_init(struct file *filp, int io_fd, int ai_dir)
{
	session_pipe_info_t *spi_ptr = current->session_pipe_info;
	struct pipe_inode_info *alloc_pipe;

	BUGG_ON(spi_ptr == NULL);
	if(io_fd >= MAX_SIM_FD) {
		printk("FD is above the limit %d\n", io_fd);
		BUGG_ON(io_fd >= MAX_SIM_FD);
	}

	if((ai_dir != SIM_IO_DIR_RECVFILE) && (ai_dir != SIM_IO_DIR_SENDFILE)) {
		printk(" Direction Corrupt : %d\n",ai_dir);
		BUGG_ON((ai_dir != SIM_IO_DIR_RECVFILE) &&  \
					(ai_dir != SIM_IO_DIR_SENDFILE));
	}

	/*
 	 * Allocate a pipe for this session and store it, this pipe will be in
 	 * use till close is called for this session.
 	 */
	mutex_lock(&fs_slab_lock);	
	alloc_pipe = alloc_pipe_from_slab(fs_pipe_cache);
	mutex_unlock(&fs_slab_lock);

	if(unlikely(!alloc_pipe)) {
		printk("%s() - %d : pipe allocation failed \n", 
                                           __func__, __LINE__);
		BUG();
		return -1;
	}
	spi_ptr->splice_pipe[ai_dir][io_fd] = alloc_pipe;

	/* 
         * Fix : When a file is opened in RW mode, it can be used for the read 
	 * and write operations subsequently without causing a close and with 
	 * just one init being done. So if the mode is rw, then duplicate both 
	 * the directions to point to the same pipe 
	 */
	if((filp->f_flags & O_ACCMODE) == O_RDWR) {
		spi_ptr->splice_pipe[!ai_dir][io_fd] = alloc_pipe;
		spi_ptr->in_use[!ai_dir][io_fd] = 0;
	}
	
	spi_ptr->in_use[ai_dir][io_fd] = 0;
	spi_ptr->err_code[ai_dir][io_fd] = 0; /* set err_code to 0 initially */
	
	SIM_SET_FD_INFO(filp, io_fd);	
	return 0;
}

/*
 * This function is called whenever a SAMBA session is closed, the function
 * cleans up the session pipe/file descriptors.
 */
void 
sim_session_level_cleanup(struct file *filp, int io_fd, int ai_dir)
{
	session_pipe_info_t *spi_ptr = current->session_pipe_info;
	struct pipe_inode_info *pipe;

	if((((unsigned int)filp->cvm_private_data) & SIO_MAGIC) != SIO_MAGIC) {
		return;
	}

	BUGG_ON(spi_ptr == NULL);
	if(io_fd >= MAX_SIM_FD) {
		printk(" IO FD is too large %d. Dir = %d\n", io_fd, ai_dir);
		BUGG_ON(io_fd >= MAX_SIM_FD);
	}
	BUGG_ON((ai_dir != SIM_IO_DIR_RECVFILE) && 
                                            (ai_dir != SIM_IO_DIR_SENDFILE));
	
	pipe = spi_ptr->splice_pipe[ai_dir][io_fd];
	BUGG_ON(pipe == NULL);

#if defined(CONFIG_AMP_ALL) || defined(CONFIG_AMP_PURENAS)
	if (cvm_async_write) 
        	wait_for_cvm_session_aio_complete(spi_ptr, io_fd, ai_dir);
#endif
	mutex_lock(&fs_slab_lock);

	if((filp->f_flags & O_ACCMODE) == O_RDWR) {	
		spi_ptr->in_use[!ai_dir][io_fd] = 0;
		spi_ptr->splice_pipe[!ai_dir][io_fd] = NULL;
	}
	
	spi_ptr->splice_pipe[ai_dir][io_fd] = NULL;

	spi_ptr->in_use[ai_dir][io_fd] = 0;
	spi_ptr->err_code[ai_dir][io_fd] = 0;

	free_pipe_to_slab(fs_pipe_cache, pipe);	
	
	mutex_unlock(&fs_slab_lock);

	filp->private_data = 0;
	filp->cvm_private_data = 0;
	return;
}

struct 
pipe_inode_info *sim_get_internal_pipe(struct file *filp, int ai_dir)
{
	session_pipe_info_t *spi;
	struct pipe_inode_info *pipe;
	int fd;

	fd = SIM_GET_FD_INFO(filp);
	if(unlikely(fd == -1)) {
		printk(" session init not done for this or corrupteid. %x\n",
							(unsigned int)filp);
		BUGG_ON(fd == -1);
		return NULL;
	}

	if(fd >= MAX_SIM_FD) {
		printk(" Too large FD = %d, Dire  = %d\n", fd, ai_dir);
		BUGG_ON(fd >= MAX_SIM_FD);
   	}
	BUGG_ON((ai_dir != SIM_IO_DIR_RECVFILE) && 
                  (ai_dir != SIM_IO_DIR_SENDFILE));
	
	spi =  ((session_pipe_info_t*)current->session_pipe_info);
	pipe = spi->splice_pipe[ai_dir][fd];

	BUGG_ON(pipe == NULL);

	return pipe;
}

/*
 * Check if we need to grow the arrays holding pages and partial page
 * descriptions.
 */
static int 
cvm_splice_grow_spd(struct pipe_inode_info *pipe, struct splice_pipe_desc *spd)
{
	if (pipe->buffers <= PIPE_DEF_BUFFERS)
		return 0;

	spd->pages = kmalloc(pipe->buffers * sizeof(struct page *), GFP_KERNEL);
	spd->partial = kmalloc(pipe->buffers * sizeof(struct partial_page), 
                                                               GFP_KERNEL);

	if (spd->pages && spd->partial)
		return 0;

	kfree(spd->pages);
	kfree(spd->partial);
	return -ENOMEM;
}

static void 
cvm_splice_shrink_spd(struct pipe_inode_info *pipe,
		       struct splice_pipe_desc *spd)
{
	if (pipe->buffers <= PIPE_DEF_BUFFERS)
		return;

	kfree(spd->pages);
	kfree(spd->partial);
}


/*
 * cvm_splice_to_pipe - Fill passed data in the splice descriptor into a pipe. 
 * @pipe:	pipe to fill
 * @spd:	data to fill
 *
 * Description:
 *    @spd contains a map of pages and len/offset tuples, along with
 *    the struct pipe_buf_operations associated with these pages. This
 *    function will link that data to the pipe.Data is filled into the splice 
 *    descriptor from a socket or disk depending on whether we are doing 
 *    client write or read.   
 */
ssize_t 
cvm_splice_to_pipe(struct pipe_inode_info *pipe,
		       struct splice_pipe_desc *spd)
{
	unsigned int spd_pages = spd->nr_pages;
	int ret, do_wakeup, page_nr;

	ret = 0;
	do_wakeup = 0;
	page_nr = 0;

	//pipe_lock(pipe);

	for (;;) {
		if (!pipe->readers) {
			send_sig(SIGPIPE, current, 0);
			if (!ret)
				ret = -EPIPE;
			break;
		}

		if (pipe->nrbufs < pipe->buffers) {
			int newbuf = (pipe->curbuf + pipe->nrbufs) & 
                                                 (pipe->buffers - 1);
			struct pipe_buffer *buf = pipe->bufs + newbuf;

			buf->page = spd->pages[page_nr];
			buf->offset = spd->partial[page_nr].offset;
			buf->len = spd->partial[page_nr].len;
			buf->private = spd->partial[page_nr].private;
			buf->ops = spd->ops;
			if (spd->flags & SPLICE_F_GIFT)
				buf->flags |= PIPE_BUF_FLAG_GIFT;

			pipe->nrbufs++;
			page_nr++;
			ret += buf->len;

			if (pipe->inode)
				do_wakeup = 1;

			if (!--spd->nr_pages)
				break;
			if (pipe->nrbufs < pipe->buffers)
				continue;

			break;
		}

		if (spd->flags & SPLICE_F_NONBLOCK) {
			if (!ret)
				ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			if (!ret)
				ret = -ERESTARTSYS;
			break;
		}

		if (do_wakeup) {
			smp_mb();
			if (waitqueue_active(&pipe->wait))
				wake_up_interruptible_sync(&pipe->wait);
			kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
			do_wakeup = 0;
		}

		pipe->waiting_writers++;
		pipe_wait(pipe);
		pipe->waiting_writers--;
	}

	//pipe_unlock(pipe);

	if (do_wakeup) {
		smp_mb();
		if (waitqueue_active(&pipe->wait))
			wake_up_interruptible(&pipe->wait);
		kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
	}

	while (page_nr < spd_pages)
		spd->spd_release(spd, page_nr++);

	return ret;
}

/*
 * Worker function for reading data from disk/socket into a pipe.
 */
static int
__cvm_generic_file_splice_read(struct file *in, loff_t *ppos,
			   struct pipe_inode_info *pipe, size_t len,
			   unsigned int flags)
{
	struct address_space *mapping = in->f_mapping;
	unsigned int loff, nr_pages, req_pages;
	struct page *pages[PIPE_DEF_BUFFERS];
	struct partial_page partial[PIPE_DEF_BUFFERS];
	struct page *page;
	pgoff_t index, end_index;
	loff_t isize;
	int error, page_nr;
	struct splice_pipe_desc spd = {
		.pages = pages,
		.partial = partial,
		.flags = flags,
		.ops = &page_cache_pipe_buf_ops,
		.spd_release = cvm_spd_release_page,
	};

	if (cvm_splice_grow_spd(pipe, &spd))
		return -ENOMEM;

	index = *ppos >> PAGE_CACHE_SHIFT;
	loff = *ppos & ~PAGE_CACHE_MASK;
	req_pages = (len + loff + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
	nr_pages = min(req_pages, pipe->buffers);

	/*
	 * Lookup the (hopefully) full range of pages we need.
	 */
	spd.nr_pages = find_get_pages_contig(mapping, index, nr_pages, 
                                                           spd.pages);
	index += spd.nr_pages;

	/*
	 * If find_get_pages_contig() returned fewer pages than we needed,
	 * readahead/allocate the rest and fill in the holes.
	 */
	if (spd.nr_pages < nr_pages)
		page_cache_sync_readahead(mapping, &in->f_ra, in,
				index, req_pages - spd.nr_pages);

	error = 0;
	while (spd.nr_pages < nr_pages) {
		/*
		 * Page could be there, find_get_pages_contig() breaks on
		 * the first hole.
		 */
		page = find_get_page(mapping, index);
		if (!page) {
			/*
			 * page didn't exist, allocate one.
			 */
			page = page_cache_alloc_cold(mapping);
			if (!page)
				break;

			error = add_to_page_cache_lru(page, mapping, index,
						GFP_KERNEL);
			if (unlikely(error)) {
				page_cache_release(page);
				if (error == -EEXIST)
					continue;
				break;
			}
			/*
			 * add_to_page_cache() locks the page, unlock it
			 * to avoid convoluting the logic below even more.
			 */
			unlock_page(page);
		}

		spd.pages[spd.nr_pages++] = page;
		index++;
	}

	/*
	 * Now loop over the map and see if we need to start IO on any
	 * pages, fill in the partial map, etc.
	 */
	index = *ppos >> PAGE_CACHE_SHIFT;
	nr_pages = spd.nr_pages;
	spd.nr_pages = 0;
	for (page_nr = 0; page_nr < nr_pages; page_nr++) {
		unsigned int this_len;

		if (!len)
			break;

		/*
		 * this_len is the max we'll use from this page
		 */
		this_len = min_t(unsigned long, len, PAGE_CACHE_SIZE - loff);
		page = spd.pages[page_nr];

		if (PageReadahead(page))
			page_cache_async_readahead(mapping, &in->f_ra, in,
					page, index, req_pages - page_nr);

		/*
		 * If the page isn't uptodate, we may need to start io on it
		 */
		if (!PageUptodate(page)) {
			/*
			 * If in nonblock mode then dont block on waiting
			 * for an in-flight io page
			 */
			if (flags & SPLICE_F_NONBLOCK) {
				if (!trylock_page(page)) {
					error = -EAGAIN;
					break;
				}
			} else
				lock_page(page);

			/*
			 * Page was truncated, or invalidated by the
			 * filesystem.  Redo the find/create, but this time the
			 * page is kept locked, so there's no chance of another
			 * race with truncate/invalidate.
			 */
			if (!page->mapping) {
				unlock_page(page);
				page = find_or_create_page(mapping, index,
						mapping_gfp_mask(mapping));

				if (!page) {
					error = -ENOMEM;
					break;
				}
				page_cache_release(spd.pages[page_nr]);
				spd.pages[page_nr] = page;
			}
			/*
			 * page was already under io and is now done, great
			 */
			if (PageUptodate(page)) {
				unlock_page(page);
				goto fill_it;
			}

			/*
			 * need to read in the page
			 */
			error = mapping->a_ops->readpage(in, page);
			if (unlikely(error)) {
				/*
				 * We really should re-lookup the page here,
				 * but it complicates things a lot. Instead
				 * lets just do what we already stored, and
				 * we'll get it the next time we are called.
				 */
				if (error == AOP_TRUNCATED_PAGE)
					error = 0;

				break;
			}
		}
fill_it:
		/*
		 * i_size must be checked after PageUptodate.
		 */
		isize = i_size_read(mapping->host);
		end_index = (isize - 1) >> PAGE_CACHE_SHIFT;
		if (unlikely(!isize || index > end_index))
			break;

		/*
		 * if this is the last page, see if we need to shrink
		 * the length and stop
		 */
		if (end_index == index) {
			unsigned int plen;

			/*
			 * max good bytes in this page
			 */
			plen = ((isize - 1) & ~PAGE_CACHE_MASK) + 1;
			if (plen <= loff)
				break;

			/*
			 * force quit after adding this page
			 */
			this_len = min(this_len, plen - loff);
			len = this_len;
		}

		spd.partial[page_nr].offset = loff;
		spd.partial[page_nr].len = this_len;
		len -= this_len;
		loff = 0;
		spd.nr_pages++;
		index++;
	}

	/*
	 * Release any pages at the end, if we quit early. 'page_nr' is how far
	 * we got, 'nr_pages' is how many pages are in the map.
	 */
	while (page_nr < nr_pages)
		page_cache_release(spd.pages[page_nr++]);
	in->f_ra.prev_pos = (loff_t)index << PAGE_CACHE_SHIFT;

	if (spd.nr_pages)
		error = cvm_splice_to_pipe(pipe, &spd);

	cvm_splice_shrink_spd(pipe, &spd);
	return error;
}

/*
 * Read data from a socket/disk into a pipe.
 */
static ssize_t 
cvm_generic_file_splice_read(struct file *in, loff_t *ppos,
				 struct pipe_inode_info *pipe, size_t len,
				 unsigned int flags)
{
	loff_t isize, left;
	int ret;

	isize = i_size_read(in->f_mapping->host);
	if (unlikely(*ppos >= isize))
		return 0;

	left = isize - *ppos;
	if (unlikely(left < len))
		len = left;

	ret = __cvm_generic_file_splice_read(in, ppos, pipe, len, flags);
	if (ret > 0) {
		*ppos += ret;
		file_accessed(in);
	}

	return ret;
}


/*
 * Move data from disk to a pipe.
 */
static long 
cvm_disk_to_pipe(struct file *in, loff_t *ppos, struct pipe_inode_info *pipe,
                                                size_t len, unsigned int flags)
{
	ssize_t (*splice_read)(struct file *, loff_t *,
			       struct pipe_inode_info *, size_t, unsigned int);
	int ret;

	if (unlikely(!(in->f_mode & FMODE_READ)))
		return -EBADF;

	ret = rw_verify_area(READ, in, ppos, len);
	if (unlikely(ret < 0))
		return ret;

	if (in->f_op && in->f_op->splice_read)
		splice_read = in->f_op->splice_read;
	else
		splice_read = default_file_splice_read;

	/*
	 * We are trying not to mix kernel code and Cavium specific code as 
	 * much as possible. Given that filesystems register splice_read 
	 * function and we don't change touch FS code, the function call below 
	 * will end up calling that generic function (normally this happens to
	 * be generic_file_splice_read) which will mean that we have to go and
	 * modify that generic function to make it call into our code.
	 *
	 * Instead of that, check whether we are trying to call the generic 
	 * function and if so go ahead and call our version straightaway. 
	 * This way the generic function can be left unmodified.
	 */
	if(splice_read == generic_file_splice_read)
		splice_read = cvm_generic_file_splice_read;	

	return splice_read(in, ppos, pipe, len, flags);
}
/*
 * Move data from socket to a pipe.
 */
static long 
cvm_socket_to_pipe(struct file *in, loff_t *ppos, struct pipe_inode_info *pipe,
                                                size_t len, unsigned int flags)
{
	ssize_t (*splice_read)(struct file *, loff_t *,
			       struct pipe_inode_info *, size_t, unsigned int);
	int ret;

	if (unlikely(!(in->f_mode & FMODE_READ)))
		return -EBADF;

	ret = rw_verify_area(READ, in, ppos, len);
	if (unlikely(ret < 0))
		return ret;

	if (in->f_op && in->f_op->splice_read)
		splice_read = in->f_op->splice_read;
	else
		splice_read = default_file_splice_read;

	/*
	 * We are trying not to mix kernel code and Cavium specific code as 
	 * much as possible. Given that filesystems register splice_read 
	 * function and we don't change touch FS code, the function call below 
	 * will end up calling that generic function (normally this happens to
	 * be generic_file_splice_read) which will mean that we have to go and
	 * modify that generic function to make it call into our code.
	 *
	 * Instead of that, check whether we are trying to call the generic 
	 * function and if so go ahead and call our version straightaway. 
	 * This way the generic function can be left unmodified.
	 */
	if(splice_read == generic_file_splice_read)
		splice_read = cvm_generic_file_splice_read;	

	return splice_read(in, ppos, pipe, len, flags);
}

/*
 * Function that offloads the data to be memcopied to DP core.
 */
void 
ptf_offload(char *dst, char *src, int len, int page_index)
{
        ptf_args_list_t *ptf_arg;

        ptf_arg = &ptf_args[ptf_idx];

        ptf_arg->dst = dst;
        ptf_arg->src = src;
        ptf_arg->len = len;
	ptf_arg->page_index = page_index;
        ptf_idx = (ptf_idx + 1) & (CNSX_IPC_PTF_RING_SIZE - 1);

	MEMCPY_INC_CNTR(MEMCPY_CP_REQ);
        ptf_tot_req++;
	ptf_offload_req(ptf_arg);
}

/*
 * Copy data from pipe pages into disk pages, the copying work is offloaded to
 * DP core. 
 */
static int
copy_buf_page(struct pipe_inode_info *pipe, struct pipe_buffer *buf, 
              page_arg *dst_pages, int page_index, int page_offset, int buf_len)
 
{
	int ret;
	struct page *dst_page;
	DECLARE_TIME_VAR(tmp_cycle);
	
	/* 
	 * Preventive check, will be removed shortly 
	 */
	if(page_offset + buf_len > PAGE_CACHE_SIZE)
	{
		dump_stack();
	}	

	/*
	 * make sure the data in this buffer is uptodate
	 */
        ret = buf->ops->confirm(pipe, buf);
        if (unlikely(ret))
                return ret;

	dst_page = dst_pages[page_index].page;	
	if (buf->page != dst_page) {
		/*
		 * Careful, ->map() uses KM_USER0!
		 */
		char *src = buf->ops->map(pipe, buf, 0);
		char *dst = kmap(dst_page);
		READ_TIME(tmp_cycle);

		/*
 		 * Start a new memcpy.
		 */
#ifndef DISABLE_MEMCPY_OFFLOAD
		NAS_INC_CNTR(NAS_OFFLOAD_MEMCPY_REQ);
	        ptf_offload(dst + page_offset, src + buf->offset, buf_len, 
                                                              page_index);
#endif 
		dst_pages[page_index].last_req = ptf_tot_req;
		
#ifdef DISABLE_MEMCPY_OFFLOAD 		 
		memcpy(dst + page_offset, src + buf->offset, buf_len);
#endif 
		
		MEMCPY_INC_CNTR(MEMCPY_CP_RECV);
		NAS_INC_CNTR(NAS_OFFLOAD_MEMCPY_RESP);
		ACCUMULATE_DIFF_TIME(NAS_MEMCPY_TT, tmp_cycle);
		memcpy_cnt++;		
	
		flush_dcache_page(dst_page);
		kunmap(dst_page);
		buf->ops->unmap(pipe, buf, src);
	}
	return 0;
}

void
release_pipe_bufs(struct pipe_inode_info *pipe)
{
	struct pipe_buffer *buf; 
	const struct pipe_buf_operations *ops; 

	while (pipe->nrbufs) {
		buf = pipe->bufs + pipe->curbuf;
		ops = buf->ops;

		buf->ops = NULL;
	 	ops->release(pipe, buf);

		pipe->curbuf = (pipe->curbuf + 1) & (pipe->buffers - 1);
		pipe->nrbufs--;
	}
	
	return;
}

/*
 * This function takes care of moving data from the pipe pages into disk I/O
 * pages. The function allocates the disk pages, then passes these pages along
 * with the pipe pages to DP core where data is actually moved from the pipe 
 * pages into disk pages. After DP core completes the copy of data, a wake up 
 * call is sent to this function which then goes ahead and submits the disk
 * pages for IO. Once the disk pages are submitted for IO, pipe pages are freed.
 * 
 * The ordering of disk page allocation, memcpy job submission to DP and disk 
 * page submission for IO is carefully controlled to ensure that page 
 * allocation and memcpy processing in DP happens in parallel.
 */
static int 
cvm_splice_from_pipe_feed(struct pipe_inode_info *pipe, struct splice_desc *sd,
			                                   splice_actor *actor)
{
	int ret;
	int i;
	int page_index = 0;
	int tmp_pipe_nrbuff = pipe->nrbufs;
	int tmp_pipe_curbuf = pipe->curbuf;	
	int tmp_sd_total_len = sd->total_len;
	loff_t tmp_sd_pos = sd->pos;	
	struct file *file = sd->u.file;
	struct address_space *mapping = file->f_mapping;
	struct page *page;
	void *fsdata;
	page_arg dst_pages[MAX_PIPE_PAGES];
	DECLARE_TIME_VAR(tmp_cycle);

	while (tmp_pipe_nrbuff) {
		struct pipe_buffer *buf = pipe->bufs + tmp_pipe_curbuf;
		const struct pipe_buf_operations *ops = buf->ops;
		unsigned int offset = 0;
		unsigned int data_written = 0;
		unsigned int destination_space = 0;
		unsigned int remaining_data = 0;

		/*
 		 * Find the amount of data that a page can hold, this will
 		 * depend on the file offset as we have to block/page align
 		 * the file offset.
 		 */
		offset = tmp_sd_pos & ~PAGE_CACHE_MASK;
		destination_space = PAGE_CACHE_SIZE - offset;
		destination_space = min((unsigned int)destination_space, 
                                        (unsigned int)tmp_sd_total_len);


		/*
 		 * Re-arranging the loop for efficient DP memcpy processing.
 		 * 1) Allocate disk IO page.
 		 * 2) Setup the pipe page and disk IO page as source and 
 		 *    destination for memcpy of data.
 		 * 3) Submit all the pages to DP core for memcpy processing
 		 * 4) Go back to step 1 till all the data is processed.
 		 *
 		 * 5) Once DP core indicates that data is copied into a disk
 		 *    page, submit the page for IO.
 		 */
	
		READ_TIME(tmp_cycle);
		/*
		 * Allocate a disk IO page, this page will be filled with
		 * multiple network packets till it is full.
		 */		
		ret = pagecache_write_begin(file, mapping, tmp_sd_pos, 
						    destination_space,
					     AOP_FLAG_UNINTERRUPTIBLE, 
						      &page, &fsdata);

		if(unlikely(ret)) {
			goto out;			
		}

		page->flags |= (1 << PG_fs_w_data);

		ACCUMULATE_DIFF_TIME(NAS_PAGE_ALLOC_TT, tmp_cycle);
		page_alloc_cnt ++;
					
		dst_pages[page_index].pos = tmp_sd_pos;
		dst_pages[page_index].len = destination_space;
		dst_pages[page_index].page = page;
		dst_pages[page_index].fsdata = fsdata;

		page_index++;

		/*
		 * If you have data to fill and the destination page has
		 * space to hold the full buffer (network packet) 
		 * copy the data into this page and free the buffer.
		 */
		while ((tmp_sd_total_len) && 
		       (buf->len <= destination_space) &&
		       (tmp_sd_total_len >= buf->len)) 
		{

			copy_buf_page(pipe, buf, dst_pages, 
				      page_index - 1, offset, buf->len);

			data_written += buf->len;

			tmp_sd_pos += buf->len;
			tmp_sd_total_len -= buf->len;
				
			offset += buf->len;
			destination_space -= buf->len;

			tmp_pipe_curbuf = 
				(tmp_pipe_curbuf + 1) & (pipe->buffers - 1);
			tmp_pipe_nrbuff--;
		
			buf = pipe->bufs + tmp_pipe_curbuf;
			ops = buf->ops;

		}
		remaining_data = destination_space;
		
		/*
		 * If the remaining data to be written is more than 
		 * what is needed to be written, then adjust the 
		 * write length accordingly.
		 */
		if(tmp_sd_total_len < buf->len)
		{
			remaining_data = min((unsigned int)tmp_sd_total_len, 
					     (unsigned int)destination_space);
		}
	
		/*
		 * If the page can't hold a full packet, we have to fill 
		 * the page with as much as it can hold and submit 
		 * the rest of the packet in the next page.
		 */
		if((tmp_sd_total_len) && (remaining_data))
		{

			copy_buf_page(pipe, buf, dst_pages, 
				            page_index - 1, 
                                                    offset, 
					   remaining_data);

			buf->offset += remaining_data;
			buf->len -= remaining_data;
			data_written += remaining_data;
			
			tmp_sd_pos += remaining_data;
			tmp_sd_total_len -= remaining_data;
				
			destination_space -= remaining_data;
		}
		
		sd->num_spliced += data_written;
		sd->len -= data_written;
		sd->pos += data_written;
		sd->total_len -= data_written;

		/*
 		 * If IO is complete, return.
 		 */
		if(!tmp_sd_total_len)
			break;
	}

        /*
         * All the allocated pages have data, now submit them for IO 
         */
        for(i=0; i<page_index; i++)
        {
		READ_TIME(tmp_cycle);
#ifndef DISABLE_MEMCPY_OFFLOAD 
	       	wait_event(ptf_waitq, (dst_pages[i].last_req <= ptf_tot_resp));
#endif 
		ACCUMULATE_DIFF_TIME(NAS_WAIT_TT, tmp_cycle);

        	READ_TIME(tmp_cycle);

		dst_pages[i].page->flags |= (1 << PG_fs_usb_data);
           	ret = pagecache_write_end(file, mapping, dst_pages[i].pos, 
                                                dst_pages[i].len, 
                                                dst_pages[i].len,
                                                dst_pages[i].page, 
                                                dst_pages[i].fsdata);
		

        	ACCUMULATE_DIFF_TIME(NAS_PAGE_WRITE_TT, tmp_cycle);
        	page_write_cnt ++;
        }
          
	/*
 	 * Free the pipe pages.
 	 */
	release_pipe_bufs(pipe);

	if(!tmp_sd_total_len)
		return 0;
	else
		return 1;
out:	
	return ret;
}

/*
 * Copy data from a pipe to disk.
 */
ssize_t
cvm_generic_file_splice_write(struct pipe_inode_info *pipe, struct file *out,
			        loff_t *ppos, size_t len, unsigned int flags)
{
	struct address_space *mapping = out->f_mapping;
	struct inode *inode = mapping->host;
	struct splice_desc sd = {
		.total_len = len,
		.flags = flags,
		.pos = *ppos,
		.u.file = out,
	};
	ssize_t ret;

	//pipe_lock(pipe);

	splice_from_pipe_begin(&sd);

	do {
		ret = splice_from_pipe_next(pipe, &sd);
		if (ret <= 0)
			break;

		mutex_lock_nested(&inode->i_mutex, I_MUTEX_CHILD);
		ret = file_remove_suid(out);
		if (!ret) {
			file_update_time(out);
			NAS_INC_CNTR(NAS_SPLICE_FROM_PIPE);
			ret = cvm_splice_from_pipe_feed(pipe, &sd, 
                                                    pipe_to_file);
		}
		mutex_unlock(&inode->i_mutex);
	} while (ret > 0);

	splice_from_pipe_end(pipe, &sd);

	//pipe_unlock(pipe);

	if (sd.num_spliced)
		ret = sd.num_spliced;

	if (ret > 0) {
		unsigned long nr_pages;
		int err;

		nr_pages = (ret + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;

		err = generic_write_sync(out, *ppos, ret);
		if (err)
			ret = err;
		else
			*ppos += ret;
		balance_dirty_pages_ratelimited_nr(mapping, nr_pages);
	}

	return ret;
}
EXPORT_SYMBOL(cvm_generic_file_splice_write);



/*
 * Move data from pipe to file.
 */
long 
cvm_pipe_to_file(struct pipe_inode_info *pipe, struct file *out, loff_t *ppos,
                                               size_t len, unsigned int flags)
{
	ssize_t (*splice_write)(struct pipe_inode_info *, struct file *,
				loff_t *, size_t, unsigned int);
	int ret;

	if (unlikely(!(out->f_mode & FMODE_WRITE)))
		return -EBADF;

	if (unlikely(out->f_flags & O_APPEND))
		return -EINVAL;

	ret = rw_verify_area(WRITE, out, ppos, len);
	if (unlikely(ret < 0))
		return ret;

	if (out->f_op && out->f_op->splice_write)
		splice_write = out->f_op->splice_write;
	else
		splice_write = default_file_splice_write;

	/*
	 * We are trying not to mix kernel code and Cavium specific code as 
	 * much as possible. Given that filesystems register splice_write 
	 * function and we don't want to modify FS code, the function call 
	 * below will end up calling that generic function (normally this 
	 * happens to be generic_file_splice_write). This will mean that we 
	 * have to modify that generic function to call into our code. 
	 *
	 * Instead of that, check whether we are trying to call the generic 
	 * function and if so go ahead and call our version straightaway. In
	 * this way the generic function can be left unmodified.
	 */
	if(splice_write == generic_file_splice_write)
	{
		splice_write = cvm_generic_file_splice_write;	
	}

	return splice_write(pipe, out, ppos, len, flags);
}

EXPORT_SYMBOL(cvm_pipe_to_file);


static int 
copy_from_pipe_to_file(struct pipe_inode_info *pipe, struct splice_desc *sd)
{
	struct file *file = sd->u.file;

	return cvm_pipe_to_file(pipe, file, &sd->pos, sd->total_len, 
                                                         sd->flags);
}


/*
 * Move data from pipe to socket.
 */
long 
cvm_pipe_to_socket(struct pipe_inode_info *pipe, struct file *out, loff_t *ppos,
                                               size_t len, unsigned int flags)
{
	ssize_t (*splice_write)(struct pipe_inode_info *, struct file *,
				loff_t *, size_t, unsigned int);
	int ret;

	if (unlikely(!(out->f_mode & FMODE_WRITE)))
		return -EBADF;

	if (unlikely(out->f_flags & O_APPEND))
		return -EINVAL;

	ret = rw_verify_area(WRITE, out, ppos, len);
	if (unlikely(ret < 0))
		return ret;

	if (out->f_op && out->f_op->splice_write)
		splice_write = out->f_op->splice_write;
	else
		splice_write = default_file_splice_write;

	/*
	 * We are trying not to mix kernel code and Cavium specific code as 
	 * much as possible. Given that filesystems register splice_write 
	 * function and we don't want to modify FS code, the function call 
	 * below will end up calling that generic function (normally this 
	 * happens to be generic_file_splice_write). This will mean that we 
	 * have to modify that generic function to call into our code. 
	 *
	 * Instead of that, check whether we are trying to call the generic 
	 * function and if so go ahead and call our version straightaway. In
	 * this way the generic function can be left unmodified.
	 */
	if(splice_write == generic_file_splice_write)
	{
		splice_write = cvm_generic_file_splice_write;	
	}

	return splice_write(pipe, out, ppos, len, flags);
}

EXPORT_SYMBOL(cvm_pipe_to_socket);




/**
 * copy_from_disk_to_sock - splices data directly from disk file to socket. 
 * @in:		disk file to copy from.
 * @sd:		Socket output related information.
 * @actor:	Function that transfers data from pipe to socket.
 *
 * Description:
 *    This is a special case helper to splice directly between two
 *    points, without requiring an explicit pipe. Internally an allocated
 *    pipe is cached in the process, and reused during the lifetime of
 *    that process.
 *
 */
ssize_t copy_from_disk_to_sock(struct file *in, struct splice_desc *sd,
			       splice_direct_actor *copy_data)
{
	struct pipe_inode_info *pipe;
	long ret, bytes;
	umode_t i_mode;
	size_t len;
	int i, flags;

	/*
	 * We require the input being a regular file, as we don't want to
	 * randomly drop data for eg socket -> socket splicing. Use the
	 * piped splicing for that!
	 */
	i_mode = in->f_path.dentry->d_inode->i_mode;
	if (unlikely(!S_ISREG(i_mode) && !S_ISBLK(i_mode)))
		return -EINVAL;

	
       /*
	 * Create/Fetch an internal pipe to be used to move data from socket 
	 * to file. 
         */
	pipe = sim_get_internal_pipe(in, SIM_IO_DIR_SENDFILE);
	if(unlikely(!pipe)) {
		printk("sim_get_internalpipe failed \n");
			return -ENOMEM;
	}
	pipe->readers = 1;

	/*
	 * Do the splice.
	 */
	ret = 0;
	bytes = 0;
	len = sd->total_len;
	flags = sd->flags;

	/*
	 * Don't block on output, we have to drain the direct pipe.
	 */
	sd->flags &= ~SPLICE_F_NONBLOCK;

	while (len) {
		size_t read_len;
		loff_t pos = sd->pos, prev_pos = pos;

		ret = cvm_disk_to_pipe(in, &pos, pipe, len, flags);
		if (unlikely(ret <= 0))
			goto out_release;

		read_len = ret;
		sd->total_len = read_len;

		/*
		 * NOTE: nonblocking mode only applies to the input. We
		 * must not do the output in nonblocking mode as then we
		 * could get stuck data in the internal pipe:
		 */
		ret = copy_data(pipe, sd);
		if (unlikely(ret <= 0)) {
			sd->pos = prev_pos;
			goto out_release;
		}

		bytes += ret;
		len -= ret;
		sd->pos = pos;

		if (ret < read_len) {
			sd->pos = prev_pos + ret;
			goto out_release;
		}
	}

done:
	pipe->nrbufs = pipe->curbuf = 0;
	file_accessed(in);
	return bytes;

out_release:
	/*
	 * If we did an incomplete transfer we must release
	 * the pipe buffers in question:
	 */
	for (i = 0; i < pipe->buffers; i++) {
		struct pipe_buffer *buf = pipe->bufs + i;

		if (buf->ops) {
			buf->ops->release(pipe, buf);
			buf->ops = NULL;
		}
	}

	if (!bytes)
		bytes = ret;

	goto done;
}

static int 
copy_from_pipe_to_socket(struct pipe_inode_info *pipe, struct splice_desc *sd)
{
	struct file *file = sd->u.file;

	return cvm_pipe_to_socket(pipe, file, &sd->pos, sd->total_len, 
                                                         sd->flags);
}

/**
 * cvm_do_sendfile - splices data from a file to socket. 
 * @in:		file to splice from
 * @ppos:	input file offset
 * @out:	Socket to which data should be sent.
 * @len:	number of bytes to splice
 * @flags:	splice modifier flags
 *
 * Description:
 *    For use by do_sendfile(). splice can easily emulate sendfile, but
 *    doing it in the application would incur an extra system call
 *    (splice in + splice out, as compared to just sendfile()). So this helper
 *    can splice directly through a process-private pipe.
 *
 */
long cvm_do_sendfile(struct file *in, loff_t *ppos, struct file *out,
		      size_t len, unsigned int flags)
{
	struct splice_desc sd = {
		.len		= len,
		.total_len	= len,
		.flags		= flags,
		.pos		= *ppos,
		.u.file		= out,
	};
	long ret;

	ret = copy_from_disk_to_sock(in, &sd, copy_from_pipe_to_socket);
	if (ret > 0)
		*ppos = sd.pos;

	return ret;
}



/*
 * copy_from_sock_to_file - Copies data directly from a socket to a disk file.
 * @in:         Socket to read data from.
 * @sd:         Data structure that holds the disk file related info.
 * @actor:      handles the data movement.
 *
 * Description:
 *
 * This function is used to transfer data directly from a socket to file, a 
 * "pipe" is used to facilitate this data movement. Data is first "moved" to 
 * the pipe from the socket and then "copied" to the disk file from this pipe. 
 * Note that there is only one copy involved in this process. 
 *
 * Depending on configuration, data copy from pipe to disk file can be 
 * handled by this function itself or a seperate task running in the background.
 * The above feature is transparent to the callers of this function, they don't
 * need to do anything special to handle background transfers.
 * 
 */
ssize_t 
copy_from_sock_to_file(struct file *in, struct splice_desc *sd,
                              splice_direct_actor *copy_data)
{
        int nas_fd;
        int fd_err;
        int i, flags;
        long ret, bytes;
        size_t len;
	struct pipe_inode_info *bg_nas_pipe;
	struct splice_desc *bg_nas_sd;
        struct pipe_inode_info *pipe;
        session_pipe_info_t *spi = (session_pipe_info_t*)
                                   (current->session_pipe_info);

	DECLARE_TIME_VAR(tmp_cycle);
	
        /*
	 * Create/Fetch an internal pipe to be used to move data from socket 
	 * to file. 
         */
	pipe = sim_get_internal_pipe(sd->u.file, SIM_IO_DIR_RECVFILE);
	if(unlikely(!pipe)) {
		printk("sim_get_internalpipe failed \n");
			return -ENOMEM;
	}
	pipe->readers = 1;

        
	/*
         * Start the data movement from Socket to Disk.
         */
        ret = 0;
        bytes = 0;
        len = sd->total_len;
        flags = sd->flags;

        /*
         * Don't block on output, we have to drain the direct pipe.
         */
        sd->flags &= ~SPLICE_F_NONBLOCK;

        while (len) {
                size_t read_len;
                loff_t pos = sd->pos, prev_pos = pos;
		pos = 0;		

drain_socket:
		/*
		 * Move data from the TCP socket to the pipe.
		 * Don't pass any offset as we are reading from a socket. 
		 */
		READ_TIME(tmp_cycle)
		NAS_INC_CNTR(NAS_SPLICE_FROM_TCP);

		ret = cvm_socket_to_pipe(in, &pos, pipe, len, flags);
		if (unlikely(ret <= 0))
			goto out_release;
		
		ACCUMULATE_DIFF_TIME(NAS_READ_TT, tmp_cycle)

                read_len = ret;
                sd->total_len = read_len;

		/*
 		 * If asynchronous IO is to be done, submit the job of 
 		 * moving data from pipe pages to disk into a queue and return.
 		 * Async NAS task will pick up those jobs and run it in the 
 		 * background. 
 		 * If no async IO is needed, complete the job of moving data
 		 * from pipe to disk before returning.
 		 */
                if (cvm_async_write) {
			/* 
			 * Check if this session is marked with ERROR by 
			 * cvm_nas_task. If it's marked error, skip processing 
			 * the request for this session and return the 
			 * err_code to Samba 
			 */
              	 	fd_err = SIM_GET_FD_ERR_INFO(sd->u.file);
                	if (fd_err == -1) {
                        	nas_fd = SIM_GET_FD_INFO(sd->u.file);
                        	if (nas_fd != -1) {
                                	ret = spi->err_code[SIM_IO_DIR_RECVFILE]
								       [nas_fd];
                                	if (unlikely(ret <= 0)) {
                                                len -= read_len;
                                                /* Keep splicing from socket until the requested len 
                                                 * bytes are read, when splice to disk fails. If this
                                                 * socket draining operation is not done correctly, then
                                                 * samba will mis-interpret previous request's undrained 
                                                 * data as NETBIOS header and will behave erratically. 
                                                 * Refer Trac #433/Bugzila #43 for more info.  */
                                                if (len > 0)
                                                        goto drain_socket;

						 /* 
                                                  * Mark the previous position 
                                                  */
                                        	sd->pos = prev_pos;
						 /* 
                                                  * set bytes with marked error
                                                  * code 
                                                  * */
                                        	bytes = ret;
                                        	goto out_release;
                                         }
                                }

                         }

			/*
 			 * Submit the pipe and disk file to background NAS 
 			 * task which will do the job of transferring data 
 			 * from the pipe to disk file.
	 		 * Background task will clean up SD and Pipe. 
	 		 * SAMBA process can go ahead and manage its pipe 
	 		 * independently as we clone that pipe. 
	 		 * The cloning job is to ease our management of pipe
	 		 * and files.
 			 */ 

			bg_nas_sd = cvm_clone_sd(sd);
			bg_nas_pipe = cvm_clone_pipe(pipe, read_len, flags);
	
			bg_job_done = 0;
			cvm_submit_bg_job(spi, bg_nas_sd, bg_nas_pipe);	
			sd->pos += read_len;

		} else {
			
			/*
 			 * Be careful while transitioning from background
 			 * job processing to foreground processing. Ensure 
 			 * that the background task has completed its job (if 
 			 * any) before starting foreground processing.
 			 */ 
			if ((atomic_read(&outstanding_job_count)) != 0)
				wait_for_cvm_aio_complete(); 

               		/* 
 			 * Copy the data from the pipe to disk file 
 			 */
               		ret = copy_data(pipe, sd);
               		if (unlikely(ret <= 0)) {
                                /* Drain socket as in async-writes case */
                                len -= read_len;
                                if (len > 0)
                                        goto drain_socket;

                       		sd->pos = prev_pos;
                       		goto out_release;
               		}
               }

                bytes += ret;
                len -= ret;

                if (ret < read_len) {
                        sd->pos = prev_pos + ret;
                        goto out_release;
                }
        }

done:
        pipe->nrbufs = pipe->curbuf = 0;
        file_accessed(in);
        return bytes;

out_release:
        /*
         * If we did an incomplete transfer we must release
         * the pipe buffers in question:
         */
        for (i = 0; i < pipe->buffers; i++) {
                struct pipe_buffer *buf = pipe->bufs + i;

                if (buf->ops) {
                        buf->ops->release(pipe, buf);
                        buf->ops = NULL;
                }
        }
        if (!bytes)
                bytes = ret;

        goto done;
}
EXPORT_SYMBOL(copy_from_sock_to_file);


static 
void init_recvfile(void)
{
#if 0
	bw_begin_cnt = 0;
	blk_read_cnt = 0;
	bw_begin_tt = 0;
	blk_read_tt = 0;

	scsi_cmd_tt = 0;
	scsi_cmd_cnt = 0;
	grab_page_tt = 0;
	grab_page_cnt = 0;
#endif

        init_waitqueue_head(&ptf_waitq);
}

int inline 
cvm_do_recvfile(struct file *sock_in, struct file *disk_out, 
                    loff_t __user * off_out, size_t len) 
{
	long ret;
	int flags = SPLICE_F_MOVE | SPLICE_F_MORE; //Check this flag setting.
	struct splice_desc sd = {
                .len            = len,
                .total_len      = len,
                .flags          = flags,
                .u.file         = disk_out,
        };
	DECLARE_TIME_VAR(tmp_cycle);

	if(recvfile_initialized == FALSE)
	{	
		recvfile_initialized = TRUE;
		init_recvfile();	
		init_bg_nas_task();
	}

	copy_from_user(&(sd.pos), off_out, sizeof(loff_t));


	READ_TIME(tmp_cycle);
	NAS_INC_CNTR(NAS_RECVFILE_CNTR);


	ret = copy_from_sock_to_file(sock_in, &sd, copy_from_pipe_to_file);
	if(ret > 0)
	{
		copy_to_user(off_out, &(sd.pos), sizeof(loff_t));
	}

	ACCUMULATE_DIFF_TIME(NAS_RECVFILE_TT, tmp_cycle);

	total_data_transmit += len;
	return ret;	
}

SYSCALL_DEFINE4(recvfile, int, sock_fd_in, int, disk_fd_out, 
                      size_t, len, loff_t __user *, off_out)
{
	long error;
	struct file *in, *out;
	int fput_in, fput_out;
	static int print_control=0;

	if(print_control < 10)
	{

		printk(KERN_DEBUG"Recvfile called with args "
                     "sock_fd: %d, write_fd: %d, len: %d, write_offset: %lld\n",
                       sock_fd_in, disk_fd_out, len, (long long int)*off_out);
		print_control++;
	}

	if (unlikely(!len))
		return 0;


	if(unlikely(!current->session_pipe_info)) {
		sim_io_init();
	}

	/*
 	 * Commenting out the error checking for optimization purposes, not
 	 * good for long term maintenance. Once the code is stabilized, 
 	 * should be tested with error checking and if the performance impact 
 	 * is minimal error handling should be included.
 	 */
#if defined(CONFIG_AMP)
	in = fget_light(sock_fd_in, &fput_in);
	out = fget_light(disk_fd_out, &fput_out);

	if(SIM_IS_NEW_SESSION(out)) {
		sim_session_level_init(out, disk_fd_out, SIM_IO_DIR_RECVFILE);
	}

	error = cvm_do_recvfile(in, out, off_out, len);

	fput_light(out, fput_out);
	fput_light(in, fput_in);

#else /* defined(CONFIG_AMP) */

	error = -EBADF;
	in = fget_light(sock_fd_in, &fput_in);
	if (in) {
		if (in->f_mode & FMODE_READ) {
			out = fget_light(disk_fd_out, &fput_out);
			if (out) {
				if (out->f_mode & FMODE_WRITE)
					error = cvm_do_recvfile(in,
							  out,
							  off_out,
							  len);
				fput_light(out, fput_out);
			}
		}

		fput_light(in, fput_in);
	}
#endif /* defined(CONFIG_AMP_ALL) */

	return error;
}


/*
 * Splice contents of ipipe to opipe.
 */
int cvm_splice_pipe_to_pipe(struct pipe_inode_info *ipipe,
			       struct pipe_inode_info *opipe,
			       size_t len, unsigned int flags)
{
	struct pipe_buffer *ibuf, *obuf;
	int ret = 0, nbuf;
	bool input_wakeup = false;


retry:
	ret = ipipe_prep(ipipe, flags);
	if (ret)
		return ret;

	ret = opipe_prep(opipe, flags);
	if (ret)
		return ret;

	/*
	 * Potential ABBA deadlock, work around it by ordering lock
	 * grabbing by pipe info address. Otherwise two different processes
	 * could deadlock (one doing tee from A -> B, the other from B -> A).
	 */
	pipe_double_lock(ipipe, opipe);

	do {
		if (!opipe->readers) {
			send_sig(SIGPIPE, current, 0);
			if (!ret)
				ret = -EPIPE;
			break;
		}

		if (!ipipe->nrbufs && !ipipe->writers)
			break;

		/*
		 * Cannot make any progress, because either the input
		 * pipe is empty or the output pipe is full.
		 */
		if (!ipipe->nrbufs || opipe->nrbufs >= opipe->buffers) {
			/* Already processed some buffers, break */
			if (ret)
				break;

			if (flags & SPLICE_F_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}

			/*
			 * We raced with another reader/writer and haven't
			 * managed to process any buffers.  A zero return
			 * value means EOF, so retry instead.
			 */
			pipe_unlock(ipipe);
			pipe_unlock(opipe);
			goto retry;
		}

		ibuf = ipipe->bufs + ipipe->curbuf;
		nbuf = (opipe->curbuf + opipe->nrbufs) % opipe->buffers;
		obuf = opipe->bufs + nbuf;

		if (len >= ibuf->len) {
			/*
			 * Simply move the whole buffer from ipipe to opipe
			 */
			*obuf = *ibuf;
			ibuf->ops = NULL;
			opipe->nrbufs++;
			ipipe->curbuf = (ipipe->curbuf + 1) % ipipe->buffers;
			ipipe->nrbufs--;
			input_wakeup = true;
		} else {
			/*
			 * Get a reference to this pipe buffer,
			 * so we can copy the contents over.
			 */
			ibuf->ops->get(ipipe, ibuf);
			*obuf = *ibuf;

			/*
			 * Don't inherit the gift flag, we need to
			 * prevent multiple steals of this page.
			 */
			obuf->flags &= ~PIPE_BUF_FLAG_GIFT;

			obuf->len = len;
			opipe->nrbufs++;
			ibuf->offset += obuf->len;
			ibuf->len -= obuf->len;
		}
		ret += obuf->len;
		len -= obuf->len;
	} while (len);

	pipe_unlock(ipipe);
	pipe_unlock(opipe);

	/*
	 * If we put data in the output pipe, wakeup any potential readers.
	 */
	if (ret > 0) {
		smp_mb();
		if (waitqueue_active(&opipe->wait))
			wake_up_interruptible(&opipe->wait);
		kill_fasync(&opipe->fasync_readers, SIGIO, POLL_IN);
	}
	if (input_wakeup)
		wakeup_pipe_writers(ipipe);

	return ret;
}
EXPORT_SYMBOL(samba_waitq);
EXPORT_SYMBOL(bg_job_done);
EXPORT_SYMBOL(ptf_offload_req);

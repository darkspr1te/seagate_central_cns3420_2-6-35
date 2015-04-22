#if defined(CONFIG_AMP_ALL) || defined(CONFIG_AMP_PURENAS)
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/pmon_counters.h>

#include <linux/cvm_nas_debug.h>
//#include "cvm_nas_debug.h"

#define MAKE_PROCDIR(procdir_entry, procdir_name) 	\
{							\
	procdir_entry = proc_mkdir(procdir_name, NULL);	\
	if (procdir_entry == NULL)			\
	{						\
		printk(KERN_EMERG "creating %s proc failed\n", procdir_name); \
		return 0;				\
	}						\
}

#define	CREATE_NAS_PROC_ENTRY(proc_name) \
	create_proc_entry(proc_name, S_IFREG | S_IRUGO, cvm_nas_proc_entry)

#define REGISTER_NAS_PROC_ENTRY(proc_entry, proc_name, read_fn, write_fn)     \
{ 									      \
	proc_entry = CREATE_NAS_PROC_ENTRY(proc_name); 			      \
	if (proc_entry == NULL) 					      \
	{ 								      \
		printk(KERN_EMERG "creating %s proc failed\n", proc_name);    \
		return 0; 						      \
	} 								      \
	proc_entry->read_proc = read_fn; 				      \
	proc_entry->write_proc = write_fn; 				      \
}

unsigned long cvm_nas_debug_cntrs[NAS_LAST];
unsigned long cvm_nas_prof_tt[NAS_END];
unsigned long cvm_nas_bg_cntrs[BG_END];
unsigned long cvm_nas_sata_cntrs[SATA_END];
unsigned long cvm_nas_memcpy_cntrs[MEMCPY_END];

EXPORT_SYMBOL(cvm_nas_sata_cntrs);
EXPORT_SYMBOL(cvm_nas_memcpy_cntrs);

struct proc_dir_entry *cvm_nas_proc_entry = NULL  ;
struct proc_dir_entry *cvm_nas_debug_proc_dir_entry = NULL ;
struct proc_dir_entry *cvm_nas_prof_proc_dir_entry = NULL ;
struct proc_dir_entry *cvm_nas_bg_proc_dir_entry = NULL ;
struct proc_dir_entry *cvm_nas_max_jobs_to_proc_dir_entry = NULL;
struct proc_dir_entry *cvm_nas_max_jobs_to_submit_dir_entry = NULL;
struct proc_dir_entry *cvm_nas_sata_proc_dir_entry = NULL ;
struct proc_dir_entry *cvm_nas_memcpy_proc_dir_entry = NULL ;


int cvm_nas_debug_proc_readfn (char *page, char **start, off_t off, int count,
                               int *eof,void *data)
{

	int num = 0;

	num += sprintf(page + num , "NAS_RECVFILE_CNTR                  %lu\n",
				cvm_nas_debug_cntrs[NAS_RECVFILE_CNTR]);
	num += sprintf(page + num , "NAS_SPLICE_FROM_TCP                %lu\n",
                                cvm_nas_debug_cntrs[NAS_SPLICE_FROM_TCP]);
	num += sprintf(page + num , "NAS_CVM_SUBMIT_BG_JOB              %lu\n",
                                cvm_nas_debug_cntrs[NAS_CVM_SUBMIT_BG_JOB]);
	num += sprintf(page + num , "NAS_PROCESS_BG_JOB                 %lu\n",
                                cvm_nas_debug_cntrs[NAS_PROCESS_BG_JOB]);
	num += sprintf(page + num , "NAS_SPLICE_FROM_PIPE               %lu\n",
                                cvm_nas_debug_cntrs[NAS_SPLICE_FROM_PIPE]);
	num += sprintf(page + num , "NAS_OFFLOAD_MEMCPY_REQ             %lu\n",
                                cvm_nas_debug_cntrs[NAS_OFFLOAD_MEMCPY_REQ]);
	num += sprintf(page + num , "NAS_OFFLOAD_MEMCPY_RESP            %lu\n",
                                cvm_nas_debug_cntrs[NAS_OFFLOAD_MEMCPY_RESP]);
	return num;

}

int cvm_nas_prof_proc_readfn (char *page, char **start, off_t off, int count,
                               int *eof,void *data)
{

	int num = 0;

	num += sprintf(page + num , "NAS_RECVFILE_TT          %lu\n", 
                                             cvm_nas_prof_tt[NAS_RECVFILE_TT]);
	num += sprintf(page + num , "NAS_READ_TT              %lu\n", 
                                                 cvm_nas_prof_tt[NAS_READ_TT]);
	num += sprintf(page + num , "NAS_PAGE_ALLOC_TT        %lu\n", 
                                           cvm_nas_prof_tt[NAS_PAGE_ALLOC_TT]);
	num += sprintf(page + num , "NAS_PAGE_WRITE_TT        %lu\n", 
                                           cvm_nas_prof_tt[NAS_PAGE_WRITE_TT]);
	num += sprintf(page + num , "NAS_MEMCPY_TT            %lu\n", 
                                               cvm_nas_prof_tt[NAS_MEMCPY_TT]);
	num += sprintf(page + num , "NAS_WAIT_TT              %lu\n",
                                                 cvm_nas_prof_tt[NAS_WAIT_TT]);

	return num;

}

int cvm_nas_bg_proc_readfn (char *page, char **start, off_t off, int count,
                            int *eof,void *data)
{

	int num = 0;

	num += sprintf(page + num , "BG_NAS_JOB_COUNT		%lu\n", 
                                              cvm_nas_bg_cntrs[BG_NAS_JOB_CNT]);
	num += sprintf(page + num , "BG_NAS_SLEEP_COUNT		%lu\n", 
                                            cvm_nas_bg_cntrs[BG_NAS_SLEEP_CNT]);
	num += sprintf(page + num , "BG_OUTSTANDING_CNT		%u\n",
                                           atomic_read(&outstanding_job_count));
	num += sprintf(page + num , "ASYNC_WRITE		%u\n",
                                                               cvm_async_write);
        
	return num ;

}

int cvm_nas_sata_readfn (char *page, char **start, off_t off, int count,
                            int *eof,void *data)
{

       int num = 0;

       num += sprintf(page + num , "SATA_CP_REQ                %lu\n", 
                                              cvm_nas_sata_cntrs[SATA_CP_REQ]);
       num += sprintf(page + num , "SATA_DP_RECV               %lu\n", 
                                            cvm_nas_sata_cntrs[SATA_DP_RECV]);
       num += sprintf(page + num , "SATA_DP_RESP               %lu\n",
                                           cvm_nas_sata_cntrs[SATA_DP_RESP]);
       num += sprintf(page + num , "SATA_CP_RECV               %lu\n",
                                           cvm_nas_sata_cntrs[SATA_CP_RECV]);
       return num ;

}

int cvm_nas_memcpy_readfn (char *page, char **start, off_t off, int count,
                            int *eof,void *data)
{

       int num = 0;

       num += sprintf(page + num , "MEMCPY_CP_REQ              %lu\n", 
                                              cvm_nas_memcpy_cntrs[MEMCPY_CP_REQ]);
       num += sprintf(page + num , "MEMCPY_DP_RECV             %lu\n", 
                                            cvm_nas_memcpy_cntrs[MEMCPY_DP_RECV]);
       num += sprintf(page + num , "MEMPCY_DP_EXIT             %lu\n",
                                           cvm_nas_memcpy_cntrs[MEMCPY_DP_RESP]);
       num += sprintf(page + num , "MEMCPY_CP_RECV             %lu\n",
                                           cvm_nas_memcpy_cntrs[MEMCPY_CP_RECV]);
        
       return num ;

}

ssize_t cvm_nas_sata_writefn (struct file *filp, const char __user *buff,
                                   unsigned long len, void *data )
{
       long val = simple_strtol(buff , NULL , 10);
       int i ;
        if(val == 0 )
        {
               for (i=0;i<SATA_END;i++)
                       cvm_nas_sata_cntrs[i] = 0;
       }

       return len;
}

ssize_t cvm_nas_memcpy_writefn (struct file *filp, const char __user *buff,
                                   unsigned long len, void *data )
{
       long val = simple_strtol(buff , NULL , 10);
       int i ;
        if(val == 0 )
        {
               for (i=0;i<MEMCPY_END;i++)
                       cvm_nas_memcpy_cntrs[i] = 0;
       }

       return len;
}


int cvm_nas_max_jobs_to_process_readfn (char *page, char **start, off_t off,
		int count, int *eof,void *data)
{

	int num = 0;
	num += sprintf(page + num , "max_jobs_to_process 	%d\n", 
                                             max_jobs_to_process); 
	return num ;
}

int cvm_nas_max_jobs_to_submit_readfn (char *page, char **start, off_t off,
		int count, int *eof,void *data)
{

	int num = 0;
	num += sprintf(page + num , "max_jobs_to_submit 	%d\n", 
                                             max_jobs_to_submit); 
	return num ;
}

ssize_t cvm_nas_debug_proc_writefn (struct file *filp, const char __user *buff,
                                    unsigned long len, void *data )
{
        long  val = simple_strtol(buff , NULL , 10 );
        int i ;
        if(val == 0 )
        {
		for (i=0;i<NAS_LAST;i++)
			cvm_nas_debug_cntrs[i] = 0;
	}

	return len;
}

ssize_t cvm_nas_prof_proc_writefn (struct file *filp, const char __user *buff,
                                   unsigned long len, void *data )
{
	long val = simple_strtol(buff , NULL , 10);
	int i ;
        if(val == 0 )
        {
		for (i=0;i<NAS_LAST;i++)
			cvm_nas_prof_tt[i] = 0;
	}

	return len;
}

ssize_t cvm_nas_bg_proc_writefn (struct file *filp, const char __user *buff,
                                 unsigned long len, void *data )
{
	long val = simple_strtol(buff, NULL, 10);
	int i ; 
        if ( val == 0 )
	{
              for (  i=0 ; i<BG_END ; i++)
                    cvm_nas_bg_cntrs[i] = 0;
		cvm_async_write=0;
	}
	else
		cvm_async_write=1;
	return len;
}	

ssize_t cvm_nas_max_jobs_to_process_writefn (struct file *filp, const char __user *buff,
                                 unsigned long len, void *data )
{
	long val = simple_strtol(buff, NULL, 10);
        max_jobs_to_process = val;
	return len;
}	



ssize_t cvm_nas_max_jobs_to_submit_writefn (struct file *filp, const char __user *buff,
                                 unsigned long len, void *data )
{
	long val = simple_strtol(buff, NULL, 10);
        max_jobs_to_submit = val;
	return len;
}	


static int __cpuinit cvm_nas_init(void)
{

        printk(KERN_EMERG "%s: cvm_nas_task init\n", __func__);

	MAKE_PROCDIR(cvm_nas_proc_entry, "cvm_nas");

	REGISTER_NAS_PROC_ENTRY(cvm_nas_debug_proc_dir_entry,
		"nas", 
			cvm_nas_debug_proc_readfn,
				cvm_nas_debug_proc_writefn);

	REGISTER_NAS_PROC_ENTRY(cvm_nas_prof_proc_dir_entry,
		"splice", 
			cvm_nas_prof_proc_readfn,
				cvm_nas_prof_proc_writefn);

	REGISTER_NAS_PROC_ENTRY(cvm_nas_bg_proc_dir_entry,
		"bg_nas",
			cvm_nas_bg_proc_readfn,
				cvm_nas_bg_proc_writefn);

	REGISTER_NAS_PROC_ENTRY(cvm_nas_max_jobs_to_proc_dir_entry, 
		"max_jobs_to_process", 
			cvm_nas_max_jobs_to_process_readfn,
				cvm_nas_max_jobs_to_process_writefn);

	REGISTER_NAS_PROC_ENTRY(cvm_nas_max_jobs_to_submit_dir_entry, 
		"max_jobs_to_submit", 
			cvm_nas_max_jobs_to_submit_readfn,
				cvm_nas_max_jobs_to_submit_writefn);		

	REGISTER_NAS_PROC_ENTRY(cvm_nas_sata_proc_dir_entry, 
               "sata_offload", 
                       cvm_nas_sata_readfn,
                               cvm_nas_sata_writefn);

	REGISTER_NAS_PROC_ENTRY(cvm_nas_memcpy_proc_dir_entry, 
               "memcpy_offload", 
                       cvm_nas_memcpy_readfn,
                               cvm_nas_memcpy_writefn);

        printk(KERN_EMERG "%s: proc init success\n", __func__);
        return 0;
        
}

early_initcall(cvm_nas_init);

#endif /* CONFIG_AMP_ALL || CONFIG_AMP_PURENAS */

#ifndef __CVM_NAS_DEBUG_H 
#define __CVM_NAS_DEBUG_H
#define DEBUG_COUNTERS
#define PROFILING /* used to find the time taken for each function */

enum {

	NAS_RECVFILE_CNTR = 0,
	NAS_SPLICE_FROM_TCP, 
	NAS_CVM_SUBMIT_BG_JOB,
	NAS_PROCESS_BG_JOB,
	NAS_SPLICE_FROM_PIPE,
	NAS_OFFLOAD_MEMCPY_REQ,
	NAS_OFFLOAD_MEMCPY_RESP,
	NAS_LAST,	
};


enum {
        NAS_RECVFILE_TT = 0 ,
	NAS_PAGE_ALLOC_TT,
	NAS_MEMCPY_TT ,
        NAS_PAGE_WRITE_TT ,
        NAS_WAIT_TT ,
	NAS_READ_TT ,
        NAS_END ,
};


enum {

	BG_NAS_JOB_CNT = 0 ,
	BG_NAS_SLEEP_CNT ,
	BG_END ,
};

enum {

       SATA_CP_REQ = 0 ,
       SATA_DP_RECV ,
       SATA_DP_RESP ,
       SATA_CP_RECV ,
       SATA_END ,
};

enum {
       
       MEMCPY_CP_REQ = 0 ,
       MEMCPY_DP_RECV ,
       MEMCPY_DP_RESP ,
       MEMCPY_CP_RECV ,
       MEMCPY_END ,
};


extern atomic_t outstanding_job_count ;
extern int max_jobs_to_process ;
extern int max_jobs_to_submit;

extern unsigned long cvm_nas_prof_tt[NAS_END];
extern unsigned long cvm_nas_debug_cntrs[NAS_LAST];
extern unsigned long cvm_nas_bg_cntrs[BG_END];
extern unsigned long cvm_nas_sata_cntrs[SATA_END];
extern unsigned long cvm_nas_memcpy_cntrs[MEMCPY_END];
extern int cvm_async_write;

#ifdef DEBUG_COUNTERS
#define NAS_INC_CNTR(x)		(cvm_nas_debug_cntrs[x]++);
#define BG_INC_CNTR(x)		(cvm_nas_bg_cntrs[x]++);
#define SATA_INC_CNTR(x)       (cvm_nas_sata_cntrs[x]++);
#define MEMCPY_INC_CNTR(x)     (cvm_nas_memcpy_cntrs[x]++);
#else
#define NAS_INC_CNTR(x) 
#define BG_INC_CNTR(x)
#define SATA_INC_CNTR(x)
#define MEMCPY_INC_CNTR(x)
#endif

#ifdef PROFILING 
#define DECLARE_TIME_VAR(t1)                     unsigned long t1;
#define READ_TIME(t1)                           t1 = arm11_read_ccnt();
#define ACCUMULATE_DIFF_TIME(t2, t1)        cvm_nas_prof_tt[(t2)] += (arm11_read_ccnt() - (t1));
#else
#define DECLARE_TIME_VAR(t1)
#define READ_TIME(t1)
#define ACCUMULATE_DIFF_TIME(t2, t1)        
#endif 

#endif

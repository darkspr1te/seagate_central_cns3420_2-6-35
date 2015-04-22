/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dmac.h>

#include <linux/delay.h>

#include "cns3xxx-pcm.h"
#include "cns3xxx-i2s.h"

//#define __DEBUG_PATH		1		//show function flow...

static const struct snd_pcm_hardware cns3xxx_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE,
#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)
	.channels_min		= 2,
#else
	.channels_min		= 2,
#endif
#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
	.channels_max = 4,
#else
	.channels_max = 2,
#endif
	.period_bytes_min	= 16,
	.period_bytes_max	= 4096-16,
	.periods_min		= 1,
	.periods_max		= 255,
	.buffer_bytes_max	= 128 * 1024,
};

struct cns3xxx_runtime_data {
	int dma_ch;
	int dma_ev;
	size_t period_size;
	struct cns3xxx_pcm_dma_params *params;
};

int loop_counter=0;
int periods=0;
static uint32_t i2s_rx_ev = 13;
static uint32_t i2s_tx_ev = 14;

static int cns3xxx_pcmda_irq_handler(void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct cns3xxx_runtime_data *prtd = runtime->private_data;
//	unsigned long flags;
	
//	local_irq_save(flags);
#ifdef __DEBUG_PATH
	printk("Test %d %d\n",loop_counter, periods);
#endif
//	loop_counter++;
//	if((loop_counter%periods)==0x00){	
    		DMAC_DMAGO(prtd->dma_ch);
//				printk("Test %d %d\n",loop_counter, periods);
//    		loop_counter = 0;
//	}
	snd_pcm_period_elapsed(substream);
//	local_irq_restore(flags);
  
	return 0;
}

static int can_go = 0;
static int cns3xxx_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct cns3xxx_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct cns3xxx_pcm_dma_params *dma = snd_soc_dai_get_dma_data(rtd->dai->cpu_dai, substream);
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
    /*The fifo offset is the fifo tx/rx register offset at which we write or 
     * receive data.This depends on sample size of audio stream.
     */
	int fifo_offset;
	dma_addr_t s_addr,d_addr;
	int s_inc,d_inc, data_swap;
	int ret;
	int move_loop, move_counter = 0;

	/* this may get called several times by oss emulation
	 * with different params */
	/* TODO - get the DMA channel and set things (if any) */

#ifdef __DEBUG_PATH
		printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
		
		if (prtd->params && (prtd->params != dma)) {
				printk("%s=>%d: Bug here...\n", __FUNCTION__, __LINE__);
        dmac_release_event(prtd->dma_ch, prtd->dma_ev);
        dmac_release_channel (prtd->dma_ch);
        prtd->params = NULL;
		}
		
    if (prtd->params == NULL)
    {
//				printk("%s=>%d: It's normal...\n", __FUNCTION__, __LINE__);
        prtd->params = dma;
        prtd->params->dma_ch = prtd->dma_ch = -1;
        prtd->dma_ev = -1;
        ret = dmac_get_channel(cns3xxx_pcmda_irq_handler, substream);
        if (ret == -1) {
					prtd->params = NULL;
					printk("%s=>%d\n", __FUNCTION__, __LINE__);
        	return ret;
        }
        
		    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) 
        	prtd->dma_ev = i2s_tx_ev;
        else
        	prtd->dma_ev = i2s_rx_ev;
				if (dmac_get_event(ret, prtd->dma_ev) == -1) {
        	printk (KERN_INFO "Event Failure .. Bailing\n");
	        dmac_release_channel (ret);
	        prtd->dma_ev = -1;
					prtd->params = NULL;
					printk("%s=>%d\n", __FUNCTION__, __LINE__);
        	return -1;
        }

        prtd->params->dma_ch = prtd->dma_ch = ret;
    }

//	printk("%s=>%d: dma_ch:%d\n", __FUNCTION__, __LINE__, prtd->dma_ch);
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;
	prtd->period_size = period;

//#ifdef CNS3XXX_AUDIO_DEBUG
//    printk ("%s:%d We have a request for %d bytes @ %d period ... \n",__FUNCTION__,__LINE__,totsize,period);
//#endif

		switch(prtd->params->size_sample) {
		case 4:
		    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
						fifo_offset = 1;
			  	 	data_swap = 0;
				}
				else {
						fifo_offset = 0;
			  	 	data_swap = 0;
				}
				break;
		case 1:
		    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
						fifo_offset = 3;
			  	 	data_swap = 0;
				}
				else {
						fifo_offset = 3;
			  	 	data_swap = 0;
				}
				break;
		case 2:
		default:
		    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
						fifo_offset = 2;
			  	 	data_swap = 0;
				}
				else {
						fifo_offset = 2;
			  	 	data_swap = 1;
				}
				break;
		}
		move_loop = 16/prtd->params->size_sample;

    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) 
    {
       s_addr=runtime->dma_addr;
       d_addr=prtd->params->dev_addr+fifo_offset;
       s_inc=1;
       d_inc=0;
    }
    else
    {
       s_addr=prtd->params->dev_addr+fifo_offset;
       d_addr=runtime->dma_addr;
       s_inc=0;
       d_inc=1;
    }
    
    {
        int ch = prtd->dma_ch;
        int lc0 = period/16;
        int lc0sz,lc1sz;
        int lc1 = totsize/period;
        int rump = totsize%period;

		periods = lc1;
//#ifdef CNS3XXX_AUDIO_DEBUG
//        if (period%16) printk ("The constraint we set seem to have not worked !! what the heck !\n");
//        if (rump) printk ("We thought rump would be zero, but totsize:%d,period %d\n",totsize,period);
//        printk ("%s:%d Preparing DMA controller to transfer %d bytes with interrupts @ %d bytes [lc0 %d, lc1 %d, rump %d src_addr %x, dst_addr %x \n",__FUNCTION__,__LINE__,totsize,period,lc0,lc1,rump,s_addr,d_addr);
//		printk("\nsrc_addr %x dst_addr %x\n", s_addr,d_addr);
        //if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
            //printk ("trying to write 2 bytes %x\n",*((uint16_t*)(phys_to_virt(s_addr))));
//#endif
        lc0sz = move_loop*(DMAWFP_INSTR_SIZE + DMALD_INSTR_SIZE+DMASTP_INSTR_SIZE);
        lc1sz = lc0sz +DMALP_INSTR_SIZE +DMALPEND_INSTR_SIZE+DMASEV_INSTR_SIZE + DMAWMB_INSTR_SIZE;
        
        DMAC_DMAMOV (ch,SAR,s_addr);
        DMAC_DMAMOV (ch,DAR,d_addr);
        DMAC_DMAMOV (ch,CCR,dmac_create_ctrlval(s_inc,prtd->params->size_sample,1,d_inc,prtd->params->size_sample,1,data_swap));


        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
            DMAC_DMAFLUSHP (ch,CNS3XXX_DMAC_I2STX_PID);
        else
            DMAC_DMAFLUSHP (ch,CNS3XXX_DMAC_I2SRX_PID);
        if (lc1)
          DMAC_DMALP (ch,1,lc1);
            DMAC_DMALP (ch,0,lc0); /* in this loop, one period is transferred*/
              if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
              {
              	for(move_counter=0; move_counter<move_loop; move_counter++) {
		        			DMAC_DMAWFP (ch, CNS3XXX_DMAC_I2STX_PID,1);
                	DMAC_DMALDB (ch);
                	DMAC_DMASTP (ch, CNS3XXX_DMAC_I2STX_PID,1);
              	}
              }
              else
              {
              	for(move_counter=0; move_counter<move_loop; move_counter++) {
	                DMAC_DMAWFP (ch, CNS3XXX_DMAC_I2SRX_PID,1);
  	              DMAC_DMALDP (ch, CNS3XXX_DMAC_I2SRX_PID,1);
    	            DMAC_DMASTB (ch);
    	          }
              }
            DMAC_DMALPEND (ch,0,lc0sz,1);
            DMAC_DMAWMB (ch);
            DMAC_DMASEV (ch,ch); // Generate an interrupt
        if (lc1)
          DMAC_DMALPEND (ch,1,lc1sz,1);
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
            DMAC_DMAFLUSHP (ch,CNS3XXX_DMAC_I2STX_PID);
        else
            DMAC_DMAFLUSHP (ch,CNS3XXX_DMAC_I2SRX_PID);

        if (rump)
        {
            panic ("Cant handle rump now .. will see later\n");
            DMAC_DMALP (ch,0,rump);
              if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
              {
                DMAC_DMAWFP (ch, CNS3XXX_DMAC_I2STX_PID,1);
                DMAC_DMALD (ch);
                DMAC_DMASTP (ch, CNS3XXX_DMAC_I2STX_PID,1);
              }
              else
              {
                DMAC_DMAWFP (ch, CNS3XXX_DMAC_I2SRX_PID,1);
                DMAC_DMALDP (ch,CNS3XXX_DMAC_I2SRX_PID,1);
                DMAC_DMAST (ch);
              }
            DMAC_DMALPEND (ch,0,DMAWFP_INSTR_SIZE+DMALDP_INSTR_SIZE+DMAST_INSTR_SIZE,1);
            DMAC_DMAWMB (ch);
            DMAC_DMASEV (ch,ch); // Generate an interrupt
        }
        DMAC_DMAEND (ch);
    }
    can_go = 1;

	return 0;
}

static int cns3xxx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct cns3xxx_runtime_data *prtd = substream->runtime->private_data;

#ifdef __DEBUG_PATH
		printk("%s=>%d: dma_ch:%d\n", __FUNCTION__, __LINE__, prtd->dma_ch);
#endif
//    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) 
//			printk("free playback channel...\n");
//		else
//			printk("free record channel...\n");
    if (prtd->dma_ch >=0)
    {
//#ifdef CNS3XXX_AUDIO_dEBUG
//        printk("Releasing the Channel %d\n",prtd->dma_ch);
//#endif
        dmac_release_event(prtd->dma_ch, prtd->dma_ev);
        dmac_release_channel (prtd->dma_ch);
//				printk("%s=>%d\n", __FUNCTION__, __LINE__);
        prtd->dma_ch = -1;
        prtd->dma_ev = -1;
        prtd->params = NULL;
        snd_pcm_set_runtime_buffer(substream,NULL);
    }

	return 0;
}

static int cns3xxx_pcm_prepare(struct snd_pcm_substream *substream)
{
	//struct cns3xxx_runtime_data *prtd = substream->runtime->private_data;
	//struct snd_pcm_runtime *runtime = substream->runtime;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	return 0;
}

static int cns3xxx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct cns3xxx_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

#ifdef __DEBUG_PATH
	printk("%s=>%d: dma_ch:%d\n", __FUNCTION__, __LINE__, prtd->dma_ch);
#endif

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
        if (can_go)
        {
            DMAC_DMAGO (prtd->dma_ch);
            can_go = 0;
        }
		break;

#if 0
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
#endif

	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t
cns3xxx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct cns3xxx_runtime_data *prtd = runtime->private_data;

	dma_addr_t ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
			            DMAC_READ_CHREGS (prtd->dma_ch, PL330_SA) : 
                        DMAC_READ_CHREGS (prtd->dma_ch, PL330_DA) ;
	snd_pcm_uframes_t x = bytes_to_frames(runtime, ptr - runtime->dma_addr);

#ifdef __DEBUG_PATH
//	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
//	static int dumponce = 1;
//
//    if(dumponce)
//		{
//    	int counter = 0;
//    	unsigned char *pBuf = (unsigned char *)__bus_to_virt(runtime->dma_addr);
//    	
//    	if(*pBuf) {
//    	printk("\n>>>>>>>>>> Dump Sound Buffer 11 <<<<<<<<<<<<\n");
//    	for(counter=0; counter<128; counter++) {
//    		if((counter%16 == 0) && counter)
//    			printk("\n");
//    		printk("%02X  ", *(pBuf+counter));
//    	}
// 			printk("\n");
//    	dumponce = 0;
//    	}
//    }

    //printk ("%s:Transmitted %d frame [sa:%x;da:%x]\n",__FUNCTION__,(int)x,DMAC_READ_CHREGS(prtd->dma_ch,PL330_SA),DMAC_READ_CHREGS(prtd->dma_ch,PL330_DA));

	if (x >= runtime->buffer_size)
    {
//#ifdef CNS3XXX_AUDIO_DEBUG
        //printk ("Transmitted all the frames [%d in total]\n",(int)x);
//#endif
		x = 0;
    }
	return x;
}

static int cns3xxx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct cns3xxx_runtime_data *prtd;
	int ret;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	snd_soc_set_runtime_hwparams(substream, &cns3xxx_pcm_hardware);

    /* Enforcing the constraint that period bytes are a multiple of 16 bytes.
     * This helps PL330 GDMA innerloop
     */
    ret = snd_pcm_hw_constraint_step (runtime, 0, 
                                      SNDRV_PCM_HW_PARAM_PERIOD_BYTES,16);
    if (ret) goto out;

    ret = snd_pcm_hw_constraint_step (runtime,0,
                                      SNDRV_PCM_HW_PARAM_BUFFER_BYTES,16);
    if (ret) goto out;

    /* To ensure that buffersize is a multiple of period size ? */
	ret = snd_pcm_hw_constraint_integer (runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct cns3xxx_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}
    prtd->dma_ch = -1;

	runtime->private_data = prtd;
	return 0;

 out:
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
	return ret;
}

static int cns3xxx_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct cns3xxx_runtime_data *prtd = runtime->private_data;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	loop_counter=0;
	kfree(prtd);
	return 0;
}

static int cns3xxx_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
//	printk("runtime->dma_area: 0x%08x\n", (unsigned int)runtime->dma_area);
//	printk("runtime->dma_addr: 0x%08x\n", runtime->dma_addr);
//	printk("runtime->dma_bytes: 0x%08x\n", runtime->dma_bytes);
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);

}

struct snd_pcm_ops cns3xxx_pcm_ops = {
	.open		= cns3xxx_pcm_open,
	.close		= cns3xxx_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= cns3xxx_pcm_hw_params,
	.hw_free	= cns3xxx_pcm_hw_free,
	.prepare	= cns3xxx_pcm_prepare,
	.trigger	= cns3xxx_pcm_trigger,
	.pointer	= cns3xxx_pcm_pointer,
	.mmap		= cns3xxx_pcm_mmap,
};

static int cns3xxx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	size_t size = cns3xxx_pcm_hardware.buffer_bytes_max;
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
//#ifdef CNS3XXX_AUDIO_DEBUG
	//printk (KERN_INFO "preallocate_dma_buffer: area=%p, addr=%x, size=%d\n",buf->area,buf->addr,size);
//#endif
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void cns3xxx_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 cns3xxx_pcm_dmamask = DMA_BIT_MASK(32); //DMA_32BIT_MASK;

static int cns3xxx_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &cns3xxx_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32); //DMA_32BIT_MASK;

	if (dai->playback.channels_min) {
		ret = cns3xxx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		ret = cns3xxx_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform cns3xxx_soc_platform = {
	.name		= "cns3xxx-audio",
	.pcm_ops 	= &cns3xxx_pcm_ops,
	.pcm_new	= cns3xxx_pcm_new,
	.pcm_free	= cns3xxx_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(cns3xxx_soc_platform);

static int __init cns3xxx_pcm_init(void)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	return snd_soc_register_platform(&cns3xxx_soc_platform);
}
module_init(cns3xxx_pcm_init);

static void __exit cns3xxx_pcm_exit(void)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	snd_soc_unregister_platform(&cns3xxx_soc_platform);
}
module_exit(cns3xxx_pcm_exit);

MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("CNS3XXX PCM DMA module");
MODULE_LICENSE("GPL");

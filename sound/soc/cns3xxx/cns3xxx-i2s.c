/*
 * cns3xxx-i2s.c  --  ALSA Soc Audio Layer
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <asm/io.h>

#include <mach/pm.h>
#include <mach/hardware.h>
#include <mach/misc.h>

#include "cns3xxx-pcm.h"
#include "cns3xxx-i2s.h"

//#define __DEBUG_PATH		1		//show function flow...

#define MISC_REG_VALUE(offset) (*((volatile unsigned int *)(CNS3XXX_MISC_BASE_VIRT+offset)))

#include <mach/cns3xxx.h>

//#define MISC_MEM_MAP_VALUE(reg_offset)	(*((uint32_t volatile *)(CNS3XXX_MISC_BASE_VIRT + reg_offset)))
#define MISC_IO_CTRL				MISC_MEM_MAP_VALUE(0x010)
#define MISC_IOCDA_CTRL			MISC_MEM_MAP_VALUE(0x01C)
#define MISC_IOCDB_CTRL			MISC_MEM_MAP_VALUE(0x020)

#define PM_MEM_MAP_VALUE(reg_offset)	(*((uint32_t volatile *)(CNS3XXX_PM_BASE_VIRT + reg_offset)))
#define PM_SYS_CLK_CTRL			PM_MEM_MAP_VALUE(0x014)

static struct cns3xxx_pcm_dma_params cns3xxx_i2s_stereo_out = {
    .name = "I2S PCM Stereo out",
    .dev_addr = (0x86000000+0x20),
    .size_sample = 2, // 16 bit audio by default
};

static struct cns3xxx_pcm_dma_params cns3xxx_i2s_stereo_in = {
    .name = "I2S PCM Stereo in",
    .dev_addr = (0x86000000+0x0),
    .size_sample = 2, //16 bit audio by default
};

static void cns3xxx_i2s_init_pwr_up(void);

static int cns3xxx_i2s_startup (struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
//	int channels;
	
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

//	channels = params_channels(params);
// 	printk("%s,%d=> channels:%d\n", __FUNCTION__, __LINE__, channels);

//	snd_pcm_hw_constraint_minmax(substream->runtime,
//			SNDRV_PCM_HW_PARAM_CHANNELS,
//			channels, channels);
	
	CWDA_TC = 0x0266;
////	if (!cpu_dai->active) 
////	{
////		/*Flushing the FIFO and resetting the registers */
////		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
////		{
////			CWDA_TC |= CWDA_TX_RST | CWDA_TX_FLUSH;
////            CWDA_TC &= ~(CWDA_TX_RST | CWDA_TX_FLUSH);
////		}
////		else
////		{
////			CWDA_TC |= CWDA_RX_RST | CWDA_RX_FLUSH;
////            CWDA_TC &= ~(CWDA_RX_RST | CWDA_RX_FLUSH);
////		}
////	}

//#ifdef CONFIG_SND_SOC_WM8991
//  CWDA_DFC = 0x4 | (0x4 << 5) | (0x1 << 8) | (0x1 << 15) | (0x1 << 19);
// 	CWDA_SSC = 0x0 | (0x7F << 4) | (0x1 << 18) | (0x3F << 20);
//#else
//  CWDA_DFC = 0x2 | (0x5 << 5) | (0x1 << 8) | (0x1 << 19);
//  CWDA_SSC = 0x0 | (0x3F << 4) | (0x1 << 18) | (0x1F << 20);
//#endif
  
	CWDA_TCI = 0x0;

	return 0;
}

//static int cns3xxx_i2s_set_dai_sysclk (struct snd_soc_dai *cpu_dai,
//							int clk_id, unsigned int freq, int dir)
//{
//
//#warning Need to set the register as per the sampling rate
//	/* Based on freq that we get, set the h/w reg to that freq */ 
//	return 0;
//	
//}

static int cns3xxx_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct cns3xxx_pcm_dma_params *dma_data;
	unsigned int clk_i2s = 12288000;
//	unsigned int fs_i2s = 64;
	unsigned int rate_sample = 0;
	unsigned int div_i2s = 0;
	int channels;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	dai->private_data = dai;

	channels = params_channels(params);
// 	printk("%s,%d=> channels:%d\n", __FUNCTION__, __LINE__, channels);

//	snd_pcm_hw_constraint_minmax(substream->runtime,
//			SNDRV_PCM_HW_PARAM_CHANNELS,
//			channels, channels);


    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        dma_data = &cns3xxx_i2s_stereo_out;
    else
        dma_data = &cns3xxx_i2s_stereo_in;

    snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);
    
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        CWDA_TFLL = 32;
		//CWDA_INT |= CWDA_INT_TXDMA | CWDA_INT_TXFFM;
		CWDA_INT = CWDA_INT_TXDMA;
		
		
    }
	else
    {
        CWDA_RFUL = 32;
		//CWDA_INT |= CWDA_INT_RXDMA | CWDA_INT_RXFFM;
		CWDA_INT = CWDA_INT_RXDMA;
		
		
    }


//#ifdef CONFIG_SND_SOC_WM8991
//	  if(channels == 4) {
//		  CWDA_DFC = 0x4 | (0x4 << 5) | (0x1 << 8) | (0x1 << 15) | (0x1 << 19);
// 			CWDA_SSC = 0x0 | (0xFF << 4) | (0x1 << 18) | (0x7F << 20);
//  	}
//  	else {
//	  	CWDA_DFC = 0x2 | (0x5 << 5) | (0x1 << 8) | (0x1 << 19);
//  		CWDA_SSC = 0x0 | (0x7F << 4) | (0x1 << 18) | (0x3F << 20);
//  	}
//#else
#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)
	  	CWDA_DFC = 0x2 | (0x5 << 5) | (0x1 << 8) | (0x1 << 19);
  		CWDA_SSC = 0x0 | (0x3F << 4) | (0x1 << 16) | (0x1 << 18) | (0x1F << 20);
#else
	  if(channels == 4) {
		  CWDA_DFC = 0x4 | (0x4 << 5) | (0x1 << 8) | (0x1 << 15) | (0x1 << 19);
 			CWDA_SSC = 0x0 | (0x7F << 4) | (0x1 << 18) | (0x3F << 20);
  	}
  	else {
	  	CWDA_DFC = 0x2 | (0x5 << 5) | (0x1 << 8) | (0x1 << 19);
  		CWDA_SSC = 0x0 | (0x3F << 4) | (0x1 << 18) | (0x1F << 20);
  	}
#endif  	
//#endif
        

    switch (params_format(params))
    {
    		case SNDRV_PCM_FORMAT_S8:
//            printk("%s:DMA data type : 8bit\n",__FUNCTION__);
            dma_data->size_sample = 1;
            CWDA_DFC &= ~0x7;
            CWDA_DFC |= 0x0; // 8 bit audio
            if(channels == 4) {
	            CWDA_DFC &= ~0xE0;
  	          CWDA_DFC |= (0x0 << 5); // 8 bit audio
            }
    				break;
        case SNDRV_PCM_FORMAT_S16_LE:
//            printk("%s:DMA data type : 16bit\n",__FUNCTION__);
            dma_data->size_sample = 2;
            CWDA_DFC &= ~0x7;
            CWDA_DFC |= 0x2; // 16 bit audio
            if(channels == 4) {
	            CWDA_DFC &= ~0xE0;
	            CWDA_DFC |= (0x2 << 5); // 16 bit audio
            }
            break;
        case SNDRV_PCM_FORMAT_S20_3LE: //should not support this format...
//            printk("%s:DMA data type : 20bit\n",__FUNCTION__);
            dma_data->size_sample = 3;
            CWDA_DFC &= ~0x7;
            CWDA_DFC |= 0x3; // 20 bit audio
            if(channels == 4) {
	            CWDA_DFC &= ~0xE0;
	            CWDA_DFC |= (0x3 << 5); // 20 bit audio
            }
            break;
        case SNDRV_PCM_FORMAT_S24_LE:
//            printk("%s:DMA data type : 24bit\n",__FUNCTION__);
            dma_data->size_sample = 4;
            CWDA_DFC &= ~0x7;
            CWDA_DFC |= 0x4; // 24 bit audio
            if(channels == 4) {
	            CWDA_DFC &= ~0xE0;
	            CWDA_DFC |= (0x4 << 5); // 24 bit audio
            }
            break;
        default:
            printk("%s:Unsupported PCM Format\n",__FUNCTION__);
            return -EINVAL;
    }

	/* TODO -- based on params, which is the sampling rate, gotta decide the
	 * the clk freq, guess the h/w guys shd provide the register to control
	 * the clk freq. 
	 */

	rate_sample = params_rate(params);
	switch (rate_sample)
	{
		case 8000:
			clk_i2s = 0; //8192000
			div_i2s = 4; //16
			break;
		case 16000:
			clk_i2s = 0; //8192000
			div_i2s = 3; //8
			break;
		case 32000:
			clk_i2s = 0; //8192000
			div_i2s = 2; //4
			break;
		case 48000:
			clk_i2s = 2; //12288000
			div_i2s = 2; //4
			break;
//		case 96000:
//			clk_i2s = 2; //12288000
//			div_i2s = 1; //2
//			break;
		case 11025:
			clk_i2s = 1; //11289600
			div_i2s = 4; //16
			break;
		case 22050:
			clk_i2s = 1; //11289600
			div_i2s = 3; //8
			break;
		case 44100:
			clk_i2s = 1; //11289600
			div_i2s = 2; //4
			break;
		case 96000:
		case 192000:
      printk("%s:Unsupported PCM Sample Rate\n",__FUNCTION__);
			return -EINVAL;
	}

//#ifdef CONFIG_SND_SOC_WM8991
//	div_i2s--;
//#endif
  if(channels == 4) {
		div_i2s--;
	}	
	PM_SYS_CLK_CTRL &= 0xF83FFFFF;
	PM_SYS_CLK_CTRL |= (0x1 << 31) | (clk_i2s << 22) | (div_i2s << 24);
//	fs_i2s = clk_i2s/rate_sample;
//	fs_i2s--;
// 	CWDA_SSC &= ~0xFFF0;
// 	fs_i2s <<= 4;
// 	CWDA_SSC |= fs_i2s;

	return 0;
}

static int cns3xxx_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	int ret = 0;


#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        //printk (KERN_INFO "%s %d\n",__FUNCTION__,__LINE__);
    
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
            CWDA_TC = 0x0101; //|= CWDA_CLK_EN | CWDA_TX_EN;
		else
            CWDA_TC = 0x0110; //|= CWDA_CLK_EN | CWDA_RX_EN;

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		//printk (KERN_INFO "%s %d\n",__FUNCTION__,__LINE__);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			CWDA_TC &= ~(CWDA_TX_EN | CWDA_CLK_EN);
		else
			CWDA_TC &= ~(CWDA_RX_EN | CWDA_CLK_EN);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void cns3xxx_i2s_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Disabling Playback and Disabling Interrupt */
        CWDA_TC = 0x0266; // &= ~(CWDA_TX_EN);
        CWDA_INT = 0;
	} else {
        CWDA_TC = 0x0266; // &= ~(CWDA_RX_EN);
        CWDA_INT = 0;
	}
	return;

}

static int cns3xxx_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
	unsigned int fmt)
{
	int ret = 0;
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	
	return ret;
}

static int cns3xxx_i2s_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int mask, unsigned int rx_mask, int slots, int slot_width)
{
//	struct ssp_priv *priv = cpu_dai->private_data;
//	struct ssp_device *ssp = priv->dev.ssp;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	printk("%s=>%d: mask:0x%08x, slot:%d\n", __FUNCTION__, __LINE__, mask, slots);

	return 0;
}

#define CNS3XXX_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

static struct snd_soc_dai_ops cns3xxx_snd_soc_dai_ops = {
	.startup = cns3xxx_i2s_startup, // flush, reset
	.shutdown = cns3xxx_i2s_shutdown,
	.hw_params = cns3xxx_i2s_hw_params,
	.trigger = cns3xxx_i2s_trigger,
	.set_fmt = cns3xxx_i2s_set_dai_fmt,
	.set_tdm_slot	= cns3xxx_i2s_set_dai_tdm_slot,
};

struct snd_soc_dai cns3xxx_i2s_dai = {
	.name = "cns3xxx-i2s",
	.id = 0,
//	.type = SND_SOC_DAI_I2S,
	.playback = {
#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)
		.channels_min	= 2,
#else
		.channels_min = 2,
#endif
#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
		.channels_max = 4,
#else
		.channels_max = 2,
#endif
		.rates = CNS3XXX_I2S_RATES,
		.formats		= SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE,},
	.capture = {
#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)
		.channels_min	= 1,
#else
		.channels_min = 2,
#endif
#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
		.channels_max = 4,
#else
		.channels_max = 2,
#endif
		.rates = CNS3XXX_I2S_RATES,
		.formats		= SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE,},
	.ops = &cns3xxx_snd_soc_dai_ops,
	// TODO - check if we really dont need these ..
//	.dai_ops = {
////		.set_fmt = cns3xxx_i2s_set_dai_fmt,
//		.set_sysclk = cns3xxx_i2s_set_dai_sysclk,
//	},
};

EXPORT_SYMBOL_GPL(cns3xxx_i2s_dai);

void *i2s_base_va = NULL;

static struct gpio i2s_gpios[] = {
	/* Pin no.   flags      label */
	{ GPIOB(22),  GPIOF_IN, "I2S CLK" },
	{ GPIOB(23),  GPIOF_IN, "I2S FS" },
	{ GPIOB(24),  GPIOF_IN, "I2S DT" },
	{ GPIOB(25),  GPIOF_IN, "I2S DR" },
};

static void cns3xxx_i2s_init_pwr_up(void)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	cns3xxx_sharepin_request_array(i2s_gpios, ARRAY_SIZE(i2s_gpios));

	PM_SYS_CLK_CTRL &= 0xF83FFFFF;
	PM_SYS_CLK_CTRL |= 0x80800000;

	//I2S Pin Drive Strength
	//(0x30000: 21mA) 
	//(0x20000: 15.7mA) 
	//(0x10000: 10.5mA) 
	//(0x00000: 5.2mA)
//	MISC_IO_PAD_DRIVE_STRENGTH_CTRL_A &= ~0x30000;
//	MISC_IO_PAD_DRIVE_STRENGTH_CTRL_A |= 0x30000; //21mA...

	cns3xxx_pwr_power_up(CNS3XXX_PWR_PLL(PLL_I2SCD));
	cns3xxx_pwr_power_up(CNS3XXX_PWR_PLL(PLL_I2S));
	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(I2S));
	cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(I2S));

	/* Enable I2S controller */
	MISC_REG_VALUE(0x704) |= 0x02; 	
}

static int cns3xxx_i2s_probe (struct platform_device *pdev)
{
	int ret;
	struct resource *mem, *ioarea;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		printk("%s: IORESOURCE_MEM not defined \n", __FUNCTION__);
		return -ENODEV;
	}
	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
				    pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2S region already claimed\n");
		return -EBUSY;
	}
		
    i2s_base_va = ioremap (mem->start,mem->end-mem->start+1);
    if (!i2s_base_va) printk ("Couldnt ioremap !! expect crash !!\n");

		cns3xxx_i2s_init_pwr_up();

    printk ("I2S version obtained Frm remap address %p: %x [==0xDA1A0100 ?]\n",i2s_base_va,*((uint32_t*)(i2s_base_va+0x50)));
    if (CWDA_VER == 0xDA1A0100)
    {
        printk ("%s: Version Read OK .. Resetting Controller\n",__FUNCTION__);
//        CWDA_TC = CWDA_RX_RST | CWDA_TX_RST | CWDA_CLK_RST | CWDA_RX_FLUSH | 
//                  CWDA_TX_FLUSH;
//        CWDA_TC &= ~(CWDA_RX_RST|CWDA_TX_RST|CWDA_CLK_RST|CWDA_RX_FLUSH|CWDA_TX_FLUSH);
//
//        CWDA_TC |= CWDA_CLK_EN;
//				CWDA_TC = 0x0266;
    }

	cns3xxx_i2s_dai.dev = &pdev->dev;
	cns3xxx_i2s_dai.private_data = NULL;
	ret = snd_soc_register_dai(&cns3xxx_i2s_dai);

    return ret;
}

static int __devexit cns3xxx_i2s_remove(struct platform_device *pdev)
{
	struct resource *mem;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		printk("%s: IORESOURCE_MEM not defined \n", __FUNCTION__);
		return -ENODEV;
	}

	snd_soc_unregister_dai(&cns3xxx_i2s_dai);
	
	iounmap(i2s_base_va);
	i2s_base_va = NULL;
	
	release_mem_region(mem->start, (mem->end - mem->start) + 1);

	/* Disable I2S controller */
	MISC_REG_VALUE(0x704) &= ~0x02; 	
	cns3xxx_pwr_clk_disable(CNS3XXX_PWR_CLK_EN(I2S));

	cns3xxx_sharepin_free_array(i2s_gpios, ARRAY_SIZE(i2s_gpios));
	
	return 0;
}

static struct platform_driver cns3xxx_i2s_driver =  {
    .probe = cns3xxx_i2s_probe,
		.remove = __devexit_p(cns3xxx_i2s_remove),
    .driver = {
        .name = "cns3xxx-i2s",
        .owner= THIS_MODULE,
    },
};

static int __init cns3xxx_i2s_init (void)
{
#ifdef __DEBUG_PATH
		printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
    return platform_driver_register (&cns3xxx_i2s_driver);
}

static void __exit cns3xxx_i2s_exit (void)
{
#ifdef __DEBUG_PATH
		printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif		
    platform_driver_unregister (&cns3xxx_i2s_driver);
}

module_init (cns3xxx_i2s_init);
module_exit (cns3xxx_i2s_exit);

/* Module information */
MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("CNS3XXX I2S SoC Interface");
MODULE_LICENSE("GPL");

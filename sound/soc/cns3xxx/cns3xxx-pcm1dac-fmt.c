/*
 * cns3xxx-pcm1dac-fmt.c  --  ALSA Soc Audio Layer
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
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <asm/io.h>

#include <mach/pm.h>
#include <mach/misc.h>
#include <mach/hardware.h>

#include <linux/delay.h>

#include "cns3xxx-pcm1dac-pcm.h"
#include "cns3xxx-pcm1dac-fmt.h"

//#define __DEBUG_PATH		1		//show function flow...

#define MISC_REG_VALUE(offset) (*((volatile unsigned int *)(CNS3XXX_MISC_BASE_VIRT+offset)))

#include <mach/cns3xxx.h>

#define PM_MEM_MAP_VALUE(reg_offset)	(*((uint32_t volatile *)(CNS3XXX_PM_BASE_VIRT + reg_offset)))
#define PM_SYS_CLK_CTRL			PM_MEM_MAP_VALUE(0x014)

static struct cns3xxx_pcm1dac_pcm_dma_params cns3xxx_pcm1dac_fmt_stereo_out = {
    .name = "FMT PCM1 Stereo out",
    .dev_addr = (0x71000000+0x198),
    .size_sample = 2, // 16 bit audio by default
};

static struct cns3xxx_pcm1dac_pcm_dma_params cns3xxx_pcm1dac_fmt_stereo_in = {
    .name = "FMT PCM1 Stereo in",
    .dev_addr = (0x71000000+0x1A0),
    .size_sample = 2, //16 bit audio by default
};

static void cns3xxx_pcm1dac_fmt_init_pwr_up(void);
static void cns3xxx_enable_pcm1(int enable);

static int cns3xxx_pcm1dac_fmt_startup (struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	return 0;
}

//static int cns3xxx_pcm1dac_fmt_set_dai_sysclk (struct snd_soc_dai *cpu_dai,
//							int clk_id, unsigned int freq, int dir)
//{
//
//#warning Need to set the register as per the sampling rate
//	/* Based on freq that we get, set the h/w reg to that freq */ 
//	return 0;
//	
//}

static int cns3xxx_pcm1dac_fmt_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
        struct cns3xxx_pcm1dac_pcm_dma_params *dma_data;
	unsigned int rate_sample = 0;
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
        dma_data = &cns3xxx_pcm1dac_fmt_stereo_out;
    else
        dma_data = &cns3xxx_pcm1dac_fmt_stereo_in;

    snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);

    
    switch (params_format(params))
    {
    		case SNDRV_PCM_FORMAT_S8:
//            printk("%s:DMA data type : 8bit\n",__FUNCTION__);
            dma_data->size_sample = 1;
					  PCM1_CH_CFG0 &= ~(0x1 << 22);
					  PCM1_CH_CFG1 &= ~(0x7F << 0);
					  PCM1_CH_CFG1 &= ~(0x7F << 8);
					  PCM1_CH_CFG1 |= ((8 & 0x7F) << 0);
					  PCM1_CH_CFG1 |= ((8 & 0x7F) << 8);
					  PCM1_CH_CFG1 &= ~(0x1 << 22);
    				break;
        case SNDRV_PCM_FORMAT_S16_LE:
//            printk("%s:DMA data type : 16bit\n",__FUNCTION__);
            dma_data->size_sample = 2;
					  PCM1_CH_CFG0 |= (0x1 << 22);
					  PCM1_CH_CFG1 &= ~(0x7F << 0);
					  PCM1_CH_CFG1 &= ~(0x7F << 8);
					  PCM1_CH_CFG1 |= ((16 & 0x7F) << 0);
					  PCM1_CH_CFG1 |= ((16 & 0x7F) << 8);
					  PCM1_CH_CFG1 |= (0x1 << 22);
            break;
        case SNDRV_PCM_FORMAT_S20_3LE: //should not support this format...
        case SNDRV_PCM_FORMAT_S24_LE:
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
			break;
		default:
      printk("%s:Unsupported PCM Sample Rate\n",__FUNCTION__);
			return -EINVAL;
	}

	return 0;
}

static int cns3xxx_pcm1dac_fmt_trigger(struct snd_pcm_substream *substream, int cmd,
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
		cns3xxx_enable_pcm1(1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void cns3xxx_pcm1dac_fmt_shutdown(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	cns3xxx_enable_pcm1(0);
	return;

}

static int cns3xxx_pcm1dac_fmt_set_dai_fmt(struct snd_soc_dai *cpu_dai,
	unsigned int fmt)
{
	int ret = 0;
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	
	return ret;
}

static int cns3xxx_pcm1dac_fmt_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
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

#define CNS3XXX_PCM1DAC_FMT_RATES (SNDRV_PCM_RATE_8000)

static struct snd_soc_dai_ops cns3xxx_pcm1dac_snd_soc_dai_ops = {
	.startup = cns3xxx_pcm1dac_fmt_startup, // flush, reset
	.shutdown = cns3xxx_pcm1dac_fmt_shutdown,
	.hw_params = cns3xxx_pcm1dac_fmt_hw_params,
	.trigger = cns3xxx_pcm1dac_fmt_trigger,
	.set_fmt = cns3xxx_pcm1dac_fmt_set_dai_fmt,
	.set_tdm_slot	= cns3xxx_pcm1dac_fmt_set_dai_tdm_slot,
};

struct snd_soc_dai cns3xxx_pcm1dac_fmt_dai = {
	.name = "cns3xxx-pcm1",
	.id = 0,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = CNS3XXX_PCM1DAC_FMT_RATES,
		.formats		= SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cns3xxx_pcm1dac_snd_soc_dai_ops,
	// TODO - check if we really dont need these ..
//	.dai_ops = {
////		.set_fmt = cns3xxx_pcm1dac_fmt_set_dai_fmt,
//		.set_sysclk = cns3xxx_pcm1dac_fmt_set_dai_sysclk,
//	},
};

EXPORT_SYMBOL_GPL(cns3xxx_pcm1dac_fmt_dai);

void *pcm1_base_va = NULL;

static void cns3xxx_enable_pcm1(int enable)
{
	if(enable)
		PCM1_CFG0 |= (0x1 << 31);
	else
		PCM1_CFG0 &= ~(0x1 << 31);
}

static void cns3xxx_pcm1dac_fmt_init_pwr_up(void)
{
	unsigned int value;
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	PM_PLL_HM_PD_CTRL_REG &= ~(0x1 << 5);
  HAL_MISC_ENABLE_PCM1_PINS();
	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(SPI_PCM_I2C));
	cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(SPI_PCM_I2C));
	
  value = ((0 & 0x7) << 0) |    		/* Configure master clock rate */
          (0 << 12) |                   /* Disable loopback mode */
          (1 << 13) |                   /* Enable master mode */
          (0 << 14) |                   /* Select IDL mode */
          /*(1 << 24) | */                  /* Enable PCM data swap */
          (0 << 24) |                   /* Disable PCM data swap */
          (0 << 31);                    /* Disable PCM */
	PCM1_CFG0 = value;
	
  value = ((0 & 0x1) << 15);    /* Select FSYNC mode , 0 : short FSYNC, 1 : long FSYNC */
	PCM1_CFG1 = value;

  value = ((0 & 0x7F) << 0) |	/* Rx Delay */
          ((0 & 0x7F) << 8) | /* Tx Delay */
          ((1 & 0x1) << 22) | /* Data Width: 0: 8bits; 1: 16bits */
          (1 << 23);    /* Enable this channel */
  PCM1_CH_CFG0 = value;

  value = ((16 & 0x7F) << 0) |	/* Rx Delay */
          ((16 & 0x7F) << 8) | /* Tx Delay */
          ((1 & 0x1) << 22) | /* Data Width: 0: 8bits; 1: 16bits */
          (1 << 23);    /* Enable this channel */
  PCM1_CH_CFG1 = value;

  PCM1_CH_CFG2 = 0;
  PCM1_CH_CFG3 = 0;
  
//  PCM1_TX_DATA_L = 0;
}

static struct resource *mem;

static int cns3xxx_pcm1dac_fmt_probe (struct platform_device *pdev)
{
	int ret;
	struct resource *ioarea;

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
		dev_err(&pdev->dev, "FMT region already claimed\n");
		return -EBUSY;
	}
		
    pcm1_base_va = ioremap (mem->start,mem->end-mem->start+1);
    if (!pcm1_base_va) printk ("Couldnt ioremap !! expect crash !!\n");

		cns3xxx_pcm1dac_fmt_init_pwr_up();

	cns3xxx_pcm1dac_fmt_dai.dev = &pdev->dev;
	cns3xxx_pcm1dac_fmt_dai.private_data = NULL;
	ret = snd_soc_register_dai(&cns3xxx_pcm1dac_fmt_dai);

//	printk("%s=>%d: %d\n", __FUNCTION__, __LINE__, ret);

    return ret;
}

static int __devexit cns3xxx_pcm1dac_fmt_remove(struct platform_device *dev)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	snd_soc_unregister_dai(&cns3xxx_pcm1dac_fmt_dai);

	cns3xxx_enable_pcm1(0);
	cns3xxx_pwr_clk_disable(CNS3XXX_PWR_CLK_EN(SPI_PCM_I2C));

	return 0;
}

#ifdef CONFIG_PM
//#warning "CONFIG_PM defined: suspend and resume not implemented"
static int cns3xxx_pcm1dac_fmt_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int cns3xxx_pcm1dac_fmt_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define cns3xxx_pcm1dac_fmt_suspend	NULL
#define cns3xxx_pcm1dac_fmt_resume	NULL
#endif

static struct platform_driver cns3xxx_pcm1dac_fmt_driver =  {
    .probe = cns3xxx_pcm1dac_fmt_probe,
		.remove = __devexit_p(cns3xxx_pcm1dac_fmt_remove),
		.suspend = cns3xxx_pcm1dac_fmt_suspend,
		.resume = cns3xxx_pcm1dac_fmt_resume,
    .driver = {
        .name = "cns3xxx-pcm1",
        .owner= THIS_MODULE,
    },
};

static int __init cns3xxx_pcm1dac_fmt_init (void)
{
#ifdef __DEBUG_PATH
		printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
    return platform_driver_register (&cns3xxx_pcm1dac_fmt_driver);
}

static void __exit cns3xxx_pcm1dac_fmt_exit (void)
{
#ifdef __DEBUG_PATH
		printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif		
    platform_driver_unregister (&cns3xxx_pcm1dac_fmt_driver);
}

module_init (cns3xxx_pcm1dac_fmt_init);
module_exit (cns3xxx_pcm1dac_fmt_exit);

/* Module information */
MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("CNS3XXX PCM1 SoC Interface");
MODULE_LICENSE("GPL");

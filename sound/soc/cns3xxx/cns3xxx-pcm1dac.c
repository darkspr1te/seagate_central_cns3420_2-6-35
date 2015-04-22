/*
 * ASoC driver for CNS3XXX
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>


#include <asm/dma.h>
#include <mach/hardware.h>

#include "../codecs/wm9081.h"
#include "cns3xxx-pcm1dac-pcm.h"
#include "cns3xxx-pcm1dac-fmt.h"

//#define __DEBUG_PATH		1		//show function flow...

static int cns3xxx_pcm1dac_jack_func;
//static int cns3xxx_pcm1dac_spk_func;


static int cns3xxx_pcm1dac_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	/* The following Data is from the CNS3XXX Architecture document */
	switch (params_rate(params)) {
		case 8000:
			clk = 4096000;
			break;
		case 11025:
		case 16000:
		case 22050:
		case 32000:
		case 44100:
		case 48000:
		case 96000:
		case 192000:
			printk (KERN_INFO "params seems to have not been set properly. check this!!\n");
			return -EINVAL;
	}
	
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A | 
					SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF);
	if (ret < 0) {
printk("%s=>%d\n", __FUNCTION__, __LINE__);
		return ret;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A |
					 SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF);
	if (ret < 0) {
printk("%s=>%d\n", __FUNCTION__, __LINE__);
		return ret;
	}

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM9081_SYSCLK_MCLK, clk,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
printk("%s=>%d\n", __FUNCTION__, __LINE__);
		return ret;
	}

	return 0;
}

static void cns3xxx_pcm1dac_ext_control(struct snd_soc_codec *codec)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

#if 0
	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);
    //printk ("%s:%d: Jack END = %d\n",__FUNCTION__,__LINE__, cns3xxx_pcm1dac_jack_func);
#endif    
}

static int cns3xxx_pcm1dac_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	/* check the jack status at stream startup */
	cns3xxx_pcm1dac_ext_control(codec);
	return 0;
}

/* we need to unmute the HP at shutdown as the mute burns power on corgi */
static void cns3xxx_pcm1dac_shutdown(struct snd_pcm_substream *substream)
{
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_codec *codec = rtd->socdev->codec;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
//	return 0;
}


static struct snd_soc_ops cns3xxx_pcm1dac_ops = {
	.startup = cns3xxx_pcm1dac_startup,
	.hw_params = cns3xxx_pcm1dac_hw_params,
	.shutdown = cns3xxx_pcm1dac_shutdown,
};


static int cns3xxx_pcm1dac_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	ucontrol->value.integer.value[0] = cns3xxx_pcm1dac_jack_func;
	return 0;
}

static int cns3xxx_pcm1dac_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	if (cns3xxx_pcm1dac_jack_func == ucontrol->value.integer.value[0])
		return 0;

	cns3xxx_pcm1dac_jack_func = ucontrol->value.integer.value[0];
	cns3xxx_pcm1dac_ext_control(codec);
	return 1;
}

//static int cns3xxx_pcm1dac_get_spk(struct snd_kcontrol *kcontrol,
//	struct snd_ctl_elem_value *ucontrol)
//{
//    //printk ("%s:%d: \n",__FUNCTION__,__LINE__);
//	ucontrol->value.integer.value[0] = cns3xxx_pcm1dac_spk_func;
//	return 0;
//}
//
//static int cns3xxx_pcm1dac_set_spk(struct snd_kcontrol *kcontrol,
//	struct snd_ctl_elem_value *ucontrol)
//{
//	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
//
//    //printk ("%s:%d: \n",__FUNCTION__,__LINE__);
//	if (cns3xxx_pcm1dac_spk_func == ucontrol->value.integer.value[0])
//		return 0;
//
//	cns3xxx_pcm1dac_spk_func = ucontrol->value.integer.value[0];
//	cns3xxx_pcm1dac_ext_control(codec);
//	return 1;
//}

//static int cns3xxx_pcm1dac_amp_event(struct snd_soc_dapm_widget *w,
//	struct snd_kcontrol *k, int event)
//{
//    //printk ("%s:%d: \n",__FUNCTION__,__LINE__);
//#if 0
//	if (SND_SOC_DAPM_EVENT_ON(event))
//		set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_APM_ON);
//	else
//		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_APM_ON);
//#endif
//	return 0;
//}

static int cns3xxx_pcm1dac_mic_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	return 0;
}


static const char *jack_function[] = {"Off", "Mic", "Headphone"};
//static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum cns3xxx_pcm1dac_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	//SOC_ENUM_SINGLE_EXT(2, spk_function),
};

/* corgi machine dapm widgets */
static const struct snd_soc_dapm_widget wm9081_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_MIC("Mic Jack", cns3xxx_pcm1dac_mic_event),
//SND_SOC_DAPM_SPK("Ext Spk", cns3xxx_pcm1dac_amp_event),
//SND_SOC_DAPM_LINE("Line Jack", NULL),
//SND_SOC_DAPM_HP("Headset Jack", NULL),
};


static const struct snd_kcontrol_new wm9081_cns3xxx_pcm1dac_controls[] = {
	SOC_ENUM_EXT("Jack Function", cns3xxx_pcm1dac_enum[0], cns3xxx_pcm1dac_get_jack,
		cns3xxx_pcm1dac_set_jack),
	/*SOC_ENUM_EXT("Speaker Function", cns3xxx_pcm1dac_enum[1], cns3xxx_pcm1dac_get_spk,
		cns3xxx_pcm1dac_set_spk),*/
};


/* Corgi machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headset Jack  - LIN1 & RIN1 = micin, out = LHPOUT*/
	{"LIN1", NULL, "Mic Jack"},
	{"RIN1", NULL, "Mic Jack"},

	/* headphone connected to LOUT, ROUT */
	{"Headphone Jack", NULL, "LOUT"},
	{"Headphone Jack", NULL, "ROUT"},
};

/* Logic for a wm9081 as connected on a CNS3XXX */
static int cns3xxx_pcm1dac_wm9081_init(struct snd_soc_codec *codec)
{
//	int i, err;
//	int err;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
#if 0
	/*snd_soc_dapm_disable_pin(codec, "LLINEIN");
	snd_soc_dapm_disable_pin(codec, "RLINEIN");*/

	/* Add cns3xxx specific controls */
	err = snd_soc_add_controls(codec, wm9081_cns3xxx_pcm1dac_controls,
				ARRAY_SIZE(wm9081_cns3xxx_pcm1dac_controls));
	if (err < 0)
		return err;

//	for (i = 0; i < ARRAY_SIZE(wm9081_cns3xxx_pcm1dac_controls); i++) {
//		err = snd_ctl_add(codec->card,
//			snd_soc_cnew(&wm9081_cns3xxx_pcm1dac_controls[i], codec, NULL));
//		if (err < 0)
//			return err;
//	}

	/* Add cns3xxx specific widgets */
	snd_soc_dapm_new_controls(codec, wm9081_dapm_widgets,
				  ARRAY_SIZE(wm9081_dapm_widgets));

	/* Set up cns3xxx specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
#endif
	return 0;
}


/* cns3xxx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cns3xxx_pcm1dac_dai = {
	.name = "wm9081",
	.stream_name = "wm9081",
	.cpu_dai = &cns3xxx_pcm1dac_fmt_dai,
	.codec_dai = &wm9081_dai,
	.init = cns3xxx_pcm1dac_wm9081_init,
	.ops = &cns3xxx_pcm1dac_ops,
};

struct snd_soc_card snd_soc_card_cns3xxx_pcm1dac = {
	.name = "CNS3XXX PCM1 Audio",
	.dai_link = &cns3xxx_pcm1dac_dai,
	.num_links = 1,
	.platform = &cns3xxx_pcm1dac_soc_platform,
};

/* cns3xxx audio subsystem */
static struct snd_soc_device cns3xxx_pcm1dac_snd_devdata = {
	.card = &snd_soc_card_cns3xxx_pcm1dac,
	.codec_dev = &soc_codec_dev_wm9081,
};

static struct platform_device *cns3xxx_pcm1dac_snd_device;
extern struct i2c_adapter cns3xxx_i2c_adapter;


static int __devinit cns3xxx_pcm1dac_init(void)
{
	int ret;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	printk("I2C adapter is:%s\n", cns3xxx_i2c_adapter.name);

	cns3xxx_pcm1dac_snd_device = platform_device_alloc("soc-audio", 0);
	if (!cns3xxx_pcm1dac_snd_device)
		return -ENOMEM;

	platform_set_drvdata(cns3xxx_pcm1dac_snd_device, &cns3xxx_pcm1dac_snd_devdata);
	cns3xxx_pcm1dac_snd_devdata.dev = &cns3xxx_pcm1dac_snd_device->dev;

	ret = platform_device_add(cns3xxx_pcm1dac_snd_device);
	if (ret)
		platform_device_put(cns3xxx_pcm1dac_snd_device);

	return ret;
}

static void __exit cns3xxx_pcm1dac_exit(void)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	platform_device_put(cns3xxx_pcm1dac_snd_device);
	platform_device_unregister(cns3xxx_pcm1dac_snd_device);
}

module_init(cns3xxx_pcm1dac_init);
module_exit(cns3xxx_pcm1dac_exit);

MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("CNS3XXX PCM1 ASoC driver");
MODULE_LICENSE("GPL");

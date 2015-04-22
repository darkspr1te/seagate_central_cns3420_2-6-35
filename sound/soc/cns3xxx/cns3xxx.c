/*
 * ASoC driver for CNS3XXX
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/i2c.h>


#include <asm/dma.h>
#include <mach/hardware.h>

#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
#include "../codecs/wm8991.h"
#endif
#if defined(CONFIG_SND_SOC_WM8580) || defined(CONFIG_SND_SOC_WM8580_MODULE)
#include "../codecs/wm8580.h"
#endif
#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)
#include "../codecs/wau8822.h"
#endif
#include "cns3xxx-pcm.h"
#include "cns3xxx-i2s.h"

//#define __DEBUG_PATH		1		//show function flow...

static int cns3xxx_jack_func;
//static int cns3xxx_spk_func;


static int cns3xxx_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
	printk("%s=>%d: %d\n", __FUNCTION__, __LINE__, params_rate(params));
#endif
	/* The following Data is from the CNS3XXX Architecture document */
	switch (params_rate(params)) {
		case 8000:
		case 16000:
		case 32000:
			clk = 8192000;
			break;
		case 48000:
		case 96000:
			clk = 12288000;
			break;
		case 11025:
		case 22050:
		case 44100:
			clk = 11289600;
			break;
		case 192000:
			printk (KERN_INFO "params seems to have not been set properly. check this!!\n");
			return -EINVAL;
	}
	
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | 
					SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF);
	if (ret < 0) {
printk("%s=>%d\n", __FUNCTION__, __LINE__);
		return ret;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
					 SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF);
//	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A |
//					 SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF);
	if (ret < 0) {
printk("%s=>%d\n", __FUNCTION__, __LINE__);
		return ret;
	}

#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, clk,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
printk("%s=>%d\n", __FUNCTION__, __LINE__);
		return ret;
	}
#endif

	return 0;
}

static void cns3xxx_ext_control(struct snd_soc_codec *codec)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");

	/* signal a DAPM event */
	snd_soc_dapm_sync(codec);
#endif
    //printk ("%s:%d: Jack END = %d\n",__FUNCTION__,__LINE__, cns3xxx_jack_func);
}

static int cns3xxx_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->card->codec;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	/* check the jack status at stream startup */
	cns3xxx_ext_control(codec);
	return 0;
}

/* we need to unmute the HP at shutdown as the mute burns power on corgi */
static void cns3xxx_shutdown(struct snd_pcm_substream *substream)
{
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_codec *codec = rtd->socdev->codec;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
//	return 0;
}


static struct snd_soc_ops cns3xxx_ops = {
	.startup = cns3xxx_startup,
	.hw_params = cns3xxx_hw_params,
	.shutdown = cns3xxx_shutdown,
};


static int cns3xxx_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	ucontrol->value.integer.value[0] = cns3xxx_jack_func;
	return 0;
}

static int cns3xxx_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	if (cns3xxx_jack_func == ucontrol->value.integer.value[0])
		return 0;

	cns3xxx_jack_func = ucontrol->value.integer.value[0];
	cns3xxx_ext_control(codec);
	return 1;
}

//static int cns3xxx_get_spk(struct snd_kcontrol *kcontrol,
//	struct snd_ctl_elem_value *ucontrol)
//{
//    //printk ("%s:%d: \n",__FUNCTION__,__LINE__);
//	ucontrol->value.integer.value[0] = cns3xxx_spk_func;
//	return 0;
//}
//
//static int cns3xxx_set_spk(struct snd_kcontrol *kcontrol,
//	struct snd_ctl_elem_value *ucontrol)
//{
//	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
//
//    //printk ("%s:%d: \n",__FUNCTION__,__LINE__);
//	if (cns3xxx_spk_func == ucontrol->value.integer.value[0])
//		return 0;
//
//	cns3xxx_spk_func = ucontrol->value.integer.value[0];
//	cns3xxx_ext_control(codec);
//	return 1;
//}

//static int cns3xxx_amp_event(struct snd_soc_dapm_widget *w,
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

static int cns3xxx_mic_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
#if 0
	if (SND_SOC_DAPM_EVENT_ON(event))
		set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MIC_BIAS);
	else
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MIC_BIAS);
#endif

	return 0;
}


#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)

static const char *jack_function[] = {"Off", "Mic", "Headphone"};
//static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum cns3xxx_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	//SOC_ENUM_SINGLE_EXT(2, spk_function),
};

/* corgi machine dapm widgets */
static const struct snd_soc_dapm_widget wm8991_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_MIC("Mic Jack", cns3xxx_mic_event),
//SND_SOC_DAPM_SPK("Ext Spk", cns3xxx_amp_event),
//SND_SOC_DAPM_LINE("Line Jack", NULL),
//SND_SOC_DAPM_HP("Headset Jack", NULL),
};


static const struct snd_kcontrol_new wm8991_cns3xxx_controls[] = {
	SOC_ENUM_EXT("Jack Function", cns3xxx_enum[0], cns3xxx_get_jack,
		cns3xxx_set_jack),
	/*SOC_ENUM_EXT("Speaker Function", cns3xxx_enum[1], cns3xxx_get_spk,
		cns3xxx_set_spk),*/
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

/* Logic for a wm8991 as connected on a CNS3XXX */
static int cns3xxx_wm8991_init(struct snd_soc_codec *codec)
{
//	int i, err;
	int err;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	/*snd_soc_dapm_disable_pin(codec, "LLINEIN");
	snd_soc_dapm_disable_pin(codec, "RLINEIN");*/

	/* Add cns3xxx specific controls */
	err = snd_soc_add_controls(codec, wm8991_cns3xxx_controls,
				ARRAY_SIZE(wm8991_cns3xxx_controls));
	if (err < 0)
		return err;

//	for (i = 0; i < ARRAY_SIZE(wm8991_cns3xxx_controls); i++) {
//		err = snd_ctl_add(codec->card,
//			snd_soc_cnew(&wm8991_cns3xxx_controls[i], codec, NULL));
//		if (err < 0)
//			return err;
//	}

	/* Add cns3xxx specific widgets */
	snd_soc_dapm_new_controls(codec, wm8991_dapm_widgets,
				  ARRAY_SIZE(wm8991_dapm_widgets));

	/* Set up cns3xxx specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
	return 0;
}


/* cns3xxx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cns3xxx_dai = {
	.name = "wm8991",
	.stream_name = "WM8991",
	.cpu_dai = &cns3xxx_i2s_dai,
	.codec_dai = &wm8991_dai,
	.init = cns3xxx_wm8991_init,
	.ops = &cns3xxx_ops,
};

#endif //CONFIG_SND_SOC_WM8991

#if defined(CONFIG_SND_SOC_WM8580) || defined(CONFIG_SND_SOC_WM8580_MODULE)

static const char *jack_function[] = {"Off", "Mic", "Headphone"};
//static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum cns3xxx_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	//SOC_ENUM_SINGLE_EXT(2, spk_function),
};

/* corgi machine dapm widgets */
static const struct snd_soc_dapm_widget wm8580_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_MIC("Mic Jack", cns3xxx_mic_event),
//SND_SOC_DAPM_SPK("Ext Spk", cns3xxx_amp_event),
//SND_SOC_DAPM_LINE("Line Jack", NULL),
//SND_SOC_DAPM_HP("Headset Jack", NULL),
};


static const struct snd_kcontrol_new wm8580_cns3xxx_controls[] = {
	SOC_ENUM_EXT("Jack Function", cns3xxx_enum[0], cns3xxx_get_jack,
		cns3xxx_set_jack),
	/*SOC_ENUM_EXT("Speaker Function", cns3xxx_enum[1], cns3xxx_get_spk,
		cns3xxx_set_spk),*/
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

/* Logic for a wm8580 as connected on a CNS3XXX */
static int cns3xxx_wm8580_init(struct snd_soc_codec *codec)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
#if 0
//	int i, err;
	int err;

	//printk ("%s:%d: \n",__FUNCTION__,__LINE__);
	/*snd_soc_dapm_disable_pin(codec, "LLINEIN");
	snd_soc_dapm_disable_pin(codec, "RLINEIN");*/

	/* Add cns3xxx specific controls */
	err = snd_soc_add_controls(codec, wm8580_cns3xxx_controls,
				ARRAY_SIZE(wm8580_cns3xxx_controls));
	if (err < 0)
		return err;

//	for (i = 0; i < ARRAY_SIZE(wm8580_cns3xxx_controls); i++) {
//		err = snd_ctl_add(codec->card,
//			snd_soc_cnew(&wm8580_cns3xxx_controls[i], codec, NULL));
//		if (err < 0)
//			return err;
//	}

	/* Add cns3xxx specific widgets */
	snd_soc_dapm_new_controls(codec, wm8580_dapm_widgets,
				  ARRAY_SIZE(wm8580_dapm_widgets));

	/* Set up cns3xxx specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
#endif	
	return 0;
}


/* cns3xxx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cns3xxx_dai = {
	.name = "wm8580",
	.stream_name = "WM8580",
	.cpu_dai = &cns3xxx_i2s_dai,
	.codec_dai = &wm8580_dai[0],
	.init = cns3xxx_wm8580_init,
	.ops = &cns3xxx_ops,
};

#endif //CONFIG_SND_SOC_WM8580

#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)

static const char *jack_function[] = {"Off", "Mic", "Headphone"};
//static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum cns3xxx_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	//SOC_ENUM_SINGLE_EXT(2, spk_function),
};

/* corgi machine dapm widgets */
static const struct snd_soc_dapm_widget wau8822_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_MIC("Mic Jack", cns3xxx_mic_event),
//SND_SOC_DAPM_SPK("Ext Spk", cns3xxx_amp_event),
//SND_SOC_DAPM_LINE("Line Jack", NULL),
//SND_SOC_DAPM_HP("Headset Jack", NULL),
};


static const struct snd_kcontrol_new wau8822_cns3xxx_controls[] = {
	SOC_ENUM_EXT("Jack Function", cns3xxx_enum[0], cns3xxx_get_jack,
		cns3xxx_set_jack),
	/*SOC_ENUM_EXT("Speaker Function", cns3xxx_enum[1], cns3xxx_get_spk,
		cns3xxx_set_spk),*/
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

/* Logic for a wau8822 as connected on a CNS3XXX */
static int cns3xxx_wau8822_init(struct snd_soc_codec *codec)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
#if 0
//	int i, err;
	int err;

	//printk ("%s:%d: \n",__FUNCTION__,__LINE__);
	/*snd_soc_dapm_disable_pin(codec, "LLINEIN");
	snd_soc_dapm_disable_pin(codec, "RLINEIN");*/

	/* Add cns3xxx specific controls */
	err = snd_soc_add_controls(codec, wau8822_cns3xxx_controls,
				ARRAY_SIZE(wau8822_cns3xxx_controls));
	if (err < 0)
		return err;

//	for (i = 0; i < ARRAY_SIZE(wau8822_cns3xxx_controls); i++) {
//		err = snd_ctl_add(codec->card,
//			snd_soc_cnew(&wau8822_cns3xxx_controls[i], codec, NULL));
//		if (err < 0)
//			return err;
//	}

	/* Add cns3xxx specific widgets */
	snd_soc_dapm_new_controls(codec, wau8822_dapm_widgets,
				  ARRAY_SIZE(wau8822_dapm_widgets));

	/* Set up cns3xxx specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
#endif	
	return 0;
}


/* cns3xxx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link cns3xxx_dai = {
	.name = "wau8822",
	.stream_name = "WAU8822",
	.cpu_dai = &cns3xxx_i2s_dai,
	.codec_dai = &wau8822_dai,
	.init = cns3xxx_wau8822_init,
	.ops = &cns3xxx_ops,
};

static struct wau8822_setup_data wau8822_setup = {
	.i2c_bus = 0,
	.i2c_address = 0x0d,
};

#endif //CONFIG_SND_SOC_WAU8822

struct snd_soc_card snd_soc_card_cns3xxx = {
	.name = "CNS3XXX Audio",
	.dai_link = &cns3xxx_dai,
	.num_links = 1,
	.platform = &cns3xxx_soc_platform,
};

/* cns3xxx audio subsystem */
static struct snd_soc_device cns3xxx_snd_devdata = {
	.card = &snd_soc_card_cns3xxx,
#if defined(CONFIG_SND_SOC_WM8991) || defined(CONFIG_SND_SOC_WM8991_MODULE)
	.codec_dev = &soc_codec_dev_wm8991,
#endif
#if defined(CONFIG_SND_SOC_WM8580) || defined(CONFIG_SND_SOC_WM8580_MODULE)
	.codec_dev = &soc_codec_dev_wm8580,
#endif
#if defined(CONFIG_SND_SOC_WAU8822) || defined(CONFIG_SND_SOC_WAU8822_MODULE)
	.codec_dev = &soc_codec_dev_wau8822,
	.codec_data = &wau8822_setup,
#endif
};

static struct platform_device *cns3xxx_snd_device = NULL;
extern struct i2c_adapter cns3xxx_i2c_adapter;


static int __devinit cns3xxx_init(void)
{
	int ret;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	printk("I2C adapter is:%s\n", cns3xxx_i2c_adapter.name);

	cns3xxx_snd_device = platform_device_alloc("soc-audio", 0);
	if (!cns3xxx_snd_device)
		return -ENOMEM;

	platform_set_drvdata(cns3xxx_snd_device, &cns3xxx_snd_devdata);
	cns3xxx_snd_devdata.dev = &cns3xxx_snd_device->dev;

	ret = platform_device_add(cns3xxx_snd_device);
	if (ret)
		platform_device_put(cns3xxx_snd_device);

	return ret;
}

static void __exit cns3xxx_exit(void)
{
#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif
	platform_device_unregister(cns3xxx_snd_device);
}

module_init(cns3xxx_init);
module_exit(cns3xxx_exit);

MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("CNS3XXX ASoC driver");
MODULE_LICENSE("GPL");

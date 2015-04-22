/*
 * wau8822.c  --  WAU8822 ALSA Soc Audio driver
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 *
 * Authors: Liam Girdwood <lg@opensource.wolfsonmicro.com>
 * 2008.12.05 Henry Lo <CYLo@nuvoton.com> modified for WAU8822
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
//#include <linux/version.h>
//#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/slab.h>

#include "wau8822.h"

//#define __DEBUG_PATH		1

//#define AUDIO_NAME "wau8822"
#define WAU8822_VERSION "0.1"

struct snd_soc_codec_device soc_codec_dev_wau8822;
static struct snd_soc_codec *wau8822_codec;

struct wau8822_priv {
	struct snd_soc_codec codec;
	u16 reg_cache[WAU8822_CACHEREGNUM];
};

/*
 * wau8822 register cache
 * We can't read the WAU8822 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
#if 1
static const u16 wau8822_reg[WAU8822_CACHEREGNUM] = {
    0x0000, 0x01ed, 0x01bf, 0x01ff, //0x00-0x03
    0x0050, 0x0000, 0x0000, 0x0000, //0x04-0x07
    0x0000, 0x0000, 0x000c, 0x01ff, //0x08-0x0b
    0x01ff, 0x0000, 0x0100, 0x00ff, //0x0c-0x0f
    0x00ff, 0x0006, 0x012c, 0x002c, //0x10-0x13
    0x002c, 0x002c, 0x002c, 0x0000, //0x14-0x17
    0x0032, 0x0000, 0x0000, 0x0000, //0x18-0x1b
    0x0000, 0x0000, 0x0000, 0x0000, //0x1c-0x1f
    0x0038, 0x000b, 0x0132, 0x0010, //0x20-0x23
    0x0008, 0x000c, 0x0093, 0x00e9, //0x24-0x27
    0x0000, 0x0000, 0x0000, 0x0020, //0x28-0x2b
    0x0033, 0x0010, 0x0010, 0x0100, //0x2c-0x2f
    0x0100, 0x0003, 0x0001, 0x0001, //0x30-0x33
    0x0131, 0x0131, 0x0139, 0x0139, //0x34-0x37
    0x0140, 0x0140, 0x0000, 0x0000, //0x38-0x3b
    0x0020, 0x0020, //0x3c-0x3d
};
#else
static const u16 wau8822_reg[WAU8822_CACHEREGNUM] = {
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0050, 0x0000, 0x0140, 0x0000,
    0x0000, 0x0000, 0x0000, 0x00ff,
    0x00ff, 0x0000, 0x0100, 0x00ff,
    0x00ff, 0x0000, 0x012c, 0x002c,
    0x002c, 0x002c, 0x002c, 0x0000,
    0x0032, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0038, 0x000b, 0x0032, 0x0000,
    0x0008, 0x000c, 0x0093, 0x00e9,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0033, 0x0010, 0x0010, 0x0100,
    0x0100, 0x0002, 0x0001, 0x0001,
    0x0039, 0x0039, 0x0039, 0x0039,
    0x0001, 0x0001, 0x0000, 0x0000,
    0x0020, 0x0000,
};
#endif

void Wau8822DumpRegister(struct snd_soc_codec *codec)
{
	int i;
	
	printk("\n");
  printk("\n>>>>>>>>>> Dump Reg Cache <<<<<<<<<<<<\n");
	for(i=0; i<WAU8822_CACHEREGNUM; i++) {
 		if((i%8 == 0) && i)
 			printk("\n");
		printk("R%d:%04x  ", i, ((u16 *)codec->reg_cache)[i]);
	}
	printk("\n");
	printk("\n");
}

/*
 * read wau8822 register cache
 */
static inline unsigned int wau8822_read_reg_cache(struct snd_soc_codec  *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WAU8822_RESET)
		return 0;
	if (reg >= WAU8822_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wau8822 register cache
 */
static inline void wau8822_write_reg_cache(struct snd_soc_codec  *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WAU8822_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WAU8822 register space
 */
static int wau8822_write(struct snd_soc_codec  *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WAU8822 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wau8822_write_reg_cache (codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2) {
#ifdef __DEBUG_PATH
		printk("%s=> reg:0x%08x,value:0x%08x success\n", __FUNCTION__, reg, value);
#endif
		return 0;
	}
	else {
#ifdef __DEBUG_PATH
		printk("%s=> reg:0x%08x,value:0x%08x error\n", __FUNCTION__, reg, value);
#endif
		return -1;
	}
}

#define wau8822_reset(c)	wau8822_write(c, WAU8822_RESET, 0)

static const char *wau8822_companding[] = {"Off", "NC", "u-law", "A-law" };
static const char *wau8822_deemp[] = {"None", "32kHz", "44.1kHz", "48kHz" };
static const char *wau8822_eqmode[] = {"Capture", "Playback" };
static const char *wau8822_bw[] = {"Narrow", "Wide" };
static const char *wau8822_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz" };
static const char *wau8822_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz" };
static const char *wau8822_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz" };
static const char *wau8822_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz" };
static const char *wau8822_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz" };
static const char *wau8822_alc[] =
    {"ALC both on", "ALC left only", "ALC right only", "Limiter" };

static const struct soc_enum wau8822_enum[] = {
	SOC_ENUM_SINGLE(WAU8822_COMP, 1, 4, wau8822_companding), /* adc */
	SOC_ENUM_SINGLE(WAU8822_COMP, 3, 4, wau8822_companding), /* dac */
	SOC_ENUM_SINGLE(WAU8822_DAC,  4, 4, wau8822_deemp),
	SOC_ENUM_SINGLE(WAU8822_EQ1,  8, 2, wau8822_eqmode),

	SOC_ENUM_SINGLE(WAU8822_EQ1,  5, 4, wau8822_eq1),
	SOC_ENUM_SINGLE(WAU8822_EQ2,  8, 2, wau8822_bw),
	SOC_ENUM_SINGLE(WAU8822_EQ2,  5, 4, wau8822_eq2),
	SOC_ENUM_SINGLE(WAU8822_EQ3,  8, 2, wau8822_bw),

	SOC_ENUM_SINGLE(WAU8822_EQ3,  5, 4, wau8822_eq3),
	SOC_ENUM_SINGLE(WAU8822_EQ4,  8, 2, wau8822_bw),
	SOC_ENUM_SINGLE(WAU8822_EQ4,  5, 4, wau8822_eq4),
	SOC_ENUM_SINGLE(WAU8822_EQ5,  8, 2, wau8822_bw),

	SOC_ENUM_SINGLE(WAU8822_EQ5,  5, 4, wau8822_eq5),
	SOC_ENUM_SINGLE(WAU8822_ALC3,  8, 2, wau8822_alc),
};

static const struct snd_kcontrol_new wau8822_snd_controls[] = {
SOC_SINGLE("Digital Loopback Switch", WAU8822_COMP, 0, 1, 0),

SOC_ENUM("ADC Companding", wau8822_enum[0]),
SOC_ENUM("DAC Companding", wau8822_enum[1]),

SOC_SINGLE("Jack Detection Enable", WAU8822_JACK1, 6, 1, 0),

SOC_SINGLE("DAC Right Inversion Switch", WAU8822_DAC, 1, 1, 0),
SOC_SINGLE("DAC Left Inversion Switch", WAU8822_DAC, 0, 1, 0),

SOC_SINGLE("Left Playback Volume", WAU8822_DACVOLL, 0, 127, 0),
SOC_SINGLE("Right Playback Volume", WAU8822_DACVOLR, 0, 127, 0),

//SOC_SINGLE("High Pass Filter Switch", WAU8822_ADC, 8, 1, 0),
//SOC_SINGLE("High Pass Filter Switch", WAU8822_ADC, 8, 1, 0),
//SOC_SINGLE("High Pass Cut Off", WAU8822_ADC, 4, 7, 0),
//SOC_SINGLE("Right ADC Inversion Switch", WAU8822_ADC, 1, 1, 0),
//SOC_SINGLE("Left ADC Inversion Switch", WAU8822_ADC, 0, 1, 0),
//
//SOC_SINGLE("Left Capture Volume", WAU8822_ADCVOLL,  0, 127, 0),
//SOC_SINGLE("Right Capture Volume", WAU8822_ADCVOLR,  0, 127, 0),

SOC_ENUM("Equaliser Function", wau8822_enum[3]),
SOC_ENUM("EQ1 Cut Off", wau8822_enum[4]),
SOC_SINGLE("EQ1 Volume", WAU8822_EQ1,  0, 31, 1),

SOC_ENUM("Equaliser EQ2 Bandwith", wau8822_enum[5]),
SOC_ENUM("EQ2 Cut Off", wau8822_enum[6]),
SOC_SINGLE("EQ2 Volume", WAU8822_EQ2,  0, 31, 1),

SOC_ENUM("Equaliser EQ3 Bandwith", wau8822_enum[7]),
SOC_ENUM("EQ3 Cut Off", wau8822_enum[8]),
SOC_SINGLE("EQ3 Volume", WAU8822_EQ3,  0, 31, 1),

SOC_ENUM("Equaliser EQ4 Bandwith", wau8822_enum[9]),
SOC_ENUM("EQ4 Cut Off", wau8822_enum[10]),
SOC_SINGLE("EQ4 Volume", WAU8822_EQ4,  0, 31, 1),

SOC_ENUM("Equaliser EQ5 Bandwith", wau8822_enum[11]),
SOC_ENUM("EQ5 Cut Off", wau8822_enum[12]),
SOC_SINGLE("EQ5 Volume", WAU8822_EQ5,  0, 31, 1),

SOC_SINGLE("DAC Playback Limiter Switch", WAU8822_DACLIM1,  8, 1, 0),
SOC_SINGLE("DAC Playback Limiter Decay", WAU8822_DACLIM1,  4, 15, 0),
SOC_SINGLE("DAC Playback Limiter Attack", WAU8822_DACLIM1,  0, 15, 0),

SOC_SINGLE("DAC Playback Limiter Threshold", WAU8822_DACLIM2,  4, 7, 0),
SOC_SINGLE("DAC Playback Limiter Boost", WAU8822_DACLIM2,  0, 15, 0),

SOC_SINGLE("ALC Enable Switch", WAU8822_ALC1,  8, 1, 0),
SOC_SINGLE("ALC Capture Max Gain", WAU8822_ALC1,  3, 7, 0),
SOC_SINGLE("ALC Capture Min Gain", WAU8822_ALC1,  0, 7, 0),

SOC_SINGLE("ALC Capture ZC Switch", WAU8822_ALC2,  8, 1, 0),
SOC_SINGLE("ALC Capture Hold", WAU8822_ALC2,  4, 7, 0),
SOC_SINGLE("ALC Capture Target", WAU8822_ALC2,  0, 15, 0),

SOC_ENUM("ALC Capture Mode", wau8822_enum[13]),
SOC_SINGLE("ALC Capture Decay", WAU8822_ALC3,  4, 15, 0),
SOC_SINGLE("ALC Capture Attack", WAU8822_ALC3,  0, 15, 0),

SOC_SINGLE("ALC Capture Noise Gate Switch", WAU8822_NGATE,  3, 1, 0),
SOC_SINGLE("ALC Capture Noise Gate Threshold", WAU8822_NGATE,  0, 7, 0),

SOC_SINGLE("Left Capture PGA ZC Switch", WAU8822_INPPGAL,  7, 1, 0),
SOC_SINGLE("Left Capture PGA Volume", WAU8822_INPPGAL,  0, 63, 0),

SOC_SINGLE("Right Capture PGA ZC Switch", WAU8822_INPPGAR,  7, 1, 0),
SOC_SINGLE("Right Capture PGA Volume", WAU8822_INPPGAR,  0, 63, 0),

SOC_SINGLE("Left Headphone Playback ZC Switch", WAU8822_HPVOLL,  7, 1, 0),
SOC_SINGLE("Left Headphone Playback Switch", WAU8822_HPVOLL,  6, 1, 1),
SOC_SINGLE("Left Headphone Playback Volume", WAU8822_HPVOLL,  0, 63, 0),

SOC_SINGLE("Right Headphone Playback ZC Switch", WAU8822_HPVOLR,  7, 1, 0),
SOC_SINGLE("Right Headphone Playback Switch", WAU8822_HPVOLR,  6, 1, 1),
SOC_SINGLE("Right Headphone Playback Volume", WAU8822_HPVOLR,  0, 63, 0),

SOC_SINGLE("Left Speaker Playback ZC Switch", WAU8822_SPKVOLL,  7, 1, 0),
SOC_SINGLE("Left Speaker Playback Switch", WAU8822_SPKVOLL,  6, 1, 1),
SOC_SINGLE("Left Speaker Playback Volume", WAU8822_SPKVOLL,  0, 63, 0),

SOC_SINGLE("Right Speaker Playback ZC Switch", WAU8822_SPKVOLR,  7, 1, 0),
SOC_SINGLE("Right Speaker Playback Switch", WAU8822_SPKVOLR,  6, 1, 1),
SOC_SINGLE("Right Speaker Playback Volume", WAU8822_SPKVOLR,  0, 63, 0),

SOC_DOUBLE_R("Capture Boost(+20dB)", WAU8822_ADCBOOSTL, WAU8822_ADCBOOSTR,
	8, 1, 0),
};

/* Left Output Mixer */
static const struct snd_kcontrol_new wau8822_left_mixer_controls[] = {
SOC_DAPM_SINGLE("Right PCM Playback Switch", WAU8822_OUTPUT, 6, 1, 1),
SOC_DAPM_SINGLE("Left PCM Playback Switch", WAU8822_MIXL, 0, 1, 1),
SOC_DAPM_SINGLE("Line Bypass Switch", WAU8822_MIXL, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WAU8822_MIXL, 5, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wau8822_right_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WAU8822_OUTPUT, 5, 1, 1),
SOC_DAPM_SINGLE("Right PCM Playback Switch", WAU8822_MIXR, 0, 1, 1),
SOC_DAPM_SINGLE("Line Bypass Switch", WAU8822_MIXR, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WAU8822_MIXR, 5, 1, 0),
};

/* BIASGEN control */
static const struct snd_kcontrol_new wau8822_biasg_en_controls =
SOC_DAPM_SINGLE("BIAS Gen", WAU8822_BIASGEN, 4, WAU8822_5V_MODE?1:0, 0);

/* Speaker boost control */
static const struct snd_kcontrol_new wau8822_speaker_boost_controls =
SOC_DAPM_SINGLE("Speaker Boost", WAU8822_SPKBOOST, 2, WAU8822_5V_MODE?0:1, 0);
/* Aux1 boost control */
static const struct snd_kcontrol_new wau8822_out3_boost_controls =
SOC_DAPM_SINGLE("Out3 Boost", WAU8822_AUX1BOOST, 3, WAU8822_5V_MODE?0:1, 0);
/* Aux2 boost control */
static const struct snd_kcontrol_new wau8822_out4_boost_controls =
SOC_DAPM_SINGLE("Out4 Boost", WAU8822_AUX2BOOST, 4, WAU8822_5V_MODE?0:1, 0);

/* Left AUX Input boost vol */
static const struct snd_kcontrol_new wau8822_laux_boost_controls =
SOC_DAPM_SINGLE("Left Aux Volume", WAU8822_ADCBOOSTL, 0, 3, 0);

/* Right AUX Input boost vol */
static const struct snd_kcontrol_new wau8822_raux_boost_controls =
SOC_DAPM_SINGLE("Right Aux Volume", WAU8822_ADCBOOSTR, 0, 3, 0);

/* Left Input boost vol */
static const struct snd_kcontrol_new wau8822_lmic_boost_controls =
SOC_DAPM_SINGLE("Left Input Volume", WAU8822_ADCBOOSTL, 4, 3, 0);

/* Right Input boost vol */
static const struct snd_kcontrol_new wau8822_rmic_boost_controls =
SOC_DAPM_SINGLE("Right Input Volume", WAU8822_ADCBOOSTR, 4, 3, 0);

/* Left Aux In to PGA */
static const struct snd_kcontrol_new wau8822_laux_capture_boost_controls =
SOC_DAPM_SINGLE("Left Capture Switch", WAU8822_ADCBOOSTL,  8, 1, 0);

/* Right  Aux In to PGA */
static const struct snd_kcontrol_new wau8822_raux_capture_boost_controls =
SOC_DAPM_SINGLE("Right Capture Switch", WAU8822_ADCBOOSTR,  8, 1, 0);

/* Left Input P In to PGA */
static const struct snd_kcontrol_new wau8822_lmicp_capture_boost_controls =
SOC_DAPM_SINGLE("Left Input P Capture Boost Switch", WAU8822_INPUT,  0, 1, 0);

/* Right Input P In to PGA */
static const struct snd_kcontrol_new wau8822_rmicp_capture_boost_controls =
SOC_DAPM_SINGLE("Right Input P Capture Boost Switch", WAU8822_INPUT,  4, 1, 0);

/* Left Input N In to PGA */
static const struct snd_kcontrol_new wau8822_lmicn_capture_boost_controls =
SOC_DAPM_SINGLE("Left Input N Capture Boost Switch", WAU8822_INPUT,  1, 1, 0);

/* Right Input N In to PGA */
static const struct snd_kcontrol_new wau8822_rmicn_capture_boost_controls =
SOC_DAPM_SINGLE("Right Input N Capture Boost Switch", WAU8822_INPUT,  5, 1, 0);

// TODO Widgets
static const struct snd_soc_dapm_widget wau8822_dapm_widgets[] = {
#if 0
//SND_SOC_DAPM_MUTE("Mono Mute", WAU8822_MONOMIX, 6, 0),
//SND_SOC_DAPM_MUTE("Speaker Mute", WAU8822_SPKMIX, 6, 0),

SND_SOC_DAPM_MIXER("Speaker Mixer", WAU8822_POWER3, 2, 0,
	&wau8822_speaker_mixer_controls[0],
	ARRAY_SIZE(wau8822_speaker_mixer_controls)),
SND_SOC_DAPM_MIXER("Mono Mixer", WAU8822_POWER3, 3, 0,
	&wau8822_mono_mixer_controls[0],
	ARRAY_SIZE(wau8822_mono_mixer_controls)),
SND_SOC_DAPM_DAC("DAC", "HiFi Playback", WAU8822_POWER3, 0, 0),
SND_SOC_DAPM_ADC("ADC", "HiFi Capture", WAU8822_POWER3, 0, 0),
SND_SOC_DAPM_PGA("Aux Input", WAU8822_POWER1, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkN Out", WAU8822_POWER3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkP Out", WAU8822_POWER3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mono Out", WAU8822_POWER3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic PGA", WAU8822_POWER2, 2, 0, NULL, 0),

SND_SOC_DAPM_PGA("Aux Boost", SND_SOC_NOPM, 0, 0,
	&wau8822_aux_boost_controls, 1),
SND_SOC_DAPM_PGA("Mic Boost", SND_SOC_NOPM, 0, 0,
	&wau8822_mic_boost_controls, 1),
SND_SOC_DAPM_SWITCH("Capture Boost", SND_SOC_NOPM, 0, 0,
	&wau8822_capture_boost_controls),

SND_SOC_DAPM_MIXER("Boost Mixer", WAU8822_POWER2, 4, 0, NULL, 0),

SND_SOC_DAPM_MICBIAS("Mic Bias", WAU8822_POWER1, 4, 0),

SND_SOC_DAPM_INPUT("MICN"),
SND_SOC_DAPM_INPUT("MICP"),
SND_SOC_DAPM_INPUT("AUX"),
SND_SOC_DAPM_OUTPUT("MONOOUT"),
SND_SOC_DAPM_OUTPUT("SPKOUTP"),
SND_SOC_DAPM_OUTPUT("SPKOUTN"),
#endif
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Mono output mixer */
	{"Mono Mixer", "PCM Playback Switch", "DAC"},
	{"Mono Mixer", "Aux Playback Switch", "Aux Input"},
	{"Mono Mixer", "Line Bypass Switch", "Boost Mixer"},

	/* Speaker output mixer */
	{"Speaker Mixer", "PCM Playback Switch", "DAC"},
	{"Speaker Mixer", "Aux Playback Switch", "Aux Input"},
	{"Speaker Mixer", "Line Bypass Switch", "Boost Mixer"},

	/* Outputs */
	{"Mono Out", NULL, "Mono Mixer"},
	{"MONOOUT", NULL, "Mono Out"},
	{"SpkN Out", NULL, "Speaker Mixer"},
	{"SpkP Out", NULL, "Speaker Mixer"},
	{"SPKOUTN", NULL, "SpkN Out"},
	{"SPKOUTP", NULL, "SpkP Out"},

	/* Boost Mixer */
	{"Boost Mixer", NULL, "ADC"},
	{"Capture Boost Switch", "Aux Capture Boost Switch", "AUX"},
	{"Aux Boost", "Aux Volume", "Boost Mixer"},
	{"Capture Boost", "Capture Switch", "Boost Mixer"},
	{"Mic Boost", "Mic Volume", "Boost Mixer"},

	/* Inputs */
	{"MICP", NULL, "Mic Boost"},
	{"MICN", NULL, "Mic PGA"},
	{"Mic PGA", NULL, "Capture Boost"},
	{"AUX", NULL, "Aux Input"},
};

static int wau8822_add_widgets(struct snd_soc_codec *codec)
{
#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	snd_soc_dapm_new_controls(codec, wau8822_dapm_widgets,
				  ARRAY_SIZE(wau8822_dapm_widgets));

//	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

struct pll_ {
	unsigned int in_hz, out_hz;
	unsigned int pre:4; /* prescale - 1 */
	unsigned int n:4;
	unsigned int k;
};

static struct pll_ pll[] = {
	{12000000, 11289600, 0, 7, 0x86c220},
	{12000000, 12288000, 0, 8, 0x3126e8},
	{13000000, 11289600, 0, 6, 0xf28bd4},
	{13000000, 12288000, 0, 7, 0x8fd525},
	{12288000, 11289600, 0, 7, 0x59999a},
	{11289600, 12288000, 0, 8, 0x80dee9},
	/* TODO: liam - add more entries */
};

static int wau8822_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int i;
	u16 reg;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	if(freq_in == 0 || freq_out == 0) {
		reg = wau8822_read_reg_cache(codec, WAU8822_POWER1);
		wau8822_write(codec, WAU8822_POWER1, reg & 0x1df);
		return 0;
	}

	for(i = 0; i < ARRAY_SIZE(pll); i++) {
		if (freq_in == pll[i].in_hz && freq_out == pll[i].out_hz) {
			wau8822_write(codec, WAU8822_PLLN, (pll[i].pre << 4) | pll[i].n);
			wau8822_write(codec, WAU8822_PLLK1, pll[i].k >> 18);
			wau8822_write(codec, WAU8822_PLLK1, (pll[i].k >> 9) && 0x1ff);
			wau8822_write(codec, WAU8822_PLLK1, pll[i].k && 0x1ff);
			reg = wau8822_read_reg_cache(codec, WAU8822_POWER1);
			wau8822_write(codec, WAU8822_POWER1, reg | 0x020);
			return 0;
		}
	}
	return -EINVAL;
}

static int wau8822_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = wau8822_read_reg_cache(codec, WAU8822_IFACE) & 0x7;
	u16 clk = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0xfffe;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		clk |= 0x0001;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0010;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0008;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x00018;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0180;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0100;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0080;
		break;
	default:
		return -EINVAL;
	}

	wau8822_write(codec, WAU8822_IFACE, iface);
	wau8822_write(codec, WAU8822_CLOCK, clk);
	return 0;
}

static int wau8822_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u16 iface = wau8822_read_reg_cache(codec, WAU8822_IFACE) & 0xff9f;
	u16 adn = wau8822_read_reg_cache(codec, WAU8822_ADD) & 0x1f1;

#ifdef __DEBUG_PATH
	printk("%s=>%d\n", __FUNCTION__, __LINE__);
	printk("%s=>%d: rate=>%d\n", __FUNCTION__, __LINE__, params_rate(params));
	printk("%s=>%d: format=>%d\n", __FUNCTION__, __LINE__, params_format(params));
#endif

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0020;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0040;
		break;
	}

	/* filter coefficient */
	switch (params_rate(params)) {
	case 8000: //SNDRV_PCM_RATE_8000:
		adn |= 0x5 << 1;
		break;
	case 11025: //SNDRV_PCM_RATE_11025:
		adn |= 0x4 << 1;
		break;
	case 16000: //SNDRV_PCM_RATE_16000:
		adn |= 0x3 << 1;
		break;
	case 22050: //SNDRV_PCM_RATE_22050:
		adn |= 0x2 << 1;
		break;
	case 32000: //SNDRV_PCM_RATE_32000:
		adn |= 0x1 << 1;
		break;
	case 48000: //SNDRV_PCM_RATE_48000:
		break;
	default:
//		printk("%s=>%d: not support rate...\n", __FUNCTION__, __LINE__);		
		break;
	}

	/* set iface */
	wau8822_write(codec, WAU8822_IFACE, iface);
	wau8822_write(codec, WAU8822_ADD, adn);
	return 0;
}

static int wau8822_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	switch (div_id) {
	case WAU8822_MCLKDIV:
		reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x11f;
		wau8822_write(codec, WAU8822_CLOCK, reg | div);
		break;
	case WAU8822_BCLKDIV:
		reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x1c7;
		wau8822_write(codec, WAU8822_CLOCK, reg | div);
		break;
	case WAU8822_OPCLKDIV:
		reg = wau8822_read_reg_cache(codec, WAU8822_GPIO) & 0x1cf;
		wau8822_write(codec, WAU8822_GPIO, reg | div);
		break;
	case WAU8822_DACOSR:
		reg = wau8822_read_reg_cache(codec, WAU8822_DAC) & 0x1f7;
		wau8822_write(codec, WAU8822_DAC, reg | div);
		break;
	case WAU8822_ADCOSR:
		reg = wau8822_read_reg_cache(codec, WAU8822_ADC) & 0x1f7;
		wau8822_write(codec, WAU8822_ADC, reg | div);
		break;
	case WAU8822_MCLKSEL:
		reg = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0x0ff;
		wau8822_write(codec, WAU8822_CLOCK, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wau8822_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wau8822_read_reg_cache(codec, WAU8822_DAC) & 0xffbf;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	if(mute) {
		wau8822_write(codec, WAU8822_DAC, mute_reg | 0x40);
	}
	else {
		wau8822_write(codec, WAU8822_DAC, mute_reg);
#ifdef __DEBUG_PATH
		Wau8822DumpRegister(codec);
		Wau8822DumpRegister(codec);
#endif
	}

	return 0;
}

/* TODO: liam need to make this lower power with dapm */
static int wau8822_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
//	u16 reg = 0;
	
#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		wau8822_write(codec, WAU8822_POWER1, 0x1ff);
		wau8822_write(codec, WAU8822_POWER2, 0x1ff);
		wau8822_write(codec, WAU8822_POWER3, 0x1ff);
//		wau8822_write(codec, WAU8822_POWER3, 0x10);
//		reg = wau8822_read_reg_cache(codec, WAU8822_OUTPUT);
//		reg |= 0x04;
//		wau8822_write(codec, WAU8822_OUTPUT, reg);
//		wau8822_write(codec, WAU8822_POWER1, 0x04);
//		wau8822_write(codec, WAU8822_POWER1, 0x0d);
//		msleep(300);
//		wau8822_write(codec, WAU8822_POWER3, 0x7f);
		break;
	case SND_SOC_BIAS_OFF:
		wau8822_write(codec, WAU8822_POWER1, 0x0);
		wau8822_write(codec, WAU8822_POWER2, 0x0);
		wau8822_write(codec, WAU8822_POWER3, 0x0);
		break;
	}
	codec->bias_level = level;
	return 0;
}

#define WAU8822_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define WAU8822_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops wau8822_dai_ops = {
	.hw_params	= wau8822_hw_params,
	.digital_mute	= wau8822_mute,
	.set_fmt	= wau8822_set_dai_fmt,
	.set_clkdiv	= wau8822_set_dai_clkdiv,
	.set_pll	= wau8822_set_dai_pll,
};

struct snd_soc_dai wau8822_dai = {
	.name = "WAU8822 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WAU8822_RATES,
		.formats = WAU8822_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WAU8822_RATES,
		.formats = WAU8822_FORMATS,},
	.ops = &wau8822_dai_ops,
};
EXPORT_SYMBOL_GPL(wau8822_dai);

//static int wau8822_suspend(struct platform_device *pdev, pm_message_t state)
//{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
//	struct snd_soc_codec *codec = socdev->card->codec;
//
//	wau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);
//	return 0;
//}
//
//static int wau8822_resume(struct platform_device *pdev)
//{
//	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
//	struct snd_soc_codec *codec = socdev->card->codec;
//	int i;
//	u8 data[2];
//	u16 *cache = codec->reg_cache;
//
//	/* Sync reg_cache with the hardware */
//	for (i = 0; i < ARRAY_SIZE(wau8822_reg); i++) {
//		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
//		data[1] = cache[i] & 0x00ff;
//		codec->hw_write(codec->control_data, data, 2);
//	}
//	wau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
//	wau8822_set_bias_level(codec, codec->suspend_bias_level);
//	return 0;
//}

/*
 * initialise the WAU8822 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wau8822_register(struct wau8822_priv *wau8822)
{
	struct snd_soc_codec *codec = &wau8822->codec;
//	u16 clk = 0;
//	u16 reg = 0;
	int ret = 0;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	if (wau8822_codec) {
		dev_err(codec->dev, "Another WAU8822 is registered\n");
		ret = -EINVAL;
		goto err;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	//codec->private_data = wau8822;
	snd_soc_codec_set_drvdata(codec, wau8822);
	codec->name = "WAU8822";
	codec->owner = THIS_MODULE;
	codec->read = wau8822_read_reg_cache;
	codec->write = wau8822_write;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = wau8822_set_bias_level;
	codec->dai = &wau8822_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(wau8822->reg_cache);
	codec->reg_cache = &wau8822->reg_cache;

	memcpy(codec->reg_cache, wau8822_reg, sizeof(wau8822_reg));

	/* Get the codec into a known state */
	ret = wau8822_reset(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to reset codec: %d\n", ret);
		goto err;
	}
	
	wau8822_dai.dev = codec->dev;
		
	/* power on device */
//	wau8822_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	{
		int iLoop;
		
		for(iLoop=1; iLoop<WAU8822_CACHEREGNUM; iLoop++) {
			wau8822_write(codec, iLoop, wau8822_reg[iLoop]);
		}
	}

//	clk = wau8822_read_reg_cache(codec, WAU8822_CLOCK) & 0xff;
//	wau8822_write(codec, WAU8822_CLOCK, clk);
	
#if 0
	wau8822_write(codec, WAU8822_MIXR, 0);
	reg = wau8822_read_reg_cache(codec, WAU8822_OUTPUT);
	reg |= 0x20;
	wau8822_write(codec, WAU8822_OUTPUT, reg);	
	reg = wau8822_read_reg_cache(codec, WAU8822_BEEP);
	reg |= 0x10;
	wau8822_write(codec, WAU8822_BEEP, reg);	
#endif	
	
	wau8822_codec = codec;
	
	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}

	ret = snd_soc_register_dais(&wau8822_dai, 1);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		goto err_codec;
	}

	return 0;

err_codec:
	snd_soc_unregister_codec(codec);
err:
	kfree(wau8822);
	return ret;
}

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

static void wau8822_unregister(struct wau8822_priv *wau8822)
{
#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	wau8822_set_bias_level(&wau8822->codec, SND_SOC_BIAS_OFF);
	snd_soc_unregister_dais(&wau8822_dai, 1);
	snd_soc_unregister_codec(&wau8822->codec);
	kfree(wau8822);
	wau8822_codec = NULL;
}

/*
 * WAU8822 2 wire address is 0x1a
 */

static int wau8822_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct wau8822_priv *wau8822;
	struct snd_soc_codec *codec;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	wau8822 = kzalloc(sizeof(struct wau8822_priv), GFP_KERNEL);
	if (wau8822 == NULL)
		return -ENOMEM;

	codec = &wau8822->codec;
	codec->hw_write = (hw_write_t)i2c_master_send;

	i2c_set_clientdata(i2c, wau8822);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;

	return wau8822_register(wau8822);
}

static int wau8822_i2c_remove(struct i2c_client *client)
{
	struct wau8822_priv *wau8822 = i2c_get_clientdata(client);
#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	wau8822_unregister(wau8822);
	return 0;
}

static const struct i2c_device_id wau8822_i2c_id[] = {
	{ "wau8822", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wau8822_i2c_id);

static struct i2c_driver wau8822_i2c_driver = {
	.driver = {
		.name = "WAU8822 I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe =    wau8822_i2c_probe,
	.remove =   wau8822_i2c_remove,
	.id_table = wau8822_i2c_id,
};
#endif

static int wau8822_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	if (wau8822_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = wau8822_codec;
	codec = wau8822_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, wau8822_snd_controls,
			     ARRAY_SIZE(wau8822_snd_controls));
	wau8822_add_widgets(codec);

	return 0;

pcm_err:
	return ret;
}

/* power down chip */
static int wau8822_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

	if (codec->control_data)
		wau8822_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_unregister_device(codec->control_data);
	i2c_del_driver(&wau8822_i2c_driver);
#endif
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wau8822 = {
	.probe = 	wau8822_probe,
	.remove = 	wau8822_remove,
//	.suspend = 	wau8822_suspend,
//	.resume =	wau8822_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_wau8822);

static int __init wau8822_modinit(void)
{
	int ret;

#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&wau8822_i2c_driver);
	if (ret != 0) {
		pr_err("Failed to register WAU8822 I2C driver: %d\n", ret);
	}
#endif

	return 0;
}
module_init(wau8822_modinit);

static void __exit wau8822_exit(void)
{
#ifdef __DEBUG_PATH
printk("%s=>%d\n", __FUNCTION__, __LINE__);
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&wau8822_i2c_driver);
#endif
}
module_exit(wau8822_exit);

MODULE_DESCRIPTION("ASoC WAU8822 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");

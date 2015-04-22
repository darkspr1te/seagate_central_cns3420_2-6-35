#ifndef _CNS3XXX_FMT_H
#define _CNS3XXX_FMT_H

extern struct snd_soc_dai cns3xxx_pcm1dac_fmt_dai;
extern void *pcm1_base_va;


#define FMT_MEM_MAP_VALUE(reg_offset)		(*((uint32_t volatile *)(pcm1_base_va + reg_offset)))

/* Reception */
#define PCM1_CFG0           FMT_MEM_MAP_VALUE(0x00)
#define PCM1_CFG1           FMT_MEM_MAP_VALUE(0x04)
#define PCM1_CH_CFG0        FMT_MEM_MAP_VALUE(0x08)
#define PCM1_CH_CFG1        FMT_MEM_MAP_VALUE(0x0C)
#define PCM1_CH_CFG2        FMT_MEM_MAP_VALUE(0x10)
#define PCM1_CH_CFG3        FMT_MEM_MAP_VALUE(0x14)
#define PCM1_TX_DATA_L      FMT_MEM_MAP_VALUE(0x18)
#define PCM1_TX_DATA_H      FMT_MEM_MAP_VALUE(0x1C)
#define PCM1_RX_DATA_L      FMT_MEM_MAP_VALUE(0x20)
#define PCM1_RX_DATA_H      FMT_MEM_MAP_VALUE(0x24)

#endif

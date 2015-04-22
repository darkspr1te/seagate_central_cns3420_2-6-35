#ifndef _CNS3XXX_SNDPCM_H
#define _CNS3XXX_SNDPCM_H

struct cns3xxx_pcm1dac_pcm_dma_params {
    char *name;
    int dma_ch;
    int rx_ch;
    int size_sample;
    uint32_t dev_addr;
};
extern struct snd_soc_platform cns3xxx_pcm1dac_soc_platform;
#endif

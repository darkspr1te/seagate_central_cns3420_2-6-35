#ifndef _CNS3XXX_I2S_H
#define _CNS3XXX_I2S_H

extern struct snd_soc_dai cns3xxx_i2s_dai;
extern void *i2s_base_va;


#define I2S_MEM_MAP_VALUE(reg_offset)		(*((uint32_t volatile *)(i2s_base_va + reg_offset)))

/* Reception */
#define CWDA_DR             I2S_MEM_MAP_VALUE(0)
#define CWDA_RCI            I2S_MEM_MAP_VALUE(0x4)
#define CWDA_RFSTATUS       I2S_MEM_MAP_VALUE(0x8)
#define CWDA_RFSIZE         I2S_MEM_MAP_VALUE(0xC)
#define CWDA_RFUL           I2S_MEM_MAP_VALUE(0x10)

/* Transmission */
#define CWDA_DT             I2S_MEM_MAP_VALUE(0x20)
#define CWDA_TCI            I2S_MEM_MAP_VALUE(0x24)
#define CWDA_TFSTATUS       I2S_MEM_MAP_VALUE(0x28)
#define CWDA_TFSIZE         I2S_MEM_MAP_VALUE(0x2C)
#define CWDA_TFLL           I2S_MEM_MAP_VALUE(0x30)

/* Common Registers */
#define CWDA_DFC            I2S_MEM_MAP_VALUE(0X40)
#define CWDA_SSC            I2S_MEM_MAP_VALUE(0X44)
#define CWDA_TC             I2S_MEM_MAP_VALUE(0x48)
#define CWDA_INT            I2S_MEM_MAP_VALUE(0x4C)
#define CWDA_VER            I2S_MEM_MAP_VALUE(0x50)


/* Transceiver Control bits */
#define CWDA_TX_EN          (0x1 << 0)
#define CWDA_TX_RST         (0x1 << 1)
#define CWDA_TX_FLUSH       (0x1 << 2)
#define CWDA_RX_EN          (0x1 << 4)
#define CWDA_RX_RST         (0x1 << 5)
#define CWDA_RX_FLUSH       (0x1 << 6)
#define CWDA_CLK_EN         (0x1 << 8)
#define CWDA_CLK_RST        (0x1 << 9)

/* Interrupt/Interrupt Mask Register bits */
#define CWDA_INT_RXFFM      (0x1 << 0)
#define CWDA_INT_TXFFM      (0x1 << 1)
#define CWDA_INT_RXFM       (0x1 << 2)
#define CWDA_INT_TXEM       (0x1 << 3)
#define CWDA_INT_RXDMA      (0x1 << 4)
#define CWDA_INT_TXDMA      (0x1 << 5)

#endif

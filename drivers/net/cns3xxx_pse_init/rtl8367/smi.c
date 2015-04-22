/*
* Copyright c                  Realtek Semiconductor Corporation, 2006 
* All rights reserved.
* 
* Program : Control  smi connected RTL8366
* Abstract : 
* Author : Yu-Mei Pan (ympan@realtek.com.cn)                
*  $Id: smi.c,v 1.2 2008-04-10 03:04:19 shiehyy Exp $
*/


#include <rtk_types.h>
#if 0
#include <gpio.h>
#endif
#include <smi.h>
#include "rtk_error.h"
#include "../cns3xxx_symbol.h"
#include <linux/delay.h>
  
#define MDC_MDIO_DUMMY_ID           0
#define MDC_MDIO_CTRL0_REG          31
#define MDC_MDIO_CTRL1_REG          21
#define MDC_MDIO_ADDRESS_REG        23
#define MDC_MDIO_DATA_WRITE_REG     24
#define MDC_MDIO_DATA_READ_REG      25
#define MDC_MDIO_PREAMBLE_LEN       32

#define MDC_MDIO_ADDR_OP           0x000E
#define MDC_MDIO_READ_OP           0x0001
#define MDC_MDIO_WRITE_OP          0x0003


#ifdef CONFIG_RTL8367

#if 0
#define DELAY                        10000
#define CLK_DURATION(clk)            { int i; for(i=0; i<clk; i++); }
#else
#define DELAY                        100
#define CLK_DURATION(clk)            udelay(clk)
#define GPIO_DIR_OUT    (1)
#define GPIO_DIR_IN     (0)
#endif
#define _SMI_ACK_RESPONSE(ok)        { /*if (!(ok)) return RT_ERR_FAILED; */}

#if 0
gpioID smi_SCK;        /* GPIO used for SMI Clock Generation */
gpioID smi_SDA;        /* GPIO used for SMI Data signal */
gpioID smi_RST;     /* GPIO used for reset swtich */
#else
#define smi_SCK (20)    /* GPIOB20 */
#define smi_SDA (21)    /* GPIOB21 */
#define smi_RST (3)     /* GPIOA 3 */
#endif


#define ack_timer                    5
#define max_register                0x018A 

static inline void gpio_b_dir(uint32 id, uint32 dir)
{
    volatile unsigned int reg;
    if (31 < id) return;

    reg = (0x1 << id);

    if (dir) { /* output */
        GPIOB_REG_VALUE(0x8) |= reg;
//		SWITCH_REG_VALUE
    } else { /* input */
        GPIOB_REG_VALUE(0x8) &= ~reg;
    }
}

static inline void gpio_b_set(uint32 id, uint32 set)
{
    volatile unsigned int reg;
    if (31 < id) return;

    reg = (0x1 << id);

    if (set) {
        /* data set */
        GPIOB_REG_VALUE(0x0) |= reg;
    } else {
        /* data clear */
        GPIOB_REG_VALUE(0x0) &= ~reg;
    }
}

static inline void gpio_b_get(uint32 id, uint16 *data)
{
    volatile unsigned int result;

    if (31 < id) return;

    result = (GPIOB_REG_VALUE(0x4) >> id) & 0x1;
    *data = result;
}


#define _rtl865x_initGpioPin(A,B,C,D)   gpio_b_dir(A, C)
#define _rtl865x_setGpioDataBit gpio_b_set
#define _rtl865x_getGpioDataBit gpio_b_get
#define rtlglue_drvMutexLock() {}
#define rtlglue_drvMutexUnlock() {}



void _smi_start(void)
{

    /* change GPIO pin to Output only */
#if 0
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    _rtl865x_initGpioPin(smi_SCK, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
#else
	/*XXX: for timming issue*/
	_rtl865x_initGpioPin(smi_SCK, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
	_rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
#endif
    
    /* Initial state: SCK: 0, SDA: 1 */
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    _rtl865x_setGpioDataBit(smi_SDA, 1);
    CLK_DURATION(DELAY);

    /* CLK 1: 0 -> 1, 1 -> 0 */
    _rtl865x_setGpioDataBit(smi_SCK, 1);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);

    /* CLK 2: */
    _rtl865x_setGpioDataBit(smi_SCK, 1);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 1);

}



void _smi_writeBit(uint16 signal, uint32 bitLen)
{
    for( ; bitLen > 0; bitLen--)
    {
        CLK_DURATION(DELAY);

        /* prepare data */
        if ( signal & (1<<(bitLen-1)) ) 
            _rtl865x_setGpioDataBit(smi_SDA, 1);    
        else 
            _rtl865x_setGpioDataBit(smi_SDA, 0);    
        CLK_DURATION(DELAY);

        /* clocking */
        _rtl865x_setGpioDataBit(smi_SCK, 1);
        CLK_DURATION(DELAY);
        _rtl865x_setGpioDataBit(smi_SCK, 0);
    }
}



void _smi_readBit(uint32 bitLen, uint32 *rData) 
{
    uint32 u = 0;

    /* change GPIO pin to Input only */
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);

    for (*rData = 0; bitLen > 0; bitLen--)
    {
        CLK_DURATION(DELAY);

        /* clocking */
        _rtl865x_setGpioDataBit(smi_SCK, 1);
        CLK_DURATION(DELAY);
        _rtl865x_getGpioDataBit(smi_SDA, &u);
        _rtl865x_setGpioDataBit(smi_SCK, 0);

        *rData |= (u << (bitLen - 1));
    }

    /* change GPIO pin to Output only */
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
}



void _smi_stop(void)
{

    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 0);    
    _rtl865x_setGpioDataBit(smi_SCK, 1);    
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 1);    
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 1);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 1);

    /* add a click */
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 1);


    /* change GPIO pin to Output only */
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);
    _rtl865x_initGpioPin(smi_SCK, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);


}

#if 0
int32 smi_reset(uint32 port, uint32 pinRST)
{
    gpioID gpioId;
    int32 res;

    /* Initialize GPIO port A, pin 7 as SMI RESET */
    gpioId = GPIO_ID(port, pinRST);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (res != RT_ERR_OK)
        return res;
    smi_RST = gpioId;

    _rtl865x_setGpioDataBit(smi_RST, 1);
    CLK_DURATION(1000000);
    _rtl865x_setGpioDataBit(smi_RST, 0);    
    CLK_DURATION(1000000);
    _rtl865x_setGpioDataBit(smi_RST, 1);
    CLK_DURATION(1000000);

    /* change GPIO pin to Input only */
    _rtl865x_initGpioPin(smi_RST, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);

    return RT_ERR_OK;
}


int32 smi_init(uint32 port, uint32 pinSCK, uint32 pinSDA)
{
    gpioID gpioId;
    int32 res;

    /* change GPIO pin to Input only */
    /* Initialize GPIO port C, pin 0 as SMI SDA0 */
    gpioId = GPIO_ID(port, pinSDA);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (res != RT_ERR_OK)
        return res;
    smi_SDA = gpioId;


    /* Initialize GPIO port C, pin 1 as SMI SCK0 */
    gpioId = GPIO_ID(port, pinSCK);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (res != RT_ERR_OK)
        return res;
    smi_SCK = gpioId;


    _rtl865x_setGpioDataBit(smi_SDA, 1);    
    _rtl865x_setGpioDataBit(smi_SCK, 1);    
    return RT_ERR_OK;
}
#endif



int32 smi_read(uint32 mAddrs, uint32 *rData)
{
#ifdef MDC_MDIO_OPERATION

    /* Write address control code to register 31 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write read control code to register 21 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_READ_OP);

    /* Read data from register 25 */
    MDC_MDIO_READ(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_DATA_READ_REG, rData);

    return SUCCESS;
#else
    uint32 rawData=0, ACK;
    uint8  con;
    uint32 ret = RT_ERR_OK;
/*
    if ((mAddrs > max_register) || (rData == NULL))  return    RT_ERR_FAILED;
*/

    /*Disable CPU interrupt to ensure that the SMI operation is atomic. 
      The API is based on RTL865X, rewrite the API if porting to other platform.*/
       rtlglue_drvMutexLock();

    _smi_start();                                /* Start SMI */

    _smi_writeBit(0x0b, 4);                     /* CTRL code: 4'b1011 for RTL8370 */

    _smi_writeBit(0x4, 3);                        /* CTRL code: 3'b100 */

    _smi_writeBit(0x1, 1);                        /* 1: issue READ command */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for issuing READ command*/
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs&0xff), 8);             /* Set reg_addr[7:0] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[7:0] */    
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs>>8), 8);                 /* Set reg_addr[15:8] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK by RTL8369 */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_readBit(8, &rawData);                    /* Read DATA [7:0] */
    *rData = rawData&0xff; 

    _smi_writeBit(0x00, 1);                        /* ACK by CPU */

    _smi_readBit(8, &rawData);                    /* Read DATA [15: 8] */

    _smi_writeBit(0x01, 1);                        /* ACK by CPU */
    *rData |= (rawData<<8);

    _smi_stop();

    rtlglue_drvMutexUnlock();/*enable CPU interrupt*/

    return ret;
#endif /* end of #ifdef MDC_MDIO_OPEARTION */
}



int32 smi_write(uint32 mAddrs, uint32 rData)
{
#ifdef MDC_MDIO_OPERATION

    /* Write address control code to register 31 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write data to register 24 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_DATA_WRITE_REG, rData);

    /* Write data control code to register 21 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_WRITE_OP);

    return SUCCESS;
#else

/*
    if ((mAddrs > 0x018A) || (rData > 0xFFFF))  return    RT_ERR_FAILED;
*/
    int8 con;
    uint32 ACK;
    uint32 ret = RT_ERR_OK;    

    /*Disable CPU interrupt to ensure that the SMI operation is atomic. 
      The API is based on RTL865X, rewrite the API if porting to other platform.*/
       rtlglue_drvMutexLock();

    _smi_start();                                /* Start SMI */

    _smi_writeBit(0x0b, 4);                     /* CTRL code: 4'b1011 for RTL8370*/

    _smi_writeBit(0x4, 3);                        /* CTRL code: 3'b100 */

    _smi_writeBit(0x0, 1);                        /* 0: issue WRITE command */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for issuing WRITE command*/
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs&0xff), 8);             /* Set reg_addr[7:0] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs>>8), 8);                 /* Set reg_addr[15:8] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit(rData&0xff, 8);                /* Write Data [7:0] out */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for writting data [7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit(rData>>8, 8);                    /* Write Data [15:8] out */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                        /* ACK for writting data [15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_stop();    

    rtlglue_drvMutexUnlock();/*enable CPU interrupt*/
    
    return ret;
#endif /* end of #ifdef MDC_MDIO_OPEARTION */
}


#endif // #ifdef CONFIG_RTL8367


#ifdef CONFIG_RTL8367_ONE_LEG


#if 0
#define DELAY                        10000
#define CLK_DURATION(clk)            { int i; for(i=0; i<clk; i++); }
#else
#define DELAY                        100
#define CLK_DURATION(clk)            { int i; for(i=0; i<clk; i++); }
//#define CLK_DURATION(clk)            udelay(clk)
#define GPIO_DIR_OUT    (1)
#define GPIO_DIR_IN     (0)
#define u32	uint32
#define u16	uint16
#endif
#define _SMI_ACK_RESPONSE(ok)        { /*if (!(ok)) return RT_ERR_FAILED; */}


#if 0
gpioID smi_SCK;        /* GPIO used for SMI Clock Generation */
gpioID smi_SDA;        /* GPIO used for SMI Data signal */
gpioID smi_RST;     /* GPIO used for reset swtich */
#else
#define smi_SCK (20)    /* GPIOB20 */
#define smi_SDA (21)    /* GPIOB21 */
#define smi_RST (3)     /* GPIOA 3 */
#define gpioID 	int32					//cypress
#endif


#define ack_timer                    5
#define max_register                0x018A 

//cypress
static inline void gpio_a_dir(uint32 id, uint32 dir)
{
    volatile unsigned int reg;
    if (31 < id) return;

    reg = (0x1 << id);

    if (dir) { /* output */
        GPIOA_REG_VALUE(0x8) |= reg;
    } else { /* input */
        GPIOA_REG_VALUE(0x8) &= ~reg;
    }
}
//cypress

static inline void gpio_b_dir(uint32 id, uint32 dir)
{
    volatile unsigned int reg;
    if (31 < id) return;

    reg = (0x1 << id);

    if (dir) { /* output */
        GPIOB_REG_VALUE(0x8) |= reg;
    } else { /* input */
        GPIOB_REG_VALUE(0x8) &= ~reg;
    }
}

static inline void gpio_b_set(u32 id, u32 set)
{
    volatile unsigned int reg;
    if (31 < id) return;

    reg = (0x1 << id);

    if (set) {
        /* data set */
        GPIOB_REG_VALUE(0x0) |= reg;
    } else {
        /* data clear */
        GPIOB_REG_VALUE(0x0) &= ~reg;
    }
}

//cypress
static inline void gpio_a_set(u32 id, u32 set)
{
    volatile unsigned int reg;
    if (31 < id) return;

    reg = (0x1 << id);

    if (set) {
        /* data set */
        //GPIOA_REG_VALUE(0x0) |= reg;
        GPIOA_REG_VALUE(0x0) |= reg;
    } else {
        /* data clear */
        //GPIOA_REG_VALUE(0x10) |= reg;
        GPIOA_REG_VALUE(0x00) &= ~reg;
    }
}
//cypress

static inline void gpio_b_get(u32 id, u16 *data)
{
    volatile unsigned int result;

    if (31 < id) return;

    result = (GPIOB_REG_VALUE(0x4) >> id) & 0x1;
    *data = result;
}


#define _rtl865x_initGpioPin(A,B,C,D)   gpio_b_dir(A, C)

#define _rtl865x_initGpioPinA(A,B,C,D)   gpio_a_dir(A, C)			//cypress
#define _rtl865x_setGpioDataBitA gpio_a_set						//cypress

#define _rtl865x_setGpioDataBit gpio_b_set
#define _rtl865x_getGpioDataBit gpio_b_get
#define rtlglue_drvMutexLock() {}
#define rtlglue_drvMutexUnlock() {}

void _smi_start(void)
{

    /* change GPIO pin to Output only */
#if 0
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    _rtl865x_initGpioPin(smi_SCK, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
#else
	/*XXX: for timming issue*/
    _rtl865x_initGpioPin(smi_SCK, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
#endif
    
    /* Initial state: SCK: 0, SDA: 1 */
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    _rtl865x_setGpioDataBit(smi_SDA, 1);
    CLK_DURATION(DELAY);

    /* CLK 1: 0 -> 1, 1 -> 0 */
    _rtl865x_setGpioDataBit(smi_SCK, 1);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);

    /* CLK 2: */
    _rtl865x_setGpioDataBit(smi_SCK, 1);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 1);

}



void _smi_writeBit(uint16 signal, uint32 bitLen)
{
    for( ; bitLen > 0; bitLen--)
    {
        CLK_DURATION(DELAY);

        /* prepare data */
        if ( signal & (1<<(bitLen-1)) ) 
            _rtl865x_setGpioDataBit(smi_SDA, 1);    
        else 
            _rtl865x_setGpioDataBit(smi_SDA, 0);    
        CLK_DURATION(DELAY);

        /* clocking */
        _rtl865x_setGpioDataBit(smi_SCK, 1);
        CLK_DURATION(DELAY);
        _rtl865x_setGpioDataBit(smi_SCK, 0);
    }
}



void _smi_readBit(uint32 bitLen, uint32 *rData) 
{
    uint32 u = 0;

    /* change GPIO pin to Input only */
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);

    for (*rData = 0; bitLen > 0; bitLen--)
    {
        CLK_DURATION(DELAY);

        /* clocking */
        _rtl865x_setGpioDataBit(smi_SCK, 1);
        CLK_DURATION(DELAY);
        _rtl865x_getGpioDataBit(smi_SDA, &u);
        _rtl865x_setGpioDataBit(smi_SCK, 0);

        *rData |= (u << (bitLen - 1));
    }

    /* change GPIO pin to Output only */
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
}



void _smi_stop(void)
{

    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 0);    
    _rtl865x_setGpioDataBit(smi_SCK, 1);    
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SDA, 1);    
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 1);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 1);

    /* add a click */
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 0);
    CLK_DURATION(DELAY);
    _rtl865x_setGpioDataBit(smi_SCK, 1);


    /* change GPIO pin to Output only */
    _rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);
    _rtl865x_initGpioPin(smi_SCK, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);


}

#if 1
int32 smi_reset(uint32 port, uint32 pinRST)
//int32 smi_reset(void)
{
    gpioID gpioId;
    int32 res;

#if 0
    /* Initialize GPIO port A, pin 7 as SMI RESET */
    gpioId = GPIO_ID(port, pinRST);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (res != RT_ERR_OK)
        return res;
    smi_RST = gpioId;

#else

	_rtl865x_initGpioPinA(smi_RST, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);	//cypress

    _rtl865x_setGpioDataBitA(smi_RST, 1);
    CLK_DURATION(1000000);
    _rtl865x_setGpioDataBitA(smi_RST, 0);    
    CLK_DURATION(1000000);
    _rtl865x_setGpioDataBitA(smi_RST, 1);
    CLK_DURATION(1000000);

#endif

    /* change GPIO pin to Input only */
   // _rtl865x_initGpioPinA(smi_RST, GPIO_PERI_GPIO, GPIO_DIR_IN, GPIO_INT_DISABLE);

    return RT_ERR_OK;
}
#endif

#if 1
int32 smi_init(uint32 port, uint32 pinSCK, uint32 pinSDA)
//int32 smi_init(void)													//cypress
{
    gpioID gpioId;
    int32 res;

#if 0
    /* change GPIO pin to Input only */
    /* Initialize GPIO port C, pin 0 as SMI SDA0 */
    gpioId = GPIO_ID(port, pinSDA);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (res != RT_ERR_OK)
        return res;
    smi_SDA = gpioId;


    /* Initialize GPIO port C, pin 1 as SMI SCK0 */
    gpioId = GPIO_ID(port, pinSCK);
    res = _rtl865x_initGpioPin(gpioId, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
    if (res != RT_ERR_OK)
        return res;
    smi_SCK = gpioId;
#else

	_rtl865x_initGpioPin(smi_SDA, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);
	_rtl865x_initGpioPin(smi_SCK, GPIO_PERI_GPIO, GPIO_DIR_OUT, GPIO_INT_DISABLE);

    _rtl865x_setGpioDataBit(smi_SDA, 1);    
    _rtl865x_setGpioDataBit(smi_SCK, 1);    
#endif
    return RT_ERR_OK;
}

#endif



int32 smi_read(uint32 mAddrs, uint32 *rData)
{
#ifdef MDC_MDIO_OPERATION

    /* Write address control code to register 31 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write read control code to register 21 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_READ_OP);

    /* Read data from register 25 */
    MDC_MDIO_READ(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_DATA_READ_REG, rData);

    return SUCCESS;
#else
    uint32 rawData=0, ACK;
    uint8  con;
    uint32 ret = RT_ERR_OK;
/*
    if ((mAddrs > max_register) || (rData == NULL))  return    RT_ERR_FAILED;
*/

    /*Disable CPU interrupt to ensure that the SMI operation is atomic. 
      The API is based on RTL865X, rewrite the API if porting to other platform.*/
       rtlglue_drvMutexLock();

    _smi_start();                                /* Start SMI */

    _smi_writeBit(0x0b, 4);                     /* CTRL code: 4'b1011 for RTL8370 */

    _smi_writeBit(0x4, 3);                        /* CTRL code: 3'b100 */

    _smi_writeBit(0x1, 1);                        /* 1: issue READ command */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for issuing READ command*/
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs&0xff), 8);             /* Set reg_addr[7:0] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[7:0] */    
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs>>8), 8);                 /* Set reg_addr[15:8] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK by RTL8369 */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_readBit(8, &rawData);                    /* Read DATA [7:0] */
    *rData = rawData&0xff; 

    _smi_writeBit(0x00, 1);                        /* ACK by CPU */

    _smi_readBit(8, &rawData);                    /* Read DATA [15: 8] */

    _smi_writeBit(0x01, 1);                        /* ACK by CPU */
    *rData |= (rawData<<8);

    _smi_stop();

    rtlglue_drvMutexUnlock();/*enable CPU interrupt*/

    return ret;
#endif /* end of #ifdef MDC_MDIO_OPEARTION */
}



int32 smi_write(uint32 mAddrs, uint32 rData)
{
#ifdef MDC_MDIO_OPERATION

    /* Write address control code to register 31 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write data to register 24 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_DATA_WRITE_REG, rData);

    /* Write data control code to register 21 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_DUMMY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_WRITE_OP);

    return SUCCESS;
#else

/*
    if ((mAddrs > 0x018A) || (rData > 0xFFFF))  return    RT_ERR_FAILED;
*/
    int8 con;
    uint32 ACK;
    uint32 ret = RT_ERR_OK;    

    /*Disable CPU interrupt to ensure that the SMI operation is atomic. 
      The API is based on RTL865X, rewrite the API if porting to other platform.*/
       rtlglue_drvMutexLock();

    _smi_start();                                /* Start SMI */

    _smi_writeBit(0x0b, 4);                     /* CTRL code: 4'b1011 for RTL8370*/

    _smi_writeBit(0x4, 3);                        /* CTRL code: 3'b100 */

    _smi_writeBit(0x0, 1);                        /* 0: issue WRITE command */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for issuing WRITE command*/
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs&0xff), 8);             /* Set reg_addr[7:0] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs>>8), 8);                 /* Set reg_addr[15:8] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit(rData&0xff, 8);                /* Write Data [7:0] out */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for writting data [7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit(rData>>8, 8);                    /* Write Data [15:8] out */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                        /* ACK for writting data [15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_stop();    

    rtlglue_drvMutexUnlock();/*enable CPU interrupt*/
    
    return ret;
#endif /* end of #ifdef MDC_MDIO_OPEARTION */
}

#endif // #ifdef CONFIG_RTL8367_ONE_LEG


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <mach/pm.h>
#include <mach/misc.h>

#include <mach/cns3xxx.h>

#define I2C_PCLK                15000000
//#define SYSVA_POWER_MANAGEMENT_BASE_ADDR     (0xFFF0C000)
//#define PWRMGT_MEM_MAP_VALUE(x) (*((u32 volatile *)(SYSVA_POWER_MANAGEMENT_BASE_ADDR + x )))
//#define PMU_POWER_STATUS_REG    PWRMGT_MEM_MAP_VALUE(0x10)
//#define PMU_POWER_CLK_GATE      PWRMGT_MEM_MAP_VALUE(0x0)
//#define PMU_POWER_CACTIVE       PWRMGT_MEM_MAP_VALUE(0x0c)

#define I2C_MEM_MAP_ADDR(x)         (CNS3XXX_SSP_BASE_VIRT + x)
#define I2C_MEM_MAP_VALUE(x)        (*((unsigned int volatile*)I2C_MEM_MAP_ADDR(x)))

#define I2C_CONTROLLER_REG_ADDR               I2C_MEM_MAP_ADDR(0x20)
#define I2C_TIME_OUT_REG_ADDR                 I2C_MEM_MAP_ADDR(0x24)
#define I2C_SLAVE_ADDRESS_REG_ADDR            I2C_MEM_MAP_ADDR(0x28)
#define I2C_WRITE_DATA_REG_ADDR               I2C_MEM_MAP_ADDR(0x2C)
#define I2C_READ_DATA_REG_ADDR                I2C_MEM_MAP_ADDR(0x30)
#define I2C_INTERRUPT_STATUS_REG_ADDR         I2C_MEM_MAP_ADDR(0x34)
#define I2C_INTERRUPT_ENABLE_REG_ADDR         I2C_MEM_MAP_ADDR(0x38)
#define I2C_TWI_OUT_DLY_REG_ADDR         			I2C_MEM_MAP_ADDR(0x3C)

#define I2C_CONTROLLER_REG                    I2C_MEM_MAP_VALUE(0x20)
#define I2C_TIME_OUT_REG                      I2C_MEM_MAP_VALUE(0x24)
#define I2C_SLAVE_ADDRESS_REG                 I2C_MEM_MAP_VALUE(0x28)
#define I2C_WRITE_DATA_REG                    I2C_MEM_MAP_VALUE(0x2C)
#define I2C_READ_DATA_REG                     I2C_MEM_MAP_VALUE(0x30)
#define I2C_INTERRUPT_STATUS_REG              I2C_MEM_MAP_VALUE(0x34)
#define I2C_INTERRUPT_ENABLE_REG              I2C_MEM_MAP_VALUE(0x38)
#define I2C_TWI_OUT_DLY_REG			         			I2C_MEM_MAP_VALUE(0x3C)

#define I2C_READ_ONLY_CMD      (0)
#define I2C_WRITE_ONLY_CMD     (1)
#define I2C_WRITE_READ_CMD     (2)
#define I2C_READ_WRITE_CMD     (3)

#define I2C_DATA_LEN_1_BYTE    (0)
#define I2C_DATA_LEN_2_BYTE    (1)
#define I2C_DATA_LEN_3_BYTE    (2)
#define I2C_DATA_LEN_4_BYTE    (3)

#define I2C_BUS_ERROR_FLAG     (0x1)
#define I2C_ACTION_DONE_FLAG   (0x2)

#define CNS3xxx_I2C_ENABLE()          (I2C_CONTROLLER_REG) |= ((unsigned int)0x1 << 31)
#define CNS3xxx_I2C_DISABLE()         (I2C_CONTROLLER_REG) &= ~((unsigned int)0x1 << 31)
#define CNS3xxx_I2C_ENABLE_INTR()     (I2C_INTERRUPT_ENABLE_REG) |= 0x03
#define CNS3xxx_I2C_DISABLE_INTR()    (I2C_INTERRUPT_ENABLE_REG) &= 0xfc

#define CNS3xxx_I2C_ENABLE_DATA_SWAP()    (I2C_CONTROLLER_REG) |= (0x1 << 24)
#define CNS3xxx_I2C_DISABLE_DATA_SWAP()   (I2C_CONTROLLER_REG) &= ~(0x1 << 24)
#define CNS3xxx_I2C_START_TRANSFER()      (I2C_CONTROLLER_REG) |= (0x1 << 6)
#define CNS3xxx_I2C_STOP_TRANSFER()       (I2C_CONTROLLER_REG) &= ~(0x1 << 6)

//#define CNS3xxx_I2C_POWER_ON()            ((PMU_POWER_STATUS_REG) | 0x4)
//#define CNS3xxx_I2C_POWER_ENABLE()        ((PMU_POWER_CLK_GATE) |= 0x4)
//#define CNS3xxx_I2C_CLOCK()               ((PMU_POWER_CACTIVE) | 0x4)
//#define CNS3xxx_I2C_CLOCK_ENABLE()        ((PMU_POWER_CLK_GATE) |= 0x4)

#define TWI_TIMEOUT         (10*HZ)
#define I2C_100KHZ          100000
#define I2C_200KHZ          200000
#define I2C_300KHZ          300000
#define I2C_400KHZ          400000

#define CNS3xxx_I2C_CLK     I2C_100KHZ

#define STATE_DONE		0
#define STATE_START		1
#define STATE_WRITE		2
#define STATE_READ		3
#define STATE_ERROR		4

struct cns3xxx_i2c {
	void __iomem *base;
	wait_queue_head_t wait;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	int pos;
	int nmsgs;
	int state;		/* see STATE_ */
	int clock_khz;
	uint32_t valid_byte_mask;
#ifdef CNS3xxx_I2C_DEBUG
	uint32_t intr_status;
#endif
};

static u32 cns3xxx_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static int cns3xxx_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			    int num)
{
	struct cns3xxx_i2c *i2c = i2c_get_adapdata(adap);
	uint32_t data = 0, read_data_len = 0, write_data_len = 0, i2c_cmd_type;
#if 0
	printk(KERN_INFO "addr: 0x%04x, len: %d, flags: 0x%x, data: %x %x %x\n",
	       msgs->addr, msgs->len, msgs->flags, msgs->buf[0], msgs->buf[1],
	       msgs->buf[2]);
#endif

	i2c->msg = msgs;
	i2c->pos = 0;
	i2c->nmsgs = num;
	if (i2c->msg->flags & I2C_M_TEN) {
		printk
		    ("%s:%d: Presently the driver does not handle extended addressing\n",
		     __FUNCTION__, __LINE__);
		return -EIO;
	}
	//printk("\nxxx num: %d ## i2c->msg->flags: %x ## msgs->buf[0]: %x\n", num, i2c->msg->flags, msgs->buf[0]);

	if (num > 1) {
		/* In general, the num=2 send W_R sequence message */
		i2c_cmd_type = I2C_WRITE_READ_CMD;
		i2c->state = STATE_READ;
		data = msgs->buf[0];	// index
		if (i2c->msg[1].len > 1) {
			i2c->valid_byte_mask = 0xffff;	// TWO byte at a time
			read_data_len = I2C_DATA_LEN_2_BYTE;
		} else {
			i2c->valid_byte_mask = 0xff;	// ONE byte at a time
			read_data_len = I2C_DATA_LEN_1_BYTE;
		}
	} else {
		if (i2c->msg->flags & I2C_M_RD) {
			i2c_cmd_type = I2C_READ_ONLY_CMD;
			i2c->state = STATE_READ;
			i2c->valid_byte_mask = 0xff;	// ONE byte at a time
			//data = 0;
			data = msgs->buf[0];	// index
		} else {
			i2c_cmd_type = I2C_WRITE_ONLY_CMD;
			i2c->state = STATE_WRITE;

		}
		switch (i2c->msg->len) {
		case 1:
			i2c->valid_byte_mask = 0xff;	// ONE byte at a time
			write_data_len = I2C_DATA_LEN_1_BYTE;
			data = i2c->msg->buf[i2c->pos] & i2c->valid_byte_mask;
			I2C_CONTROLLER_REG = (0x1 << 31) | 0x10;

			break;
		case 2:
			i2c->valid_byte_mask = 0xffff;
			write_data_len = I2C_DATA_LEN_2_BYTE;
			data = (i2c->msg->buf[1] << 8) | i2c->msg->buf[0];
			break;
		case 3:
			i2c->valid_byte_mask = 0xffffff;
			write_data_len = I2C_DATA_LEN_3_BYTE;
			data =
			    (i2c->msg->buf[2] << 16) | (i2c->msg->
							buf[1] << 8) | i2c->
			    msg->buf[0];
			break;
		default:
			printk(KERN_INFO " %s Data length is over two bytes\n",
			       __FUNCTION__);

		}
	}

	//printk("\nxx data: %x\n", data);
	if (i2c->state == STATE_READ) {
		/* 1. Write TWI Serial slave address
		 * 2. Write CFG Reg with Read Data Len, TWI transfer command,
		 * 3. Write CFG Reg with RUN_START
		 */

		I2C_SLAVE_ADDRESS_REG = (i2c->msg->addr << 1);

		/* Well, setting the controller for TWI_EN = 1; TWI_SWAP_EN = 0;
		 * RUN_START = 0; TWI_WR_DATA_LEN = 0;
		 * TWI_RD_DATA_LEN = 1 or 2 bytes;
		 */
		I2C_WRITE_DATA_REG = data;
		I2C_CONTROLLER_REG =
		    ((0x1 << 31) | (i2c_cmd_type << 4) | (read_data_len << 0));
		CNS3xxx_I2C_START_TRANSFER();
	} else {
		//printk("\n567\n");
		/* 1. Write TWI Serial slave address
		 * 2. Write CFG Reg with Write Data Len, TWI transfer command,
		 * 3. Write TWI WRITE DATA Reg with data to be written
		 * 4. Write CFG Reg with RUN_START
		 */
		I2C_SLAVE_ADDRESS_REG = (i2c->msg->addr << 1);
		if (i2c->msg->len == 1) {
			//i2c->valid_byte_mask = 0xff; // ONE byte at a time
			I2C_WRITE_DATA_REG = data;
			I2C_CONTROLLER_REG =
			    ((0x1 << 31) | (i2c_cmd_type << 4) |
			     (write_data_len << 2));
		} else {
			//i2c->valid_byte_mask = 0xffff; // ONE byte at a time
			I2C_WRITE_DATA_REG = data;
			I2C_CONTROLLER_REG =
			    ((0x1 << 31) | (i2c_cmd_type << 4) |
			     (write_data_len << 2));
		}
		CNS3xxx_I2C_START_TRANSFER();
		if (((i2c->msg->addr >> 4) & 0xf) == 5)
			mdelay(5);
	}

	if (wait_event_timeout(i2c->wait, (i2c->state == STATE_ERROR) ||
			       (i2c->state == STATE_DONE), TWI_TIMEOUT)) {
		//printk(KERN_INFO "buf0_0=%x buf0_1=%x buf0_2=%x bu1_0=%x buf1_1=%x\n", msgs[0].buf[0],msgs[0].buf[1],msgs[0].buf[2],msgs[1].buf[0],msgs[1].buf[1]);
		return (i2c->state == STATE_DONE) ? num : -EIO;
	} else
		return -ETIMEDOUT;
}

static struct i2c_algorithm cns3xxx_i2c_algo = {
	.master_xfer = cns3xxx_i2c_xfer,
	.functionality = cns3xxx_i2c_func,
};

struct i2c_adapter cns3xxx_i2c_adapter = {
	.owner = THIS_MODULE,
	.algo = &cns3xxx_i2c_algo,
	.algo_data = NULL,
	.nr = 0,
	.name = "CNS3xxx I2C 0",
};
EXPORT_SYMBOL(cns3xxx_i2c_adapter);

static struct gpio i2c_gpios[] = {
	/* Pin no.   flags      label */
	{ GPIOB(12),  GPIOF_IN, "I2C SCL" },
	{ GPIOB(13),  GPIOF_IN, "I2C SDA" },
};

static void cns3xxx_i2c_adapter_init(struct cns3xxx_i2c *i2c)
{
	/* Steps
	 * 1. Check if the power is enabled to the module (PMU_BASE + 0x010)
	 * 2. Enable the clock (Enabled by default (PMU doc
	 *    but check clk status anyway PMU_BASE + 0X00C)
	 * 3. Configure the registers of i2c
	 */

	//   if (!CNS3xxx_I2C_POWER_ON())
//        CNS3xxx_I2C_POWER_ENABLE();

	//  if (!CNS3xxx_I2C_CLOCK())
	//      CNS3xxx_I2C_CLOCK_ENABLE();

        cns3xxx_pwr_clk_en(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);
        cns3xxx_pwr_power_up(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);
        cns3xxx_pwr_soft_rst(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);

	/* Disable the I2C */
	I2C_CONTROLLER_REG = 0;	/* Disabled the I2C */

	/* Enable SCL and SDA share pin function */
	cns3xxx_sharepin_request_array(i2c_gpios, ARRAY_SIZE(i2c_gpios));

	//TWI Pin Drive Strength
	//(0x300: 21mA) 
	//(0x200: 15.7mA) 
	//(0x100: 10.5mA) 
	//(0x000: 5.2mA)
	MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B &= ~0x300;
	MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B |= 0x300; //21mA...

	/* Check the Reg Dump when testing */
	I2C_TIME_OUT_REG =
	    ((((((cns3xxx_cpu_clock()*(1000000/8)) / (2 * CNS3xxx_I2C_CLK)) -
		1) & 0x3FF) << 8) | (1 << 7) | 0x7F);
	I2C_TWI_OUT_DLY_REG |= 0x3;

	/* Clear Interrupt Status (0x2 | 0x1) */
	I2C_INTERRUPT_STATUS_REG |= (I2C_ACTION_DONE_FLAG | I2C_BUS_ERROR_FLAG);

	/* Enable The Interrupt */
	CNS3xxx_I2C_ENABLE_INTR();

	/* Enable the I2C Controller */
	CNS3xxx_I2C_ENABLE();
}

static irqreturn_t cns3xxx_i2c_isr(int irq, void *dev_id)
{
	struct cns3xxx_i2c *i2c = dev_id;
	struct i2c_msg *msg = i2c->msg;
	uint32_t data = 0;
	uint32_t stat =
#ifdef CNS3xxx_I2C_DEBUG
	    i2c->intr_status =
#endif
	    I2C_INTERRUPT_STATUS_REG;

	if (stat & I2C_BUS_ERROR_FLAG) {
		i2c->state = STATE_ERROR;

		/* Clear the interrupt */
		I2C_INTERRUPT_STATUS_REG |= 0x1;

		wake_up(&i2c->wait);
		return IRQ_HANDLED;
	}

	if (i2c->state == STATE_READ) {
		data = I2C_READ_DATA_REG;
		if (msg->flags & I2C_M_RD)
			msg[0].buf[0] = data & 0x000000FF;
		else {
			msg[1].buf[0] = data & 0x000000FF;
			msg[1].buf[1] = ((data & 0x0000FF00) >> 8);
		}
	} else {
#ifdef CNS3xxx_I2C_DEBUG
		if (i2c->state != STATE_WRITE)
			BUG();
#endif
		i2c->pos++;
	}

	i2c->state = STATE_DONE;
	/* Clear Interrupt */
	I2C_INTERRUPT_STATUS_REG |= 0x1;
	wake_up(&i2c->wait);

#if 0
	/* End of msg, go to next msg */
	if (i2c->pos == msg->len) {
		i2c->nmsgs--;
		i2c->msg++;
		i2c->pos = 0;
		msg = i2c->msg;

		if (i2c->nmsgs) {	/* Not an End yet */
			i2c->state =
			    (msg->flags & I2C_M_RD) ? STATE_READ : STATE_WRITE;

			/* Prepare and Say RUN_START */
			if (i2c->state == STATE_READ) {
				/* 1. Write TWI Serial slave address
				 * 2. Write CFG Reg with Read Data Len, TWI transfer command,
				 * 3. Write CFG Reg with RUN_START
				 */

				I2C_SLAVE_ADDRESS_REG = (msg->addr << 1);
				i2c->valid_byte_mask = 0xff;	// ONE byte at a time

				/* Well, setting the controller for TWI_EN = 1; TWI_SWAP_EN = 0;
				 * RUN_START = 0; TRANSFER_CMD =0 (READ); TWI_DATA_LEN = 0;
				 * TWI_RD_DATA_LEN = 0 (1 byte);
				 */
				I2C_CONTROLLER_REG = (0x1 << 31);
				CNS3xxx_I2C_START_TRANSFER();
			} else {
				/* 1. Write TWI Serial slave address
				 * 2. Write CFG Reg with Write Data Len, TWI transfer command,
				 * 3. Write TWI WRITE DATA Reg with data to be written
				 * 4. Write CFG Reg with RUN_START
				 */
				I2C_SLAVE_ADDRESS_REG = (msg->addr << 1);
				i2c->valid_byte_mask = 0xffff;	// ONE byte at a time
				data = i2c->msg->buf[i2c->pos++];
				data |=
				    (i2c->msg->buf[i2c->pos] << 8) & i2c->
				    valid_byte_mask;
				I2C_WRITE_DATA_REG = data;
				I2C_CONTROLLER_REG = (0x1 << 31) | 0x14;
				CNS3xxx_I2C_START_TRANSFER();
			}

			return IRQ_HANDLED;
		} else {
			i2c->state = STATE_DONE;

			/* Clear Interrupt */
			I2C_INTERRUPT_STATUS_REG |= 0x1;
			wake_up(&i2c->wait);
			return IRQ_HANDLED;
		}
	}

	/* Prepare and RUN_START */
	if (i2c->state == STATE_READ) {
		/* 1. Write TWI Serial slave address
		 * 2. Write CFG Reg with Read Data Len, TWI transfer command,
		 * 3. Write CFG Reg with RUN_START
		 */

		I2C_SLAVE_ADDRESS_REG = (msg->addr << 1);
		i2c->valid_byte_mask = 0xff;	// ONE byte at a time

		/* Well, setting the controller for TWI_EN = 1; TWI_SWAP_EN = 0;
		 * RUN_START = 0; TRANSFER_CMD =0 (READ); TWI_DATA_LEN = 0;
		 * TWI_RD_DATA_LEN = 0 (1 byte);
		 */
		I2C_CONTROLLER_REG = (0x1 << 31);
		CNS3xxx_I2C_START_TRANSFER();
	} else {
		/* 1. Write TWI Serial slave address
		 * 2. Write CFG Reg with Write Data Len, TWI transfer command,
		 * 3. Write TWI WRITE DATA Reg with data to be written
		 * 4. Write CFG Reg with RUN_START
		 */
		I2C_SLAVE_ADDRESS_REG = (msg->addr << 1);
		i2c->valid_byte_mask = 0xffff;	// ONE byte at a time
		data =
		    ((i2c->msg->buf[0] +
		      i2c->pos) | (i2c->msg->buf[i2c->pos] << 8)) & i2c->
		    valid_byte_mask;
		// I2C_WRITE_DATA_REG = ((i2c->msg->buf[0] + 1) | (i2c->msg->buf[i2c->pos] << 8)) & i2c->valid_byte_mask;
		printk("%s:%d: next byte %x\n", __FUNCTION__, __LINE__, data);
		I2C_WRITE_DATA_REG = data;
		I2C_CONTROLLER_REG = (0x1 << 31) | 0x14;
		CNS3xxx_I2C_START_TRANSFER();
	}
#endif
	return IRQ_HANDLED;
}

static int __devinit cns3xxx_i2c_probe(struct platform_device *pdev)
{
	struct cns3xxx_i2c *i2c;
	struct resource *res, *res2;
	int ret;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk("%s: IORESOURCE_MEM not defined \n", __FUNCTION__);
		return -ENODEV;
	}

	res2 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res2) {
		printk("%s: IORESOURCE_IRQ not defined \n", __FUNCTION__);
		return -ENODEV;
	}

	i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	if (!request_mem_region(res->start, res->end - res->start + 1,
				pdev->name)) {
		dev_err(&pdev->dev, "Memory region busy\n");
		ret = -EBUSY;
		goto request_mem_failed;
	}

	i2c->base = ioremap(res->start, res->end - res->start + 1);
	if (!i2c->base) {
		dev_err(&pdev->dev, "Unable to map registers\n");
		ret = -EIO;
		goto map_failed;
	}

	cns3xxx_i2c_adapter_init(i2c);

	init_waitqueue_head(&i2c->wait);
	ret = request_irq(res2->start, cns3xxx_i2c_isr, 0, pdev->name, i2c);
	if (ret) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto request_irq_failed;
	}

	platform_set_drvdata(pdev, i2c);
	i2c->adap = cns3xxx_i2c_adapter;
	i2c_set_adapdata(&i2c->adap, i2c);
	i2c->adap.dev.parent = &pdev->dev;

	/* add i2c adapter to i2c tree */
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add adapter\n");
		goto add_adapter_failed;
	}

	return 0;

      add_adapter_failed:
	free_irq(res2->start, i2c);
      request_irq_failed:
	iounmap(i2c->base);
      map_failed:
	release_mem_region(res->start, res->end - res->start + 1);
      request_mem_failed:
	kfree(i2c);

	return ret;
}

static int __devexit cns3xxx_i2c_remove(struct platform_device *pdev)
{
	struct cns3xxx_i2c *i2c = platform_get_drvdata(pdev);
	struct resource *res;

	/* disable i2c logic */
	CNS3xxx_I2C_DISABLE_INTR();
	CNS3xxx_I2C_DISABLE();

	cns3xxx_pwr_clk_disable(0x1 << PM_CLK_GATE_REG_OFFSET_SPI_PCM_I2C);

	cns3xxx_sharepin_free_array(i2c_gpios, ARRAY_SIZE(i2c_gpios));

	/* remove adapter & data */
	i2c_del_adapter(&i2c->adap);
	platform_set_drvdata(pdev, NULL);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res)
		free_irq(res->start, i2c);

	iounmap(i2c->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, res->end - res->start + 1);

	kfree(i2c);

	return 0;
}

#ifdef CONFIG_PM
static int cns3xxx_i2c_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int cns3xxx_i2c_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define cns3xxx_i2c_suspend	NULL
#define cns3xxx_i2c_resume	NULL
#endif

static struct platform_driver cns3xxx_i2c_driver = {
	.probe = cns3xxx_i2c_probe,
	.remove = cns3xxx_i2c_remove,
	.suspend = cns3xxx_i2c_suspend,
	.resume = cns3xxx_i2c_resume,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "cns3xxx-i2c",
		   },
};

static int __init cns3xxx_i2c_init(void)
{
	return platform_driver_register(&cns3xxx_i2c_driver);
}

static void __exit cns3xxx_i2c_exit(void)
{
	platform_driver_unregister(&cns3xxx_i2c_driver);
}

module_init(cns3xxx_i2c_init);
module_exit(cns3xxx_i2c_exit);

MODULE_AUTHOR("Cavium Networks");
MODULE_DESCRIPTION("Cavium CNS3XXX I2C Controller");
MODULE_LICENSE("GPL");

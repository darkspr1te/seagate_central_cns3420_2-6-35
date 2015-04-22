/*******************************************************************************
 *
 *  drivers/gpio/cns3xxx-gpio.c
 *
 *  GPIO driver for the CNS3XXX SOCs
 *
 *  Copyright (c) 2011 Cavium
 *
 *  This file is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, Version 2, as
 *  published by the Free Software Foundation.
 *
 *  This file is distributed in the hope that it will be useful,
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 *  NONINFRINGEMENT.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this file; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or
 *  visit http://www.gnu.org/licenses/.
 *
 *  This file may also be available under a different license from Cavium.
 *  Contact Cavium for more information
 *
 ******************************************************************************/

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <asm/mach/irq.h>


#define GPIO_PROC_NAME		"gpio"
static struct proc_dir_entry	*proc_cns3xxx_gpio;

struct chog_gpio_chip {
	struct gpio_chip	chip;
	int			irq;
	void __iomem		*reg_base;
	void __iomem		*reg_sharepin_en;
	void __iomem		*reg_pud;
};

#define INIT_CHOG_GPIO_CHIP(name, base_no, nr_gpio)			\
	{								\
		.chip = {						\
			.label			= name,			\
			.owner			= THIS_MODULE,		\
			.request		= cns3xxx_request,	\
			.direction_input	= cns3xxx_direction_in,	\
			.direction_output	= cns3xxx_direction_out,\
			.get			= cns3xxx_get,		\
			.set			= cns3xxx_set,		\
			.to_irq			= cns3xxx_to_irq,	\
			.base			= base_no,		\
			.ngpio			= nr_gpio,		\
			.can_sleep		= 0,			\
		},							\
	}


#define to_chog_gpio_chip(c)	container_of(c, struct chog_gpio_chip, chip)
#define nr_banks		ARRAY_SIZE(cns3xxx_gc)

/* The CNS3XXX GPIO pins are shard with special functions which is described in
 * the following table. "none" in this table represent the corresponding pins
 * are dedicate GPIO.
 */
const char *sharepin_desc[] = {
	/* GPIOA group */
/*  0 */ "none",	"none",		"SD_PWR_ON",	"OTG_DRV_VBUS",
/*  4 */ "Don't use",	"none",		"none",		"none",
/*  8 */ "CIM_nOE",	"LCD_Power",	"SMI_nCS3",	"SMI_nCS2",
/* 12 */ "SMI_Clk",	"SMI_nADV",	"SMI_CRE",	"SMI_Addr[26]",
/* 16 */ "SD_nCD",	"SD_nWP",	"SD_CLK",	"SD_CMD",
/* 20 */ "SD_DT[7]",	"SD_DT[6]",	"SD_DT[5]",	"SD_DT[4]",
/* 24 */ "SD_DT[3]",	"SD_DT[2]",	"SD_DT[1]",	"SD_DT[0]",
/* 28 */ "SD_LED",	"UR_RXD1",	"UR_TXD1",	"UR_RTS2",
	/* GPIOB group */
/*  0 */ "UR_CTS2",	"UR_RXD2",	"UR_TXD2",	"PCMCLK",
/*  4 */ "PCMFS",	"PCMDT",	"PCMDR",	"SPInCS1",
/*  8 */ "SPInCS0",	"SPICLK",	"SPIDT",	"SPIDR",
/* 12 */ "SCL",		"SDA",		"GMII2_CRS",	"GMII2_COL",
/* 16 */ "RGMII1_CRS",	"RGMII1_COL",	"RGMII0_CRS",	"RGMII0_COL",
/* 20 */ "MDC",		"MDIO",		"I2SCLK",	"I2SFS",
/* 24 */ "I2SDT",	"I2SDR",	"ClkOut",	"Ext_Intr2",
/* 28 */ "Ext_Intr1",	"Ext_Intr0",	"SATA_LED1",	"SATA_LED0",
};

struct cns3xxx_regs {
	const char *name;
	volatile unsigned int *addr;
	u32 offset;
};

struct cns3xxx_regs gpio_regs[] =  {
	{"Data Out",			0, GPIO_OUTPUT_OFFSET},
	{"Data In",			0, GPIO_INPUT_OFFSET},
	{"Direction",			0, GPIO_DIR_OFFSET},
	{"Interrupt Enable",		0, GPIO_INTR_ENABLE_OFFSET},
	{"Interrupt Raw Status",	0, GPIO_INTR_RAW_STATUS_OFFSET},
	{"Interrupt Masked Status",	0, GPIO_INTR_MASKED_STATUS_OFFSET},
	{"Interrupt Level Trigger",	0, GPIO_INTR_TRIGGER_METHOD_OFFSET},
	{"Interrupt Both Edge",		0, GPIO_INTR_TRIGGER_BOTH_EDGES_OFFSET},
	{"Interrupt Falling Edge",	0, GPIO_INTR_TRIGGER_TYPE_OFFSET},
	{"Interrupt MASKED",		0, GPIO_INTR_MASK_OFFSET},
	{"GPIO Bounce Enable",		0, GPIO_BOUNCE_ENABLE_OFFSET},
	{"GPIO Bounce Prescale",	0, GPIO_BOUNCE_PRESCALE_OFFSET},
};

struct cns3xxx_regs misc_regs[] =  {
	{"Drive Strength Register A",	&MISC_IO_PAD_DRIVE_STRENGTH_CTRL_A},
	{"Drive Strength Register B",	&MISC_IO_PAD_DRIVE_STRENGTH_CTRL_B},
	{"Pull Up/Down Ctrl A[15:0]",	&MISC_GPIOA_15_0_PULL_CTRL_REG},
	{"Pull Up/Down Ctrl A[31:16]",	&MISC_GPIOA_16_31_PULL_CTRL_REG},
	{"Pull Up/Down Ctrl B[15:0]",	&MISC_GPIOB_15_0_PULL_CTRL_REG},
	{"Pull Up/Down Ctrl B[31:16]",	&MISC_GPIOB_16_31_PULL_CTRL_REG},
};

static DEFINE_SPINLOCK(gpio_lock);


static int cns3xxx_request(struct gpio_chip *chip, unsigned offset)
{
	/* GPIOA4 is reserved for chip bonding configuration. Please don't use
	 * and configure GPIOA4.
	 */
	if ((strcmp(chip->label, "GPIOA") == 0) && (offset == 4))
		return -EINVAL;
	return 0;
}

/*
 * Configure the GPIO line as an input.
 */
static int cns3xxx_direction_in(struct gpio_chip *chip, unsigned offset)
{
	struct chog_gpio_chip *cgc = to_chog_gpio_chip(chip);
	u32 reg;

	spin_lock(&gpio_lock);
	/* Clear corresponding register bit to set as input pin. */
	reg = readl(cgc->reg_base + GPIO_DIR_OFFSET);
	reg &= ~(1 << offset);
	writel(reg, cgc->reg_base + GPIO_DIR_OFFSET);
	spin_unlock(&gpio_lock);

	return 0;
}

/*
 * Set the state of an output GPIO line.
 */
static void cns3xxx_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct chog_gpio_chip *cgc = to_chog_gpio_chip(chip);

	if (value)
		/* Write 1 to set corresponding bit output "HIGH"
		 * Multi-bit write is allowed. Write 0 makes no change.
		 */
		writel(1 << offset, cgc->reg_base + GPIO_BIT_SET_OFFSET);
	else
		/* Write 1 to set corresponding bit output "LOW"
		 * Multi-bit write is allowed. Write 0 makes no change.
		 */
		writel(1 << offset, cgc->reg_base + GPIO_BIT_CLEAR_OFFSET);
}

/*
 * Configure the GPIO line as an output, with default state.
 */
static int cns3xxx_direction_out(struct gpio_chip *chip,
		unsigned offset, int value)
{
	struct chog_gpio_chip *cgc = to_chog_gpio_chip(chip);
	u32 reg;

	cns3xxx_set(chip, offset, value);

	spin_lock(&gpio_lock);
	/* Set corresponding register bit to set as output pin. */
	reg = readl(cgc->reg_base + GPIO_DIR_OFFSET);
	reg |= 1 << offset;
	writel(reg, cgc->reg_base + GPIO_DIR_OFFSET);
	spin_unlock(&gpio_lock);

	return 0;
}

/*
 * Read the state of a GPIO line.
 */
static int cns3xxx_get(struct gpio_chip *chip, unsigned offset)
{
	struct chog_gpio_chip *cgc = to_chog_gpio_chip(chip);
	u32 reg;
	int ret;

	reg = readl(cgc->reg_base + GPIO_INPUT_OFFSET);
	ret = reg & (1 << offset);

	return ret;
}

/*
 * GPIO interrtups are remapped to unused irq number.
 * The remapped GPIO IRQ number start from NR_IRQS_CNS3XXX (96). Here is the
 * table of GPIO to irq number mapping table.
 *
 *	GPIOA	GPIOB	|	GPIOA	GPIOB
 * No.	IRQ	IRQ	|  No.	IRQ	IRQ
 * ===================  |  ===================
 *  0	 96	128	|  16	112	144
 *  1	 97	129	|  17	113	145
 *  2	 98	130	|  18	114	146
 *  3	 99	131	|  19	115	147
 *  4	100	132	|  20	116	148
 *  5	101	133	|  21	117	149
 *  6	102	134	|  22	118	150
 *  7	103	135	|  23	119	151
 *  8	104	136	|  24	120	152
 *  9	105	137	|  25	121	153
 * 10	106	138	|  26	122	154
 * 11	107	139	|  27	123	155
 * 12	108	140	|  28	124	156
 * 13	109	141	|  29	125	157
 * 14	110	142	|  30	126	158
 * 15	111	143	|  31	127	159
 */

static int cns3xxx_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return offset + NR_IRQS_CNS3XXX + chip->base;
}

static unsigned __irq_to_gpio_offset(struct gpio_chip *chip, int irq)
{
	return irq - NR_IRQS_CNS3XXX - chip->base;
}

static int cns3xxx_gpio_set_irq_type(unsigned int irq, unsigned int type)
{
	struct chog_gpio_chip *gc = get_irq_chip_data(irq);
	void __iomem *base = gc->reg_base;
	unsigned offset = __irq_to_gpio_offset(&gc->chip, irq);
	u32 reg_level, reg_both, reg_low, index;

	index = 1 << offset;

	spin_lock(&gpio_lock);
	reg_level = readl(base + GPIO_INTR_TRIGGER_METHOD_OFFSET);
	reg_both = readl(base + GPIO_INTR_TRIGGER_BOTH_EDGES_OFFSET);
	reg_low = readl(base + GPIO_INTR_TRIGGER_TYPE_OFFSET);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		reg_level &= ~index;
		reg_both &= ~index;
		reg_low &= ~index;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		reg_level &= ~index;
		reg_both &= ~index;
		reg_low |= index;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		reg_level &= ~index;
		reg_both |= index;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		reg_level |= index;
		reg_low |= index;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		reg_level |= index;
		reg_low &= ~index;
		break;
	default:
		return -EINVAL;
	}

	cns3xxx_direction_in(&gc->chip, offset);
	writel(reg_level, base + GPIO_INTR_TRIGGER_METHOD_OFFSET);
	writel(reg_both, base + GPIO_INTR_TRIGGER_BOTH_EDGES_OFFSET);
	writel(reg_low, base + GPIO_INTR_TRIGGER_TYPE_OFFSET);
	spin_unlock(&gpio_lock);

	return 0;
}

static void cns3xxx_gpio_ack(unsigned int irq)
{
	struct chog_gpio_chip *gc = get_irq_chip_data(irq);
	void __iomem *base = gc->reg_base;
	unsigned offset = __irq_to_gpio_offset(&gc->chip, irq);
	u32 val;

	spin_lock(&gpio_lock);
	val = readl(base + GPIO_INTR_CLEAR_OFFSET);
	val |= (1 << offset);
	writel(val, base + GPIO_INTR_CLEAR_OFFSET);
	spin_unlock(&gpio_lock);
}

static void cns3xxx_gpio_mask(unsigned int irq)
{
	struct chog_gpio_chip *gc = get_irq_chip_data(irq);
	void __iomem *base = gc->reg_base;
	unsigned offset = __irq_to_gpio_offset(&gc->chip, irq);
	u32 val;

	spin_lock(&gpio_lock);
	val = readl(base + GPIO_INTR_ENABLE_OFFSET);
	val &= ~(1 << offset);
	writel(val, base + GPIO_INTR_ENABLE_OFFSET);
	spin_unlock(&gpio_lock);
}

static void cns3xxx_gpio_unmask(unsigned int irq)
{
	struct chog_gpio_chip *gc = get_irq_chip_data(irq);
	void __iomem *base = gc->reg_base;
	unsigned offset = __irq_to_gpio_offset(&gc->chip, irq);
	u32 val;

	spin_lock(&gpio_lock);
	val = readl(base + GPIO_INTR_ENABLE_OFFSET);
	val |= (1 << offset);
	writel(val, base + GPIO_INTR_ENABLE_OFFSET);
	spin_unlock(&gpio_lock);
}

static struct irq_chip cns3xxx_gpio_irq_chip = {
	.name = "GPIO",
	.ack = cns3xxx_gpio_ack,
	.mask = cns3xxx_gpio_mask,
	.unmask = cns3xxx_gpio_unmask,
	.set_type = cns3xxx_gpio_set_irq_type,
};

static struct chog_gpio_chip cns3xxx_gc[] = {
			  /* label,  base,	   ngpio */
	INIT_CHOG_GPIO_CHIP("GPIOA", 0x00,	   MAX_GPIOA_NO),
	INIT_CHOG_GPIO_CHIP("GPIOB", MAX_GPIOA_NO, MAX_GPIOB_NO),
};


/*
 * Turn on corresponding shared pin function.
 * Turn on shared pin function will also disable GPIO function. Related GPIO
 * control registers are still accessable but not reflect to pin.
 */
int cns3xxx_sharepin_request(unsigned gpio, const char *label)
{
	struct gpio_chip *chip;
	int i, reg, ret, offset = gpio;

	if (!label)
		label = sharepin_desc[gpio];

	ret = gpio_request(gpio, label);
	if (ret) {
		printk(KERN_INFO "gpio-%d already in use! err=%d\n", gpio, ret);
		return ret;
	}

	for (i = 0; i < nr_banks; i++) {
		chip = &cns3xxx_gc[i].chip;
		if (offset >= chip->ngpio) {
			offset -= chip->ngpio;
			continue;
		}
		spin_lock(&gpio_lock);
		reg = readl(cns3xxx_gc[i].reg_sharepin_en);
		reg |= (1 << offset);
		writel(reg, cns3xxx_gc[i].reg_sharepin_en);
		spin_unlock(&gpio_lock);
		pr_debug("%s[%d] is occupied by %s function!\n",
				chip->label, offset, label);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(cns3xxx_sharepin_request);

/**
 * cns3xxx_sharepin_request_array - request multiple GPIOs in a single call
 * @array:	array of the 'struct gpio'
 * @num:	how many GPIOs in the array
 */
int cns3xxx_sharepin_request_array(struct gpio *array, size_t num)
{
	int i, err;

	for (i = 0; i < num; i++, array++) {
		err = cns3xxx_sharepin_request(array->gpio, array->label);
		if (err)
			goto err_free;
		if (array->flags & GPIOF_DIR_IN)
			err = gpio_direction_input(array->gpio);
		else
			err = gpio_direction_output(array->gpio,
				(array->flags & GPIOF_INIT_HIGH) ? 1 : 0);
		if (err)
			goto err_free;
	}
	return 0;

err_free:
	while (i--)
		cns3xxx_sharepin_free((--array)->gpio);
	return err;
}
EXPORT_SYMBOL_GPL(cns3xxx_sharepin_request_array);

/*
 * Turn off corresponding share pin function.
 */
void cns3xxx_sharepin_free(unsigned gpio)
{
	struct gpio_chip *chip;
	int i, reg, offset = gpio;

	gpio_free(gpio);

	for (i = 0; i < nr_banks; i++) {
		chip = &cns3xxx_gc[i].chip;
		if (offset >= chip->ngpio) {
			offset -= chip->ngpio;
			continue;
		}
		spin_lock(&gpio_lock);
		reg = readl(cns3xxx_gc[i].reg_sharepin_en);
		reg &= ~(1 << offset);
		writel(reg, cns3xxx_gc[i].reg_sharepin_en);
		spin_unlock(&gpio_lock);
		pr_debug(KERN_INFO "%s[%d] share pin function (%s) disabled!\n",
				chip->label, offset, sharepin_desc[gpio]);
		break;
	}
}
EXPORT_SYMBOL(cns3xxx_sharepin_free);

/**
 * cns3xxx_sharepin_free_array - release multiple GPIOs in a single call
 * @array:	array of the 'struct gpio'
 * @num:	how many GPIOs in the array
 */
void cns3xxx_sharepin_free_array(struct gpio *array, size_t num)
{
	while (num--)
		cns3xxx_sharepin_free((array++)->gpio);
}
EXPORT_SYMBOL_GPL(cns3xxx_sharepin_free_array);

static int cns3xxx_gpio_read_proc(char *page, char **start,  off_t off,
		int count, int *eof, void *data)
{
	int num = 0, i, nr_regs;

	nr_regs = ARRAY_SIZE(gpio_regs);
	num += sprintf(page + num,
			"Register Description        GPIOA     GPIOB\n"
			"====================        =====     =====\n");
	num += sprintf(page + num, "%-26.26s: %08x  %08x\n", "GPIO Disable",
		readl(cns3xxx_gc[0].reg_sharepin_en),
		readl(cns3xxx_gc[1].reg_sharepin_en));
	for (i = 0; i < nr_regs; i++) {
		num += sprintf(page + num, "%-26.26s: %08x  %08x\n",
			gpio_regs[i].name,
			readl(cns3xxx_gc[0].reg_base + gpio_regs[i].offset),
			readl(cns3xxx_gc[1].reg_base + gpio_regs[i].offset));
	}

	num += sprintf(page + num, "\n"
			"Register Description        Value\n"
			"====================        =====\n");
	nr_regs = ARRAY_SIZE(misc_regs);
	for (i = 0; i < nr_regs; i++) {
		num += sprintf(page + num, "%-26.26s: %08x\n",
			misc_regs[i].name,
			*misc_regs[i].addr);
	}

	return num;
}


#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

const char *pull_state[] = {
	"--",
	"PD",
	"PU",
	""
};

static int get_bits(void __iomem *addr, int shift)
{
	u32 val;

	if (!addr)
		return 0;

	if (shift < 16)
		val = readl(addr);
	else {
		shift -= 16;
		val = readl(addr + 4);
	}

	return (val >> (shift * 2)) & 0x03;
}

#define header0	"           Label                 Mode      Dir Val\n"
#define header1	" ====================================================\n"
#define format0	" %s%-3d [%-20.20s] %-10.10s%s %s  %s\n"

static int cns3xxx_dbg_gpio_show_all(struct seq_file *s, void *unused)
{
	int i, j, is_out, disabled;
	unsigned gpio;
	const char *gpio_label;
	struct gpio_chip *chip;
	int bits;

	for (j = 0; j < nr_banks; j++) {
		chip = &cns3xxx_gc[j].chip;
		seq_printf(s, header0);
		seq_printf(s, header1);
		for (i = 0; i < chip->ngpio; i++) {
			gpio = chip->base + i;
			disabled = test_bit(i, cns3xxx_gc[j].reg_sharepin_en);
			gpio_label = gpiochip_is_requested(chip, i);
			if (!gpio_label) {
				if (disabled)
					gpio_label = sharepin_desc[gpio];
				else
					gpio_label = "";
			}
			is_out = test_bit(i, cns3xxx_gc[j].reg_base + GPIO_DIR_OFFSET);
			bits = get_bits(cns3xxx_gc[j].reg_pud, i);
			seq_printf(s, format0, chip->label, i, gpio_label,
					disabled ? "Function" : "GPIO",
					is_out ? "out" : "in ",
					chip->get(chip, i) ? "hi" : "lo",
					pull_state[bits]);
		}
		seq_printf(s, "\n");
	}

	return 0;
}

static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, cns3xxx_dbg_gpio_show_all, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init cns3xxx_gpio_debuginit(void)
{
	debugfs_create_file("gpio-cns3xxx", S_IRUGO, NULL, NULL, &debug_fops);
	return 0;
}
late_initcall(cns3xxx_gpio_debuginit);

#endif /* CONFIG_DEBUG_FS */

static void chained_gpio_isr(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = get_irq_chip(irq);
	struct chog_gpio_chip *gc = get_irq_data(irq);
	unsigned i;
	int target_irq;
	u32 status;

	chip->ack(irq);

	spin_lock(&gpio_lock);
	status = readl(gc->reg_base + GPIO_INTR_MASKED_STATUS_OFFSET);
	writel(status, gc->reg_base + GPIO_INTR_CLEAR_OFFSET);
	spin_unlock(&gpio_lock);

	for (i = 0; i < gc->chip.ngpio; i++) {
		if (status & (1 << i)) {
			target_irq = cns3xxx_to_irq(&gc->chip, i);
			pr_debug(KERN_INFO "Invoke cascaded irq %d from irq %d\n",
					target_irq, gc->irq);
			generic_handle_irq(target_irq);
		}
	}

	chip->unmask(irq);
}

static void __iomem *gpio_map(struct platform_device *pdev,
		const char *name, int *err)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	resource_size_t start;
	resource_size_t sz;
	void __iomem *ret;

	*err = 0;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!r)
		return NULL;

	sz = resource_size(r);
	start = r->start;

	if (!devm_request_mem_region(dev, start, sz, r->name)) {
		*err = -EBUSY;
		return NULL;
	}

	ret = devm_ioremap(dev, start, sz);
	if (!ret) {
		*err = -ENOMEM;
		return NULL;
	}

	return ret;
}

static int __devinit gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *misc_reg;
	int i, j, err, nr_gpios = 0, irq = 0;

	cns3xxx_pwr_clk_en(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);
	cns3xxx_pwr_soft_rst(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);

	if (cns3xxx_proc_dir) {
		proc_cns3xxx_gpio = create_proc_entry(GPIO_PROC_NAME,
				S_IFREG | S_IRUGO, cns3xxx_proc_dir);
		if (proc_cns3xxx_gpio)
			proc_cns3xxx_gpio->read_proc = cns3xxx_gpio_read_proc;
	}

	misc_reg = gpio_map(pdev, "MISC", &err);
	if (!misc_reg) {
		dev_dbg(dev, "%s gpio_map \"MISC\" failure! err=%d\n",
				__func__, err);
		return err;
	}

	/* Scan and match GPIO resources */
	for (i = 0; i < nr_banks; i++) {
		/* Fetech GPIO base address */
		cns3xxx_gc[i].reg_base = gpio_map(pdev, cns3xxx_gc[i].chip.label, &err);
		if (!cns3xxx_gc[i].reg_base) {
			dev_dbg(dev, "%s gpio_map %s failure! err=%d\n",
					__func__, cns3xxx_gc[i].chip.label, err);
			return err;
		}

		/* Fetech GPIO interrupt number */
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				cns3xxx_gc[i].chip.label);
		if (!res)
			continue;
		irq = res->start;
		cns3xxx_gc[i].irq = irq;
		cns3xxx_gc[i].chip.dev = &pdev->dev;
		cns3xxx_gc[i].reg_sharepin_en = misc_reg + i * 4 +
			MISC_GPIOA_PIN_DISABLE_OFFSET;
		cns3xxx_gc[i].reg_pud = misc_reg + i * 8 +
			MISC_GPIOA_15_0_PULL_CTRL_OFFSET;

		gpiochip_add(&cns3xxx_gc[i].chip);

		/* Initial irq_chip to handle virtual GPIO irqs. */
		for (j = 0; j < cns3xxx_gc[i].chip.ngpio; j++) {
			irq = cns3xxx_to_irq(&cns3xxx_gc[i].chip, j);
			set_irq_chip(irq, &cns3xxx_gpio_irq_chip);
			set_irq_handler(irq, handle_simple_irq);
			set_irq_flags(irq, IRQF_VALID);
			set_irq_chip_data(irq, &cns3xxx_gc[i]);
		}
		set_irq_chained_handler(cns3xxx_gc[i].irq, chained_gpio_isr);
		set_irq_type(cns3xxx_gc[i].irq, IRQ_TYPE_LEVEL_HIGH);
		set_irq_data(cns3xxx_gc[i].irq, &cns3xxx_gc[i]);

		nr_gpios += cns3xxx_gc[i].chip.ngpio;
		if (nr_gpios >= MAX_GPIO_NO)
			break;
	}

	return 0;
}


static struct platform_driver gpio_driver = {
	.probe		= gpio_probe,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "cns3xxx-gpio",
	},
};

#define GPIO_PORTS_NO sizeof(cns3xxx_gc) / sizeof(struct gpio_chip)

#define VALIDATE_PORT_AND_BIT(port, bit)                \
    if (port > GPIO_PORTS_NO) {                         \
        printk(KERN_INFO "Invalid port: %u\n", port);   \
        return -EINVAL;                                 \
    }                                                   \
                                                        \
    if (bit > 31) {                                     \
        printk(KERN_INFO "Invalid bit: %u\n", bit);     \
        return -EINVAL;                                 \
    }

/**
 * Same functionality as cns3xxx_direction_in.
 * It is meant to be called from other modules.
 * port: 0 - GPIOA or 1 - GPIOB.
 * bit: between 0-31
 */
int cns3xxx_gpio_direction_in(unsigned port, unsigned bit)
{
    VALIDATE_PORT_AND_BIT(port, bit)

    return cns3xxx_direction_in(&cns3xxx_gc[port].chip, bit);
}

EXPORT_SYMBOL(cns3xxx_gpio_direction_in);

/**
 * Same functionality as cns3xxx_set.
 * It is meant to be called from other modules.
 * port: 0 - GPIOA or 1 - GPIOB.
 * bit: between 0-31
 * value: 0 or 1
 */
int cns3xxx_gpio_set(unsigned port, unsigned bit, int value)
{
    VALIDATE_PORT_AND_BIT(port, bit)

    cns3xxx_set(&cns3xxx_gc[port].chip, bit, value);

    return 0;
}

EXPORT_SYMBOL(cns3xxx_gpio_set);

/**
 * Same functionality as cns3xxx_direction_out.
 * It is meant to be called from other modules.
 * port: 0 - GPIOA or 1 - GPIOB.
 * bit: between 0-31
 * value: 0 or 1
 */
int cns3xxx_gpio_direction_out(unsigned port, unsigned bit, int value)
{
    VALIDATE_PORT_AND_BIT(port, bit)

    return cns3xxx_direction_out(&cns3xxx_gc[port].chip, bit, value);
}

EXPORT_SYMBOL(cns3xxx_gpio_direction_out);

/**
 * Same functionality as cns3xxx_get.
 * It is meant to be called from other modules.
 * port: 0 - GPIOA or 1 - GPIOB.
 * bit: between 0-31
 * return: 0 or 1
 */
int cns3xxx_gpio_get(unsigned port, unsigned bit)
{
    VALIDATE_PORT_AND_BIT(port, bit)

    return cns3xxx_get(&cns3xxx_gc[port].chip, bit);
}

EXPORT_SYMBOL(cns3xxx_gpio_get);

int __init cns3xxx_gpio_init(void)
{
	return platform_driver_register(&gpio_driver);
}

void __exit cns3xxx_gpio_exit(void)
{
	if (proc_cns3xxx_gpio)
		remove_proc_entry(GPIO_PROC_NAME, cns3xxx_proc_dir);
	platform_driver_unregister(&gpio_driver);
}

module_init(cns3xxx_gpio_init);
module_exit(cns3xxx_gpio_exit);

MODULE_LICENSE("GPL");


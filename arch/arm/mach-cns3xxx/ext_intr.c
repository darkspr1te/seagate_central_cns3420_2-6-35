/*******************************************************************************
 *
 *  arch/arm/mach-cns3xxx/ext_intr.c
 *
 *  External interrupt driver for the CNS3XXX SOCs
 *
 *  Copyright (c) 2011 Cavium
 *		       Tommy Lin <tommy.lin@caviumnetworks.com>
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>


/*   CNS3XXX has three external interrupt pins, Ext_Intr0, Ext_Intr1 and
 * Ext_Intr3. Each pin has its own interrupt ID which is not shared with other
 * function. It is not like 32 gpio pins in group A or B shared one irq. The
 * external interrupt pins are special functions that shared with GPIOB[27:29].
 * Users could turn on shared pin function to enable external interrupt pin with
 * cns3xxx_sharepin_request API.
 *
 *   The external interrupt trigger method is controlled by ARM MPCore GIC. GIC
 * only suppot high active level trigger and rising edge trigger type. Configure
 * GIC with other trigger type will have no effect.
 *
 *   Name	irq	Shared gpio pin
 *   Ext_Intr0	93	GPIOB[29]
 *   Ext_Intr1	94	GPIOB[28]
 *   Ext_Intr2	95	GPIOB[27]
 *
 *   Users can monitor interrupt status via /proc/interrups.
 * # cat /proc/interrupts
 * (Messages that turn on FUNCTION_MODE)
 *            CPU0
 * ......
 *  93:          0         GIC  Ext_Intr0
 *  94:    1795957         GIC  Ext_Intr1
 *  95:     244789         GIC  Ext_Intr2
 *
 * (Messages that turn off FUNCTION_MODE)
 *            CPU0
 * ......
 *  93:          0         GIC  Ext_Intr0
 * 155:         71        GPIO  GPIOB-27
 * 156:         30        GPIO  GPIOB-28
 */


#define FUNCTION_MODE

struct ext_intr_data {
	int		irq;
	unsigned	gpio;
	char		*name;
	char		*alias;
};

static unsigned led_D6 = GPIOA(6);
static unsigned led_D7 = GPIOA(5);

static struct ext_intr_data eintr_data[] = {
	{IRQ_CNS3XXX_EXTERNAL_PIN0, GPIOB(29), "Ext_Intr0", "HIB_REQ"},
	{IRQ_CNS3XXX_EXTERNAL_PIN1, GPIOB(28), "Ext_Intr1", "Push Button S3"},
	{IRQ_CNS3XXX_EXTERNAL_PIN2, GPIOB(27), "Ext_Intr2", "Push Button S4"},
};

static irqreturn_t cns3xxx_ext_intr0_handler(int irq, void *d)
{
	struct ext_intr_data *data = d;

	printk(KERN_INFO "%s detected!\n", data->alias);

	return IRQ_HANDLED;
}

static irqreturn_t cns3xxx_ext_intr1_handler(int irq, void *d)
{
	int value;

	value = gpio_get_value(led_D6);

	gpio_set_value(led_D6, !value);

	return IRQ_HANDLED;
}

static irqreturn_t cns3xxx_ext_intr2_handler(int irq, void *d)
{
	int value;

	value = gpio_get_value(led_D7);

	gpio_set_value(led_D7, !value);

	return IRQ_HANDLED;
}


int __init cns3xxx_eintr_init(void)
{
	int ret;

	ret = gpio_request(led_D6, "EINTR Test LED(D6)");
	if (ret)
		goto err1;
	gpio_direction_output(led_D6, 0);

	ret = gpio_request(led_D7, "EINTR Test LED(D7)");
	if (ret)
		goto err2;
	gpio_direction_output(led_D7, 0);

	/************ Ext_Intr0 ************/
	ret = cns3xxx_sharepin_request(eintr_data[0].gpio, eintr_data[0].alias);
	if (ret)
		goto err3;

	ret = request_irq(eintr_data[0].irq, cns3xxx_ext_intr0_handler,
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			eintr_data[0].name, &eintr_data[0]);
	if (ret)
		goto err4;

#ifdef FUNCTION_MODE
	/************ Ext_Intr1 ************/
	ret = cns3xxx_sharepin_request(eintr_data[1].gpio, eintr_data[1].alias);
	if (ret)
		goto err5;

	ret = request_irq(eintr_data[1].irq, cns3xxx_ext_intr1_handler,
			/* Avaliable trigger type settings
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			IRQF_SHARED | IRQF_TRIGGER_HIGH,
			*/
			IRQF_SHARED | IRQF_TRIGGER_RISING,
			eintr_data[1].name, &eintr_data[1]);
	if (ret)
		goto err6;

	/************ Ext_Intr2 ************/
	ret = cns3xxx_sharepin_request(eintr_data[2].gpio, eintr_data[2].alias);
	if (ret)
		goto err7;

	ret = request_irq(eintr_data[2].irq, cns3xxx_ext_intr2_handler,
			IRQF_SHARED | IRQF_TRIGGER_HIGH,
			eintr_data[2].name, &eintr_data[2]);
	if (ret)
		goto err8;
#else
	/* Using GPIOB[28] to do the similar job with Ext_Intr1.
	 * Here source code setup GPIOB[28] interrupt trigger type to both edge.
	 * The both edge interrupt type will trigger interrupt with rising or
	 * falling edge which is not acceptable with Ext_IntrX pins(GIC).
	 */
	cns3xxx_sharepin_free(eintr_data[1].gpio);
	ret = gpio_request(GPIOB(28), "Interrupt Test1");
	if (ret)
		goto err5;

	ret = request_irq(gpio_to_irq(GPIOB(28)), cns3xxx_ext_intr1_handler,
		/* Avaliable trigger type settings
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		IRQF_SHARED | IRQF_TRIGGER_RISING,
		IRQF_SHARED | IRQF_TRIGGER_FALLING,
		IRQF_SHARED | IRQF_TRIGGER_HIGH,
		IRQF_SHARED | IRQF_TRIGGER_LOw,
		 */
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"GPIOB-28", &eintr_data[1]);
	if (ret)
		goto err6;

	/* Using GPIOB[27] to do the same job with Ext_Intr0. */
	cns3xxx_sharepin_free(eintr_data[2].gpio);
	ret = gpio_request(GPIOB(27), "Interrupt Test0");
	if (ret)
		goto err9;

	ret = request_irq(gpio_to_irq(GPIOB(27)), cns3xxx_ext_intr2_handler,
			IRQF_SHARED | IRQF_TRIGGER_HIGH,
			"GPIOB-27", &eintr_data[2]);
	if (ret)
		goto err10;
#endif

	return 0;

#ifdef FUNCTION_MODE
err8:
	cns3xxx_sharepin_free(eintr_data[2].gpio);
err7:
	free_irq(eintr_data[1].irq, &eintr_data[1]);
#else
err10:
	gpio_free(GPIOB(27));
err9:
	free_irq(gpio_to_irq(GPIOB(28)), &eintr_data[1]);
#endif
err6:
	cns3xxx_sharepin_free(eintr_data[1].gpio);
err5:
	free_irq(eintr_data[0].irq, &eintr_data[0]);
err4:
	cns3xxx_sharepin_free(eintr_data[0].gpio);
err3:
	gpio_free(led_D7);
err2:
	gpio_free(led_D6);
err1:
	return ret;
}

void __exit cns3xxx_eintr_exit(void)
{
#ifdef FUNCTION_MODE
	free_irq(eintr_data[2].irq, &eintr_data[2]);
	free_irq(eintr_data[1].irq, &eintr_data[1]);
#else
	free_irq(gpio_to_irq(GPIOB(27)), &eintr_data[1]);
	free_irq(gpio_to_irq(GPIOB(28)), &eintr_data[1]);
#endif
	free_irq(eintr_data[0].irq, &eintr_data[0]);
	cns3xxx_sharepin_free(eintr_data[2].gpio);
	cns3xxx_sharepin_free(eintr_data[1].gpio);
	cns3xxx_sharepin_free(eintr_data[0].gpio);
	gpio_free(led_D7);
	gpio_free(led_D6);
}

module_init(cns3xxx_eintr_init);
module_exit(cns3xxx_eintr_exit);

MODULE_LICENSE("GPL");


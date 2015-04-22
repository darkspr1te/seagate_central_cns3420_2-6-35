/*******************************************************************************
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

#ifndef	_CNS3XXX_GPIO_H_
#define	_CNS3XXX_GPIO_H_

#include <mach/cns3xxx.h>

#define ARCH_NR_GPIOS			MAX_GPIO_NO

#include <asm-generic/gpio.h>

#define gpio_get_value			__gpio_get_value
#define gpio_set_value			__gpio_set_value
#define gpio_cansleep			__gpio_cansleep
#define gpio_to_irq			__gpio_to_irq

#define GPIOA(n)			n
#define GPIOB(n)			(MAX_GPIOA_NO + n)


/* Function prototype */
int cns3xxx_sharepin_request(unsigned gpio, const char *label);
void cns3xxx_sharepin_free(unsigned gpio);
int cns3xxx_sharepin_request_array(struct gpio *array, size_t num);
void cns3xxx_sharepin_free_array(struct gpio *array, size_t num);
int cns3xxx_gpio_direction_in(unsigned port, unsigned bit);
int cns3xxx_gpio_set(unsigned port, unsigned bit, int value);
int cns3xxx_gpio_direction_out(unsigned port, unsigned bit, int value);
int cns3xxx_gpio_get(unsigned port, unsigned bit);

#endif


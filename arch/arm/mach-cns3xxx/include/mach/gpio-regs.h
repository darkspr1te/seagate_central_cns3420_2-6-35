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

#ifndef	_CNS3XXX_GPIO_REGS_H_
#define	_CNS3XXX_GPIO_REGS_H_

#define GPIO_OUTPUT_OFFSET			0x00
#define GPIO_INPUT_OFFSET			0x04
#define GPIO_DIR_OFFSET				0x08
#define GPIO_BIT_SET_OFFSET			0x10
#define GPIO_BIT_CLEAR_OFFSET			0x14
#define GPIO_INTR_ENABLE_OFFSET			0x20
#define GPIO_INTR_RAW_STATUS_OFFSET		0x24
#define GPIO_INTR_MASKED_STATUS_OFFSET		0x28
#define GPIO_INTR_MASK_OFFSET			0x2C
#define GPIO_INTR_CLEAR_OFFSET			0x30
#define GPIO_INTR_TRIGGER_METHOD_OFFSET		0x34
#define GPIO_INTR_TRIGGER_BOTH_EDGES_OFFSET	0x38
#define GPIO_INTR_TRIGGER_TYPE_OFFSET		0x3C
#define GPIO_BOUNCE_ENABLE_OFFSET		0x40
#define GPIO_BOUNCE_PRESCALE_OFFSET		0x44

#define MISC_GPIOA_PIN_DISABLE_OFFSET		0x14
#define MISC_GPIOB_PIN_DISABLE_OFFSET		0x18
#define MISC_IO_PAD_DRIVE_STR_CTRL_A_OFFSET	0x1C
#define MISC_IO_PAD_DRIVE_STR_CTRL_B_OFFFSET	0x20
#define MISC_GPIOA_15_0_PULL_CTRL_OFFSET	0x24
#define MISC_GPIOA_16_31_PULL_CTRL_OFFSET	0x28
#define MISC_GPIOB_15_0_PULL_CTRL_OFFSET	0x2C
#define MISC_GPIOB_16_31_PULL_CTRL_OFFSET	0x30

#endif

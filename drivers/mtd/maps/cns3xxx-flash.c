/*******************************************************************************
 *
 *  drivers/mtd/maps/cns3xxx-flash.c
 * 
 *  Mapping for the CNS3XXX based systems
 *
 *  Author: Scott Shu
 *
 *  Copyright (c) 2008 Cavium Networks 
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
 *  Contact Cavium Networks for more information
 *
 ******************************************************************************/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/system.h>

struct cns3xxxflash_info {
	struct flash_platform_data *plat;
	struct resource		*res;
	struct mtd_partition	*parts;
	struct mtd_info		*mtd;
	struct map_info		map;
};

static const char *probes[] = { "cmdlinepart", "Boot", NULL };

#ifdef CONFIG_SILICON
static struct mtd_partition cns3xxx_flash_partitions[] = {
	{
		.name =		"UBoot",
		.size =		0x00020000,	/* u-boot (128KiB), 0x20000-0x40000: Reserved Space */
		.offset =	0,
		.mask_flags =	MTD_WRITEABLE,	/* force read-only */
	},{
		.name =		"CaviumEnv",	/* bottom 8KiB for env vars */
		.size =		0x20000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"Kernel",	/* OS Kernel */
		.size =		0x004C0000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"FileSystem",	/* File System */
		.size =		0x7000000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"FileSystem2",	/* File System 2 */
		.size =		0x0AE0000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"UBootEnv",	/* bottom 8KiB for env vars */
		.size =		MTDPART_SIZ_FULL,
		.offset =	MTDPART_OFS_APPEND,
	}
};
#else
static struct mtd_partition cns3xxx_flash_partitions[] = {
	{
		.name =		"UBoot",
		.size =		0x00020000,	/* u-boot (128KiB), 0x20000-0x40000: Reserved Space */
		.offset =	0,
		.mask_flags =	MTD_WRITEABLE,	/* force read-only */
	},{
		.name =		"CaviumEnv",	/* bottom 8KiB for env vars */
		.size =		0x00020000,
		.offset =	MTDPART_OFS_APPEND,
	},{
		.mask_flags =	MTD_WRITEABLE,	/* force read-only */
	}, {
		.name =		"Kernel",	/* OS Kernel */
		.size =		0x004C0000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"FileSystem",	/* File System */
		.size =		0x002F0000,
		.offset =	MTDPART_OFS_APPEND,
	}, {
		.name =		"UBootEnv",	/* bottom 8KiB for env vars */
		.size =		MTDPART_SIZ_FULL,
		.offset =	MTDPART_OFS_APPEND,
	}
};
#endif

static struct mtd_partition *parsed_parts;

static int cns3xxxflash_probe(struct platform_device *dev)
{
	struct flash_platform_data *plat = dev->dev.platform_data;
	struct resource *res = dev->resource;
	unsigned int size = res->end - res->start + 1;
	struct cns3xxxflash_info *info;
	int err;
	void __iomem *base;
	struct mtd_partition *parts;
	int nb_parts = 0;
	int parsed_nr_parts = 0;

	parts = cns3xxx_flash_partitions;
	nb_parts = ARRAY_SIZE(cns3xxx_flash_partitions);

	info = kzalloc(sizeof(struct cns3xxxflash_info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto out;
	}

	info->plat = plat;
	if (plat && plat->init) {
		err = plat->init();
		if (err)
			goto no_resource;
	}

	info->res = request_mem_region(res->start, size, "flash");
	if (!info->res) {
		err = -EBUSY;
		goto no_resource;
	}

	base = ioremap(res->start, size);
	if (!base) {
		err = -ENOMEM;
		goto no_mem;
	}

	/*
	 * look for CFI based flash parts fitted to this board
	 */
	info->map.size		= size;
	info->map.bankwidth	= plat->width;
	info->map.phys		= res->start;
	info->map.virt		= base;
	info->map.name		= "cns3xxxflash";

	simple_map_init(&info->map);

	/*
	 * Also, the CFI layer automatically works out what size
	 * of chips we have, and does the necessary identification
	 * for us automatically.
	 */
	info->mtd = do_map_probe(plat->map_name, &info->map);
	if (!info->mtd) {
		err = -ENXIO;
		goto no_device;
	}

	info->mtd->owner = THIS_MODULE;

	if (parsed_nr_parts > 0) {
		parts = parsed_parts;
		nb_parts = parsed_nr_parts;
	}

	if (nb_parts == 0) {
		printk(KERN_NOTICE "CNS3XXX NOR flash: No partition info available \n");
		if (add_mtd_device(info->mtd))
			return -ENXIO;
	} else {
		printk(KERN_NOTICE "CNS3XXX NOR flash: Using static partition definition\n");
		return add_mtd_partitions(info->mtd, parts, nb_parts);
	}

	platform_set_drvdata(dev, info);

	err = parse_mtd_partitions(info->mtd, probes, &info->parts, 0);
	if (err > 0) {
		err = add_mtd_partitions(info->mtd, info->parts, err);
		if (err)
			printk(KERN_ERR
			       "mtd partition registration failed: %d\n", err);
	}

	if (err == 0)
		platform_set_drvdata(dev, info);

	/*
	 * If we got an error, free all resources.
	 */
	if (err < 0) {
		if (info->mtd) {
			del_mtd_partitions(info->mtd);
			map_destroy(info->mtd);
		}
		kfree(info->parts);

 no_device:
		iounmap(base);
 no_mem:
		release_mem_region(res->start, size);
 no_resource:
		if (plat && plat->exit)
			plat->exit();
		kfree(info);
	}
 out:
	return err;
}

static int cns3xxxflash_remove(struct platform_device *dev)
{
	struct cns3xxxflash_info *info = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if (info) {
		if (info->mtd) {
			del_mtd_partitions(info->mtd);
			map_destroy(info->mtd);
		}
		kfree(info->parts);

		iounmap(info->map.virt);
		release_resource(info->res);
		kfree(info->res);

		if (info->plat && info->plat->exit)
			info->plat->exit();

		kfree(info);
	}

	return 0;
}

static struct platform_driver cns3xxxflash_driver = {
	.probe		= cns3xxxflash_probe,
	.remove		= cns3xxxflash_remove,
	.driver		= {
		.name	= "cns3xxxflash",
		.owner	= THIS_MODULE,
	},
};

static int __init cns3xxxflash_init(void)
{
	return platform_driver_register(&cns3xxxflash_driver);
}

static void __exit cns3xxxflash_exit(void)
{
	platform_driver_unregister(&cns3xxxflash_driver);
}

module_init(cns3xxxflash_init);
module_exit(cns3xxxflash_exit);

MODULE_AUTHOR("Scott Shu");
MODULE_DESCRIPTION("CNS3XXX CFI map driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cns3xxxflash");

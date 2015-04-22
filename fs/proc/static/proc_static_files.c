/*
 * Create /proc entries for all the files listed in proc_static_files_table.h
 *
 * Author: Dale Farnsworth <dale@farnsworth.org>
 *
 * Copyright 2009 (c) MontaVista Software, Inc. This file is licensed
 * under the terms of the GNU General Public License version 2.
 * This program is licensed "as is" without any warranty of any kind,
 * whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/init.h>
#include <asm/uaccess.h>

struct psf_entry {
	char *name;
	struct psf_entry *parent;
	void *data;
	int size;
};

#include "proc_static_files_table.h"

static struct proc_dir_entry *psf_parent(struct psf_entry *p)
{
	return p->parent ? p->parent->data : NULL;
}

static int psf_open(struct inode *inode, struct file *file)
{
	file->private_data = PDE(inode)->data;

	return 0;
}

static ssize_t psf_read(struct file *file, char __user *buf,
		        size_t len, loff_t *offset)
{
	struct psf_entry *p = file->private_data;

	return simple_read_from_buffer(buf, len, offset, p->data, p->size);
}

static const struct file_operations psf_file_ops = {
	.owner = THIS_MODULE,
	.open = psf_open,
	.read = psf_read,
};

static int __init psf_create_file(struct psf_entry *p)
{
	struct proc_dir_entry *entry;

	entry = proc_create_data(p->name, S_IFREG | S_IRUGO, psf_parent(p),
				 &psf_file_ops, p);
	if (!entry)
		return -ENOMEM;

	entry->size = p->size;

	return 0;
}

static int __init psf_init(void)
{
	int i;
	struct psf_entry *p;
	int rc;

	for (i = 0; i < ARRAY_SIZE(psf_table); i++) {
		p = &psf_table[i];
		if (p->size == -1) {
			p->data = proc_mkdir(p->name, psf_parent(p));
			BUG_ON(p->data == NULL);
		} else {
			rc = psf_create_file(p);
			if (rc < 0)
				return rc;
		}
	}

	return 0;
}

static void __exit psf_cleanup(void)
{
	int i;
	struct psf_entry *p;

	for (i = (ARRAY_SIZE(psf_table) - 1); i >= 0; i--) {
		p = &psf_table[i];
		remove_proc_entry(p->name, psf_parent(p));
	}
}

module_init(psf_init);
module_exit(psf_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dale Farnsworth");
MODULE_DESCRIPTION("Create /proc static files");

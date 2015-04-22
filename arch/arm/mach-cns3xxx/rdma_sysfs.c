/*
 * rdma_sysfs.c : Sysfs of CNS3XXX RAID-DMA
 */
#ifdef	CONFIG_SYSFS

#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/mempool.h>
#include <linux/slab.h>
#include "rdma.h"

static struct class *rdma_class;
static void rdma_class_release(struct class *class)
{
	kfree(class);
}

#define LPLUS \
do {	\
	p += len;	\
	length += len;	\
} while (0);

#define PCAT(pool, name)	\
do {	\
	if ((pool) && (name)) { \
		len = sprintf(p, "%s=[%d/%d]\n", name, pool->curr_nr, pool->min_nr);	\
		LPLUS;	\
	}	\
} while (0);

#define	rdma_class_create_file(class, attr)	\
do { \
	WARN_ON(class_create_file((class), (attr)));	\
} while (0);


static ssize_t rdma_debug_show(struct class *c, struct class_attribute *attr,
		char *b)
{
	int len = 0, length = 0;
	char *p = b;

	PCAT(rdma_sg_pool, "rdma_sg_pool");

	len = sprintf(p, "reg->ParaReg(0x%p, 0x%x) = 0x%08x / 0x%08x\n",
			(void *)dma->cregs->para,
			RDMA_REGS_PHYS(REG_PARA_OFFSET),
			RDMA_REGS_VALUE(REG_PARA_OFFSET),
			*(dma->cregs->para));
    LPLUS;
	len = sprintf(p, "reg->BlkSize(0x%p, 0x%x) = 0x%08x / 0x%08x\n",
			(void *)dma->cregs->blsz,
			RDMA_REGS_PHYS(REG_BLSZ_OFFSET),
			RDMA_REGS_VALUE(REG_BLSZ_OFFSET),
			*(dma->cregs->blsz));
    LPLUS;
	len = sprintf(p, "reg->S/GAddr(0x%p, 0x%x) = 0x%08x / 0x%08x\n",
			(void *)dma->cregs->sgad,
			RDMA_REGS_PHYS(REG_SGAD_OFFSET),
			RDMA_REGS_VALUE(REG_SGAD_OFFSET),
			*(dma->cregs->sgad));
    LPLUS;
	len = sprintf(p, "reg->StatReg(0x%p, 0x%x) = 0x%08x / 0x%08x\n",
			(void *)dma->cregs->stat,
			RDMA_REGS_PHYS(REG_STAT_OFFSET),
			RDMA_REGS_VALUE(REG_STAT_OFFSET),
			*(dma->cregs->stat));
    LPLUS;
	len = sprintf(p, "reg->FrntPtr(0x%p, 0x%x) = 0x%08x / 0x%08x\n",
			(void *)dma->cregs->frnt,
			RDMA_REGS_PHYS(REG_FRNT_OFFSET),
			RDMA_REGS_VALUE(REG_FRNT_OFFSET),
			*(dma->cregs->frnt));
    LPLUS;
	len = sprintf(p, "reg->BackPtr(0x%p, 0x%x) = 0x%08x / 0x%08x\n",
			(void *)dma->cregs->back,
			RDMA_REGS_PHYS(REG_BACK_OFFSET),
			RDMA_REGS_VALUE(REG_BACK_OFFSET),
			*(dma->cregs->back));
    LPLUS;
	len = sprintf(p, "reg->QuParam(0x%p, 0x%x) = 0x%08x / 0x%08x\n",
			(void *)dma->cregs->qpar,
			RDMA_REGS_PHYS(REG_QPAR_OFFSET),
			RDMA_REGS_VALUE(REG_QPAR_OFFSET),
			*(dma->cregs->qpar));
    LPLUS;

	return length;
}

static ssize_t rdma_debug_store(struct class *c, struct class_attribute *attr,
		const char *b, size_t count)
{
	int unit_test_action = 1;
	unsigned int src_cnts = 4;
	unsigned int bytes = 4096;

	if (*b != '0' || (b[1] != 'x' && b[1] != 'X'))
		sscanf(b, "%d %u %u", &unit_test_action, &src_cnts, &bytes);
	else if (b[1] == 'x')
		sscanf(b, "0x%x 0x%x 0x%x", &unit_test_action, &src_cnts,
				&bytes);
	else
		sscanf(b, "0X%x 0X%x 0X%x", &unit_test_action, &src_cnts,
				&bytes);

	rdma_unit_test(unit_test_action, src_cnts, bytes);

	return count;
}
static CLASS_ATTR(rdma_debug, 0644, rdma_debug_show, rdma_debug_store);

static ssize_t timeout_ms_show(struct class *c, struct class_attribute *attr,
		char *b)
{
	int len = 0, length = 0;
	char *p = b;

	len = sprintf(p, "%u\n", jiffies_to_msecs(dma_timeout_jiffies));
    LPLUS;
	return length;
}

static ssize_t timeout_ms_store(struct class *c, struct class_attribute *attr,
		const char *b, size_t count)
{
	unsigned int t_ms;

	if (*b != '0' || (b[1] != 'x' && b[1] != 'X'))
		sscanf(b, "%u", &t_ms);
	else if (b[1] == 'x')
		sscanf(b, "0x%x", &t_ms);
	else
		sscanf(b, "0X%x", &t_ms);

	dma_timeout_jiffies = msecs_to_jiffies(t_ms);
	return count;
}
static CLASS_ATTR(timeout_ms, 0644, timeout_ms_show, timeout_ms_store);

void rdma_sysfs_exit(void)
{
	class_remove_file(rdma_class, &class_attr_timeout_ms);
	class_remove_file(rdma_class, &class_attr_rdma_debug);
	class_unregister(rdma_class);
	return;
}

void rdma_sysfs_init(void)
{

	rdma_class = kzalloc(sizeof(struct class), GFP_KERNEL);
	if (!rdma_class)
		return;

	rdma_class->name = "rdma";
	rdma_class->class_release = rdma_class_release,

	class_register(rdma_class);
	rdma_class_create_file(rdma_class, &class_attr_rdma_debug);
	rdma_class_create_file(rdma_class, &class_attr_timeout_ms);
	return;
}

#endif	/* CONFIG_SYSFS */

/*
 *  Based on arch/arm/kernel/smp.c
 *
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock_sop.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/cache.h>
#include <linux/profile.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/atomic.h>
#include <linux/ptrace.h>
#include <linux/mmu_context.h>

#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/processor.h>
#include <asm/tlbflush.h>
#include <asm/localtimer.h>

/* Required for DMA coherent API's*/
#include <linux/dma-mapping.h>

/* Required for Proc files */
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <asm/mach-types.h>
#include <asm/hardware/gic.h>
#include <mach/smp.h>

#ifdef CONFIG_PM
#include <mach/pm.h>
#endif /* CONFIG_PM */

/*
 * Let's use the same stack size as done in VxWorks for
 * CPU1 bring-up
 */
#define DATA_PLANE_STACK_SIZE (8*1024)

static void (*volatile dp_mainloop)(void);

/*
 * as from 2.5, kernels no longer have an init_tasks structure
 * so we need some other way of telling a new secondary core
 * where to place its SVC stack
 */
struct secondary_data secondary_data;

/* Parent proc dir for all the amp offloaded tasks */
struct proc_dir_entry *amp_proc_dir;
EXPORT_SYMBOL(amp_proc_dir);

/*TTBR values of CPU1*/
unsigned long ttbr0_cpu1 = 0x0FFFFFFF;
unsigned long ttbr1_cpu1 = 0x0FFFFFFF;
unsigned long ttbcr_cpu1 = 0x0FFFFFFF;

/*
 * Set the exception handler stack as done for CPU0 ;
 * But, not sure if this change alone is sufficient ;
 */
struct amp_stack {
	u32 irq[3];
	u32 abt[3];
	u32 und[3];
} ____cacheline_aligned;

static struct amp_stack amp_stacks[1];

/*
 * A flag used to indicate if CPU1 is UP
 * This should be replaced by lock if needed
 */
int amp_cpu;
EXPORT_SYMBOL(amp_cpu);

/*
 * Called from platform specific assembly code, this is the
 * secondary CPU entry point.
 */
asmlinkage void secondary_start_kernel(void);

/*
 * Set the values of TTBRs from CPU0 to CPU1
 */
void set_secondary_ttbr(void)
{

	__asm__("mcr p15, 0, %0, c2, c0, 0" : : "r" (ttbr0));
	__asm__("mcr p15, 0, %0, c2, c0, 1" : : "r" (ttbr1));
	__asm__("mcr p15, 0, %0, c2, c0, 2" : : "r" (ttbcr));

	__asm__("mrc p15, 0, %0, c2, c0, 0" : "=r" (ttbr0_cpu1));
	__asm__("mrc p15, 0, %0, c2, c0, 1" : "=r" (ttbr1_cpu1));
	__asm__("mrc p15, 0, %0, c2, c0, 2" : "=r" (ttbcr_cpu1));
}

/*
 * Set-up the exception stack for CPU1
 */
void amp_cpu_init(void)
{
	struct amp_stack *stk = &amp_stacks[0];

	/*
	 * setup stacks for re-entrant exception handlers
	 */
	__asm__ (
	"msr    cpsr_c, %1\n\t"
	"add    sp, %0, %2\n\t"
	"msr    cpsr_c, %3\n\t"
	"add    sp, %0, %4\n\t"
	"msr    cpsr_c, %5\n\t"
	"add    sp, %0, %6\n\t"
	"msr    cpsr_c, %7"
	    :
	    : "r" (stk),
	      "I" (PSR_F_BIT | PSR_I_BIT | IRQ_MODE),
	      "I" (offsetof(struct amp_stack, irq[0])),
	      "I" (PSR_F_BIT | PSR_I_BIT | ABT_MODE),
	      "I" (offsetof(struct amp_stack, abt[0])),
	      "I" (PSR_F_BIT | PSR_I_BIT | UND_MODE),
	      "I" (offsetof(struct amp_stack, und[0])),
	      "I" (PSR_F_BIT | PSR_I_BIT | SVC_MODE)
	    : "r14");
}

/* this is the platfrom init func for CPU1 */
void __cpuinit amp_platform_secondary_init(void)
{
	trace_hardirqs_off();

	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	if (machine_is_cns3420vb())
		gic_cpu_init(0, __io(CNS3XXX_TC11MP_GIC_CPU_BASE_VIRT));

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();

}

enum ipi_msg_type {
	IPI_HARDEVENT,
	IPI_WAKEUP_HIEVENT,
	IPI_WAKEUP_EVENT,
	IPI_CALL_FUNC_SINGLE,
};

/*
 * This is the IPI data strucutre used during message passing between Cores
 */
struct ipi_data {
	raw_spinlock_smp_t lock;
	unsigned long ipi_count;
	unsigned long bits;
};

/*
 * The IPI_WAKEUP_EVENT can be considered as a generic
 * remote processor event similar to device interrupt.
 *
 * IPI wakeup event handler has to return as fast as possible
 * and not meant for any packet processing.
 */
static void (*ipi_wakeup_event_handler[2])(void);
static void (*ipi_wakeup_hievent_handler[2])(void);

static void (*ipi_hardevent_handler[2])(void);

/*
 * Shared ipi-data initialization
 */
static struct ipi_data ipi_data[] = {
	{(raw_spinlock_smp_t)__RAW_SPIN_LOCK_UNLOCKED_SMP, 0, 0},
	{(raw_spinlock_smp_t)__RAW_SPIN_LOCK_UNLOCKED_SMP, 0, 0}
};


void install_ipi_event_handler(void (*func)(void), int cpu)
{
	ipi_wakeup_event_handler[cpu] = func;
}
EXPORT_SYMBOL(install_ipi_event_handler);

void install_ipi_hievent_handler(void (*func)(void), int cpu)
{
	ipi_wakeup_hievent_handler[cpu] = func;
}
EXPORT_SYMBOL(install_ipi_hievent_handler);

void install_ipi_hardevent_handler(void (*func)(void), int cpu)
{
	ipi_hardevent_handler[cpu] = func;
}
EXPORT_SYMBOL(install_ipi_hardevent_handler);


/*
 * Modified version of send_ipi_message.
 * Here we bother about sending IPI to only one CPU at a time.
 */
static void send_ipi_cpu_message(int cpu, enum ipi_msg_type msg)
{
/*	unsigned long flags; */
	struct ipi_data *ipi = &ipi_data[cpu];

	/*local_irq_save(flags);*/ /* optimize to no op? */

	__raw_spin_lock_smp(&ipi->lock);
	ipi->bits |= 1 << msg;
	__raw_spin_unlock_smp(&ipi->lock);

	/*
	 * Call the platform specific cross-CPU call function.
	 */
	smp_cross_call(cpumask_of(cpu)); /* optimize the mask to cpu number? */

	/*local_irq_restore(flags);*/ /* optimize to no op ? */
}

/*
 * Main handler for inter-processor interrupts
 */
asmlinkage void __exception do_IPI(struct pt_regs *regs)
{
	unsigned int cpu = hard_smp_processor_id();
	struct ipi_data *ipi;
	struct pt_regs *old_regs = set_irq_regs(regs);

	ipi = &ipi_data[cpu];
	ipi->ipi_count++;

	for (;;) {
		unsigned long msgs;

		__raw_spin_lock_smp(&ipi->lock);
		msgs = ipi->bits;
		ipi->bits = 0;
		__raw_spin_unlock_smp(&ipi->lock);

		if (!msgs)
			break;

		do {
			unsigned nextmsg; /* Optimize ? */
			nextmsg = msgs & -msgs;
			msgs &= ~nextmsg;
			nextmsg = ffz(~nextmsg);

			switch (nextmsg) {

			case IPI_HARDEVENT:
				ipi_hardevent_handler[cpu]();
				break;

			case IPI_WAKEUP_HIEVENT:
				ipi_wakeup_hievent_handler[cpu]();
				break;

			case IPI_WAKEUP_EVENT:
				ipi_wakeup_event_handler[cpu]();
				break;

			case IPI_CALL_FUNC_SINGLE:
				generic_amp_call_function_single_interrupt();
				break;

			default:
				printk(KERN_CRIT "CPU%u: Unknown IPI message"
				       "0x%x\n",
				       cpu, nextmsg);
				break;
			}
		} while (msgs);
	}

	set_irq_regs(old_regs);
}

void arch_send_call_function_single_ipi(int cpu)
{
	send_ipi_cpu_message(cpu, IPI_CALL_FUNC_SINGLE);
}

void smp_send_reschedule(int cpu)
{
	BUG();
}

void smp_send_stop(void)
{
	BUG();
}

/*
 *  * not supported here
 *   */
int setup_profiling_timer(unsigned int multiplier)
{
	return -EINVAL;
}


/*
 * This function brings the CPU1 out of WFI
 */
int __cpuinit __cpu_up_amp(unsigned int cpu)
{
	/*TODO: This needs to be removed later*/
	struct task_struct *idle = NULL;
	pgd_t *pgd;
	pmd_t *pmd;
	int ret;
	int cpunum = hard_smp_processor_id();

	pgd = pgd_alloc(&init_mm);
	pmd = pmd_offset(pgd + pgd_index(PHYS_OFFSET), PHYS_OFFSET);
	*pmd = __pmd((PHYS_OFFSET & PGDIR_MASK) |
		     PMD_TYPE_SECT | PMD_SECT_AP_WRITE);

	secondary_data.stack = (void *)kmalloc(DATA_PLANE_STACK_SIZE,
			GFP_KERNEL);

	if (secondary_data.stack == NULL) {
		printk(KERN_EMERG "CPU%d: Stack Allocation failed for Secondary"
				" Core\n", cpunum);
		return -ENOMEM;
	}

	secondary_data.stack += DATA_PLANE_STACK_SIZE;
	secondary_data.pgdir = virt_to_phys(pgd);
	wmb();

	/* creating /proc/amp dir for dpc proc entries */
	amp_proc_dir = proc_mkdir("amp", NULL);

	/*TODO: idle is not required, to be removed later*/
	ret = boot_secondary(cpu, idle);
	if (ret == 0) {
		unsigned long timeout;

		timeout = jiffies + HZ;
		while (time_before(jiffies, timeout)) {
			if (amp_cpu == 1)
				break;

			udelay(10);
			barrier();
		}

		if (amp_cpu != 1)
			ret = -EIO;
	}

	return ret;
}


void (*dp_entry_func)(void);
EXPORT_SYMBOL(dp_entry_func);

unsigned int cpu2_proc;

/*
 * This is the secondary CPU boot entry.  We're using this CPUs
 * idle thread stack, but a set of temporary page tables.
 */
asmlinkage void secondary_start_kernel(void)
{
	unsigned int cpu = 0;
	unsigned long tmp = 0;

	cpu = hard_smp_processor_id();
	set_secondary_ttbr();
	amp_cpu_init();

	printk(KERN_INFO "CPU%u: Booted secondary processor\n", cpu);
	local_flush_tlb_all();
	amp_platform_secondary_init();
	amp_cpu = 1; /*This could be replaced by a lock in future*/

	local_irq_enable();
	local_fiq_enable();

#ifndef CONFIG_CNS3XXX_PSE_WOL
	while (1)
		cpu2_proc++;
#else
dp_wfi_state:
	printk(KERN_INFO "<0> DP-core moving to WFI\n");
#ifdef CONFIG_PM
	cnsx_dp_power_event = CNSX_DP_RESUME;
	cnsx_dp_power_event_ack = CNSX_DP_PWR_EVENT_ACK;
#endif /* CONFIG_PM */


	/* Go to WFI state */
	asm volatile("mcr p15, 0, %0, c7, c10, 5\n" \
		     "mcr p15, 0, %0, c7, c10, 4\n" \
		     "mcr p15, 0, %0, c7, c0, 4\n" : : "r"(tmp));

	printk(KERN_INFO "CPU%u DP-core coming out of WFI\n", cpu);

	/* get DP core into action */
	dp_entry_func();

	/* moving to suspend mode */
	goto dp_wfi_state;

	return;
#endif /* CONFIG_CNS3XXX_PSE_WOL */
}

void send_ipi_wakeup_event(int cpu)
{
	send_ipi_cpu_message(cpu, IPI_WAKEUP_EVENT);
}
EXPORT_SYMBOL(send_ipi_wakeup_event);

void send_ipi_wakeup_hievent(int cpu)
{
	send_ipi_cpu_message(cpu, IPI_WAKEUP_HIEVENT);
}
EXPORT_SYMBOL(send_ipi_wakeup_hievent);

void send_ipi_hardevent(int cpu)
{
	send_ipi_cpu_message(cpu, IPI_HARDEVENT);
}
EXPORT_SYMBOL(send_ipi_hardevent);

void start_dp_mainloop(void *task)
{
	/* Fixed : Send IPI from CPC to DPC with no wait and CPC does'nt get
	 * stuck waiting for lock release from DPC
	 */
	dp_mainloop = task;
	dp_mainloop();
}
EXPORT_SYMBOL(start_dp_mainloop);

int get_ipi_rx_count(int cpu)
{
	return ipi_data[cpu].ipi_count;
}
EXPORT_SYMBOL(get_ipi_rx_count);

/*
 * amp.c:
 * 	Generic helper function for amp IPI calls.
 *      Based on kernel/smp.c
 */
#include <linux/rcupdate.h>
#include <linux/proc_fs.h>
#include <linux/rculist.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/cpu.h>

extern void csd_lock_wait(struct call_single_data *);
extern void csd_lock(struct call_single_data *);
extern void csd_unlock(struct call_single_data *);

static
void amp_generic_exec_single(int cpu, struct call_single_data *data, 
			     int wait)
{
	arch_send_call_function_single_ipi(cpu);
	if (wait)
		csd_lock_wait(data);
}


/* 
 * Creating two instance one each for a core 
 * DEFINE_PER_CPU is not supported in SOP platform. Also here it is not useful 
 * since the IPI traffic is unidirectional from DP to CP execept at boot time.
 */
static struct call_single_data csd_data[2];

/*
 * amp_call_function_single - Run a function on a specific CPU
 * @func: The function to run. This must be fast and non-blocking.
 * @info: An arbitrary pointer to pass to the function.
 * @wait: If true, wait until function has completed on other CPUs.
 *
 * Returns 0 on success, else a negative status code. Note that @wait
 * will be implicitly turned on in case of allocation failures, since
 * we fall back to on-stack allocation.
 */
int amp_call_function_single(int cpu, void (*func) (void *info), void *info,
                             int wait)
{
	int this_cpu = hard_smp_processor_id();
	struct call_single_data *data;

	if (cpu == this_cpu) {
		printk("XXX: amp cross call on native CPU\n");
		func(info);
		return 0;
	}
		
	data = &csd_data[cpu];
	
	csd_lock(data);
	
	data->func = func;
	data->info = info;
	smp_wmb();
	amp_generic_exec_single(cpu, data, wait);

	return 0; 
}

EXPORT_SYMBOL(amp_call_function_single);

/* Function that executes the registered handler for IPI */
void generic_amp_call_function_single_interrupt(void)
{
	struct call_single_data *data;
	int this_cpu = hard_smp_processor_id();

	data = &csd_data[this_cpu];

	smp_rmb();
	data->func(data->info);

	csd_unlock(data);	
}

void dp_csd_unlock(void)
{
	struct call_single_data *data;
	int this_cpu = hard_smp_processor_id();

	data = &csd_data[this_cpu];
	csd_unlock(data);	
}

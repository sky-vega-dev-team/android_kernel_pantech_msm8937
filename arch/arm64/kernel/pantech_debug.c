/* arch/arm/mach_msm/pantech_debug.c
 *
 * Copyright (C) 2012 PANTECH, Co. Ltd.
 * based on drivers/misc/apanic.c
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.      See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
//#include <mach/msm_iomap.h>
//#include <asm/unwind.h>
#include <asm/stacktrace.h>

#include <mach/pantech_debug.h>

int pantech_debug_enable = 0;

struct pantech_debug_log {
#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  
	atomic_t idx_sched[CONFIG_NR_CPUS];
	struct sched_log sched[CONFIG_NR_CPUS][SCHED_LOG_MAX];
#endif

#ifdef CONFIG_PANTECH_DEBUG_IRQ_LOG  
	atomic_t idx_irq[CONFIG_NR_CPUS];
	struct irq_log irq[CONFIG_NR_CPUS][SCHED_LOG_MAX];
#endif

#ifdef CONFIG_PANTECH_DEBUG_DCVS_LOG  
	atomic_t dcvs_log_idx ;
	struct dcvs_debug dcvs_log[DCVS_LOG_MAX] ;	
#endif

#ifdef CONFIG_PANTECH_DEBUG_RPM_LOG  
	atomic_t rpm_log_idx ;
	struct rpm_debug rpm_set_log[RPM_LOG_MAX] ;	
#endif

};
struct pantech_debug_log *pantech_dbg_log;

void pantech_unwind_backtrace(unsigned int *array)
{
	struct stackframe frame;
	register unsigned long current_sp asm ("sp");
	unsigned int max_depth=PANTECH_MAX_STACKFRAME_DEPTH;
	unsigned int depth=0;
	unsigned int skip_1_depth=0;

	frame.fp = (unsigned long)__builtin_frame_address(0);
	frame.sp = current_sp;
	frame.lr = (unsigned long)__builtin_return_address(0);
	frame.pc = (unsigned long)pantech_unwind_backtrace;

	while (1) {
		int urc;

		urc = unwind_frame(&frame);
		if (urc < 0)
			break;

		if (!skip_1_depth++)
			continue;

		if (depth > max_depth)
			break;

		//save frame.pc(caller)
		array[depth]=frame.pc;
		
		depth++;
	}
}


void pantech_get_stackframe(unsigned int *array)
{
	pantech_unwind_backtrace(array);
	barrier();
}

extern void set_pantech_dbg_buf_info(unsigned long long* addr, unsigned int size);

int __init pantech_debug_init(void)
{
	int i;
	struct pantech_debug_log *vaddr;
	unsigned long long paddr;
	unsigned int size;	

    if(!pantech_debug_enable)
		return 0;

	//size = sizeof(struct pantech_debug_log);
	size = PANTECH_DBG_LOG_BUF_SIZE;
	//paddr = allocate_contiguous_ebi_nomap(size, SZ_4K);
	//vaddr = ioremap_nocache(paddr, size);

	vaddr = dma_alloc_coherent(NULL, size, &paddr, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;

	// P14994
	set_pantech_dbg_buf_info(paddr, size);

	pr_info("[PANTECH_DBG] %s: vaddr=0x%x paddr=0x%x size=0x%x "
			"sizeof(struct pantech_debug_log)=0x%x\n", __func__,
			(unsigned int)vaddr, paddr, size,
			sizeof(struct pantech_debug_log));

	if ((vaddr == NULL) || (sizeof(struct pantech_debug_log) > size)) {
		pr_info("%s: ERROR! init failed!\n", __func__);
		return -EFAULT;
	}

	memset(vaddr, 0, size);
	for (i = 0; i < CONFIG_NR_CPUS; i++) {
#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  //p14291_121102
		atomic_set(&(vaddr->idx_sched[i]), -1);
#endif
#ifdef CONFIG_PANTECH_DEBUG_IRQ_LOG  //p14291_121102
		atomic_set(&(vaddr->idx_irq[i]), -1);
#endif
	}

#ifdef CONFIG_PANTECH_DEBUG_DCVS_LOG  //p14291_121102
	atomic_set(&(vaddr->dcvs_log_idx), -1);
#endif

#ifdef CONFIG_PANTECH_DEBUG_RPM_LOG  //p14291_121102
	atomic_set(&(vaddr->rpm_log_idx), -1);
#endif

	pantech_dbg_log = vaddr;
	pr_info("[PANTECH_DBG] %s: init done\n", __func__);

	printk(KERN_INFO "Android kernel / Modem panic handler initialized \n");
	return 0;
}

#ifdef CONFIG_PANTECH_DEBUG_SCHED_LOG  
void __pantech_debug_task_sched_log(int cpu, struct task_struct *task,
						char *msg)
{
	unsigned i;

	if (!pantech_dbg_log)
		return;

	if (!task && !msg)
		return;

	i = atomic_inc_return(&(pantech_dbg_log->idx_sched[cpu]))
		& (SCHED_LOG_MAX - 1);
	pantech_dbg_log->sched[cpu][i].time = cpu_clock(cpu);
	if (task) {
		strncpy(pantech_dbg_log->sched[cpu][i].comm, task->comm,
			sizeof(pantech_dbg_log->sched[cpu][i].comm));
		pantech_dbg_log->sched[cpu][i].pid = task->pid;
	} else {
		strncpy(pantech_dbg_log->sched[cpu][i].comm, msg,
			sizeof(pantech_dbg_log->sched[cpu][i].comm));
		pantech_dbg_log->sched[cpu][i].pid = -1;
	}
}
void pantech_debug_task_sched_log_short_msg(char *msg)
{
	__pantech_debug_task_sched_log(smp_processor_id(), NULL, msg);
}
void pantech_debug_task_sched_log(int cpu, struct task_struct *task)
{
	__pantech_debug_task_sched_log(cpu, task, NULL);
}
#endif

#ifdef CONFIG_PANTECH_DEBUG_IRQ_LOG  //p14291_121102
void pantech_debug_irq_sched_log(unsigned int irq, void *fn, int en, unsigned long long start_time)
{
	int cpu = smp_processor_id();
	unsigned i;

	if (!pantech_dbg_log)
		return;

	i = atomic_inc_return(&(pantech_dbg_log->idx_irq[cpu]))
		& (SCHED_LOG_MAX - 1);
	pantech_dbg_log->irq[cpu][i].time = start_time;
	pantech_dbg_log->irq[cpu][i].end_time = cpu_clock(cpu);
	pantech_dbg_log->irq[cpu][i].elapsed_time =
	pantech_dbg_log->irq[cpu][i].end_time - start_time;
	pantech_dbg_log->irq[cpu][i].irq = irq;
	pantech_dbg_log->irq[cpu][i].fn = (void *)fn;
	pantech_dbg_log->irq[cpu][i].en = en;
	pantech_dbg_log->irq[cpu][i].preempt_count = preempt_count();
	pantech_dbg_log->irq[cpu][i].context = &cpu;
}
#endif

#ifdef CONFIG_PANTECH_DEBUG_DCVS_LOG  //p14291_121102
void pantech_debug_dcvs_log(int cpu_no, unsigned int prev_freq,
						unsigned int new_freq)
{
	unsigned int i;
	if (!pantech_dbg_log)
		return;

	i = atomic_inc_return(&(pantech_dbg_log->dcvs_log_idx)) 
		& (DCVS_LOG_MAX - 1);
	pantech_dbg_log->dcvs_log[i].cpu_no = cpu_no;
	pantech_dbg_log->dcvs_log[i].prev_freq = prev_freq;
	pantech_dbg_log->dcvs_log[i].new_freq = new_freq;
	pantech_dbg_log->dcvs_log[i].time = cpu_clock(cpu_no);
}
#endif

#ifdef CONFIG_PANTECH_DEBUG_RPM_LOG  //p14291_121102
/*
void pantech_debug_rpm_log(unsigned int set, unsigned int id, unsigned int value)
{
	int cpu = smp_processor_id();
	unsigned int i;
	
	if (!pantech_dbg_log)
		return;

	i = atomic_inc_return(&(pantech_dbg_log->rpm_log_idx)) 
		& (RPM_LOG_MAX - 1);
	pantech_dbg_log->rpm_set_log[i].set = set;
	pantech_dbg_log->rpm_set_log[i].id = id;
	pantech_dbg_log->rpm_set_log[i].value = value;
	pantech_dbg_log->rpm_set_log[i].time = cpu_clock(cpu);
}
*/
void pantech_debug_rpm_ack_log(unsigned int msgid, int errno)
{
    int cpu = raw_smp_processor_id();
    unsigned int i;
    
    if (!pantech_dbg_log)
        return;

        i = atomic_inc_return(&(pantech_dbg_log->rpm_log_idx)) 
            & (RPM_LOG_MAX - 1);

        pantech_dbg_log->rpm_set_log[i].msgid = msgid;
        pantech_dbg_log->rpm_set_log[i].set = 0;
        pantech_dbg_log->rpm_set_log[i].restype = 0;
        pantech_dbg_log->rpm_set_log[i].resid = 0;
        pantech_dbg_log->rpm_set_log[i].key = 0;
        pantech_dbg_log->rpm_set_log[i].value = 0;
        pantech_dbg_log->rpm_set_log[i].time = cpu_clock(cpu);
        pantech_dbg_log->rpm_set_log[i].errno = errno;
/*
        printk(KERN_INFO "msgid : %d, errno : %d\n", pantech_dbg_log->rpm_set_log[i].msgid
            , pantech_dbg_log->rpm_set_log[i].errno);
*/
    
}

void pantech_debug_rpm_req_log(struct msm_rpm_request_debug *cdata)
{
    int cpu = raw_smp_processor_id();
    unsigned int i;
    int j;
    
    if (!pantech_dbg_log)
        return;

    for(j = 0; j< cdata->write_idx; j++){

        if(!cdata->kvp[j].valid)
            continue;
        
        i = atomic_inc_return(&(pantech_dbg_log->rpm_log_idx)) 
            & (RPM_LOG_MAX - 1);

        pantech_dbg_log->rpm_set_log[i].msgid = cdata->msg_hdr.msg_id;
        pantech_dbg_log->rpm_set_log[i].set = cdata->msg_hdr.set;
        pantech_dbg_log->rpm_set_log[i].restype = cdata->msg_hdr.resource_type;
        pantech_dbg_log->rpm_set_log[i].resid = cdata->msg_hdr.resource_id;
        pantech_dbg_log->rpm_set_log[i].key = cdata->kvp[j].key;
        pantech_dbg_log->rpm_set_log[i].value = *((unsigned int*)cdata->kvp[j].value);
        pantech_dbg_log->rpm_set_log[i].time = cpu_clock(cpu);
        pantech_dbg_log->rpm_set_log[i].errno = 0;
/*
        printk(KERN_INFO "msgid : %d, set : %d, type : %d, id : %d, key : %d, value : %d\n", pantech_dbg_log->rpm_set_log[i].msgid
            , pantech_dbg_log->rpm_set_log[i].set, pantech_dbg_log->rpm_set_log[i].restype, pantech_dbg_log->rpm_set_log[i].resid
            , pantech_dbg_log->rpm_set_log[i].key, pantech_dbg_log->rpm_set_log[i].value);
*/

    }
}

#endif


module_init(pantech_debug_init);

MODULE_AUTHOR("HeeCheol Kim <kim.heecheol@pantech.com>");
MODULE_DESCRIPTION("PANTECH Debug Module");
MODULE_LICENSE("GPL");


/* drivers/misc/pantech_apanic.c
 *
 * Copyright (C) 2011 PANTECH, Co. Ltd.
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

/*---------------------------------------------------------------------------------------------------
 ** Common Header Include
 **-------------------------------------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/cacheflush.h>
#include <mach/pantech_apanic.h>
#include <linux/input.h>
#include <soc/qcom/smd.h>
#include <mach/pantech_sys_info.h>
#include <mach/pantech_sys.h>

/*---------------------------------------------------------------------------------------------------
 ** Define
 **-------------------------------------------------------------------------------------------------*/
#define PANIC_MAGIC    0xDAEDDAED
#define PHDR_VERSION   0x01

struct fuel_gauge_info{
	int percent_soc;
	unsigned int cable_type;
};

typedef  unsigned long int  uint32;      /* Unsigned 32 bit value */
typedef  unsigned char      uint8;       /* Unsigned 8  bit value */

typedef struct {
	unsigned int partition_size_;
	char   partition_name_[ 8 ];
}partition_info_type;

typedef struct {
	uint32 version_;
	char   model_name_    [ 16 ];
	char   binary_version_[ 16 ];
	uint32 fs_version_;
	uint32 nv_version_;
	char   build_date_    [ 16 ];
	char   build_time_    [ 16 ];

	uint32 boot_loader_version_;
	uint32 boot_section_id_[4];

	uint32              efs_size_;
	uint32              partition_num_;
	partition_info_type partition_info_[ 6 ];

	uint32 FusionID;
	uint8  Imei[15];
	/* F_PANTECH_SECBOOT - DO NOT erase this comment! */
	char   secure_magic_[ 7 ];
	char   sub_binary_version_[8];

	uint32 preload_checksum; 
	uint8  reserved_2[ 42 ];
} __attribute__((packed)) phoneinfo_type;
/*
   PANTECH Shared memory region
 */
struct pantech_panic_info {
	unsigned int magic;
	/* log info - each log pointer*/
	struct msg_log msg;

	/* Dump mode setting info */
	int in_panic;
	int ramdump; /* 0:Log, 1:Log & RAM dump */
	int usbdump; /* 0:Dump via emmc 1: Dump via USB*/

	/* Force crash */
	int errortest;

	/* Fuel gauge info - percent_soc , cable_type*/
	struct fuel_gauge_info fuel_gauge;

	/* Phone info - struct phoneinfo_type */
	phoneinfo_type phoneinfo_buff_ptr;

	/*Schedule Info - index, time, task name, pid*/
	atomic_t sched_idx[CONFIG_NR_CPUS];
	struct sched_log sched[CONFIG_NR_CPUS][SCHED_LOG_MAX];

	/*IRQ Info - irq number, start time, elapsed time, irqs flag, */
	atomic_t irq_idx[CONFIG_NR_CPUS];
	struct irq_log irq[CONFIG_NR_CPUS][SCHED_LOG_MAX];
};
/*---------------------------------------------------------------------------------------------------
 ** extern function declare
 **-------------------------------------------------------------------------------------------------*/
extern pantech_log_header *get_pantech_klog_dump_address(void);
//extern pantech_log_header *get_pantech_logcat_dump_address(void);

/*---------------------------------------------------------------------------------------------------
 ** global / static variable define
 **-------------------------------------------------------------------------------------------------*/
static pantech_log_header *crash_buf_header = NULL;

/*---------------------------------------------------------------------------------------------------
 ** static function define
 **-------------------------------------------------------------------------------------------------*/
#define POS(x) (x > 0 ? x : 0)
#define WRITE_LOG(...) \
	do{\
		if(bufsize != 0) { \
			n = snprintf(s, POS(bufsize), __VA_ARGS__); \
			s+=n; \
			total +=n; \
			bufsize-=n;\
		}\
	}while(0)

static void apanic_get_mmu_info(void)
{
/*
	int  bufsize = MMU_SCRIPT_BUF_SIZE, n = 0,total=0;
	char *s;
	unsigned int mmu_transbase0,mmu_transbase1;
	unsigned int mmu_dac;//,mmu_control;
	unsigned int mmu_prrr,mmu_nmrr;
	unsigned int mmu_transbase1;
	unsigned int mmu_tcr,mmu_mair;
	unsigned int mmu_par,mmu_sctlr;

	asm("mrs %0,TTBR1_EL1" : "=r" (mmu_transbase1));
	asm("mrs %0,TCR_EL1" : "=r" (mmu_tcr));
	asm("mrs %0,MAIR_EL1" : "=r" (mmu_mair));
	asm("mrs %0,PAR_EL1" : "=r" (mmu_par));
	asm("mrs %0,SCTLR_EL1" : "=r" (mmu_sctlr));

	//asm("mrc p15,0,%0,c1,c0,0" : "=r" (mmu_control));
	//asm("mrc p15,0,%0,c2,c0,0" : "=r" (mmu_transbase0));
	//asm("mrc p15,0,%0,c3,c0,0" : "=r" (mmu_dac));
	//asm("mrc p15,0,%0,c2,c0,1" : "=r" (mmu_transbase1));
	//asm("mrc p15,0,%0,c10,c2,0" : "=r" (mmu_prrr));
	//asm("mrc p15,0,%0,c10,c2,1" : "=r" (mmu_nmrr));
	s =(char *)crash_buf_header->mmu_cmm_script;	

	//printk(KERN_DEBUG "Crash key : %d , value : %d\n", code, value);

	printk(KERN_INFO "Data.Set SPR:0x30201 %%Quad 0x%X\n",mmu_transbase1);
	printk(KERN_INFO "Data.Set SPR:0x30202 %%Quad 0x%16X\n",mmu_tcr);

	WRITE_LOG("Data.Set SPR:0x30201 %%Quad 0x%X\n",mmu_transbase1);
	WRITE_LOG("Data.Set SPR:0x30202 %%Quad 0x%16X\n",mmu_tcr);
	WRITE_LOG("Data.Set SPR:0x30A20 %%Quad 0x%16X\n",mmu_mair);
	WRITE_LOG("Data.Set SPR:0x30A30 %%Quad 0x%16X\n",mmu_par);
	WRITE_LOG("Data.Set SPR:0x30100 %%Quad 0x%16X\n",mmu_sctlr);
	WRITE_LOG("MMU.SCAN\n");
	WRITE_LOG("MMU.ON\n");
*/
//	WRITE_LOG("\n\n\n");  /* 32bit boundary */

	//WRITE_LOG("PER.S C15:0x1 %%LONG 0x%X\n",mmu_control);
	//WRITE_LOG("PER.S C15:0x2 %%LONG 0x%X\n",mmu_transbase0);
	//WRITE_LOG("PER.S C15:0x3 %%LONG 0x%X\n",mmu_dac);
	//WRITE_LOG("PER.S C15:0x102 %%LONG 0x%X\n",mmu_transbase1);
	//WRITE_LOG("PER.S C15:0x2A %%LONG 0x%X\n",mmu_prrr);
	//WRITE_LOG("PER.S C15:0x12A %%LONG 0x%X\n",mmu_nmrr);
	//WRITE_LOG("MMU.SCAN\n");
	//WRITE_LOG("MMU.ON\n");
	//WRITE_LOG("\n\n\n");  /* 32bit boundary */

	//crash_buf_header->mmu_cmm_size = total;
}

// p15060
extern int forceddump;
void pantech_force_dump_key(unsigned int code, int value)
{
    #define KEY_VOL_OK  250

	static unsigned int step = 0;
	static int usbdump = 0;
	//printk(KERN_EMERG "Crash key : %d , value : %d\n", code, value);

	usbdump = (int)IS_SYS_USBDUMP_MODE;
	//printk(KERN_EMERG "pantech_force_dump_key - code : %d, value : %d, usbdump : %d\n", code, value, usbdump);
	if( (usbdump == 0) || ( (value !=1) && (code != KEY_VOL_OK)) || (1 != forceddump) ) //temp block for forcedump
	{
		step = 0;
		return;
	}

	switch(step)
	{
		case 0:
			if(code == KEY_POWER)
				step = 1;
			else 
				step = 0;
			break;
		case 1:
			if((code == KEY_VOL_OK) &&  (value ==1))
			    step = 2;
			else if((code == KEY_VOL_OK) &&  (value ==0))
			    step = 1;
			else 
				step = 0;
			break;
		case 2:
			if((code == KEY_VOL_OK) &&  (value ==1))
			    step = 3;
			else if((code == KEY_VOL_OK) &&  (value ==0))
			    step = 2;
			else 
				step = 0;
			break;			
		case 3:
			if((code == KEY_VOL_OK) &&  (value ==1))
				step = 4;
			else if((code == KEY_VOL_OK) &&  (value ==0))
				step = 3;
			else 
				step = 0;
			break;
		case 4:
			if((code == KEY_VOL_OK) &&  (value ==1))
				step = 5;
			else if((code == KEY_VOL_OK) &&  (value ==0))
				step = 4;
			else 
				step = 0;
			break;		
		case 5:
			if((code == KEY_VOL_OK) &&  (value ==1))
			    step = 6;
			else if((code == KEY_VOL_OK) &&  (value ==0))
			    step = 5;
			else 
				step = 0;
			break;
		case 6:
			if(code == KEY_VOLUMEUP)
				panic("linux- Force dump key");
			else if((code == KEY_VOL_OK) &&  (value ==0))
			    step = 6;
			else
				step = 0;		
	}
}
EXPORT_SYMBOL(pantech_force_dump_key);

static int apanic_logging(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk(KERN_EMERG "pantech_apanic: apanic_logging start\n");

#ifdef CONFIG_PREEMPT
	/* Ensure that cond_resched() won't try to preempt anybody */
	__preempt_count_add(PREEMPT_ACTIVE);
#endif

	touch_softlockup_watchdog();

	if(crash_buf_header != NULL)
	{
		crash_buf_header->magic = PANIC_MAGIC;
		crash_buf_header->version = PHDR_VERSION;
		apanic_get_mmu_info();
	}
	else
		printk(KERN_ERR "apanic_logging : crash_buf_header is not initialized!\n");

	flush_cache_all();

#ifdef CONFIG_PREEMPT
	__preempt_count_sub(PREEMPT_ACTIVE);
#endif

	printk(KERN_EMERG "pantech_apanic: apanic_logging end\n");
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call    = apanic_logging,
};


/*---------------------------------------------------------------------------------------------------
 ** non - static function define
 **-------------------------------------------------------------------------------------------------*/
int crash_buf_header_init(void)
{
	unsigned size;

	crash_buf_header = (pantech_log_header *)smem_get_entry(SMEM_ID_VENDOR2, &size, 0, SMEM_ANY_HOST_FLAG);

	if(!crash_buf_header){
		printk(KERN_ERR "pantech_apanic: no available crash buffer , initial failed\n");
		return 0;
	}
	else
	{
		atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	}

	return 1;
}

void set_pantech_dbg_buf_info(unsigned long long addr, unsigned int size)
{
	int ret;

	if(!crash_buf_header)
	{
		ret = crash_buf_header_init();
		if(!ret)
			return ;
	}

	crash_buf_header->pantech_dbg_addr = addr;
	crash_buf_header->pantech_dbg_size = size;
}

/*****************************************************
 * PNANTEH APANIC MODULE INIT
 * **************************************************/
int __init pantech_apanic_init(void)
{
	int ret;

	pantech_log_header *klog_header/*, *logcat_log_header*/;

	klog_header = get_pantech_klog_dump_address();
//	logcat_log_header = get_pantech_logcat_dump_address();

	if(!crash_buf_header)
	{
		ret = crash_buf_header_init();
		if(!ret)
			return 0;
	}

	crash_buf_header->magic=0;
	crash_buf_header->version=0;
	crash_buf_header->klog_buf_address = klog_header->klog_buf_address;
	crash_buf_header->klog_size = klog_header->klog_size;
/*
	crash_buf_header->mlogcat_buf_address = logcat_log_header->mlogcat_buf_address;
	crash_buf_header->mlogcat_w_off = logcat_log_header->mlogcat_w_off;
	crash_buf_header->mlogcat_size = logcat_log_header->mlogcat_size;

	crash_buf_header->slogcat_buf_address = logcat_log_header->slogcat_buf_address;
	crash_buf_header->slogcat_w_off = logcat_log_header->slogcat_w_off;
	crash_buf_header->slogcat_size = logcat_log_header->slogcat_size;

	crash_buf_header->rlogcat_buf_address = logcat_log_header->rlogcat_buf_address;
	crash_buf_header->rlogcat_w_off = logcat_log_header->rlogcat_w_off;
	crash_buf_header->rlogcat_size = logcat_log_header->rlogcat_size;
*/
	printk("pantech_apanic : pantech_log_header initialized success for write to SMEM\n");

	printk(KERN_INFO "apanic_pantech_init\n");
	return 0;
}

//module_init(pantech_apanic_init);
//changed by p11219.  Change timing to initialize pantech_apanic. (module_init -> late_initcall)
//In apq8084, pantech_apanic.c is call before calling logger.c. Because of that, logcat log address is NULL.
late_initcall(pantech_apanic_init); 

MODULE_AUTHOR("Pantech LS1 PART2");
MODULE_DESCRIPTION("Pantech errlogging driver");
MODULE_LICENSE("GPL v2");

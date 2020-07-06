#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <mach/pantech_sys.h>
#include <mach/pantech_restart.h>

static int ramdump = 0;
static int usbdump = 0;
static int errortest = 0;
static int ssrmdm = 0;
static int ssrmdmdump = 0;
static int ssrwifi = 0;
static int ssrwifidump = 0;

int userdump = 0;

// p15060
int forceddump = 1;      // forecd dump default : enable
EXPORT_SYMBOL(forceddump);

typedef enum {
	MODEM_ERR_FATAL=10,
	MODEM_WATCHDOG=11,
	MODEM_EXCP_SWI=12,
	MODEM_EXCP_UNDEF=13,
	MODEM_EXCP_MIS_ALIGN=14,  
	MODEM_EXCP_PAGE_FAULT=15,
	MODEM_EXCP_EXE_FAULT=16,
	MODEM_EXCP_DIV_BY_Z=17,
	LINUX_ERR_FATAL=20,
	LINUX_WATCHDOG=21,
	LINUX_EXCP_SWI=22,
	LINUX_EXCP_UNDEF=23,
	LINUX_EXCP_MIS_ALIGN=24,  
	LINUX_EXCP_PAGE_FAULT=25,
	LINUX_EXCP_EXE_FAULT=26,
	LINUX_EXCP_DIV_BY_Z=27,
	LINUX_FS_CRASH=28,
	SUBSYSTEM_ERR_MAX_NUM
}subsystem_err_type;

// p15060
int pantech_is_usbdump_enabled(void)
{
    return usbdump;
}
EXPORT_SYMBOL(pantech_is_usbdump_enabled);

typedef void (*func_ptr)(void);
static const int _undef_inst = 0xFF0000FF;
static int div0_y=0;
static void diag_error_data_abort(void)
{
	*(int *)0 = 0;
}

static void diag_error_prefetch_abort(void)
{
	func_ptr func;
	func = (func_ptr)0xDEADDEAD;
	(*func)();
}

static void diag_error_undefined_instruction(void)
{
	func_ptr func;
	func = (func_ptr)(&_undef_inst);
	(*func)();
}

static DEFINE_SPINLOCK(state_lock);

static void diag_error_sw_watchdog(void)
{
	unsigned long irqflags;
	unsigned long  value = 0;  

	spin_lock_irqsave(&state_lock, irqflags);

	while(1){
		value = value ? 0 : 1;
	}

	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void diag_error_div0(void)
{
	int x = 1234567;
	int *y;
	y = &div0_y;
	x = x / *y;
}

static void linux_crash_test(int sel)
{
	switch(sel)
	{
		case LINUX_ERR_FATAL:
			BUG_ON(1);
			break;
		case LINUX_WATCHDOG:
			diag_error_sw_watchdog();
			break;
		case LINUX_EXCP_SWI:
			break;
		case LINUX_EXCP_UNDEF:
			diag_error_undefined_instruction();
			break;
		case LINUX_EXCP_MIS_ALIGN:
			break;            
		case LINUX_EXCP_PAGE_FAULT:
			diag_error_data_abort();
			break;
		case LINUX_EXCP_EXE_FAULT:
			diag_error_prefetch_abort();
			break;
		case LINUX_EXCP_DIV_BY_Z:
			diag_error_div0();
			break;
		default:
			break;
	}
}

static ssize_t ramdump_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ramdump = (int)IS_SYS_RAMDUMP_MODE;
	printk(KERN_INFO "%s - ramdump : %d\n", __func__, ramdump);
	return sprintf(buf, "%d\n", ramdump);
}

static ssize_t ramdump_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &ramdump);
	printk(KERN_INFO "%s - ramdump : %d\n", __func__, ramdump);
	if(ramdump == 1)
		SET_SYS_RESET_DUMP_MODE(SYS_RESET_RAMDUMP_MASK, 1);
	else
		SET_SYS_RESET_DUMP_MODE(SYS_RESET_RAMDUMP_MASK, 0);
	return count;
}

static struct kobj_attribute ramdump_attribute =
	__ATTR(ramdump, 0664, ramdump_show, ramdump_store);

static ssize_t usbdump_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	usbdump = (int)IS_SYS_USBDUMP_MODE;
	printk(KERN_INFO "%s - usbdump : %d\n", __func__, usbdump);
	return sprintf(buf, "%d\n", usbdump);
}

static ssize_t usbdump_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &usbdump);
	printk(KERN_INFO "%s - usbdump : %d\n", __func__, usbdump);
	if(usbdump == 1)
		SET_SYS_RESET_DUMP_MODE(SYS_RESET_USBDUMP_MASK, 1);
	else
		SET_SYS_RESET_DUMP_MODE(SYS_RESET_USBDUMP_MASK, 0);
	return count;
}

static struct kobj_attribute usbdump_attribute =
	__ATTR(usbdump, 0664, usbdump_show, usbdump_store);

// p15060
// LOGSET_USERDUMP_DISABLE          -> 0
// LOGSET_USERDUMP_COREDUMP         -> 1
// LOGSET_USERDUMP_FRAME_COREDUMP   -> 2
// LOGSET_USERDUMP_USER_RAMDUMP     -> 3
// LOGSET_USERDUMP_FRAME_RAMDUMP    -> 4
int get_userdump_mode(void)
{
	unsigned int mode=0;

	if(IS_SYS_RESET_MAGIC)		mode = (unsigned int)GET_SYS_USERDUMP_MODE;
	
	printk(KERN_INFO "%s - mode : %08x\n", __func__, mode);	
	return mode;
}

static ssize_t userdump_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	userdump = get_userdump_mode();
	printk(KERN_INFO "%s : , userdump : %d\n", __func__, userdump);
	return sprintf(buf, "%d\n", userdump);
}
static ssize_t userdump_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &userdump);
	printk(KERN_INFO "%s : , userdump : %d\n", __func__, userdump);
	
	SET_SYS_RESET_USERDUMP_MODE(userdump);
	return count;
}
static struct kobj_attribute userdump_attribute =
	__ATTR(userdump, 0664, userdump_show, userdump_store);

static ssize_t errortest_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", errortest);
}

static ssize_t errortest_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &errortest);
	printk(KERN_INFO "%s - errortest : %d\n", __func__, errortest);
	linux_crash_test(errortest);
	return count;
}

static struct kobj_attribute errortest_attribute =
	__ATTR(errortest, 0664, errortest_show, errortest_store);

//SSR
static ssize_t ssrmdm_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssrmdm = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_FLAG);
	printk(KERN_INFO "%s - ssrmdm : %d\n", __func__, ssrmdm);
	return sprintf(buf, "%d\n", ssrmdm);
}

static ssize_t ssrmdm_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &ssrmdm);
	printk(KERN_INFO "%s - ssrmdm : %d\n", __func__, ssrmdm);
	if(ssrmdm == 1)
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_FLAG, 1);
	else
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_FLAG, 0);
	return count;
}

static struct kobj_attribute ssrmdm_attribute =
	__ATTR(ssrmdm, 0664, ssrmdm_show, ssrmdm_store);

static ssize_t ssrmdmdump_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssrmdmdump = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_DUMP_FLAG);
	printk(KERN_INFO "%s - ssrmdmdump : %d\n", __func__, ssrmdmdump);
	return sprintf(buf, "%d\n", ssrmdmdump);
}

static ssize_t ssrmdmdump_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &ssrmdmdump);
	printk(KERN_INFO "%s - ssrmdmdump : %d\n", __func__, ssrmdmdump);
	if(ssrmdmdump == 1)
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_DUMP_FLAG, 1);
	else
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_DUMP_FLAG, 0);
	return count;
}

static struct kobj_attribute ssrmdmdump_attribute =
	__ATTR(ssrmdmdump, 0664, ssrmdmdump_show, ssrmdmdump_store);

static ssize_t ssrwifi_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssrwifi = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_FLAG);
	printk(KERN_INFO "%s - ssrwifi : %d\n", __func__, ssrwifi);
	return sprintf(buf, "%d\n", ssrwifi);
}

static ssize_t ssrwifi_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &ssrwifi);
	printk(KERN_INFO "%s - ssrwifi : %d\n", __func__, ssrwifi);
	if(ssrwifi == 1)
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_FLAG, 1);
	else
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_FLAG, 0);
	return count;
}

static struct kobj_attribute ssrwifi_attribute =
	__ATTR(ssrwifi, 0664, ssrwifi_show, ssrwifi_store);

static ssize_t ssrwifidump_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssrwifidump = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_DUMP_FLAG);
	printk(KERN_INFO "%s - ssrwifidump : %d\n", __func__, ssrwifidump);
	return sprintf(buf, "%d\n", ssrwifidump);
}

static ssize_t ssrwifidump_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &ssrwifidump);
	printk(KERN_INFO "%s - ssrwifidump : %d\n", __func__, ssrwifidump);
	if(ssrwifidump == 1)
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_DUMP_FLAG, 1);
	else
		SET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_DUMP_FLAG, 0);
	return count;
}

static struct kobj_attribute ssrwifidump_attribute =
	__ATTR(ssrwifidump, 0664, ssrwifidump_show, ssrwifidump_store);

// p15060
static ssize_t forceddump_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	printk(KERN_INFO "%s - forceddump : %d\n", __func__, forceddump);
	return sprintf(buf, "%d\n", forceddump);
}

static ssize_t forceddump_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%du", &forceddump);
	printk(KERN_INFO "%s - forceddump : %d\n", __func__, forceddump);
	return count;
}

static struct kobj_attribute forceddump_attribute =
	__ATTR(forceddump, 0664, forceddump_show, forceddump_store);

static struct attribute *attrs[] = {
	&ramdump_attribute.attr,
	&usbdump_attribute.attr,
	&userdump_attribute.attr, // p15060
	&errortest_attribute.attr,
	&ssrmdm_attribute.attr,
	&ssrmdmdump_attribute.attr,
	&ssrwifi_attribute.attr,
	&ssrwifidump_attribute.attr,
	&forceddump_attribute.attr,  // p15060
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *pantech_restart_kobj;

int __init pantech_restart_init(void)
{
    int retval;

    pantech_restart_kobj = kobject_create_and_add("pantech_restart", kernel_kobj);
    if (!pantech_restart_kobj)
        return -ENOMEM;

    /* Create the files associated with this kobject */
    retval = sysfs_create_group(pantech_restart_kobj, &attr_group);
    if (retval)
        kobject_put(pantech_restart_kobj);

#ifdef CONFIG_PANTECH_ERR_CRASH_LOGGING
    pantech_sys_imem_init();
#endif

    /* variable initialize */
    ramdump = (int)IS_SYS_RAMDUMP_MODE;
    usbdump = (int)IS_SYS_USBDUMP_MODE;
    userdump = get_userdump_mode();
    ssrmdm = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_FLAG);
    ssrmdmdump = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_MDM_DUMP_FLAG);
    ssrwifi = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_FLAG);
    ssrwifidump = (int)GET_SYS_RESET_IMEM_FLAG(SYS_SSR_WIFI_DUMP_FLAG);

    printk(KERN_INFO "initial dump mode - [ramdump mode : %x, usbdump mode : %x, userdump mode : %x]\n",
        (unsigned int)ramdump, (unsigned int)usbdump, (unsigned int)userdump);
 	
    return retval;
}

module_init(pantech_restart_init);

MODULE_AUTHOR("Pantech LS1");
MODULE_DESCRIPTION("Pantech Restart driver");
MODULE_LICENSE("GPL v2");

/* pantech_sys.h
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

/*****************
 *** INCLUDE ***
 *****************/

#include <mach/pantech_sys.h>
#include <linux/io.h>
#include <linux/export.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <soc/qcom/smem.h>

#if defined(CONFIG_PANTECH_DEBUG)
#include <mach/pantech_debug.h>
#endif

unsigned int silent_boot_mode;
unsigned int silent_boot_mode_backlight;
static uint32_t oem_prev_reset=0;

ssize_t oem_pm_read_proc_reset_info(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int len = 0;
       oem_pm_smem_vendor1_data_type *smem_id_vendor1_ptr;

	smem_id_vendor1_ptr = (oem_pm_smem_vendor1_data_type*)smem_alloc(SMEM_ID_VENDOR1, sizeof(oem_pm_smem_vendor1_data_type),0,SMEM_ANY_HOST_FLAG);

	len  = sprintf(buffer, "Power On Reason : 0x%x\n", smem_id_vendor1_ptr->power_on_reason);
	len += sprintf(buffer + len, "Factory Adc: %d\n", smem_id_vendor1_ptr->factory_dummy_adc);
	len += sprintf(buffer + len, "Factory Present: %d\n", smem_id_vendor1_ptr->factory_dummy_present);
	len += sprintf(buffer + len, "Battery Id: %d\n", smem_id_vendor1_ptr->battery_id);
	len += sprintf(buffer + len, "Battery Id Adc: %d\n", (smem_id_vendor1_ptr->battery_id_adc ? 1 : 0 ) );
	len += sprintf(buffer + len, "Power On Mode: %d\n", smem_id_vendor1_ptr->power_on_mode);
	len += sprintf(buffer + len, "SilentBoot: %d\n", smem_id_vendor1_ptr->silent_boot_mode);
	len += sprintf(buffer + len, "Backlight Off: %d\n", (smem_id_vendor1_ptr->backlight_off ? 1 : 0 ) );
	len += sprintf(buffer + len, "Reset: %d\n", oem_prev_reset);

	return len;
}

ssize_t oem_pm_write_proc_reset_info(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int len;
	char tbuffer[2];

	if(count > 1 )
		len = 1;

	memset(tbuffer, 0x00, 2);

	if(copy_from_user(tbuffer, buffer, len))
		return -EFAULT;

	tbuffer[len] = '\0';

	if(tbuffer[0] >= '0' && tbuffer[0] <= '9')
		oem_prev_reset = tbuffer[0] - '0';

	return len;
}

static const struct file_operations pantech_resetinfo_file_ops = {
	.read		= oem_pm_read_proc_reset_info,
	.write		= oem_pm_write_proc_reset_info,
};

ssize_t showPantechResetInfo(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
    printk(KERN_INFO "Reset: %d\n", GET_SYS_RESET_IMEM_FLAG(SYS_ERROR_RESET_OCCURED_FLAG));

    return 0;
}

static long pantechResetInfo_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int return_val = 0;

    switch (cmd) {
        case GET_SYS_ERROR_RESET_OCCURED_CMD:
            return_val = GET_SYS_RESET_IMEM_FLAG(SYS_ERROR_RESET_OCCURED_FLAG);
            if(copy_to_user(argp, &return_val, sizeof(int)))
                printk("[%s] cmd : %d, fail copy to user\n", __func__, GET_SYS_ERROR_RESET_OCCURED_CMD);
    }

    return 0;
}

 static const struct file_operations procPantechResetInfoOps = {
    read:showPantechResetInfo,
    unlocked_ioctl:pantechResetInfo_ioctl,
};

void pantech_sys_reset_reason_set(unsigned int reason)
{
    unsigned int reset_reason;

    reset_reason = GET_SYS_RESET_REASON_ERROR;
    printk(KERN_INFO "[%s] current_reset_reason : 0x%08X\n",__func__, reset_reason);

    if(reset_reason == SYS_RESET_REASON_ABNORMAL)
    {
        SET_SYS_RESET_REASON(reason);
    }
    printk(KERN_INFO "[%s] changed_reset_reason :0x%08X\n",__func__, GET_SYS_RESET_REASON_ALL);
}

unsigned int *pantech_sys_imem_addr = NULL;

/* this function must to be called faster than pantech_resrart_init & pantech_server */
void pantech_sys_imem_init(void)
{
    struct proc_dir_entry *pantechResetInfo = NULL; 
    struct proc_dir_entry *oem_pm_power_on_info;
    oem_pm_smem_vendor1_data_type *smem_vendor1_data;
	
#if defined(CONFIG_PANTECH_DEBUG)
    if(IS_SYS_RAMDUMP_MODE)
        pantech_debug_enable = 1;
    else
        pantech_debug_enable = 0;
#endif

    pantech_sys_imem_addr = (unsigned int *)ioremap(0x08600FF4, 0x10);
	
    if(pantech_sys_imem_addr)
        printk(KERN_INFO "[%s] pantech_sys_imem ioremap success [virtual address : %p, physical address : %x, size : %x]\n", 
            __func__, pantech_sys_imem_addr, 0x8600FF4, 0x10);
			//__func__, (unsigned int)pantech_sys_imem_addr, 0x8600FF4, 0x10);
    else
        printk(KERN_ERR "[%s] pantech_sys_imem ioremap fail\n", __func__);

    /* section - related with silent reboot info */
    pantechResetInfo = proc_create_data("pantechResetInfo", S_IRUGO | S_IWUSR | S_IWGRP, NULL, &procPantechResetInfoOps, NULL);

    if (pantechResetInfo == NULL) {
        printk(KERN_INFO "[%s] proc_create_data failed for %s\n", __func__, "pantechResetInfo");
    }    

    smem_vendor1_data = (oem_pm_smem_vendor1_data_type*)smem_alloc(SMEM_ID_VENDOR1, sizeof(oem_pm_smem_vendor1_data_type),0,SMEM_ANY_HOST_FLAG);

    silent_boot_mode_backlight =smem_vendor1_data->backlight_off;
    silent_boot_mode = smem_vendor1_data->silent_boot_mode;

     /* section - related with silent reboot info */
    oem_pm_power_on_info = proc_create_data("pantech_resetinfo", S_IRUGO | S_IWUSR | S_IWGRP, NULL, &pantech_resetinfo_file_ops, NULL);

    if (oem_pm_power_on_info == NULL) {
        printk(KERN_INFO "[%s] proc_create_data failed for %s\n", __func__, "pantech_resetinfo");
    }    

}


unsigned int pantech_sys_reset_backlight_flag_get(void)
{
	return silent_boot_mode_backlight;
}
EXPORT_SYMBOL(pantech_sys_reset_backlight_flag_get);

void pantech_sys_reset_backlight_flag_set(unsigned int flag)
{
	SET_SYS_RESET_IMEM_FLAG(SYS_RESET_BACKLIGHT_OFF_FLAG, flag);
	silent_boot_mode_backlight = flag;
}
EXPORT_SYMBOL(pantech_sys_reset_backlight_flag_set);
unsigned int pantech_sys_rst_is_silent_boot_mode(void)
{
	return silent_boot_mode;
}

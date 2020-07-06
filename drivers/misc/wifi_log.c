/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/uaccess.h>


#define DEVICE "wifi_log"
#define VERSION "1.00"

/* module params */
#undef PLATFORM_DEVICE

#define KERNEL_BUFFER_SIZE 512

char log_buffer[KERNEL_BUFFER_SIZE];

static struct {
#ifdef PLATFORM_DEVICE
    struct platform_device *pdev;
#endif
    int	device_opened;
    struct mutex dev_lock;
} *penv = NULL;

bool block_mode;

DECLARE_WAIT_QUEUE_HEAD(ReadWaitQueue);

void wifi_to_log_driver( const char *str) 
{ 
    memset(log_buffer,0x00,KERNEL_BUFFER_SIZE);
    if(str != NULL)
    {
        if(strlen(str) < KERNEL_BUFFER_SIZE)
        {
            sprintf(log_buffer, "%s", str);
            if(block_mode)
            {
                wake_up_interruptible(&ReadWaitQueue);
            }
        }
    }
}
EXPORT_SYMBOL(wifi_to_log_driver);


static int wifi_log_open(struct inode *inode, struct file *file)
{
	int rc = 0;

       if (!penv)
		return -EFAULT;

       if((file->f_flags & O_NONBLOCK) || (file->f_flags & O_NDELAY))
       {
           block_mode = false;
       }
       else block_mode = true;

       penv->device_opened = 1;
	pr_info("%s(%d)\n", __func__, __LINE__); 
	return rc;
}

static int wifi_log_close(struct inode *inode, struct file *file)
{
	int rc = 0;

	if (!penv)
		return -EFAULT;

       penv->device_opened = 0;
	pr_info("%s(%d) \n", __func__, __LINE__);
	return rc;
}

static ssize_t wifi_log_read(struct file *fp, char __user
			*buffer, size_t count, loff_t *position)
{
	int cnt = 0, err = 0;
	
	if (!penv || !penv->device_opened)
		return -EFAULT;

       cnt = sizeof(log_buffer);
       if(count < cnt)
          cnt = count;

       if(block_mode)
       {
           if(log_buffer[0] == 0x00)
           {
               wait_event_interruptible(ReadWaitQueue, log_buffer[0] != 0x00);
           }
       }
       else
       {
           if(log_buffer[0] == 0x00) return -EAGAIN;
       }

       if((err = copy_to_user(buffer,log_buffer,cnt)) < 0) return err;
       
       memset(log_buffer,0x00,KERNEL_BUFFER_SIZE);
       
       return count ;


}

static ssize_t wifi_log_write(struct file *fp, const char __user
			*user_buffer, size_t count, loff_t *position)
{
	int err = 0;
	char *buff;
	
	if (!penv || !penv->device_opened)
		return -EFAULT;

       buff = kmalloc(count, GFP_KERNEL);
	if((err = copy_from_user(buff,user_buffer,count)) < 0) 
	{ 
	   kfree(buff);
	   return err;
	}

       kfree(buff);
	return count ;
}

static const struct file_operations wifi_log_fops = {
	.owner = THIS_MODULE,
	.open = wifi_log_open,
	.release = wifi_log_close,
	.read = wifi_log_read,
	.write = wifi_log_write,
};

static struct miscdevice wifi_log_miscdev = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     DEVICE,
	.fops =     &wifi_log_fops
};

#ifdef PLATFORM_DEVICE
static int __devinit
wifi_log_probe(struct platform_device *pdev)
{
	int ret = 0;
	pr_info("%s(%d)\n", __func__, __LINE__);

       if(penv){
		dev_err(&pdev->dev, "cannot handle multiple devices.\n");
		return -ENODEV;
	}
	penv = devm_kzalloc(&pdev->dev, sizeof(*penv), GFP_KERNEL);
	if (!penv) {
		dev_err(&pdev->dev, "cannot allocate device memory.\n");
		return -ENOMEM;
	}
	penv->pdev = pdev;
		
       mutex_init(&penv->dev_lock);

	pr_info("%s(%d): probed in built-in mode\n", __func__, __LINE__);

       ret = misc_register(&wifi_log_miscdev);
	if (ret) 
	    pr_err("%s(%d): misc_register failed \n", __func__, __LINE__);

	return ret ;
}

static int __devexit
wifi_log_remove(struct platform_device *pdev)
{
	pr_info("%s(%d)\n", __func__, __LINE__);
	penv = NULL;
	misc_deregister(&wifi_log_miscdev);
	return 0;
}

static struct platform_driver wifi_log_driver = {
	.driver = {
		.name   = DEVICE,
		.owner  = THIS_MODULE,
	},
	.probe  = wifi_log_probe,
	.remove = __devexit_p(wifi_log_remove),
};
#endif

static int __init wifi_log_init(void)
{
	int ret = 0;

	pr_info("%s(%d)\n", __func__, __LINE__);
#ifdef PLATFORM_DEVICE
	ret = platform_driver_register(&wifi_log_driver);
	if (ret) 
	    pr_err("%s(%d): platform_driver_register failed \n", __func__, __LINE__);
#else
       ret = misc_register(&wifi_log_miscdev);
	if (ret) 
	    pr_err("%s(%d): misc_register failed \n", __func__, __LINE__);	
	else
	    penv = kmalloc(sizeof(*penv), GFP_KERNEL);
#endif
	return ret;
}

static void __exit wifi_log_exit(void)
{
	pr_info("%s(%d)\n", __func__, __LINE__);
#ifdef PLATFORM_DEVICE
	if (penv) {
	    penv = NULL;	
	}
	platform_driver_unregister(&wifi_log_driver);
#else
	if (penv) {
	    kfree(penv);
	    penv = NULL;	
	}
	misc_deregister(&wifi_log_miscdev);
#endif
}

module_init(wifi_log_init);
module_exit(wifi_log_exit);


MODULE_LICENSE("GPL v2");
MODULE_VERSION(VERSION);
MODULE_DESCRIPTION(DEVICE "Driver");

/*
 *  drivers/switch/switch_class.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/switch.h>


//ps1 team shs : add driver
struct class *sky_class;
static struct switch_dev sky_switch_dev;
static atomic_t device_count;

#define apr_dbg(fmt, args...)   printk("[SWITCH SKY] " fmt, ##args)
void sky_set_state(struct switch_dev *sdev, int state);
static ssize_t state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);

	if (sdev->print_state) {
		int ret = sdev->print_state(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return sprintf(buf, "%d\n", sdev->state);
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);

	if (sdev->print_name) {
		int ret = sdev->print_name(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return sprintf(buf, "%s\n", sdev->name);
}
static ssize_t event_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)
		dev_get_drvdata(dev);

	if (sdev->print_name) {
		int ret = sdev->print_name(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return sprintf(buf, "%s\n", sdev->name);
}
static ssize_t event_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	
	u32 akey = simple_strtoul(buf, NULL, 10);
	struct switch_dev *sdev = (struct switch_dev *)
	dev_get_drvdata(dev);	
	apr_dbg("event_store CALLING\n");
	if(akey==0xF23F)
	{
	sky_set_state(&sky_switch_dev,1);	
	sdev->state = 0;	
	}
	return count;
}


static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, name_show, NULL);
static DEVICE_ATTR(event, S_IRUGO | S_IWUSR, event_show, event_store);

void sky_set_state(struct switch_dev *sdev, int state)
{
	char name_buf[120];
	char state_buf[120];
	char *prop_buf;
	char *envp[3];
	int env_offset = 0;
	int length;

	if (sdev->state != state) {
		sdev->state = state;

		prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
		if (prop_buf) {
			length = name_show(sdev->dev, NULL, prop_buf);
			if (length > 0) {
				if (prop_buf[length - 1] == '\n')
					prop_buf[length - 1] = 0;
				snprintf(name_buf, sizeof(name_buf),
					"APR_NAME=%s", prop_buf);
				envp[env_offset++] = name_buf;
			}
			length = state_show(sdev->dev, NULL, prop_buf);
			if (length > 0) {
				if (prop_buf[length - 1] == '\n')
					prop_buf[length - 1] = 0;
				snprintf(state_buf, sizeof(state_buf),
					"APR_STATE=%s", prop_buf);
				envp[env_offset++] = state_buf;
			}
			envp[env_offset] = NULL;
			apr_dbg("kobject_uevent_env calling");			
			kobject_uevent_env(&sdev->dev->kobj, KOBJ_CHANGE, envp);
			free_page((unsigned long)prop_buf);
		} else {
			printk(KERN_ERR "out of memory in sky_set_state\n");
			kobject_uevent(&sdev->dev->kobj, KOBJ_CHANGE);
		}
	}
}

static int create_sky_class(void)
{
	if (!sky_class) {
		sky_class = class_create(THIS_MODULE, "sky");
		if (IS_ERR(sky_class))
			return PTR_ERR(sky_class);
		atomic_set(&device_count, 0);
	}

	return 0;
}

static int sky_dev_register(struct switch_dev *sdev)
{
	int ret;

	if (!sky_class) {
		ret = create_sky_class();
		if (ret < 0)
			return ret;
	}

	sdev->index = atomic_inc_return(&device_count);
	sdev->dev = device_create(sky_class, NULL,
		MKDEV(0, sdev->index), NULL, sdev->name);
	if (IS_ERR(sdev->dev))
		return PTR_ERR(sdev->dev);

	ret = device_create_file(sdev->dev, &dev_attr_state);
	if (ret < 0)
		goto err_create_file_1;
	ret = device_create_file(sdev->dev, &dev_attr_name);
	if (ret < 0)
		goto err_create_file_2;
	ret = device_create_file(sdev->dev, &dev_attr_event);
	if (ret < 0)	
		goto err_create_file_3;	
	dev_set_drvdata(sdev->dev, sdev);
	sdev->state = 0;
	return 0;

err_create_file_3:
	device_remove_file(sdev->dev, &dev_attr_name);	
	device_remove_file(sdev->dev, &dev_attr_state);
err_create_file_2:
	device_remove_file(sdev->dev, &dev_attr_state);
err_create_file_1:
	device_destroy(sky_class, MKDEV(0, sdev->index));
	printk(KERN_ERR "switch: Failed to register driver %s\n", sdev->name);

	return ret;
}


void sky_switch_dev_unregister(struct switch_dev *sdev)
{
	device_remove_file(sdev->dev, &dev_attr_name);
	device_remove_file(sdev->dev, &dev_attr_state);
	device_remove_file(sdev->dev, &dev_attr_event);	
	atomic_dec(&device_count);
	dev_set_drvdata(sdev->dev, NULL);
	device_destroy(sky_class, MKDEV(0, sdev->index));
}


static int __init sky_switch_class_init(void)
{
	int ret;
	sky_switch_dev.name="apr";
	ret=sky_dev_register(&sky_switch_dev);	
	return ret;
}

static void __exit sky_switch_class_exit(void)
{
	sky_switch_dev_unregister(&sky_switch_dev);	
	class_destroy(sky_class);
	
}

module_init(sky_switch_class_init);
module_exit(sky_switch_class_exit);

MODULE_AUTHOR("shin hyung sik<shin.hyungsik@pantech.com>");
MODULE_DESCRIPTION("Sky Switch class driver");
MODULE_LICENSE("GPL");

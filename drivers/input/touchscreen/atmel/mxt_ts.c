/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2011 Atmel Corporation
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <asm/io.h>
#include <linux/gpio.h>
//#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
//#include <mach/gpio.h>
//#include <asm/mach-types.h>
#include <linux/hrtimer.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
//#include <linux/gpio_switch.h> //15.01.26 temp blocked because of missing this file.

/* ATMEL Common File */
#include "mxt_ts.h"
#include "mxt_sec.c"
#if TSP_PATCH
#include "mxt_patch.c"
#endif

/* TSP Power & Local feature */
#include "pantech_ts/pantech_ts.h"

/* Model CFG File */

#if MXT_CFG_WRITE_BIN
#if (CONFIG_BOARD_VER == CONFIG_PT10 )
#include "config/EF78/mxt_cfg_ef78.c" //EF78
#else
#include "config/EF71/mxt_cfg_ef71.c" //EF71
#endif
#else
#include "config/EF67/mxt_cfg_ef67.c"
#endif

/* Pantech Dev File */
#include "pantech_ts/touch_fops.h"
#include "pantech_ts/touch_monitor.h"

#if MXT_CFG_WRITE_BIN
/* Config BIN File */
#ifdef MXT_CFG_EF71
#if (CONFIG_BOARD_VER == CONFIG_PT10 )
#include "config/EF78/ef78_cfg_finger.h" //EF78
#else
#include "config/EF71/ef71_cfg_finger.h" //EF71 
//#include "config/EF71/ef71_cfg_charger.h" //EF71 
#include "config/EF71/ef71_cfg_charger_AC.h" //EF71 
#include "config/EF71/ef71_cfg_charger_Wireless.h" //EF71 
#endif
#else
#include "config/EF67/ef67_cfg_finger.h"
#include "config/EF67/ef67_cfg_stylus.h"
#include "config/EF67/ef67_cfg_glove.h"
#endif
#endif

#ifdef TOUCH_DEV_TREE_SUPPORT
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#endif

#ifdef OFFLINE_CHARGER_TOUCH_DISABLE
#include <soc/qcom/smsm.h>
#endif

/*function proto types*/
void  clear_event(struct mxt_data *data, uint8_t clear);

#ifdef TOUCH_DEV_TREE_SUPPORT
static int mxt_parse_dt(struct i2c_client *client)
{
	rst_gpio = of_get_named_gpio(client->dev.of_node,	 "atmel,reset-gpio", 0);
	if (rst_gpio< 0) {
		dbg_cr("unable to atmel-reset-gpio\n");
		return 0;
	}

	iovdd_gpio = of_get_named_gpio(client->dev.of_node, "atmel,iovdd-gpio", 0);
	if (iovdd_gpio<0) {
		dbg_cr("unable to atmel-iovdd-gpio\n");
		return 0;
	}

	chg_gpio = of_get_named_gpio(client->dev.of_node, "atmel,irq-gpio", 0);
	if (chg_gpio<0) {
		dbg_cr("unable to atmel-chg-gpio\n");
		return 0;
	}

	dbg_hw("rst_gpio : [%d]\n", rst_gpio);
	dbg_hw("iovdd_gpio : [%d]\n", iovdd_gpio);	
	dbg_hw("chg_gpio : [%d]\n", chg_gpio);
	
	return 1;
}
#endif

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
		       message, data->T5_msg_size, false);
}

static int mxt_bootloader_read(struct mxt_data *data, u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);

	return (ret == 1) ? 0 : ret;
}

static int mxt_bootloader_write(struct mxt_data *data, const u8 * const val,
	unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);

	return (ret == 1) ? 0 : ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, u8 retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = 0;

	if (data->info) {
		family_id = data->info->family_id;
	} else {
		family_id = MXT_CURRENT_FAMILY_ID;
	}

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if ((retry % 2) || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 val;
	bool crc_failure;

	dbg_func_in();

	ret = mxt_lookup_bootloader_address(data, 0);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret) {
		dev_err(dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
		val, crc_failure ? ", APP_CRC_FAIL" : "");

	dbg_func_out();

	return 0;
}

static int mxt_wait_for_chg(struct mxt_data *data)
{
/*
	int timeout_counter = 0;
	int count = 1E6;

	if (data->pdata->read_chg == NULL) {
		msleep(10);
		return 0;
	}

	while ((timeout_counter++ <= count) && data->pdata->read_chg())
		udelay(20);

	if (timeout_counter > count) {
		dev_err(&data->client->dev, "mxt_wait_for_chg() timeout!\n");
		return -EIO;
	}
*/
	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	struct device *dev = &data->client->dev;
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			dev_err(dev, "%s: i2c failure\n", __func__);
			return -EIO;
		}

		dev_info(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		dev_info(dev, "Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data,
				unsigned int state)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 val;

recheck:
	ret = mxt_bootloader_read(data, &val, 1);
	if (ret) {
		dev_err(dev, "%s: i2c recv failed, ret=%d\n",
			__func__, ret);
		return ret;
	}

	if (state == MXT_WAITING_BOOTLOAD_CMD) {
		val = mxt_get_bootloader_version(data, val);
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			mxt_wait_for_chg(data);
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			dev_err(dev, "Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader mode state 0x%02X\n", val);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret) {
		dev_err(&data->client->dev, "%s: i2c send failed, ret=%d\n",
				__func__, ret);
		return ret;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;
	bool retry = false;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
				__func__, ret);
			return -EIO;
		}
	}

	return 0;
}

#if TSP_PATCH
static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object = NULL;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}
#endif

static int mxt_read_mem(struct i2c_client *client, u16 reg, u16 len, u8* buf)
{
	return __mxt_read_reg(client, reg, len, buf);
}

static int mxt_read_message(struct mxt_data *data, struct mxt_message *message)
{
	struct mxt_object *object = NULL;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
		return -EINVAL;

	return mxt_read_mem(data->client, object->start_address, sizeof(struct mxt_message), (u8*)message);
}


static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int ret;
	bool retry = false;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

retry_write:
	ret = i2c_master_send(client, buf, count);
	if (ret == count) {
		ret = 0;
	} else {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}
	}

	kfree(buf);
	return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

#if TSP_PATCH
static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_make_reportid_table(struct mxt_data *data)
{
//	struct mxt_object *object = data->object_table;
	struct mxt_reportid *reportids = data->reportids;		
	int i, j;
	int id = 0;

	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = data->object_table + i;
		for (j = 0; j < object->num_report_ids * OBP_INSTANCES(object); j++) {
			id++;

			reportids[id].type = object->type;
			reportids[id].index = j;

			dev_dbg(&data->client->dev, "Report_id[%d]:\tType=%d\tIndex[%d]\n",
					id, reportids[id].type, reportids[id].index);
		}
	}
}

#endif

static int mxt_write_mem(struct i2c_client *client, u16 reg, u16 len, u8* buf)
{
	return __mxt_write_reg(client, reg, len, buf);
}


static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);
	
	dbg_func_in();
	
	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_dbg(dev, "T6 Config Checksum: 0x%06X\n", crc);
	}

	if (status)
		dev_err(dev, "T6 Status 0x%02X%s%s%s%s%s%s\n",
				status,
				(status & MXT_T6_STATUS_RESET) ? " RESET" : "",
				(status & MXT_T6_STATUS_OFL) ? " OFL" : "",
				(status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
				(status & MXT_T6_STATUS_CAL) ? " CAL" : "",
				(status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
				(status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");		
}

static void mxt_input_sync(struct input_dev *input_dev)
{
	dbg_func_in();

	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);
}

#if PAN_HAVE_TOUCH_KEY
int get_touch_key_pos(int x, int y) 
{	
	if ( x >= (PAN_1ST_TOUCH_KEY_X - PAN_1ST_TOUCH_KEY_MARGIN_X) && 
			x <= (PAN_1ST_TOUCH_KEY_X + PAN_1ST_TOUCH_KEY_MARGIN_X) &&
			y >= (PAN_TOUCH_KEY_Y - PAN_TOUCH_KEY_MARGIN_Y) && 
			y <= (PAN_TOUCH_KEY_Y + PAN_TOUCH_KEY_MARGIN_Y) ) {
		return 0;
	}

	if ( x >= (PAN_2ND_TOUCH_KEY_X - PAN_2ND_TOUCH_KEY_MARGIN_X) && 
			x <= (PAN_2ND_TOUCH_KEY_X + PAN_2ND_TOUCH_KEY_MARGIN_X) &&
			y >= (PAN_TOUCH_KEY_Y - PAN_TOUCH_KEY_MARGIN_Y) && 
			y <= (PAN_TOUCH_KEY_Y + PAN_TOUCH_KEY_MARGIN_Y) ) {
		return 1;
	}

	if ( x >= (PAN_3RD_TOUCH_KEY_X - PAN_3RD_TOUCH_KEY_MARGIN_X) && 
			x <= (PAN_3RD_TOUCH_KEY_X + PAN_3RD_TOUCH_KEY_MARGIN_X) &&
			y >= (PAN_TOUCH_KEY_Y - PAN_TOUCH_KEY_MARGIN_Y) && 
			y <= (PAN_TOUCH_KEY_Y + PAN_TOUCH_KEY_MARGIN_Y) ) {
		return 2;	
	}	

	return -1;
}
#endif

void report_fingerInfo (struct input_dev *input_dev, report_finger_info_t fingerInfo) {
	input_mt_slot(input_dev, fingerInfo.id);						
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, fingerInfo.id);	
	input_report_abs(input_dev, ABS_MT_POSITION_X, fingerInfo.x);
#if(CONFIG_BOARD_VER == CONFIG_PT10 )	
	input_report_abs(input_dev, ABS_MT_POSITION_Y, (SCREEN_RESOLUTION_SCREEN_Y - fingerInfo.y)); 
#else
	input_report_abs(input_dev, ABS_MT_POSITION_Y, fingerInfo.y /*(SCREEN_RESOLUTION_SCREEN_Y - fingerInfo.y)*/); //(EF78)temp, because of reversed y pattern.
#endif	
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo.area);	
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo.area);		
}

void report_release (struct input_dev *input_dev, report_finger_info_t fingerInfo) {
	input_mt_slot(input_dev, fingerInfo.id);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
}

void  clear_event(struct mxt_data *data, uint8_t clear)
{
	uint8_t valid_input_count=0;
	int i;   

	dbg_func_in();
	for ( i= 0; i<MAX_NUM_FINGER; i++ )
	{
		if(fingerInfo[i].mode == TSC_EVENT_WINDOW)
		{
			dbg_op("clear_event U:(%d, %d) (id:%d)\n", fingerInfo[i].x, fingerInfo[i].y, fingerInfo[i].id);
			report_release(data->input_dev, fingerInfo[i]);
			fingerInfo[i].mode = TSC_EVENT_NONE;
			fingerInfo[i].status= -1;
		}
		else{
			valid_input_count++;
		}
	}
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	dbg_op(" touch event num => %d\n",valid_input_count);
	input_sync(data->input_dev);

	if(clear == TSC_CLEAR_ALL)
	{
		for ( i= 0; i<MAX_NUM_FINGER; i++ )
		{
			fingerInfo[i].mode = TSC_EVENT_NONE;
			fingerInfo[i].status = -1;
			fingerInfo[i].area = 0;
		}     
	}
	dbg_func_out();
}

void report_input (struct mxt_data *data) {
	int i;
	int valid_input_count=0;

	for ( i= 0; i<MAX_NUM_FINGER; i++ )	{

		if ( fingerInfo[i].status == -1 || (fingerInfo[i].mode == TSC_EVENT_NONE && fingerInfo[i].status == 0)) 
			continue;

		if(fingerInfo[i].mode == TSC_EVENT_NONE ){			// TOUCH_EVENT_PRESS (DOWN)

			if(fingerInfo[i].y < SCREEN_RESOLUTION_SCREEN_Y){
				report_fingerInfo(data->input_dev,fingerInfo[i]);
				fingerInfo[i].mode = TSC_EVENT_WINDOW;
			}
#if PAN_HAVE_TOUCH_KEY
			else {

				fingerInfo[i].mode = TSC_EVENT_NONE;
				if ( get_touch_key_pos(fingerInfo[i].x, fingerInfo[i].y) == 0 ){	
					input_report_key(data->input_dev, PAN_1ST_TOUCH_KEY_TYPE, 1);
					fingerInfo[i].mode = TSC_EVENT_1ST_KEY;
				}
				if ( get_touch_key_pos(fingerInfo[i].x, fingerInfo[i].y) == 1 ){
					input_report_key(data->input_dev, PAN_2ND_TOUCH_KEY_TYPE, 1);
					fingerInfo[i].mode = TSC_EVENT_2ND_KEY;
				}
				if ( get_touch_key_pos(fingerInfo[i].x, fingerInfo[i].y) == 2 ){
					input_report_key(data->input_dev, PAN_3RD_TOUCH_KEY_TYPE, 1);
					fingerInfo[i].mode = TSC_EVENT_3RD_KEY;
				}				
			}
#endif

			valid_input_count++;
		}
		else
		{
			// TOUCH_EVENT_RELEASE (UP)
			if (fingerInfo[i].status == TOUCH_EVENT_RELEASE && fingerInfo[i].mode == TSC_EVENT_WINDOW) { 		 
				dbg_op(" U:(%d, %d) (id:%d)\n", fingerInfo[i].x, fingerInfo[i].y, fingerInfo[i].id);
				report_release(data->input_dev, fingerInfo[i]);
				fingerInfo[i].mode = TSC_EVENT_NONE;
				fingerInfo[i].status= -1;				
			}
#if PAN_HAVE_TOUCH_KEY
			else if (fingerInfo[i].status == TOUCH_EVENT_RELEASE && fingerInfo[i].mode == TSC_EVENT_1ST_KEY) {
				input_report_key(data->input_dev, PAN_1ST_TOUCH_KEY_TYPE, 0);
				fingerInfo[i].mode = TSC_EVENT_NONE; fingerInfo[i].status= -1;				
			}
			else if (fingerInfo[i].status == TOUCH_EVENT_RELEASE && fingerInfo[i].mode == TSC_EVENT_2ND_KEY) {
				input_report_key(data->input_dev, PAN_2ND_TOUCH_KEY_TYPE, 0);
				fingerInfo[i].mode = TSC_EVENT_NONE; fingerInfo[i].status= -1;				
			}
			else if (fingerInfo[i].status == TOUCH_EVENT_RELEASE && fingerInfo[i].mode == TSC_EVENT_3RD_KEY) {
				input_report_key(data->input_dev, PAN_3RD_TOUCH_KEY_TYPE, 0);
				fingerInfo[i].mode = TSC_EVENT_NONE; fingerInfo[i].status= -1;				
			}
#endif
			// TOUCH_EVENT_MOVE
			else if(fingerInfo[i].status == TOUCH_EVENT_MOVE && fingerInfo[i].mode == TSC_EVENT_WINDOW)
			{
				if(fingerInfo[i].y>(SCREEN_RESOLUTION_SCREEN_Y)) {
					fingerInfo[i].y=SCREEN_RESOLUTION_SCREEN_Y;
					dbg_op("Finger[%d] Move(TSC_EVENT_WINDOW) XY(%d, %d) - disabled\n", i, fingerInfo[i].x, fingerInfo[i].y);
				}
				else {
				    dbg_op("Finger[%d] Move(TSC_EVENT_WINDOW) XY(%d, %d)\n", i, fingerInfo[i].x, fingerInfo[i].y);
				    report_fingerInfo(data->input_dev,fingerInfo[i]);
				}				

				fingerInfo[i].status= TOUCH_EVENT_PRESS;
				valid_input_count++;
			}
			// TOUCH_EVENT_PRESS 
			else if (fingerInfo[i].status == TOUCH_EVENT_PRESS)	{
				valid_input_count++;
			}
			else {
			}
		}
	}

	input_report_key(data->input_dev, BTN_TOUCH, !!valid_input_count);  // mirinae_ICS
	dbg_op(" touch event num => %d\n",valid_input_count);
	input_sync(data->input_dev);

}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	dbg_func_in();

	/* do not report events if input device not yet registered */
	if (!input_dev)
		return;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	/* A reported size of zero indicates that the reported touch
	 * is a stylus from a linked Stylus T47 object. */
	if (message[5] == 0) {
		area = 1;
		tool = MT_TOOL_PEN;
	} else {
		area = message[5];
		tool = MT_TOOL_FINGER;
	}

	amplitude = message[6];
	vector = message[7];

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/* Multiple bits may be set if the host is slow to read the
		 * status messages, indicating all the events that have
		 * happened */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev, tool, 0);
			mxt_input_sync(input_dev);
		}

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, tool, 0);
	}

	data->t9_update_input = true;
}

static void mxt_proc_t100_messages(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
//	struct input_dev *input_dev = data->input_dev;
	u8 id, index;
	u8 touch_type = 0, touch_event = 0, touch_detect = 0;
	int x = 0;
	int y = 0;
	int area = 0;
	u8 vector = 0;
//	int tool = 0;

	index = message[0] - data->T100_reportid_min;

	/* Treate screen messages */
	if (index < MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID) {
		if (index == MXT_T100_SCREEN_MSG_FIRST_RPT_ID)
			dev_dbg(dev, "SCRSTATUS:[%02X] %02X %04X %04X %04X\n",
				 message[0], message[1], (message[3] << 8) | message[2],
				 (message[5] << 8) | message[4],
				 (message[7] << 8) | message[6]);

		return;
	}

	/* Treate touch status messages */
	id = index - MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID;
	touch_detect = (message[1] & 0x80) >> 7;
	touch_type = (message[1] & 0x70) >> 4;
	touch_event = (message[1] & 0x0F);

	dev_dbg(dev, "TCHSTATUS [%d] : DETECT[%d] TYPE[%d] EVENT[%d] %d,%d,%d,%d,%d\n",
		id, touch_detect, touch_type, touch_event,
		message[2] | (message[3] << 8),	message[4] | (message[5] << 8),
		message[6], message[7], message[8]);

	switch (touch_type)	{
	case MXT_T100_TYPE_FINGER:
	case MXT_T100_TYPE_PASSIVE_STYLUS:
	case MXT_T100_TYPE_HOVERING_FINGER:
	case MXT_T100_TYPE_GLOVE:
		/* There are no touch on the screen */
		if (!touch_detect) {
			if (touch_event == MXT_T100_EVENT_UP
				|| touch_event == MXT_T100_EVENT_SUPPRESS) {
				fingerInfo[id].status= 0;
				//printk("[641T] TOUCH_RELEASE || TOUCH_SUPPRESS !!, 0x%x\n", touch_event);

			} else {
				dev_dbg(dev,"Untreated Undetectd touch : type[%d], event[%d]\n",
					touch_type, touch_event);
				return;
			}
			break;
		}

		/* There are touch on the screen */
		if (touch_event == MXT_T100_EVENT_DOWN
			|| touch_event == MXT_T100_EVENT_UNSUPPRESS
			|| touch_event == MXT_T100_EVENT_MOVE
			|| touch_event == MXT_T100_EVENT_NONE) {

			x = message[2] | (message[3] << 8);
			y = message[4] | (message[5] << 8);
			//vector = message[6];
			area = message[6]; // p11774 14.06.10 for update area data

			if (touch_type == MXT_T100_TYPE_HOVERING_FINGER) {
				vector = 0;
				area = 0;
			}

			fingerInfo[id].id = id;
			fingerInfo[id].status = TOUCH_EVENT_PRESS;
			fingerInfo[id].type	= touch_type;
			fingerInfo[id].area = area;
			fingerInfo[id].x = (int16_t)x;
			fingerInfo[id].y = (int16_t)y;

			
		} else {
			dev_err(dev, "Untreated Detectd touch : type[%d], event[%d]\n",
				touch_type, touch_event);
			return;
		}
		break;
		default: return;
	}

	if ( touch_event == MXT_T100_EVENT_UP || touch_event == MXT_T100_EVENT_SUPPRESS )    
	{
		if(data->pre_debug_enabled && !data->move_dbg){
			data->debug_enabled = true;
		}
		fingerInfo[id].status= TOUCH_EVENT_RELEASE;
		fingerInfo[id].area= 0;
	}
	else if ( touch_event == MXT_T100_EVENT_MOVE )  
	{
		if(data->pre_debug_enabled && !data->move_dbg){
			data->debug_enabled = false;
		}
		fingerInfo[id].id= id;
		fingerInfo[id].status= TOUCH_EVENT_MOVE;
	}
	else if ( touch_event == MXT_T100_EVENT_DOWN || touch_event == MXT_T100_EVENT_UNSUPPRESS )
	{                               
		fingerInfo[id].id= id;
		fingerInfo[id].status= TOUCH_EVENT_PRESS;
	}                 
	
	report_input(data);

}

static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;

#if PAN_T15_KEYARRAY_ENABLE
#ifdef PAN_TOUCH_KEY_REJECTION_ON_DISPLAY
	int i;
#endif
	if(msg[1] == 0x80)
	{
//++ p11309 - 2013.09.01 for Optional
#ifdef PAN_TOUCH_KEY_REJECTION_ON_DISPLAY
		for(i=0;i<MAX_NUM_FINGER;i++){
			if(fingerInfo[i].status > 0){
				dbg_op("[KeyArray] Even if Menu/back key is pressed, Menu/back key is ignored because there is touch event on window,\n");
				return;
			}
		}
#endif

		if(msg[2] == 1){
			mPan_KeyArray[0].key_state=true;
			input_report_key(input_dev, mPan_KeyArray[0].key_num, 1);
			input_sync(input_dev);
		}else if(msg[2] == 2)  {
			mPan_KeyArray[1].key_state=true;
			input_report_key(input_dev,  mPan_KeyArray[1].key_num, 1);
			input_sync(input_dev); 
		}  
	}else if(msg[1] == 0x0){
		if(mPan_KeyArray[0].key_state==true){
			mPan_KeyArray[0].key_state=false;
			input_report_key(input_dev, mPan_KeyArray[0].key_num, 0);
			input_sync(input_dev);
		}else if(mPan_KeyArray[1].key_state==true){
			mPan_KeyArray[1].key_state=false;
			input_report_key(input_dev, mPan_KeyArray[1].key_num, 0);
			input_sync(input_dev);
		}
	}
#else
	struct device *dev = &data->client->dev;
	u8 key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	for (key = 0; key < ARRAY_SIZE(mxt_t15_keys); key++) {
		curr_state = test_bit(key, &mxt_t15_keystatus);
		new_state = test_bit(key, &keystates);

		if (!curr_state && new_state) {
			dev_dbg(dev, "T15 key press: %u\n", key);
			__set_bit(key, &mxt_t15_keystatus);
			input_event(input_dev, EV_KEY, mxt_t15_keys[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
			dev_dbg(dev, "T15 key release: %u\n", key);
			__clear_bit(key, &mxt_t15_keystatus);
			input_event(input_dev, EV_KEY, mxt_t15_keys[key], 0);
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
#endif
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	dev_dbg(dev, "T48 state %d status %02X %s%s%s%s%s\n",
			state,
			status,
			(status & 0x01) ? "FREQCHG " : "",
			(status & 0x02) ? "APXCHG " : "",
			(status & 0x04) ? "ALGOERR " : "",
			(status & 0x10) ? "STATCHG " : "",
			(status & 0x20) ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	if (!input_dev)
		return;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		dev_err(dev, "invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_STYLUS_PRESSURE_MASK;

	dev_dbg(dev,
		"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		(msg[1] & MXT_STYLUS_SUPPRESS) ? 'S' : '.',
		(msg[1] & MXT_STYLUS_MOVE)     ? 'M' : '.',
		(msg[1] & MXT_STYLUS_RELEASE)  ? 'R' : '.',
		(msg[1] & MXT_STYLUS_PRESS)    ? 'P' : '.',
		x, y, pressure,
		(msg[2] & MXT_STYLUS_BARREL) ? 'B' : '.',
		(msg[2] & MXT_STYLUS_ERASER) ? 'E' : '.',
		(msg[2] & MXT_STYLUS_TIP)    ? 'T' : '.',
		(msg[2] & MXT_STYLUS_DETECT) ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS, (msg[2] & MXT_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2, (msg[2] & MXT_STYLUS_BARREL));

	mxt_input_sync(input_dev);
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool handled = false;

	if (data->state != APPMODE)
		return 0;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;


	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
		handled = true;
	} else if (report_id >= data->T9_reportid_min
	    && report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
		handled = true;
	} else if (report_id >= data->T63_reportid_min
		   && report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
		handled = true;
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
		handled = true;
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
		handled = true;
	} else if (report_id >= data->T15_reportid_min
		   && report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
	}else if (report_id >= data->T100_reportid_min
	    && report_id <= data->T100_reportid_max) {
		mxt_proc_t100_messages(data, message);
		handled = true;
	}

	if (/* !handled || */data->debug_enabled)
		mxt_dump_message(data, message);
		
#if TSP_PATCH
	{
		struct mxt_message pMsg;
		pMsg.reportid = report_id;
		memcpy(pMsg.message, &message[1], 8);	
		mxt_patch_message(data, &pMsg);
	}	
#endif

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) { 
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	if (count == 0) {
		dev_warn(dev, "Interrupt triggered but zero messages\n");
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		dev_err(dev, "T44 count exceeded max report id\n");
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0) {
			goto end;
		} else if (ret != num_left) {
			dev_warn(dev, "Unexpected invalid message\n");
		}
	}

end:
	if (data->t9_update_input) {
		mxt_input_sync(data->input_dev);
		data->t9_update_input = false;
	}

	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	dbg_func_in();

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		dev_err(dev, "read %d\n", read);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->t9_update_input) {
		mxt_input_sync(data->input_dev);
		data->t9_update_input = false;
	}

	if (data->t100_update_input) {
		mxt_input_sync(data->input_dev);
		data->t100_update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* read two at a time until an invalid message or else we reach
	 * reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->t9_update_input) {
		mxt_input_sync(data->input_dev);
		data->t9_update_input = false;
	}

	if (data->t100_update_input) {
		mxt_input_sync(data->input_dev);
		data->t100_update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	irqreturn_t ret = IRQ_NONE;

	mutex_lock(&data->lock);	

	if (data->T44_address)
		ret = mxt_process_messages_t44(data);
	else
		ret = mxt_process_messages(data);

	mutex_unlock(&data->lock);	

	return ret;
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset, u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int ret;
	int timeout_counter = 0;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_backup(struct mxt_data *data)
{
	int ret;
	u8 value = 0x55u;
	
	ret = mxt_t6_command(data, MXT_COMMAND_BACKUPNV, value, false);
	msleep(50);
	
	return ret;
}

static int mxt_calibrate(struct mxt_data *data)
{
	int ret;
	u8 value = 0x01u;
	
	ret = mxt_t6_command(data, MXT_COMMAND_CALIBRATE, value, false);
	
	return ret;
}

static int mxt_soft_reset(struct mxt_data *data, u8 value)
{
	int ret;
	struct device *dev = &data->client->dev;

	dbg_func_in();		

	dev_info(dev, "Resetting chip\n");

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, value, false);
	if (ret)
		return ret;

	msleep(MXT_RESET_TIME);

	preTouchMode = TOUCH_MODE_NORMAL;

	dbg_func_out();

	return 0;
}

static int mxt_soft_reset_resume(struct mxt_data *data, u8 value)
{
	int ret;
	struct device *dev = &data->client->dev;

	dev_info(dev, "Resetting chip from resume\n");

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, value, false);
	if (ret)
		return ret;

	msleep(MXT_RESET_TIME_RESUME);

	preTouchMode = TOUCH_MODE_NORMAL;

	return 0;
}

static int mxt_read_message_reportid(struct mxt_data *data, struct mxt_message *message, u8 reportid)
{
	struct device *dev = &data->client->dev;
	int try = 0;
	int error = 0;
	int fail_count = 0;

	fail_count = data->max_reportid * 2;

	while (++try < fail_count) {
		error = mxt_read_message(data, message);
		if (error) {
			dev_err(dev, "mxt_read_message error\n");
			print_hex_dump(KERN_DEBUG, "[Touch] CRC : ", DUMP_PREFIX_NONE, 16, 1,
				   message, sizeof(struct mxt_message), false);
			return error;
		}

		if (message->reportid == 0xff)
			continue;

		if (message->reportid == reportid)
			return 0;
	}

	return -EINVAL;
}

static int mxt_read_config_crc(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message = {0};
	int error = 0;

	/* on failure, CRC is set to 0 and config will always be downloaded */
	data->config_crc = 0;

	mxt_t6_command(data, MXT_COMMAND_REPORTALL, 1, true);

	/* Read all messages until invalid, this will update the config crc
	 * stored in mxt_data. On failure, CRC is set to 0 and config will
	 * always be downloaded */
	//mxt_process_messages_until_invalid(data);
	error = mxt_read_message_reportid(data, &message, 1);
	if (error) {
		dev_err(dev, "Failed to retrieve CRC\n");
		return error;
	}

	data->config_crc = message.message[1] | (message.message[2] << 8) | (message.message[3] << 16);

	return error;
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	return (crc & 0x00FFFFFF);
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &data->t7_cfg;

	error = __mxt_write_reg(data->client, data->T7_address,
			sizeof(data->t7_cfg),
			new_config);
	if (error)
		return error;

	dev_err(dev, "Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

//20140703_for_selftest +
#if TSP_PATCH //0
static int mxt_get_t8_self_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error = 0;
	u8 val = 0;
	
	error = mxt_read_object(data, MXT_GEN_ACQUIRE_T8, 10, &val);

	dev_err(dev, "T8_SELF: %d\n", ((val & 0x02)>>1));

	return ((val & 0x02)>>1);
}

static int mxt_set_t8_self_cfg(struct mxt_data *data, int onoff)
{
	struct device *dev = &data->client->dev;
	int error = 0;
	u8 val = 0;
	
	error = mxt_read_object(data, MXT_GEN_ACQUIRE_T8, 10, &val);
	dev_err(dev, "T8_OFFSET[10] : %d\n", val);

	if(onoff)
		val |= (0x01<<1);
	else
		val &= (~(0x01<<1));

	error = mxt_write_object(data, MXT_GEN_ACQUIRE_T8, 10, val);
	dev_err(dev, "T8_OFFSET[10] SET : %d\n", val);

	msleep(10);

	return error;
}
#endif
//20140703_for_selftest -

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			dev_err(dev, "T7 cfg zero, resetting\n");
			mxt_soft_reset(data, MXT_RESET_VALUE);
			retry = true;
			goto recheck;
		} else {
		    dev_err(dev, "T7 cfg zero after reset, overriding\n");
		    data->t7_cfg.active = 255;
		    data->t7_cfg.idle = 255;
		    return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	} else {
		dev_err(dev, "Initialised power cfg: ACTV %d, IDLE %d\n",
				data->t7_cfg.active, data->t7_cfg.idle);
		return 0;
	}
}

static int mxt_parse_object_table(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	dbg_func_in();

	dev_dbg(&client->dev, "Object num: %d\n", data->info->object_num);

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = data->object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					OBP_INSTANCES(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(&data->client->dev,
			"T%u Start:%u Size:%u Instances:%u Report IDs:%u-%u\n",
			object->type, object->start_address, OBP_SIZE(object),
			OBP_INSTANCES(object), min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80) {
				/* On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned */
				data->T5_msg_size = OBP_SIZE(object);
			} else {
				/* CRC not enabled, therefore don't read last byte */
				data->T5_msg_size = OBP_SIZE(object) - 1;
			}
			data->T5_address = object->start_address;
			break;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = max_id;
			data->num_touchids =
				object->num_report_ids * OBP_INSTANCES(object);
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_DEBUG_DIAGNOSTIC_T37:
			data->T37_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = max_id;
			data->num_stylusids =
				object->num_report_ids * OBP_INSTANCES(object);
			break;
		case MXT_SPT_GOLDENREFERENCES_T66:
			data->T66_reportid = min_id;
			break;
		case MXT_TOUCH_MULTI_T100:
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			data->num_touchids =
				object->num_report_ids * OBP_INSTANCES(object);
			break;
		}

		end_address = object->start_address
			+ OBP_SIZE(object) * OBP_INSTANCES(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	dev_err(&client->dev, "mem_size : %d\n", data->mem_size);

	/* If T44 exists, T9 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid, data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static void mxt_free_object_table(struct mxt_data *data)
{
	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	data->info = NULL;
	data->object_table = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_address = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T37_address = 0;
	data->T66_reportid = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
#if TSP_PATCH
	kfree(data->reportids);
#endif
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *buf;
	struct mxt_info *info;
	u32 calculated_crc;
	u8 *crc_ptr;

	dbg_func_in();

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, 0, size, buf);
	if (error)
		goto err_free_mem;

	/* Resize buffer to give space for rest of info block */
	info = (struct mxt_info *)buf;
	size += (info->object_num * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(buf, size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = __mxt_read_reg(client, MXT_OBJECT_START,
			       size - MXT_OBJECT_START,
			       buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0, size - MXT_INFO_CHECKSUM_SIZE);

	/* CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol */
	if (data->info_crc != calculated_crc) {
		dev_err(&client->dev, "Info Block CRC error"
			" calculated=0x%06X read=0x%06X\n",
			data->info_crc, calculated_crc);
		return -EIO;
	}

	/* Save pointers in device data structure */
	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;
	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	dev_info(&client->dev,
			"Family ID: %u Variant ID: %u Firmware: V%u.%u.%02X\n",
			data->info->family_id, data->info->variant_id,
			data->info->version >> 4, data->info->version & 0xf,
			data->info->build);

	/* Parse object table information */
	error = mxt_parse_object_table(data);
	if (error) {
		dev_err(&client->dev, "Error %d reading object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

#if TSP_PATCH
	data->reportids = kcalloc(data->max_reportid + 1,
			sizeof(struct mxt_reportid),
			GFP_KERNEL);
	if (!data->reportids) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Make report id table */
	mxt_make_reportid_table(data);
#endif

	dbg_func_out();

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(range.x);
	le16_to_cpus(range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 1023;

	if (range.y == 0)
		range.y = 1023;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	dev_info(&client->dev, "Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_read_t100_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	
	data->max_x = (SCREEN_RESOLUTION_X-1);
	data->max_y = (SCREEN_RESOLUTION_SCREEN_Y-1);

	dev_info(&client->dev, "Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

#if MXT_CFG_WRITE_BIN
int mxt_verify_fw(struct mxt_cfg_info *cfg_info, const struct firmware *fw)
{
	struct mxt_data *data = cfg_info->data;
	struct device *dev = &data->client->dev;
	struct mxt_cfg_image *cfg_img;

	if (!fw) {
		dev_err(dev, "could not find firmware file\n");
		return -ENOENT;
	}

	cfg_img = (struct mxt_cfg_image *)fw->data;

	cfg_info->fw_ver = cfg_img->fw_ver;
	cfg_info->build_ver = cfg_img->build_ver;
	cfg_info->cfg_crc = le32_to_cpu(cfg_img->cfg_crc);
	if(sizeof(finger_cfg) != 0)
		cfg_info->cfg_len = sizeof(finger_cfg) - sizeof(struct mxt_cfg_image);
	else
	cfg_info->cfg_len = CFG_DATA_SIZE;

	if (!cfg_info->cfg_len) {
		dev_err(dev, "Firmware file dose not include configuration data\n");
		return -EINVAL;
	}

	cfg_info->cfg_raw_data = cfg_img->data;

	return 0;
}

static int mxt_write_config_from_bin(struct mxt_cfg_info *cfg_info)
{
	struct mxt_data *data = cfg_info->data;
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	struct mxt_cfg_data *cfg_data;
	u8 i, val = 0;
	u16 reg, index;
	int ret;

	if (!cfg_info->cfg_raw_data) {
		dev_err(dev, "No cfg data in file\n");
		return -ENOMEM;
	}

	/* Check Version information */
	if (cfg_info->fw_ver != data->info->version) {
		dev_err(dev, "Warning: version mismatch! %s\n", __func__);
		return 0;
	}
	if (cfg_info->build_ver != data->info->build) {
		dev_err(dev, "Warning: build num mismatch! %s\n", __func__);
		return 0;
	}

	/* Write config info */
	for (index = 0; index < cfg_info->cfg_len;) {

		if (index + sizeof(struct mxt_cfg_data) >= cfg_info->cfg_len) {
//modify 0203 - 160114 P16094 Block for fix build err
			dev_err(dev, "index(%ld) of cfg_data exceeded total size(%d)!!\n",
			(unsigned long int)(index + sizeof(struct mxt_cfg_data)), cfg_info->cfg_len);
			return -EINVAL;
		}

		/* Get the info about each object */
		cfg_data = (struct mxt_cfg_data *)(&cfg_info->cfg_raw_data[index]);

		index += sizeof(struct mxt_cfg_data) + cfg_data->size;
		if (index > cfg_info->cfg_len) {
			dev_err(dev, "index(%d) of cfg_data exceeded total size(%d) in T%d object!!\n",
				index, cfg_info->cfg_len, cfg_data->type);
			return -EINVAL;
		}

		object = mxt_get_object(data, cfg_data->type);
		if (!object) {
			dev_err(dev, "T%d is Invalid object type\n",
				cfg_data->type);
			return -EINVAL;
		}

		/* Check and compare the size, instance of each object */
		if (cfg_data->size > OBP_SIZE(object)) {
			dev_err(dev, "T%d Object length exceeded!\n",
				cfg_data->type);
			return -EINVAL;
		}
		if (cfg_data->instance >= OBP_INSTANCES(object)) {
			dev_err(dev, "T%d Object instances exceeded!\n",
				cfg_data->type);
			return -EINVAL;
		}

		dev_dbg(dev, "Writing config for obj %d len %d instance %d (%d/%d)\n",
			cfg_data->type, OBP_SIZE(object),
			cfg_data->instance, index, cfg_info->cfg_len);

		reg = object->start_address + (OBP_SIZE(object) * cfg_data->instance);

		/* Write register values of each object */
		ret = mxt_write_mem(data->client, reg, cfg_data->size,
					 cfg_data->register_val);
		if (ret) {
			dev_err(dev, "Write T%d Object failed\n",
				object->type);
			return ret;
		}

		/*
		 * If firmware is upgraded, new bytes may be added to end of
		 * objects. It is generally forward compatible to zero these
		 * bytes - previous behaviour will be retained. However
		 * this does invalidate the CRC and will force a config
		 * download every time until the configuration is updated.
		 */
		if (cfg_data->size < OBP_SIZE(object)) {
			dev_err(dev, "Warning: zeroing %d byte(s) in T%d\n",
				 OBP_SIZE(object) - cfg_data->size, cfg_data->type);

			for (i = cfg_data->size + 1; i < OBP_SIZE(object); i++) {
				ret = mxt_write_mem(data->client, reg + i, 1, &val);
				if (ret)
					return ret;
			}
		}
	}
	dev_err(dev, "Updated configuration\n");

	return ret;
}

int mxt_update_cfg(struct mxt_data* data, touch_mode_type touchMode)
{
	struct firmware *fw = NULL;
	struct mxt_cfg_info cfg_info;
	int error = 0;
	struct device *dev = &data->client->dev;
	bool irq_disabled=false;
	
	dbg_func_in();

	dev_err(dev, "preMode %d, Mode %d\n", (int)preTouchMode, (int)touchMode);

	if(data->no_cfg_update)
		return 0;
	
	if(preTouchMode == touchMode)
		return 0;

	if(data->flag_probe && data->irq)
	{
		disable_irq(data->irq);
		irq_disabled=true;
		//dbg_cr("disable irq from mxt_update_cfg.\n");
	}
	

	memset(&cfg_info, 0, sizeof(struct mxt_cfg_info));
	cfg_info.data = data;

	fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);

	switch(touchMode)
	{
		case TOUCH_MODE_NORMAL:
		fw->data = finger_cfg;
		fw->size = sizeof(finger_cfg);
		break;

#ifndef MXT_CFG_EF71
		case TOUCH_MODE_HIGH_SENSITIVE:
		fw->data = glove_cfg;
		fw->size = sizeof(glove_cfg);
		break;

		case TOUCH_MODE_PEN_LETTER:
		fw->data = stylus_cfg;
		fw->size = sizeof(stylus_cfg);
		break;

		case TOUCH_MODE_PEN_DRAW:
		fw->data = stylus_cfg;
		fw->size = sizeof(stylus_cfg);
		break;
#endif

#ifdef MXT_CFG_EF71
		case TOUCH_MODE_CHARGER:
		if(data->batt_mode == BATTERY_PLUGGED_AC) {
			fw->data = charger_ac_cfg;
			fw->size = sizeof(charger_ac_cfg);
		} else if(data->batt_mode == BATTERY_PLUGGED_WIRELESS) {
			fw->data = charger_wireless_cfg;
			fw->size = sizeof(charger_wireless_cfg);
		}
		break;
#endif

		default: 
			error = -EINVAL;
			goto exit;
	}
	
	error = mxt_verify_fw(&cfg_info, fw);
	if (error) {
		dev_err(dev, "Failed to verify fw\n");
		goto exit;
	}
		
	error = mxt_write_config_from_bin(&cfg_info);
	if (error) {
		dev_err(dev, "Failed to write cfg\n");
		goto exit;
	}

	/* Backup to memory */
	error = mxt_backup(data);
	if (error) {
		dev_err(dev, "Failed backup NV data\n");
		goto exit;
	}

	preTouchMode = touchMode;
#if TSP_PATCH	
	t255_user[4] = touchMode;
#endif

	dbg_func_out();

exit:
	kfree(fw);
	if(irq_disabled)
	{	
			enable_irq(data->irq);
			//dbg_cr("enable irq from mxt_update_cfg.\n");
	}
	dev_err(dev, " error %d\n", error);
	return error;
}

int mxt_update_cfg_no_backup(struct mxt_data* data, touch_mode_type touchMode)
{
	struct firmware *fw = NULL;
	struct mxt_cfg_info cfg_info;
	int error = 0;
	struct device *dev = &data->client->dev;
	bool irq_disabled=false;
	
	dbg_func_in();

	dev_err(dev, "preMode %d, Mode %d\n", (int)preTouchMode, (int)touchMode);

	if(data->no_cfg_update)
		return 0;
	
	if(preTouchMode == touchMode)
		return 0;

	if(data->flag_probe && data->irq)
	{
		disable_irq(data->irq);
		irq_disabled=true;
		//dbg_cr("disable irq from mxt_update_cfg.\n");
	}
	

	memset(&cfg_info, 0, sizeof(struct mxt_cfg_info));
	cfg_info.data = data;

	fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);

	switch(touchMode)
	{
		case TOUCH_MODE_NORMAL:
		fw->data = finger_cfg;
		fw->size = sizeof(finger_cfg);
		break;

#ifndef MXT_CFG_EF71
		case TOUCH_MODE_HIGH_SENSITIVE:
		fw->data = glove_cfg;
		fw->size = sizeof(glove_cfg);
		break;

		case TOUCH_MODE_PEN_LETTER:
		fw->data = stylus_cfg;
		fw->size = sizeof(stylus_cfg);
		break;

		case TOUCH_MODE_PEN_DRAW:
		fw->data = stylus_cfg;
		fw->size = sizeof(stylus_cfg);
		break;
#endif

#ifdef MXT_CFG_EF71
		case TOUCH_MODE_CHARGER:
		if(data->batt_mode == BATTERY_PLUGGED_AC) {
			fw->data = charger_ac_cfg;
			fw->size = sizeof(charger_ac_cfg);
		} else if(data->batt_mode == BATTERY_PLUGGED_WIRELESS) {
			fw->data = charger_wireless_cfg;
			fw->size = sizeof(charger_wireless_cfg);
		}
		break;
#endif

		default: 
			error = -EINVAL;
			goto exit;
	}
	
	error = mxt_verify_fw(&cfg_info, fw);
	if (error) {
		dev_err(dev, "Failed to verify fw\n");
		goto exit;
	}
		
	error = mxt_write_config_from_bin(&cfg_info);
	if (error) {
		dev_err(dev, "Failed to write cfg\n");
		goto exit;
	}

	preTouchMode = touchMode;
#if TSP_PATCH	
	t255_user[4] = touchMode;
#endif

	dbg_func_out();

exit:
	kfree(fw);
	if(irq_disabled)
	{	
			enable_irq(data->irq);
			//dbg_cr("enable irq from mxt_update_cfg.\n");
	}
	dev_err(dev, " error %d\n", error);
	return error;
}
#endif

int  mxt_rest_initialize(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	/* Get config CRC from device */
	ret = mxt_read_config_crc(data);
	if(ret)
		dev_err(dev, "Failed Read Config CRC\n");
	
	/* Check config CRC */
	if (data->config_crc == MXT_LATEST_CONFIG_CRC) {
		dev_err(dev, "Skip writing Config:[CRC 0x%06X]\n",
			data->config_crc);
		goto out;
	}

	dev_err(dev, "Writing Config:[CRC 0x%06X!=0x%06X]\n",
					data->config_crc, MXT_LATEST_CONFIG_CRC);

#if MXT_CFG_WRITE_BIN
	/* default finger mode */
    //ymlee_test
	preTouchMode = TOUCH_MODE_NONE;    
	ret = mxt_update_cfg(data, TOUCH_MODE_NORMAL);
	if (ret) {
		dev_err(dev, "Failed update CFG data\n");
		goto out;
	}
#else

	init_touch_config();

#endif

out:
	ret = mxt_init_t7_power_cfg(data);
	if (ret) {
		dev_err(dev, "Failed to initialize power cfg\n");
		return ret;
	}

	/* Backup to memory */
	ret = mxt_backup(data);
	if (ret) {
		dev_err(dev, "Failed backup NV data\n");
		return ret;
	}

	/* Soft reset */
	ret = mxt_soft_reset(data, MXT_RESET_VALUE);
	if (ret) {
		dev_err(dev, "Failed Reset IC\n");
		return ret;
	}

#if TSP_PATCH
	if(sizeof(patch_bin))
		data->patch.patch = patch_bin;
	else
		dev_info(dev, "No patch file %p\n", data->patch.patch);

	if (data->patch.patch){
		dev_info(dev, "mxt_patch_init on probe, size:%ld patch:%p\n", sizeof(patch_bin), data->patch.patch);
		ret = mxt_patch_init(data, data->patch.patch);
		if (ret) {
			dev_err(dev, "Failed to mxt_patch_init\n");
		}
	}
#endif

	return ret;
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	u8 retry_count = 0;

	dbg_func_in();

retry_probe:
	error = mxt_read_info_block(data);
	if (error) {
		error = mxt_probe_bootloader(data);
		if (error) {
			/* Chip is not in appmode or bootloader mode */
			return error;
		} else {
			if (++retry_count > 10) {
				dev_err(&client->dev,
					"Could not recover device from "
					"bootloader mode\n");
				data->state = BOOTLOADER;
				/* this is not an error state, we can reflash
				 * from here */
				 //ymlee_test
				//return -EIO;
				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FWRESET_TIME);
			goto retry_probe;
		}
	}

	//data->state = APPMODE;
	data->state = INIT; //change mode because booting fail 

	dbg_func_out();

	return 0;
}

static ssize_t mxt_update_fw(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;
	bool irq_disabled=false;	

	if(data->flag_probe && data->irq)
	{
		disable_irq(data->irq);
		irq_disabled=true;
		dbg_cr("disable irq from mxt_update_fw.\n");
	}

	error = mxt_load_fw(dev, NULL);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		mxt_soft_reset(data, MXT_RESET_VALUE);

		mxt_free_object_table(data);

		mxt_initialize(data);
	}

	if(irq_disabled)
	{	
		enable_irq(data->irq);
		dbg_cr("enable irq from mxt_update_fw.\n");
	}

	return error;
}

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned int num_mt_slots;
	int id;

	num_mt_slots = data->num_touchids + data->num_stylusids;

	for (id = 0; id < num_mt_slots; id++) {
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	mxt_input_sync(input_dev);
}

static void mxt_start(struct mxt_data *data)
{
	printk("641T_start\n");
	
	/* Discard any messages still in message buffer from before chip went
	 * to sleep */
	mxt_process_messages_until_invalid(data);

	mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
	
	/* Recalibrate since chip has been in deep sleep */
	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);

	if (data->state != APPMODE) {
		enable_irq(data->irq);
		//dbg_cr("enable irq from mxt_start.\n");
		data->state = APPMODE;
	}
}

static void mxt_stop(struct mxt_data *data)
{
	printk("641T_stop\n");

	mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);

	disable_irq(data->irq);
	//dbg_cr("disable irq from mxt_stop.\n");
	
	data->state = SUSPEND;

	mxt_reset_slots(data);
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
  data->cnt_mxt_input_open_func++;
	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
  data->cnt_mxt_input_close_func++;
	mxt_stop(data);
}

static int mxt_initialize_input_device(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct input_dev *input_dev;
	int error = 0xFF;
	unsigned int num_mt_slots;
	int key;

	if(data->T9_reportid_min){
		error = mxt_read_t9_resolution(data);
	}
	
	if (data->T100_reportid_min) {
		error = mxt_read_t100_resolution(data);
	}
	
	if (error) {
		dev_warn(&client->dev, "Failed to initialize T9 resolution\n");
	}

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "atmel_mxt_641t";
	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);
	input_dev->phys = data->phys;
	
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	set_bit(KEY_MENU, input_dev->keybit);	
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_HOMEPAGE, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
//	set_bit(KEY_APP_SWITCH, input_dev->keybit);

#ifdef SKY_PROCESS_CMD_KEY
	set_bit(KEY_SEARCH, input_dev->keybit);
	set_bit(KEY_HOMEPAGE, input_dev->keybit);
	set_bit(KEY_0, input_dev->keybit);
	set_bit(KEY_1, input_dev->keybit);
	set_bit(KEY_2, input_dev->keybit);
	set_bit(KEY_3, input_dev->keybit);
	set_bit(KEY_4, input_dev->keybit);
	set_bit(KEY_5, input_dev->keybit);
	set_bit(KEY_6, input_dev->keybit);
	set_bit(KEY_7, input_dev->keybit);
	set_bit(KEY_8, input_dev->keybit);
	set_bit(KEY_9, input_dev->keybit);
	set_bit(0xe3, input_dev->keybit); /* '*' */
	set_bit(0xe4, input_dev->keybit); /* '#' */
	set_bit(0xe5, input_dev->keybit); /* 'KEY_END' p13106 120105 */
	set_bit(KEY_POWER, input_dev->keybit);
	set_bit(KEY_LEFTSHIFT, input_dev->keybit);
	set_bit(KEY_RIGHTSHIFT, input_dev->keybit);
	set_bit(KEY_LEFT, input_dev->keybit);
	set_bit(KEY_RIGHT, input_dev->keybit);
	set_bit(KEY_UP, input_dev->keybit);
	set_bit(KEY_DOWN, input_dev->keybit);
	set_bit(KEY_ENTER, input_dev->keybit);
	set_bit(KEY_SEND, input_dev->keybit);
	set_bit(KEY_END, input_dev->keybit);
	set_bit(KEY_F1, input_dev->keybit);
	set_bit(KEY_F2, input_dev->keybit);
	set_bit(KEY_F3, input_dev->keybit);				
	set_bit(KEY_F4, input_dev->keybit);
	set_bit(KEY_VOLUMEUP, input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, input_dev->keybit);
	set_bit(KEY_CLEAR, input_dev->keybit);
	set_bit(KEY_CAMERA, input_dev->keybit);
	set_bit(KEY_VT_CALL, input_dev->keybit);       // P13106 VT_CALL
#endif // SKY_PROCESS_CMD_KEY
	
	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids + data->num_stylusids;
	error = input_mt_init_slots(input_dev, num_mt_slots,0);
	if (error)
		goto err_free_mem;
		
	input_set_abs_params(input_dev, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,  0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION, 0, 255, 0, 0);

	/* For T63 active stylus */
	if (data->T63_reportid_min) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
			0, MT_TOOL_MAX, 0, 0);
	}

	/* For T15 key array */
	if (data->T15_reportid_min) {
		mxt_t15_keystatus = 0;
		for (key = 0; key < ARRAY_SIZE(mxt_t15_keys); key++) {
			input_set_capability(input_dev, EV_KEY, mxt_t15_keys[key]);
		}
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&client->dev, "Error %d registering input device\n",
			error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct mxt_data *data;
	int error;

#if defined (OFFLINE_CHARGER_TOUCH_DISABLE)   
	oem_pm_smem_vendor1_data_type *smem_id_vendor1_ptr;  
	smem_id_vendor1_ptr =  (oem_pm_smem_vendor1_data_type*)smem_alloc(SMEM_ID_VENDOR1,sizeof(oem_pm_smem_vendor1_data_type),0,SMEM_ANY_HOST_FLAG);
	 
	if(smem_id_vendor1_ptr->power_on_mode == 0)
	{    
			dbg_cr(" OFFLINE_CHARGER is enabled. And Touch Driver IRQ is disabled\n");   
			TSP_PowerOff();    
			return -1;  
	}
#endif

#ifdef TOUCH_DEV_TREE_SUPPORT
	if(mxt_parse_dt(client) != 1){
		dbg_cr("parse device tree fail....!!\n");
		return 0;		
	}
	
	TSP_PowerOn();	
#endif
	
	dbg_func_in();
	
	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if(!data)
		return -EINVAL;
		
	data->state = INIT;

	data->client = client;
	data->irq = client->irq;

	dbg_hw("data->irq : [%d], client->irq : [%d]\n", data->irq, client->irq);
	
	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);	

	error = mxt_initialize(data);
	if (error)
		goto err_free_mem;

#if PAN_T15_KEYARRAY_ENABLE
	mPan_KeyArray[0].key_state = false;
	mPan_KeyArray[0].key_num = KEY_MENU;
	mPan_KeyArray[1].key_state = false;
	mPan_KeyArray[1].key_num = KEY_BACK;
#endif


	/* update fw */
	//ymlee_test
	if(data->info || data->state == BOOTLOADER){		
 
		bool isUpdate = false;

		if(data->state == BOOTLOADER){
			isUpdate = true;
		} 
		else if((data->info->version != MXT_LATEST_FW_VERSION)
			|| (data->info->build != MXT_LATEST_FW_BUILD)){
			isUpdate = true;

			dev_err(&client->dev, "Current FW: 0x%2x, BUILD: 0x%2x\n",
				data->info->version, data->info->build);
			dev_err(&client->dev, "LATEST FW: 0x%2x, BUILD: 0x%2x\n",
				MXT_LATEST_FW_VERSION, MXT_LATEST_FW_BUILD);
		}
#if  1 // FIRMWARE_UPDATE

		if(isUpdate){
			int i = 0;
			int retry_cnt = 5;
			for(i=0; i<retry_cnt; i++){
				error = mxt_update_fw(&client->dev);
				
				if((data->state == INIT) && 
					(data->info->version == MXT_LATEST_FW_VERSION) &&
			 		(data->info->build == MXT_LATEST_FW_BUILD)){
					dbg_cr("After FW update: 0x%2x, BUILD: 0x%2x\n",data->info->version, data->info->build);
					break;
				}
				dev_err(&client->dev, "FAIL FW upgrade, retry count  %d/%d ", i, retry_cnt);
				if(i==(retry_cnt-1)) return -EIO;
			}
		}
		
#endif 		
	}

	data_common = data;

	error = mxt_rest_initialize(data);
	if (error) {
		dev_err(&client->dev, "Error %d MXT CFG WRITE FAIL\n",
			error);
		goto err_free_object;
	}

	error = mxt_initialize_input_device(data);
	if (error) {
		dev_err(&client->dev, "Error %d registering input device\n",
			error);
		goto err_free_object;
	}


	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_unregister_device;
	}

#if TSP_ITDEV
	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			error);
		goto err_free_irq;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}
#endif

#ifdef SKY_PROCESS_CMD_KEY
	touch_fops_init();
	touch_monitor_init();
#endif

	//mxt_Multitouchscreen_T100_Init(); //temp blocked 

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	data->flag_probe=true;

	dbg_func_out();

	return 0;
	
#if TSP_ITDEV
err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
#endif
err_free_irq:
	free_irq(client->irq, data);
err_unregister_device:
	input_unregister_device(data->input_dev);
	data->input_dev = NULL;
err_free_object:
	mxt_free_object_table(data);
err_free_mem:
	kfree(data);
	return error;
}


static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	
#ifdef SKY_PROCESS_CMD_KEY
	touch_fops_exit();
	touch_monitor_exit();
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

#if TSP_ITDEV
	sysfs_remove_bin_file(&client->dev.kobj, &data->mem_access_attr);
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
#endif

	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	mxt_free_object_table(data);
	kfree(data);

	return 0;
}

static int mxt_suspend(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
    
	if (data->state == SUSPEND) {
		return -1;
	}
	
	mutex_lock(&input_dev->mutex);
	
	if (input_dev->users)
		mxt_stop(data);
	
	clear_event(data, TSC_CLEAR_ALL);
	
	input_mt_destroy_slots(data->input_dev);
	
#if PAN_T15_KEYARRAY_ENABLE
	if(mPan_KeyArray[0].key_state)
	mPan_KeyArray[0].key_state=false;
	if(mPan_KeyArray[1].key_state)
	mPan_KeyArray[1].key_state=false;    
#endif

	mutex_unlock(&input_dev->mutex);

	TSP_PowerOff();
	
	dbg_cr("%s : Touch suspend.\n",__func__);
	
	return 0;
}

static int  mxt_resume(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	int error = 0;
  
	if (data->state == APPMODE) {
		return -1;
	}
	
	TSP_PowerOn();
	
	mutex_lock(&input_dev->mutex);
	
	input_mt_init_slots(data->input_dev, MAX_NUM_FINGER, 1);
	
	clear_event(data, TSC_CLEAR_ALL);

	error = mxt_soft_reset_resume(data, MXT_RESET_VALUE);
	if (error) {
		printk("641T Failed to Reset\n");
	}
	
	if (input_dev->users)
		mxt_start(data);

	if (mxt_update_cfg_no_backup(data, mTouch_mode)) {
		printk("%s : Failed update CFG data to set touch mode to %d\n", __func__, mTouch_mode);
	}

	mutex_unlock(&input_dev->mutex);
	
	dbg_cr("%s : Touch resume.\n",__func__);
	
	return 0;
}

#define MXT_DEV_NAME	"atmel_mxt_641t"

void  mxt_front_test_init(void)
{
	struct mxt_data *data = data_common; //i2c_get_clientdata(client);
	bool irq_disabled=false;
	if(data->flag_probe && data->irq)
	{	
		irq_disabled=true;
		disable_irq(data->irq);
	}

	TSP_PowerOff();
	msleep(20);	  
	TSP_PowerOn();
	msleep(100); 	
	mxt_rest_initialize(data);

	if(irq_disabled)
	{	
		enable_irq(data->irq);
	}
	dbg_cr("641T init finished!\n");
	return ;
}

void mxt_shutdown(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

  data->cnt_mxt_shutdown_func++;
	if((data->state == APPMODE) && (data->irq) )
	{	
		disable_irq(data->irq);
	}
	data->state = SHUTDOWN;
}

static const struct i2c_device_id mxt_id[] = {
	{ MXT_DEV_NAME, 0 },
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "mXT224", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct of_device_id mxt_match_table[] = {
	{ .compatible = "atmel,atmel_mxt_641t",},
	{ },
};

//p11774 name was changed according to the atmel test app execution.
//p11774 14.07.03 added for PM suspend&resume
static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "Atmel MXT540S",
		.owner	= THIS_MODULE,
		.of_match_table = mxt_match_table,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
#ifndef TOUCH_DEV_TREE_SUPPORT
	int rc;

	rc = TSP_PowerOn();
	
	dbg_cr("%s : TSP PWR (rc=%d)\n", __func__, rc);

	if(rc<0){
		dbg_cr("%s : TSP PWR FAIL!! (rc=%d)\n", __func__, rc);
		return rc;
	}
#endif
	printk("641T mxt_init. in\n");
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

late_initcall(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");

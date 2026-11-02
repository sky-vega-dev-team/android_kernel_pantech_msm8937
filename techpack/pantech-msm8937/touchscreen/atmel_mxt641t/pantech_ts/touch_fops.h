#ifndef _TOUCH_FOPS_
#define _TOUCH_FOPS_

#include <linux/miscdevice.h>

static int ts_fops_open(struct inode *inode, struct file *filp)
{
	filp->private_data = data_common;
	return 0;
}

static long ts_fops_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;	
	int touch_mode_flag = 0;

	switch (cmd) 
	{
		case TOUCH_IOCTL_READ_LASTKEY:
			break;
		case TOUCH_IOCTL_DO_KEY:
			dbg_ioctl("TOUCH_IOCTL_DO_KEY  = %ld\n",(unsigned long int)argp);			
			if ( (unsigned long int)argp == KEY_NUMERIC_STAR )
				input_report_key(data_common->input_dev, 0xe3, 1);
			else if ( (unsigned long int)argp == KEY_NUMERIC_POUND )
				input_report_key(data_common->input_dev, 0xe4, 1);
			else
				input_report_key(data_common->input_dev, (unsigned long int)argp, 1);
			input_sync(data_common->input_dev); 
			break;
		case TOUCH_IOCTL_RELEASE_KEY:		
			dbg_ioctl("TOUCH_IOCTL_RELEASE_KEY  = %ld\n",(unsigned long int)argp);
			if ( (unsigned long int)argp == 0x20a )
				input_report_key(data_common->input_dev, 0xe3, 0);
			else if ( (unsigned long int)argp == 0x20b )
				input_report_key(data_common->input_dev, 0xe4, 0);
			else
				input_report_key(data_common->input_dev, (unsigned long int)argp, 0);
			input_sync(data_common->input_dev); 
			break;		
		case TOUCH_IOCTL_DEBUG:
			dbg_ioctl("Touch Screen Read Queue ~!!\n");	
			break;
		case TOUCH_IOCTL_CLEAN:
			dbg_ioctl("Touch Screen Previous Data Clean ~!!\n");
			//clear_event(TSC_CLEAR_ALL);
			break;
		case TOUCH_IOCTL_RESTART:
			dbg_ioctl("Touch Screen Calibration Restart ~!!\n");			
			mxt_start(data_common);
			break;
		case TOUCH_IOCTL_PRESS_TOUCH:
			input_report_abs(data_common->input_dev, ABS_MT_POSITION_X, (int)(arg&0x0000FFFF));
			input_report_abs(data_common->input_dev, ABS_MT_POSITION_Y, (int)((arg >> 16) & 0x0000FFFF));
			input_report_abs(data_common->input_dev, ABS_MT_TOUCH_MAJOR, 255);
			input_report_abs(data_common->input_dev, ABS_MT_WIDTH_MAJOR, 1);			
			input_sync(data_common->input_dev);
			break;
		case TOUCH_IOCTL_RELEASE_TOUCH:		
			input_report_abs(data_common->input_dev, ABS_MT_POSITION_X, (int)(arg&0x0000FFFF));
			input_report_abs(data_common->input_dev, ABS_MT_POSITION_Y, (int)((arg >> 16) & 0x0000FFFF));
			input_report_abs(data_common->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(data_common->input_dev, ABS_MT_WIDTH_MAJOR, 1);			
			input_sync(data_common->input_dev); 
			break;			
		case POWER_OFF:
			pm_power_off();
			break;
		case TOUCH_IOCTL_DELETE_ACTAREA:
			touchscreen_config.yloclip = 0;		// Change Active area
			touchscreen_config.yhiclip = 0;
			if (write_multitouchscreen_T100_config(0, touchscreen_config)<0){
				dbg_ioctl("mxt_Multitouchscreen_config Error!!!\n");
			}
			break;
		case TOUCH_IOCTL_RECOVERY_ACTAREA:
			touchscreen_config.yloclip = 15;	// Change Active area
			touchscreen_config.yhiclip = 15;
			if (write_multitouchscreen_T100_config(0, touchscreen_config)<0)
				dbg_ioctl("mxt_Multitouchscreen_config Error!!!\n");
			break;
		case TOUCH_IOCTL_INIT:
			mxt_front_test_init();
			dbg_ioctl("Touch init \n");
			break;

		case TOUCH_IOCTL_OFF:
			dbg_ioctl("Touch off \n");
			TSP_PowerOff();
			break;		

		case TOUCH_IOCTL_SENSOR_X:
			{
				int send_data;
//#if MXT_CFG_WRITE_BIN
				send_data = 26; //21;
//#else
//				send_data = touchscreen_config.xsize;
//				dbg_cr("sensor_X entered! %d \n",touchscreen_config.xsize);
//#endif
				dbg_cr("sensor_X entered! %d \n",send_data);

				if (copy_to_user(argp, &send_data, sizeof(send_data)))
					return false;
			}

			break;
		case TOUCH_IOCTL_SENSOR_Y:
			{
				int send_data;
//#if MXT_CFG_WRITE_BIN
				send_data = 15; //12;
//#else
//				send_data = touchscreen_config.ysize;
//				dbg_cr("sensor_Y entered! %d \n",touchscreen_config.ysize);
//#endif
				dbg_cr("sensor_Y entered! %d \n",send_data);

				if (copy_to_user(argp, &send_data, sizeof(send_data)))
					return false;
			}
			break;

		case TOUCH_IOCTL_CHECK_BASE:
		case TOUCH_IOCTL_START_UPDATE:
			break;

		case TOUCH_IOCTL_SELF_TEST:
			{
				int* send_byte;
				disable_irq(data_common->client->irq);
				mutex_lock(&data_common->lock);
				dbg_cr("selftest entered!\n");
				send_byte = diag_debug(MXT_REFERENCE_MODE);
				dbg_cr("reference data sent!\n");
				mutex_unlock(&data_common->lock);
				enable_irq(data_common->client->irq);
				diagnostic_min = MXT_REFERENCE_MIN;
				diagnostic_max = MXT_REFERENCE_MAX;

				if (copy_to_user(argp, send_byte, sizeof(int) * MXT_USED_CHANNEL_NUM)){ // MXT_MAX_CHANNEL_NUM
					dbg_cr("copy_to_user false\n");	
					return false;
				}
				dbg_cr("copy_to_user finish!\n");
				return touch_diagnostic_ret;
				break;
			}
		case TOUCH_IOCTL_DIAGNOSTIC_MIN_DEBUG:
			return MXT_REFERENCE_MIN;
			break;
		case TOUCH_IOCTL_DIAGNOSTIC_MAX_DEBUG:
			return MXT_REFERENCE_MAX;
			break;
		case TOUCH_IOCTL_DIAG_DELTA:
			{
				int* send_byte;
				disable_irq(data_common->client->irq);
				mutex_lock(&data_common->lock);
				send_byte = diag_debug(MXT_DELTA_MODE);
				mutex_unlock(&data_common->lock);
				enable_irq(data_common->client->irq);
				if (copy_to_user(argp, send_byte, sizeof(int) * MXT_USED_CHANNEL_NUM))
					return false;

				return touch_diagnostic_ret;
				break;
			}
//20140703_for_selftest +
		case TOUCH_IOCTL_SELF_CAP_GET_ONOFF:
		{
#if TSP_PATCH //0			
			int ret = mxt_get_t8_self_cfg(data_common);
			return ret;
#endif
		}

		case TOUCH_IOCTL_SELF_CAP_SET_ONOFF:
#if TSP_PATCH //0			
			mxt_set_t8_self_cfg(data_common, arg);
			msleep(10);
#endif			
			break;
//20140703_for_selftest -
			
		case TOUCH_IOCTL_MULTI_TSP_OBJECT_SEL:
#if MXT_CFG_WRITE_BIN
			if ( arg < TOUCH_MODE_MAX_NUM ) {

				if(mTouch_mode == (int)arg) 
				{
					dbg_cr("[touch_monitor] Touch Mode selection is equal, mTouch_mode -> %d\n",mTouch_mode);
				}
				else
				{
					mTouch_mode = (unsigned long int)argp;
					if (mxt_update_cfg(data_common, mTouch_mode)) {
						dbg_cr("Failed update CFG data\n");
					}
					if (data_common->state == SUSPEND) {
						mxt_set_t7_power_cfg(data_common, MXT_POWER_CFG_DEEPSLEEP);
					}
					dbg_cr("Mode changing finished.\n");
				}

			}
					
			// format: obj_sel_# (0~9)
			if ( arg < TOUCH_MODE_MAX_NUM ) 
			{
				if(mTouch_mode == (int)arg) 
				{
					dbg_cr("[touch_monitor] Touch Mode selection is equal, mTouch_mode -> %d\n",mTouch_mode);
				}
				else 
				{
					mTouch_mode = arg;
					init_touch_config();
					dbg_cr("[touch_monitor] Suspend Touch Mode selection is %d\n", mTouch_mode);
				}
			
			}
		
			else if ( arg == 20 || arg == 21 ) 
			{
		  		if(mTouch_mode ==2 || mTouch_mode ==3)
				{
		    			if(arg == 20)
					{
		      				dbg_cr("Touch Pen Idle mode\n");
		      				mxt_Stylus_T47_Config_Init();
		    			}
					else if(arg == 21)
					{
					      dbg_cr("Touch Pen Text mode\n");
		      				touchscreen_config = obj_multi_touch_t100[mTouch_mode];
		      				touchscreen_config.movhysti=2;
		      				touchscreen_config.movhystn=1;
		      				if (write_multitouchscreen_T100_config(0, touchscreen_config) != CFG_WRITE_OK)
		        				dbg_cr("T100 Configuration Fail!!!, %s, Line %d \n", __func__, __LINE__);
		      				stylus_t47_config = obj_stylus_t47[mTouch_mode];
					       stylus_t47_config.cfg=3;
		      				if (write_stylus_T47_config(stylus_t47_config) != CFG_WRITE_OK)
			      				dbg_cr("T47 Configuration Fail!!! , Line %d \n\r", __LINE__);		      
		    			}
		  		}
			}
#endif
			break;
			
		case TOUCH_IOCTL_MULTI_TSP_OBJECT_GET:
			touch_mode_flag = mTouch_mode;
			if (copy_to_user(argp, &touch_mode_flag, sizeof(touch_mode_flag))) {
				dbg_ioctl(" TOUCH_IOCTL_MULTI_TSP_OBJECT_GET\n");
			}		
			break;

		case TOUCH_IOCTL_CHARGER_MODE:
			dbg_cr("TOUCH_IOCTL_CHARGER_MODE %d\n", (int)arg);
			data_common->batt_mode = (int)arg;
			if ((int)arg == BATTERY_PLUGGED_AC || (int)arg == BATTERY_PLUGGED_WIRELESS){
				mTouch_mode = TOUCH_MODE_CHARGER;
				if (mxt_update_cfg_no_backup(data_common, mTouch_mode)) {
					dbg_cr("Failed update TOUCH_IOCTL_CHARGER_MODE CFG data\n");
				}
			}
			else{
				mTouch_mode = TOUCH_MODE_NORMAL;
				if (mxt_update_cfg_no_backup(data_common, mTouch_mode)) {
					dbg_cr("Failed update TOUCH_MODE_NORMAL CFG data\n");
				}
			}
#if TSP_PATCH // 0
			data_common->charging_mode = (int)arg;
			dbg_cr("charging_mode %d\n", data_common->charging_mode);
			if (data_common->charging_mode) {
				if (data_common->patch.event_cnt) {
					mxt_patch_test_event(data_common, MXT_PATCH_EVENT_TA_PLUGGED);
				}
			} else {
				if (data_common->patch.event_cnt) {
					mxt_patch_test_event(data_common, MXT_PATCH_EVENT_TA_UNPLUGGED);
				}
			}
#endif
			break;

		case TOUCH_IOCTL_HALL_IC_GPIO_GET:

			touch_mode_flag = gpio_get_value(9);
			if (copy_to_user(argp, &touch_mode_flag, sizeof(touch_mode_flag))) {
				dbg_ioctl("TOUCH_IOCTL_HALL_IC_GPIO_GET\n");
			}
			dbg_op("[%s] Hall ic Status: %d\n", __func__, touch_mode_flag);
    			break;

		case TOUCH_IOCTL_RESUME:
			dbg_cr("TOUCH_IOCTL_RESUME %d\n", (int)arg);
			mxt_resume(data_common);
			break;

		case TOUCH_IOCTL_SUSPEND:
			dbg_cr("TOUCH_IOCTL_SUSPEND %d\n", (int)arg);
			mxt_suspend(data_common);
			break;

		default:
			break;
	}
	return true;
}

static ssize_t ts_fops_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int i=0,j,nBufSize=0;
	if((size_t)(*ppos) > 0) return 0;	
  	dbg_ioctl(" Touch IO Write function\n");

	if(buf!=NULL)
	{
		nBufSize=strlen(buf);

		if(strncmp(buf, "debug_",6)==0)
		{ 
			if(buf[6] > '0'){
				i = buf[6] - '1';
				if(pan_debug_state & 0x00000001 <<i) {
					pan_debug_state &= ~(0x00000001 <<i);	
				}else{
					pan_debug_state |= (0x00000001 <<i);
				}
			}
			dbg_cr(" pan_debug_state -> %x, i-> %d\n",pan_debug_state,i);
		}
/*
		if(strncmp(buf, "wifion", 6)==0)
		{		 
			if(sysfs_chmod_file(&mxt_fw30_data->client->dev.kobj,&mxt_fw30_data->mem_access_attr.attr,S_IRWXUGO))
				printk("[TOUCH] sysfs_chmod_file is failed\n");
			i=sysfs_chmod_file(&mxt_fw30_data->client->dev.kobj,&(*mxt_attrs[0]), S_IRWXUGO);
		}
		if(strncmp(buf, "wifioff", 7)==0)
		{
			if(sysfs_chmod_file(&mxt_fw30_data->client->dev.kobj,&mxt_fw30_data->mem_access_attr.attr,S_IRUGO | S_IWUSR))
				printk("[TOUCH] sysfs_chmod_file is failed\n");		
			i=sysfs_chmod_file(&mxt_fw30_data->client->dev.kobj,&(*mxt_attrs[0]), S_IWUSR | S_IRUSR);
		}
		*/	
#if MXT_CFG_WRITE_BIN
			
		if(strncmp(buf, "pen",3)==0)
		{
			mTouch_mode = 2;
			if (mxt_update_cfg(data_common, mTouch_mode)) 
			{
				dbg_cr("Failed update CFG data\n");
			}
		}
		if(strncmp(buf, "glove",5)==0)
		{
			mTouch_mode = 1;
			if (mxt_update_cfg(data_common, mTouch_mode)) 
			{
				dbg_cr("Failed update CFG data\n");
			}
		}
		if(strncmp(buf, "touch",5)==0)
		{
			mTouch_mode = 0;
			if (mxt_update_cfg(data_common, mTouch_mode)) 
			{
				dbg_cr("Failed update CFG data\n");
			}
		}
#else

		if(strncmp(buf, "pen",3)==0)
		{
			mTouch_mode = 2;
			init_touch_config();
			dbg_cr("[touch_test] Touch Mode selection is %d\n", mTouch_mode);
		}
		if(strncmp(buf, "glove",5)==0)
		{
			mTouch_mode = 1;
			init_touch_config();
			dbg_cr("[touch_test] Touch Mode selection is %d\n", mTouch_mode);
		}
		if(strncmp(buf, "touch",5)==0)
		{
			mTouch_mode = 0;
			init_touch_config();
			dbg_cr("[touch_test] Touch Mode selection is %d\n", mTouch_mode);
		}
#endif
		if(strncmp(buf, "touchid", 7)==0)
		{	
			j=0;
			for (i = 0 ; i < MAX_NUM_FINGER; ++i) { 
				if (fingerInfo[i].status == -1)
					continue;
				j++;
				dbg_ioctl("[TOUCH] TOUCH ID => %d, fingerInfo[i].status=> %d, fingerInfo[i].mode=> %d\n",i,fingerInfo[i].status,fingerInfo[i].mode); 
			}
		//	dbg_ioctl("[TOUCH] TOUCH ID CNT => %d, cal_correction_limit=> %d\n", j, cal_correction_limit); 
		}
/*
		if(strncmp(buf, "checkcal", 8)==0)
		{			
			check_chip_calibration();
		}
		if(strncmp(buf, "cal", 3)==0)
		{			
			calibrate_chip();
		}
*/
		if(strncmp(buf, "reset", 5)==0)
		{			
			mxt_soft_reset(data_common, MXT_RESET_VALUE);
		}
/*
		if(strncmp(buf, "reference", 9)==0)
		{			
			disable_irq(data_common->client->irq);
			mutex_lock(&data_common->lock);
			diag_debug(MXT_REFERENCE_MODE);
			mutex_unlock(&data_common->lock);
			enable_irq(data_common->client->irq);
		}
		if(strncmp(buf, "delta", 5)==0)
		{			
			disable_irq(data_common->client->irq);
			mutex_lock(&data_common->lock);
			diag_debug(MXT_DELTA_MODE);
			mutex_unlock(&data_common->lock);
			enable_irq(data_common->client->irq);
		}
*/
		if(strncmp(buf, "off",3)==0)
		{			
			TSP_PowerOff();   
		}		
		if(strncmp(buf, "on",2)==0)
		{			
			TSP_PowerOn();
		}		
		if(strncmp(buf, "obj_sel", 7)==0)
		{
			// format: obj_sel_# (0~9)
			i = buf[7] - '0';
			if ( i < TOUCH_MODE_MAX_NUM ) 
			{
				mTouch_mode = i;
                		disable_irq(data_common->client->irq);
                		init_touch_config();
	            		enable_irq(data_common->client->irq);
	      
				dbg_cr(" TOUCH MODE selection is %d\n", mTouch_mode);
			}			

		}
		if(strncmp(buf, "init",4)==0)
		{			
			mxt_front_test_init();   
		}

#ifdef ITO_TYPE_CHECK
		if(strncmp(buf, "id",2)==0)
		{			
			read_touch_id();   
		}
#endif
#if TSP_PATCH
		if(strncmp(buf, "patch_update",12)==0)
		{			
			mxt_load_patch_from_ums(&data_common->client->dev, NULL,NULL,0);   
		}
#endif
		if(strncmp(buf, "movedbg_on",10)==0)
		{			
			data_common->move_dbg = true;   
		}
		if(strncmp(buf, "movedbg_off",11)==0)
		{			
			data_common->move_dbg = false;   
		}
		if(strncmp(buf, "cfg_update_on",13)==0)
		{			
			data_common->no_cfg_update = false;   
		}
		if(strncmp(buf, "cfg_update_off",14)==0)
		{			
			data_common->no_cfg_update = true;   
		}
		if(strncmp(buf, "cnt_func",8)==0)
		{			
			dbg_cr(" mxt_input_open -> %d, mxt_input_close -> %d, mxt_shutdown -> %d\n"
			,data_common->cnt_mxt_input_open_func,data_common->cnt_mxt_input_close_func,
			data_common->cnt_mxt_shutdown_func)
		}
		if(strncmp(buf, "suspend", 7) == 0)
		{
				mxt_suspend(data_common);
		}
		if(strncmp(buf, "resume", 6) == 0)
		{
				mxt_resume(data_common);
  		}

	}

	*ppos +=nBufSize;
	return nBufSize;
}

static struct file_operations ts_fops = {
	.owner = THIS_MODULE,
	.open = ts_fops_open,
	.write = ts_fops_write,
	.unlocked_ioctl = ts_fops_ioctl, // mirinae
#ifdef CONFIG_COMPAT
	.compat_ioctl = ts_fops_ioctl, // mirinae
#endif
};

static struct miscdevice touch_event = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch_fops",
	.fops = &ts_fops,
};


void touch_fops_init(void) {
	int rc;
	rc = misc_register(&touch_event);
	if (rc) {
		pr_err("::::::::: can''t register touch_fops\n");
	}
}

// call in driver remove function
void touch_fops_exit(void) {
	misc_deregister(&touch_event);
}

#endif //_TOUCH_FOPS_

/* 
 * Touch Monitor Interface 
 * Ver 0.1
 */
#include <linux/version.h>
#include "touch_log.h"

#include <linux/miscdevice.h>
#include <asm/uaccess.h>

static int monitor_open(struct inode *inode, struct file *file);
//static ssize_t monitor_read(struct file *file, char *buf, size_t count, loff_t *ppos);
//static ssize_t monitor_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
//static int monitor_release(struct inode *inode, struct file *file);
#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(3,0,0))
static int monitor_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#else
static long monitor_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#endif
static ssize_t monitor_write(struct file *file, const char *buf, size_t count, loff_t *ppos);


/*
 * vendor_id : 
 * ateml(1) cypress(2)
 * model_id : 
 * ef39s(0390) ef40s(0400) ef40k (0401)
 * presto(9000) kelly(9100)
 * type_id : 
 * model manager would manage ito or color type.

 * return vendor_id*100*10000 + model_id*100 + type_id;
 */ 
static int vendor_id = 1;
static int model_id = 9100;
static int type_id = 0;

static struct file_operations monitor_fops = 
{
	.owner =    THIS_MODULE,
#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(3,0,0))
	.ioctl =    monitor_ioctl,
#else
	.unlocked_ioctl =   monitor_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =   monitor_ioctl,
#endif
#endif
//	.read =     monitor_read,
	.write =    monitor_write,
	.open =     monitor_open,
//	.release =  monitor_release
};

static struct miscdevice touch_monitor = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "touch_monitor",
	.fops =     &monitor_fops
};

typedef struct
{
	int touch_count; 

} touch_monitor_info_t;
//static touch_monitor_info_t *touch_monitor_info; 

static int ioctl_debug(unsigned long arg) 
{

	//pm_message_t pm_null={0}; //p11774 14.07.03 blocked for PM suspend&resume
	switch (arg)
	{
	case IOCTL_DEBUG_SUSPEND:
		mxt_suspend(data_common);
		break;
	case IOCTL_DEBUG_RESUME:
		mxt_resume(data_common);
		break;
// 	case IOCTL_DEBUG_GET_TOUCH_ANTITOUCH_INFO:
// 		check_chip_calibration();
// 		return get_touch_antitouch_info();
// 		break;
// 	case IOCTL_DEBUG_TCH_CH:
// 		return debugInfo.tch_ch;
// 		break;
// 	case IOCTL_DEBUG_ATCH_CH:
// 		return debugInfo.atch_ch;
// 		break;
// 	case IOCTL_GET_CALIBRATION_CNT:
// 		return debugInfo.calibration_cnt;
// 		break;
	default:
		break;
	}

	return 0;
}

#ifdef MXT_CFG_EF71
#include "../config/EF71/ef71_cfg_reference.h"
int mxt_verify_fw(struct mxt_cfg_info *cfg_info, const struct firmware *fw);
static int mxt_write_config_from_bin(struct mxt_cfg_info *cfg_info);
#endif

static int* diag_debug(int command) 
{
	/*command 0x10: Delta, 0x11: Reference*/
	uint8_t data_buffer[130] = { 0 };
	uint8_t data_byte = 0; /* dianostic command to get touch refs*/
	uint16_t diag_address;
	uint8_t page;
	int i;
	int j=0, k=0;
	uint16_t value;
	uint16_t max_page = (MXT_MAX_CHANNEL_NUM/MXT_CHANNELS_PER_PAGE); // max_page = ceil(540 / (128/2) );
	int16_t signed_value;
	int rc=0;
	struct mxt_data *data = data_common;
	struct device *dev = &data->client->dev;

	dbg_func_in();

#ifdef MXT_CFG_EF71
	if (command == MXT_REFERENCE_MODE){
		struct firmware *fw = NULL;
		struct mxt_cfg_info cfg_info;
		
		memset(&cfg_info, 0, sizeof(struct mxt_cfg_info));
		cfg_info.data = data;
		
		fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);
		fw->data = ref_cfg;
		fw->size = sizeof(ref_cfg);

		rc = mxt_verify_fw(&cfg_info, fw);
		if (rc) {
			dev_err(dev, "Failed to verify for REF CFG\n");
		}
			
		rc = mxt_write_config_from_bin(&cfg_info);
		if (rc) {
			dev_err(dev, "Failed to write for REF CFG\n");
		}

		kfree(fw);
	}
#endif

	diagnostic_min = 0;
	diagnostic_max = 0;
	/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
	memset(reference_data, 0, sizeof(reference_data));
	rc = mxt_t6_command(data, MXT_COMMAND_DIAGNOSTIC, command, false);
	if (rc)
		dev_err(dev, "[TOUCH] diag mode is failed.\n");

	msleep(20); 

	/* read touch flags from the diagnostic object - clear buffer so the while loop can run first time */
	diag_address = data->T37_address;
 	/* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/
	/* count up the channels/bits if we recived the data properly */

	printk("diag_debug_start\n");
	
	for (page = 0; page < max_page; page++) {
		
	    //rc=read_mem(diag_address, 130,data_buffer);
	    rc = __mxt_read_reg(data->client, diag_address,130, data_buffer);
	    msleep(20);
	    if(rc)
	      dev_err(dev, "[TOUCH] read_mem is failed.\n");
	    
		for(i = 2; i < 130; i+=2) /* check X lines - data is in words so increment 2 at a time */
		{
			value =  (data_buffer[1+i]<<8) + data_buffer[i];
			//if(j>=MXT_MAX_CHANNEL_NUM)continue;
			if (((j%MXT_MAX_Y_CHANNEL) < MXT_USED_Y_CHANNEL)  && (k < MXT_MAX_CHANNEL_NUM))
			{
				if (command == MXT_REFERENCE_MODE){
					reference_data[k] = value;			
				}
				else if (command == MXT_DELTA_MODE){
					signed_value = value;
					reference_data[k] = (int16_t)value;
					diagnostic_min = min(diagnostic_min, reference_data[k] );
					diagnostic_max = max(diagnostic_max, reference_data[k] );
				}
				else{
					reference_data[k] = value;
				}
				//printk("[%d]:[%d]\n",k,reference_data[k]);
				k++;
			}else{
			}
			j++;
		}
   		
		data_byte = 0x01;
		mxt_t6_command(data, MXT_COMMAND_DIAGNOSTIC, data_byte, false);
		msleep(20);
	}

#ifdef MXT_CFG_EF71
	if (command == MXT_REFERENCE_MODE){
		mxt_soft_reset(data_common, MXT_RESET_VALUE);
	}
#endif
	printk("diag_debug_finish\n");
	return (int *)reference_data;;
}

static int* diag_debug_for_selfcap(int command) 
{
	uint8_t data_buffer[130] = { 0 };
	uint16_t diag_address;
	uint8_t page;
	int i = 0 , k = 0;
	uint16_t value;
	uint16_t max_page = 3; //touch, hover, prox
	int16_t signed_value;
	int rc=0;
	struct mxt_data *data = data_common;
	struct device *dev = &data->client->dev;

	diagnostic_min = 0;
	diagnostic_max = 0;
	memset(reference_data, 0, sizeof(reference_data));

	rc = mxt_t6_command(data, MXT_COMMAND_DIAGNOSTIC, command, false);
	if (rc)
		dev_err(dev, "[TOUCH] diag mode is failed.\n");
	msleep(20); 

	diag_address = data->T37_address;
	for (page = 0; page < max_page; page++) {
		
	    rc = __mxt_read_reg(data->client, diag_address,130, data_buffer);
	    msleep(20);
	    if(rc)
	      dev_err(dev, "[TOUCH] read_mem is failed.\n");
	    
		for(i = 2; i < 130; i+=2) /* check X lines - data is in words so increment 2 at a time */
		{
			value =  (data_buffer[1+i]<<8) + data_buffer[i];

			if(k < MXT_MAX_SELFCAP_DATA)
			{
				if (command == MXT_SELF_REFERENCE_MODE){
					reference_data[k] = value;			
				}
				else if (command == MXT_SELF_DELTA_MODE){
					signed_value = value;
					reference_data[k] = (int16_t)value;
					diagnostic_min = min(diagnostic_min, reference_data[k] );
					diagnostic_max = max(diagnostic_max, reference_data[k] );
				}
				else{
					reference_data[k] = value;
				}
				k++;
			}
		}
	   		
		mxt_t6_command(data, MXT_COMMAND_DIAGNOSTIC, (u8)0x01, false);
		msleep(20);
	}
	return (int *)reference_data;;
}

static int monitor_open(struct inode *inode, struct file *filp)
{
	filp->private_data = data_common;
	return 0;
}

static ssize_t monitor_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int ret=0, nBufSize=0;
	struct mxt_data *data = data_common;
	struct device *dev = &data->client->dev;
	int i=0;

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

		if(strncmp(buf, "wifion", 6)==0)
		{		 
			if(sysfs_chmod_file(&data_common->client->dev.kobj,&data_common->mem_access_attr.attr,S_IRWXUGO))
				printk("[TOUCH] sysfs_chmod_file is failed\n");
			i=sysfs_chmod_file(&data_common->client->dev.kobj,&(*mxt_attrs[0]), S_IRWXUGO);
		}
		if(strncmp(buf, "wifioff", 7)==0)
		{
			if(sysfs_chmod_file(&data_common->client->dev.kobj,&data_common->mem_access_attr.attr,S_IRUGO | S_IWUSR))
				printk("[TOUCH] sysfs_chmod_file is failed\n");		
			i=sysfs_chmod_file(&data_common->client->dev.kobj,&(*mxt_attrs[0]), S_IWUSR | S_IRUSR);
		}
		
#ifdef VIBRATOR_PANTECH_PATCH

		if(strncmp(buf, "vibon", 5)==0)
		{
		  pantech_vib_debug_enable();
		}
		if(strncmp(buf, "viboff", 6)==0)
		{
		  pantech_vib_debug_disable();
		}
#endif
		if(strncmp(buf, "touchid", 7)==0)
		{	
			for (i = 0 ; i < MAX_NUM_FINGER; ++i) { 
				if (fingerInfo[i].status == -1)
					continue;
				dbg_ioctl("[TOUCH] TOUCH ID => %d, fingerInfo[i].status=> %d, fingerInfo[i].mode=> %d\n",i,fingerInfo[i].status,fingerInfo[i].mode); 
			}
		}
		if(strncmp(buf, "checkcal", 8)==0)
		{			
		}
		if(strncmp(buf, "cal", 3)==0)
		{			
			mxt_calibrate(data_common);
		}
		if(strncmp(buf, "reset", 5)==0)
		{			
			mxt_soft_reset(data_common, MXT_RESET_VALUE);
		}
		if(strncmp(buf, "save", 4)==0)
		{			
			mxt_backup(data_common);	    
		}
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
		if(strncmp(buf, "init",4)==0)
		{			
		}
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
		}			

//++ p11309 - 2013.07.19 Check Noise Mode shake
#ifdef PAN_SUPPORT_SOFT_DEAD_ZONE
		if(strncmp(buf, "soft_dz_on", 10)==0)
		{ 
			pan_support_soft_dead_zone = 1;
		}		
		if(strncmp(buf, "soft_dz_off", 11)==0)
		{ 
			pan_support_soft_dead_zone = 0;
		}	
#endif

#ifdef ITO_TYPE_CHECK
		if(strncmp(buf, "id",2)==0)
		{			
			read_touch_id();   
		}
#endif
		if(strncmp(buf, "read_hallic",11)==0){
		}

		if(strncmp(buf, "fw",2)==0)
		{			
			ret = mxt_update_fw(dev);
			printk(" monitor_write ret %d\n", ret);
		}

	}
	*ppos +=nBufSize;
	return nBufSize;
}

#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(3,0,0))
static int monitor_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long monitor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	void __user *argp = (void __user *)arg;	
	struct mxt_data *data = data_common;
	struct device *dev = &data->client->dev;
	// Read Command 
	// Write, Etc.
	int return_value = -1;
	int cfg, object_type, field_index;
	int touch_mode_flag = 0;
//	pm_message_t pm_null={0}; //p11774 14.07.03 blocked for PM suspend&resume
	config_table_element config;

	switch (cmd)
	{
		case READ_TOUCH_ID:
		    return ((vendor_id<<16) + (model_id<<4) + type_id);
			break;

		case APPLY_TOUCH_CONFIG:
			apply_touch_config();
			break;
			
		case DIAG_DEBUG:
				disable_irq(data->client->irq);
			if (arg == TOUCH_IOCTL_DIAG_DEBUG_DELTA) 
				diag_debug(MXT_DELTA_MODE);
			else if (arg == TOUCH_IOCTL_DIAG_DEBUG_REF) 
				diag_debug(MXT_REFERENCE_MODE);
			else if (arg == TOUCH_IOCTL_DIAG_DEBUG_SELF_DELTA) 
				diag_debug_for_selfcap(MXT_SELF_DELTA_MODE);
			else if (arg == TOUCH_IOCTL_DIAG_DEBUG_SELF_REF) 
				diag_debug_for_selfcap(MXT_SELF_REFERENCE_MODE);
				enable_irq(data->client->irq);
				return 0;
			break;

		case RESET_TOUCH_CONFIG:
			init_touch_config();
			break;

		case GET_TOUCH_CONFIG:
			object_type 	= (int)((arg & 0x0000FF00) >> 8);
			field_index 	= (int)((arg & 0x000000FF) >> 0);
			if (config_table[object_type] == 0) {
				dbg_cr("[TOUCH] Get Touch Config is Error! undefined object type! %d\n", object_type);
				break;
			}
			config = config_table[object_type][field_index];
			if (config.size == UINT8 || config.size == INT8) {
				return_value = ((int16_t)*(config.value) & 0xFF) + (config.size << 16);
			}
			else if (config.size == UINT16 || config.size == INT16) {
				return_value = ((int16_t)*(config.value) & 0xFFFF) + (config.size << 16);
			}
			else {
				// Error
			}
			dbg_ioctl("Touch IO IOCTL GET config: %d-%d: %d (%d)\n", object_type, field_index, (return_value & 0xFFFF), (return_value & 0xF0000)>>16);
			return return_value;
			break;

		case SET_TOUCH_CONFIG:
			cfg    = (int)((arg & 0xFFFF0000) >> 16);
			object_type   = (int)((arg & 0x0000FF00) >> 8);
			field_index   = (int)((arg & 0x000000FF) >> 0);
			if (config_table[object_type] == 0) {
			  dbg_cr("[TOUCH] Error! undefined object type! %d\n", object_type);
			break;
			}
			config = config_table[object_type][field_index];
			if (config.size == UINT8) {
			  *((uint8_t*)config_table[object_type][field_index].value) = (cfg & 0xFF);
			}
			else if (config.size == UINT16) {
			  *((uint16_t*)config_table[object_type][field_index].value) = (cfg & 0xFFFF);
			}
			else if (config.size == INT8) {
			  *((int8_t*)config_table[object_type][field_index].value) = (cfg & 0xFF);
			}
			else if (config.size == INT16) {
			  *((int16_t*)config_table[object_type][field_index].value) = (cfg & 0xFFFF);
			}
			else {
			  // Error
			}
			dbg_ioctl("Touch IC IOCTL set %d-%d with %d\n", object_type, field_index, cfg);
			break;
			
		case ATMEL_GET_REFERENCE_DATA:
		  	{
		  	  if (copy_to_user(argp, &reference_data, sizeof(int)*MXT_USED_CHANNEL_NUM)){
		  	    dev_err(dev,"[TOUCH IO] ATMEL_GET_REFERENCE_DATA is failed\n");
		  	  }
					break;
		  	}

		case ATMEL_GET_SELFCAP_DATA:
			if (copy_to_user(argp, &reference_data, sizeof(int)*MXT_MAX_SELFCAP_DATA))
				dev_err(dev,"[TOUCH IO] ATMEL_GET_SELFCAP_DATA is failed\n");
			break;

		case TOUCH_IOCTL_DEBUG:
			return ioctl_debug(arg);
			break;
		case TOUCH_CHARGER_MODE:
			break;

		case TOUCH_IOCTL_DO_KEY:
			input_report_key(data->input_dev, (int)arg, 1);
			break;
			
		case TOUCH_IOCTL_RELEASE_KEY:		
			input_report_key(data->input_dev, (int)arg, 0);
			break;
			
		case TOUCH_IOCTL_INIT:
			dbg_cr("[TOUCH] Touch IC init \n");
			//mxt_front_test_init();
			break;
			
		case TOUCH_IOCTL_OFF:
			dbg_cr("[TOUCH] Touch IC off \n");
			TSP_PowerOff();
			break;   
		 
		case TOUCH_IOCTL_MULTI_TSP_OBJECT_SEL:
#if MXT_CFG_WRITE_BIN
			mTouch_mode = (unsigned long int)argp;
			if (mxt_update_cfg(data_common, mTouch_mode)) {
				dbg_cr("Failed update CFG data\n");
			}
			break;
#endif			
			break;

		case TOUCH_IOCTL_SUSPEND :
			mxt_suspend(data_common);
			break;

		case TOUCH_IOCTL_RESUME :
			mxt_resume(data_common);
			break;
			
		//++ p11309 - 2013.07.10 for Get Touch Mode
		case TOUCH_IOCTL_MULTI_TSP_OBJECT_GET:

			touch_mode_flag = mTouch_mode;
	
			if (copy_to_user(argp, &touch_mode_flag, sizeof(touch_mode_flag))) {
				dbg_ioctl(" TOUCH_IOCTL_MULTI_TSP_OBJECT_GET\n");
			}		
			break;
		//-- p11309

		case TOUCH_IOCTL_WIFI_DEBUG_APP_ENABLE :
			if(sysfs_chmod_file(&data->client->dev.kobj,&data->mem_access_attr.attr,S_IRWXUGO))
				printk("[TOUCH] sysfs_chmod_file is failed\n");
			return_value=sysfs_chmod_file(&data->client->dev.kobj,&(*mxt_attrs[0]), S_IRWXUGO);
			break;

		case TOUCH_IOCTL_WIFI_DEBUG_APP_DISABLE :
			if(sysfs_chmod_file(&data->client->dev.kobj,&data->mem_access_attr.attr,S_IRUGO | S_IWUSR))
				printk("[TOUCH] sysfs_chmod_file is failed\n");		
			return_value=sysfs_chmod_file(&data->client->dev.kobj,&(*mxt_attrs[0]), S_IWUSR | S_IRUSR);
			break;
		//++ P13106 TOUCH MODE READ
		case TOUCH_IOCTL_TOUCH_MODE:   
		if (copy_to_user(argp, &mTouch_mode, sizeof(mTouch_mode))) {
		  dbg_cr("TOUCH_IOCTL_TOUCH_MODE mTouch_mode %d\n", mTouch_mode);
		}   
		break;
		//-- P13106 TOUCH MODE READ

		case TOUCH_IOCTL_DIAG_DEBUG_CALIBRATION:   
			mxt_calibrate(data_common); 
		break;

		default:
			return 0;
			break;
	}
	return 0;
}

// call in driver init function
void touch_monitor_init(void) {
	int rc;
	rc = misc_register(&touch_monitor);
	if (rc) {
		pr_err("::::::::: can''t register touch_monitor\n");
	}
	init_proc();
}

// call in driver remove function
void touch_monitor_exit(void) {
	misc_deregister(&touch_monitor);
	remove_proc();
}

/*
 * lp5523.c - LP5523 LED Driver
 *
 * Copyright (C) 2010 Nokia Corporation
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/leds.h>
#include <linux/platform_data/leds-lp5523.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
//++ p11309 - 2013.04.25 for Device Tree
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
//-- p11309
#ifdef OFFLINE_CHARGER_LED_CONTROL
#include <soc/qcom/smem.h>
#include <linux/timer.h>
#include <linux/time.h>
#endif

//++ p11309 - 2013.04.24 for debug
#define PAN_DEBUGGER_ENABLE
#ifdef PAN_DEBUGGER_ENABLE 
#define pan_dbg(fmt, args...) if(dbg_flag)printk("[+++ LED IC] " fmt, ##args)
#define pan_err(fmt, args...) printk("[+++ LED IC] " fmt, ##args)
#define pan_func_in() if(dbg_flag) printk("[+++ LED IC] %s Func In\n", __func__)
#define pan_func_out() if(dbg_flag) printk("[+++ LED IC] %s Func Out\n", __func__)
#else
#define pan_dbg(fmt, args...) 
#endif
//-- p11309
static bool dbg_flag=false;

#ifdef PAN_LED_SYSFS_DEBUG
#include <linux/fs.h>
#include <linux/miscdevice.h>
static int led_open(struct inode *inode, struct file *file);
static ssize_t led_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
static long led_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

struct lp5523_chip *pan_led;

#endif
#define PAN_LED_NAME_DEBUG

#ifdef OFFLINE_CHARGER_LED_CONTROL
static struct timer_list offline_led_check_timer;
static void offline_led_check_timer_func(unsigned long data);
//extern int get_batt_status(void);
#define POWER_SUPPLY_STATUS_CHARGING       1
#define POWER_SUPPLY_STATUS_DISCHARGING    2
#define POWER_SUPPLY_STATUS_NOT_CHARGING   3
#define POWER_SUPPLY_STATUS_FULL           4
struct lp5523_led *control_led;
static int offline_led_state=0;
#define OFFLINE_LED_ALL_OFF 0
#define OFFLINE_LED_RED     1
#define OFFLINE_LED_GREEN   2
#define OFFLINE_LED_BLUE    3

#define OFFLINE_LEC_CHECK_INTERVAL 3000
#define BOOTING_LEC_CHECK_INTERVAL 60000
#endif
// p13106 130902 Allocation led name.
int check_dimming_channel(void);


static int boot_led_off=0;


#ifdef LED_OFF_CHECK_TIMER_USAGE
//alloc variable
static struct timer_list led_off_check_timer;
static int led_off_check=0;
struct work_struct	led_off_check_work;
#define LED_OFF_CHECK_INTERVAL 50
#endif
struct workqueue_struct *pan_led_single_thread_queue;

static int lp5523_set_mode(struct lp5523_engine *engine, u8 mode);
static int lp5523_set_engine_mode(struct lp5523_engine *engine, u8 mode);
static int lp5523_load_program(struct lp5523_engine *engine, const u8 *pattern);

static void lp5523_led_brightness_work(struct work_struct *work);
static int lp5523_write(struct i2c_client *client, u8 reg, u8 value);
static int lp5523_read(struct i2c_client *client, u8 reg, u8 *buf);
static int lp5523_write(struct i2c_client *client, u8 reg, u8 value);
static int lp5523_set_engine_mode(struct lp5523_engine *engine, u8 mode);
static void lp5523_en_pin_enable(unsigned gpio, bool state);
static int lp5523_detect(struct i2c_client *client);
static int __init lp5523_init_engine(struct lp5523_engine *engine, int id);
static int lp5523_configure(struct i2c_client *client);

static u8 pan_pattern[][LP5523_PROGRAM_LENGTH] =  {
	{ 0x00, 0x00, 0x9c, 0x20, 0x00, 0xff, 0xc0, 0x00}, //pan_pattern[0] : Menu&Back keyramp on/off
	{ 0x00, 0x00, 0x9c, 0x00, 0x00, 0xff, 0xc0, 0x00}, //pan_pattern[1] : Home key ramp on
	{ 0x00, 0x00, 0x9c, 0x10, 0x00, 0xff, 0xc0, 0x00}, //pan_pattern[2] : Home key ramp off      

	{ 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x7e, 0x00, 0x05, 0xff, 0x7e, 0x00, 0xa9, 0x85, 0xa0, 0x02, 0x00},  // pan_pattern[3] : dimming one color and off 10s, repeated.
	{ 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x7e, 0x00, 0x05, 0xff, 0x7e, 0x00, 0xb3, 0x85, 0xa0, 0x02, 0x00},  // pan_pattern[4] : dimming one color and off 20s, repeated.
	{ 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x7e, 0x00, 0x05, 0xff, 0x7e, 0x00, 0xbd, 0x85, 0xa0, 0x02, 0x00},  // pan_pattern[5] : dimming one color and off 30s, repeated.
	{ 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x7e, 0x00, 0x05, 0xff, 0xa0, 0x02},                                // pan_pattern[6] : dimming one color.
	{ 0x00, 0x00, 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x7e, 0x00, 0x05, 0xff, 0x9d, 0x80, 0x04, 0xff, 0x7e, 0x00, 0xa9, 0x88, 0x05, 0xff, 0x9d, 0xc0, 0xa0, 0x03, 0x00},  // pan_pattern[7] : dimming two color and off 10s, repeated.
	{ 0x00, 0x00, 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x7e, 0x00, 0x05, 0xff, 0x9d, 0x80, 0x04, 0xff, 0x7e, 0x00, 0xb3, 0x88, 0x05, 0xff, 0x9d, 0xc0, 0xa0, 0x03, 0x00},  // pan_pattern[8] : dimming two color and off 20s, repeated.
	{ 0x00, 0x00, 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x7e, 0x00, 0x05, 0xff, 0x9d, 0x80, 0x04, 0xff, 0x7e, 0x00, 0xbd, 0x88, 0x05, 0xff, 0x9d, 0xc0, 0xa0, 0x03, 0x00},  // pan_pattern[9] : dimming two color and off 30s, repeated.
	{ 0x00, 0xc0, 0x00, 0xc5, 0x00, 0x05, 0x00, 0x0f, 0x00, 0x0a, 0x00, 0xca, 0x9c, 0x00, 0x40, 0xff, 0x72, 0x00, 0x40, 0x00, 0x9d, 0x80, 0xa2, 0x87, 0xa0, 0x06, 0xc0, 0x00},// color 
	{ 0x00, 0xc0, 0x00, 0xc5, 0x00, 0x05, 0x00, 0x0f, 0x00, 0x0a, 0x00, 0xca, 0x9c, 0x00, 0x04, 0xff, 0x05, 0xff, 0x9d, 0x80, 0xa2, 0x87, 0xa0, 0x06, 0xc0, 0x00},// color 

	{ 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0xc0, 0x00}, //pan_pattern[12] : Home key ramp on
	{ 0x00, 0x00, 0x9c, 0x10, 0x05, 0xff, 0xc0, 0x00}, //pan_pattern[13] : Home key ramp off  

	{ 0x00, 0x00, 0x9c, 0x00, 0x05, 0xff, 0x7e, 0x00, 0x04, 0xff, 0x7e, 0x00, 0xa9, 0x85, 0xa0, 0x02, 0x00},  // pan_pattern[14] : dimming one color and off 10s, repeated.
	{ 0x00, 0x00, 0x9c, 0x00, 0x05, 0xff, 0x7e, 0x00, 0x04, 0xff, 0x7e, 0x00, 0xb3, 0x85, 0xa0, 0x02, 0x00},  // pan_pattern[15] : dimming one color and off 20s, repeated.
	{ 0x00, 0x00, 0x9c, 0x00, 0x05, 0xff, 0x7e, 0x00, 0x04, 0xff, 0x7e, 0x00, 0xbd, 0x85, 0xa0, 0x02, 0x00},  // pan_pattern[16] : dimming one color and off 30s, repeated.
//++ define PAN_LOGARITHMIC_DIMMING
	{ 0x00, 0xc0, 0x9c, 0x00, 0x1e, 0xe0, 0x54, 0x00, 0x1f, 0xe0, 0x54, 0x00, 0xa0, 0x02, 0x00},  // pan_pattern[17] : red dimming in charging.
//-- define PAN_LOGARITHMIC_DIMMING
  { 0x00, 0x00, 0x9c, 0x00, 0x40, 0xff, 0x42, 0x00, 0xa9, 0x83, 0x40, 0x00, 0x7e, 0x00, 0xa9, 0x86, 0xa0, 0x02, 0x00},  // pan_pattern[18] : dimming one color and off repeated.
  { 0x00, 0x00, 0x9c, 0x00, 0x40, 0xff, 0x42, 0x00, 0xa9, 0x83, 0x40, 0x00, 0x42, 0x00, 0xa9, 0x86, 0xa0, 0x02, 0x00},  // pan_pattern[19] : dimming one color and off short timer repeated.
  { 0x00, 0x00, 0x9c, 0x00, 0x04, 0xff, 0x3e, 0x00, 0x05, 0xff, 0x3e, 0x00, 0xa9, 0x85, 0xa0, 0x02, 0x00},  // pan_pattern[20] : dimming one color and off repeated.
	//{ 0x00, 0xc0, 0x00, 0xc5, 0x00, 0x05, 0x00, 0x0f, 0x00, 0x0a, 0x00, 0xca, 0x9c, 0x00, 0x40, 0xff, 0x60, 0x00, 0x40, 0x00, 0x60, 0x00, 0x9d, 0x80, 0xa2, 0x87, 0xa0, 0x06, 0xc0, 0x00},// color and black
};

#ifdef EF60_BOOT_RAINBOW_LED
// rainbow pattern 
static u8 pan_boot_pattern[][LP5523_PROGRAM_LENGTH] =  {
	{ 0x00, 0xc0, 0x9c, 0x00, 0x04, 0xff, 0xc0, 0x00},  // start RED ON
	{ 0x00, 0x05, 0x9c, 0x00, 0x04, 0xff, 0xc0, 0x00},  // pattern 1 GREEN ON
	{ 0x00, 0xc0, 0x9c, 0x00, 0x05, 0xff, 0xc0, 0x00},  // pattern 2 RED OFF
	{ 0x00, 0x0a, 0x9c, 0x00, 0x04, 0xff, 0xc0, 0x00},  // pattern 3 BLUE ON
	{ 0x00, 0x05, 0x9c, 0x00, 0x05, 0xff, 0xc0, 0x00},  // pattern 4 GREEN OFF
	{ 0x00, 0xc0, 0x9c, 0x00, 0x04, 0xff, 0xc0, 0x00},  // pattern 5 RED ON
	{ 0x00, 0x0a, 0x9c, 0x00, 0x05, 0xff, 0xc0, 0x00},  // pattern 6 BLUE OFF
};
// rainbow step. 9step.
typedef enum {	
	EF71_BOOT_START_PATTERN = 0,
	EF71_BOOT_RAINBOW_PATTERN_1,
	EF71_BOOT_RAINBOW_PATTERN_2,
	EF71_BOOT_RAINBOW_PATTERN_3,
	EF71_BOOT_RAINBOW_PATTERN_4,
	EF71_BOOT_RAINBOW_PATTERN_5,
	EF71_BOOT_RAINBOW_PATTERN_6,
}PAN_LED_EF71_BOOT_STATE_CMD;

static int boot_led_pattern_num=0;
struct work_struct  boot_led_work;
static struct timer_list boot_led_check_timer;
#define BOOTING_RAINBOW_LED_CHECK_INTERVAL 240
static void ef71_boot_led_work(struct work_struct *work);
static void boot_rainbow_led_timer_func(unsigned long data);
#endif

#ifdef LED_DIMMING_IN_CHARGING
#if defined(CONFIG_MACH_MSM8937_EF71IE) || defined(CONFIG_MACH_MSM8937_EF71S) || defined(CONFIG_MACH_MSM8937_EF71K)
static int base_brightness = 105;
#else
static int base_brightness = 31;
#endif
/* for dimming parameter
static int dimming_time = 15;
static int wait_time =10;
*/
static bool is_logarithmic_dimming = false;
void enable_logarithmic_dimming(void);
void enable_linear_dimming(void);
#endif

#ifdef PAN_LED_SYSFS_DEBUG
static struct file_operations led_lp5523_fops = 
{
	.owner          =     THIS_MODULE,
	.open           =     led_open,
	.unlocked_ioctl =     led_ioctl,  
	.write          =     led_write,
};

static struct miscdevice led_lp5523_io = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "led_fops",
	.fops =     &led_lp5523_fops
};
static int led_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t led_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int nBufSize=0;
	int i,j;
	int ret;
	u8 temp;
	//struct lp5523_chip *chip = i2c_get_clientdata(pan_led->client);
	if((size_t)(*ppos) > 0) return 0;	

	if(buf!=NULL)
	{
		nBufSize=strlen(buf);

		if(strncmp(buf, "read_reg",8)==0)
		{
			printk("[LED_LP5523] Read LP5523 LED IC Register\n");
			for(i=0;i<=LP5523_GAIN_CHANGE_CONTROL;i++){
				lp5523_read(pan_led->client, i,&temp);
				printk("LED ID [%3x] value => %x\n",i,temp);
			}
		}

		if(strncmp(buf, "read_3a",7)==0)
		{
			printk("[LED_LP5523] Read LP5523 LED IC 3A Register\n");
			lp5523_read(pan_led->client,0x3A,&temp);
			printk("LED ID [%3x] value => %x\n",0x3A,temp);			
		}

		if(strncmp(buf, "dbg_on",6)==0)
		{
			printk("[LED_LP5523] debug on\n");
			dbg_flag=true;
		}

		if(strncmp(buf, "dbg_off",7)==0)
		{
			printk("[LED_LP5523] debug off\n");
			dbg_flag=false;
		}
#ifdef EF60_BOOT_RAINBOW_LED
    // test for boot rainbow led.
		if(strncmp(buf, "rain_on",7)==0)
		{
			printk("[LED_LP5523] rainbow on\n");
			boot_led_pattern_num=0;
			boot_led_off = 1;

		  queue_work(pan_led_single_thread_queue, &boot_led_work);
		}
		if(strncmp(buf, "rain_off",8)==0)
		{
			printk("[LED_LP5523] rainbow off\n");
			boot_led_off = 0;
		}		
#endif    
		if(strncmp(buf, "test", 4)==0)
		{
			printk("[LED_LP5523] TEST\n");
			for(j=0;j<6;j++){
				lp5523_write(pan_led->client, LP5523_REG_PROG_PAGE_SEL, j);
				for(i=0x50;i<0x70;i++){
					lp5523_read(pan_led->client, (u8)i, &temp);
					printk(" [%d] -> %x\n",i,(int)temp);
				}
			}

		}
		if(strncmp(buf, "upper", 5)==0)
		{
			pan_pattern[10][16]+=3;

			pan_dbg("Test Led start. pan_pattern[10][16] -> %x\n",pan_pattern[10][16]);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("Booting led error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}
			ret = lp5523_load_program(&pan_led->engines[0], pan_pattern[10]);
			if(ret)
				pan_err("Booting led red load progrma error ret -> %d\n",ret);
			/*
			   ret = lp5523_load_program(&pan_led->engines[1], pan_pattern[11]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);

			   ret = lp5523_load_program(&pan_led->engines[2], pan_pattern[12]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);
			 */	
			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
			if(ret)
				pan_err("Booting led write  LP5523_REG_ENABLE error ret -> %d\n",ret);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE,LP5523_CMD_RUN_ENGINE1);
			if(ret)
				pan_err("Booting led write  LP5523_REG_OP_MODE error ret -> %d\n",ret);	


		}

		if(strncmp(buf, "lower", 5)==0)
		{
			pan_pattern[10][16]-=3;

			pan_dbg("Test Led start. pan_pattern[10][16] -> %x\n",pan_pattern[10][16]);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("Booting led error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}
			ret = lp5523_load_program(&pan_led->engines[0], pan_pattern[10]);
			if(ret)
				pan_err("Booting led red load progrma error ret -> %d\n",ret);
			/*
			   ret = lp5523_load_program(&pan_led->engines[1], pan_pattern[11]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);

			   ret = lp5523_load_program(&pan_led->engines[2], pan_pattern[12]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);
			 */	
			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
			if(ret)
				pan_err("Booting led write  LP5523_REG_ENABLE error ret -> %d\n",ret);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE,LP5523_CMD_RUN_ENGINE1);
			if(ret)
				pan_err("Booting led write  LP5523_REG_OP_MODE error ret -> %d\n",ret);	


		}
		if(strncmp(buf, "test1", 5)==0){
			pan_dbg("Booting Led start.\n");
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("Booting led error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}
			ret = lp5523_load_program(&pan_led->engines[0], pan_pattern[10]);
			if(ret)
				pan_err("Booting led red load progrma error ret -> %d\n",ret);
			/*
			   ret = lp5523_load_program(&pan_led->engines[1], pan_pattern[11]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);

			   ret = lp5523_load_program(&pan_led->engines[2], pan_pattern[12]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);
			 */	
			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
			if(ret)
				pan_err("Booting led write  LP5523_REG_ENABLE error ret -> %d\n",ret);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE,LP5523_CMD_RUN_ENGINE1);
			if(ret)
				pan_err("Booting led write  LP5523_REG_OP_MODE error ret -> %d\n",ret);	
		}
		if(strncmp(buf, "test2", 5)==0){
			pan_dbg("Booting Led start.\n");
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("Booting led error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}
			ret = lp5523_load_program(&pan_led->engines[0], pan_pattern[11]);
			if(ret)
				pan_err("Booting led red load progrma error ret -> %d\n",ret);
			/*
			   ret = lp5523_load_program(&pan_led->engines[1], pan_pattern[11]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);

			   ret = lp5523_load_program(&pan_led->engines[2], pan_pattern[12]);
			   if(ret)
			   pan_err("Booting led red load progrma error ret -> %d\n",ret);
			 */	
			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
			if(ret)
				pan_err("Booting led write  LP5523_REG_ENABLE error ret -> %d\n",ret);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE,LP5523_CMD_RUN_ENGINE1);
			if(ret)
				pan_err("Booting led write  LP5523_REG_OP_MODE error ret -> %d\n",ret);	
		}
		if(strncmp(buf, "run1", 4)==0)
		{
			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
			ret = lp5523_set_engine_mode(&pan_led->engines[0], LP5523_CMD_RUN_ENGINE1);
			//ret = lp5523_set_engine_mode(&chip->engines[1], LP5523_CMD_RUN);
			//  ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_RUN_ENGINE1|LP5523_CMD_RUN_ENGINE2);

		}
		if(strncmp(buf, "run2", 4)==0)
		{
			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  LP5523_CMD_RUN_ENGINE2 | LP5523_ENABLE);
			ret = lp5523_set_engine_mode(&pan_led->engines[1], LP5523_CMD_RUN_ENGINE2);
			//ret = lp5523_set_engine_mode(&chip->engines[1], LP5523_CMD_RUN);
			//  ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_RUN_ENGINE1|LP5523_CMD_RUN_ENGINE2);

		}
		if(strncmp(buf, "writeon", 7)==0)
		{
			if(buf[7] >= '0'){
				i = buf[7] - '1';
				printk("LED DRIVER Num -> %d On\n",i);
				mutex_lock(&pan_led->lock);
				lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,255);
				mutex_unlock(&pan_led->lock);
			}
		}
		if(strncmp(buf, "writeoff", 8)==0)
		{
			if(buf[8] >= '0'){
				i = buf[8] - '1';
				printk("LED DRIVER Num -> %d Off\n",i);
				mutex_lock(&pan_led->lock);
				lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,0);
				mutex_unlock(&pan_led->lock);
			}
		}
		if(strncmp(buf, "reset", 5)==0)
		{
			printk("LED Driver Reset\n");
			lp5523_en_pin_enable(pan_led->pdata->led_en_gpio, 0);
			usleep_range(1000, 2000); /* Keep enable down at least 1ms */
			lp5523_en_pin_enable(pan_led->pdata->led_en_gpio, 1);
			usleep_range(1000, 2000); /* 500us abs min. */

			ret = lp5523_write(pan_led->client, LP5523_REG_RESET, 0xff);
			if (ret) {
				pan_dbg("lp5523_write: LP5523 REG Reset fail. rc=%d\n", ret);
			}
			usleep_range(10000, 20000); 
			/*
			 * Exact value is not available. 10 - 20ms
			 * appears to be enough for reset.
			 */
			ret = lp5523_detect(pan_led->client);
			if (ret) {
				pan_dbg("lp5523_detect fail!\n");
				return 0 ;
			}

			pan_dbg("LP5523 Programmable led chip found\n");
			/* Initialize engines */
			for (i = 0; i < ARRAY_SIZE(pan_led->engines); i++) {
				ret = lp5523_init_engine(&pan_led->engines[i], i + 1);
				if (ret) {
					pan_dbg("error initializing engine\n");
					return 0;
				}
			}
			ret = lp5523_configure(pan_led->client);
			if (ret < 0) {
				pan_dbg("error configuring chip\n");
				return 0;
			}
			for (i = 0; i < pan_led->pdata->num_channels; i++) {
				ret = lp5523_write(pan_led->client,LP5523_REG_LED_CURRENT_BASE + pan_led->leds[i].chan_nr,pan_led->leds[i].led_current);
				if (ret) {
					pan_dbg("lp5523_write: REG_LED_CURRENT, chan=%u, current=%u\n",pan_led->leds[i].chan_nr, pan_led->leds[i].led_current);
				}
			}
			printk("LED Driver Reset end\n");
		}
#ifdef LED_DIMMING_IN_CHARGING		
    // RED DIMMING TEST.
		if(strncmp(buf, "red_dim", 7)==0){
	  pan_dbg("DIMMING_RED_CHARGING \n");
    
    // disable LED IC engine.
    ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
		if(ret)
			pan_err("DIMMING_RED_CHARGING error ret -> %d\n",ret);
			
    // led off for dimming event.
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
    // register set logarithmic dimming.
    enable_logarithmic_dimming();

    // set base brightness.
    lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 6,base_brightness);
    lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 7,base_brightness);

    temp=LP5523_CMD_RUN_ENGINE1;
    // load engine program
    if(lp5523_load_program(&pan_led->engines[0], pan_pattern[17])){
			pan_err("EF60_BOOT_START_PATTERN load error \n");
    }
    /* for dimming parameter
    pan_pattern[17][4]=(dimming_time<<1);
    pan_pattern[17][5]=(255-base_brightness);
    pan_pattern[17][6]=(wait_time<<1|0x40);      
    pan_pattern[17][8]=(dimming_time<<1)+1;
    pan_pattern[17][9]=(255-base_brightness);
    pan_pattern[17][10]=(wait_time<<1|0x40);
    */

    // engine program reg set.
    if(lp5523_write(pan_led->client, LP5523_REG_ENABLE,	temp | LP5523_ENABLE)){
      pan_err("DIMMING_RED_CHARGING program reg enable is error\n");	  
	  }

	  // engine program excute.
	  if( lp5523_write(pan_led->client, LP5523_REG_OP_MODE, temp)){
      pan_err("DIMMING_RED_CHARGING program reg mode is error\n");
	  }

	  // clear led data for next dimming event.
    for(i=0;i<PAN_NUM_LEDS;i++){
			if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
			pan_led->leds[i].before_brightness =pan_led->leds[i].brightness;
		}
		pan_led->home_key_led_current_state = 0;
		pan_led->home_key_led_before_state = pan_led->home_key_led_current_state;   
		}
#endif		
	}
	*ppos +=nBufSize;
	return nBufSize;
}

static long led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int i=0;
	int ret = 0;//add for fixing compile error by KwonOhYoon 2013/09/06
	printk("[LED_LP5523] IOCTL function. cmd => %d, arg -> %d\n",cmd,(int)arg);
	switch (cmd)
	{
	case PAN_LED_LP5523_LED_ALL_OFF :
		pan_dbg("LED OFF \n");
		ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
		if(ret)
			pan_err("EF59_LED_OFF error ret -> %d\n",ret);

		for(i=0;i<PAN_NUM_LEDS;i++){
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		pan_led->home_key_led_current_state = 0;
		pan_led->home_key_led_before_state = pan_led->home_key_led_current_state;
		pan_led->menu_back_led_state = 0;
		break;
	case PAN_LED_LP5523_LED_FRONT_RED :
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 6,MAX_BRIGHTNESS);
		break;	  
	case PAN_LED_LP5523_LED_FRONT_GREEN :
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE,MAX_BRIGHTNESS);
		break;
	case PAN_LED_LP5523_LED_FRONT_BLUE :
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 1,MAX_BRIGHTNESS);
		break;
	case PAN_LED_LP5523_LED_REAR_RED :
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 7,MAX_BRIGHTNESS);
		break;	  
	case PAN_LED_LP5523_LED_REAR_GREEN :
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 2,MAX_BRIGHTNESS);
		break;
	case PAN_LED_LP5523_LED_REAR_BLUE :
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 3,MAX_BRIGHTNESS);
		break;
	case PAN_LED_LP5523_LED_MENU_BACK_LED_ON :
		for(i=0;i<PAN_NUM_LEDS;i++){
			pan_led->leds[i].brightness=0;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 4,MAX_BRIGHTNESS);
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 5,MAX_BRIGHTNESS);
		break;	  
	case PAN_LED_LP5523_SET_MAX_CURRENT : 
		for (i = 0; i < LP5523_LEDS; i++) {
			/* Skip non-existing channels */
			if (pan_led->pdata->led_config[i].led_current == 0)
				continue;

			/* Set default current */
			lp5523_write(pan_led->client,LP5523_REG_LED_CURRENT_BASE + i,pan_led->pdata->led_config[i].max_current);
		}

		break;

	case PAN_LED_LP5523_SET_DEFAULT_CURRENT:
		for (i = 0; i < LP5523_LEDS; i++) {
			/* Skip non-existing channels */
			if (pan_led->pdata->led_config[i].led_current == 0)
				continue;

			/* Set default current */
			lp5523_write(pan_led->client,LP5523_REG_LED_CURRENT_BASE + i,pan_led->pdata->led_config[i].led_current);
		}
		break;

	default:
		break;
	}
	return 0;
}

#endif

#ifdef LED_DIMMING_IN_CHARGING
void enable_logarithmic_dimming(void)
{
  // register set logarithmic dimming
  int i;
  pan_dbg("enable_logarithmic_dimming\n");
  for(i=0;i<PAN_NUM_LEDS;i++){
    if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
    if(lp5523_write(pan_led->client, LP5523_REG_LED_CNTRL_BASE+i,32)){
      pan_err("enable_logarithmic_dimming write failed \n");
    }
  }
  is_logarithmic_dimming = true;
  return;
}

void enable_linear_dimming(void)
{
  // register set linear dimming
  int i;
  pan_dbg("enable_linear_dimming\n");
  for(i=0;i<PAN_NUM_LEDS;i++){
    if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
    if(lp5523_write(pan_led->client, LP5523_REG_LED_CNTRL_BASE+i,0)){
      pan_err("enable_linear_dimming write failed \n");
    }
  }
  is_logarithmic_dimming = false;
  return;
}
#endif
int check_dimming_channel(void){
	int rc=1,i;
	if(pan_led->leds[0].brightness != pan_led->leds[2].brightness || pan_led->leds[1].brightness != pan_led->leds[3].brightness || pan_led->leds[6].brightness != pan_led->leds[7].brightness){
		return 0;
	}
	for(i=0;i<PAN_NUM_LEDS;i++){
		if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
		if(pan_led->leds[i].brightness >MIN_BRIGHTNESS && pan_led->leds[i].brightness < MAX_BRIGHTNESS){
			return rc;
		}
	}
	return 0;
}

#ifdef LED_OFF_CHECK_TIMER_USAGE
// Callback function of led off timer.
static void led_off_check_timer_func(unsigned long data)
{
	pan_dbg("led_off_check_timer_func\n");
	if(led_off_check){	  
	  //control_led->brightness = PWM_HOMEKEY_ON_OFF;	  
	  queue_work(pan_led_single_thread_queue, &led_off_check_work);
  }
	return;
}
#endif


#ifdef OFFLINE_CHARGER_LED_CONTROL
static void offline_led_check_timer_func(unsigned long data)
{
	pan_err("offline_led_check_timer_func\n");
	control_led->brightness = EF59_LED_OFFLINE_LED_CHECK;
	queue_work(pan_led_single_thread_queue, &control_led->brightness_work);
	return;
}

static void booting_led_check_timer_func(unsigned long data)
{
	pan_err("booting_led_check_timer_func boot_led_off -> %d\n",boot_led_off);
	if(boot_led_off){
		control_led->brightness = EF59_LED_OFF;
		queue_work(pan_led_single_thread_queue, &control_led->brightness_work);
	}  
	return;
}

#endif

#ifdef EF60_BOOT_RAINBOW_LED 
static void boot_rainbow_led_timer_func(unsigned long data)
{
	pan_dbg("boot_rainbow_led_timer_func boot_led_off -> %d\n",boot_led_off);
	if(boot_led_off){
	  queue_work(pan_led_single_thread_queue, &boot_led_work);
	}  
	return;
}
#endif
static inline struct lp5523_led *cdev_to_led(struct led_classdev *cdev)
{
	return container_of(cdev, struct lp5523_led, cdev);
}

static inline struct lp5523_chip *engine_to_lp5523(struct lp5523_engine *engine)
{
	return container_of(engine, struct lp5523_chip,
			engines[engine->id - 1]);
}

static inline struct lp5523_chip *led_to_lp5523(struct lp5523_led *led)
{
	return container_of(led, struct lp5523_chip,
			leds[led->id]);
}

//++ p11309 - 2013.04.25 for led enable pin control
static int lp5523_en_pin_setup(unsigned gpio)
{
	return gpio_request_one(gpio, GPIOF_OUT_INIT_LOW, "lp5523_enable");
}

static void lp5523_en_pin_release(unsigned gpio)
{
	gpio_free(gpio);
}

static void lp5523_en_pin_enable(unsigned gpio, bool state)
{
	gpio_set_value(gpio, !!state);
}
//-- p11309

//++ p11309 - 2013.04.25 for Device Tree
#ifdef CONFIG_OF
static int lp5523_parse_dt(
		struct device *dev, 
		struct lp5523_platform_data *pdata )
{
	int rc=0, i=0;	
	struct device_node *np = dev->of_node;
	//struct property *prop;
	//int len=0;
	//u32 *temp_tri = 0;

	u32 temp_val;
	u32 led_current_val;
	u32 max_current_val;
	int gpio;

	/* clock mode */	                               
	rc = of_property_read_u32(np, "lp5523,clock_mode", &temp_val);
	if (!rc) {		
		pdata->clock_mode = temp_val;
		pan_dbg("lp5523,clock_mode=%d\n", pdata->clock_mode);
	}
	else {
		pan_dbg("failed to get lp5523,clock_mode\n");
		return rc;
	}

	/* led en gpio */	                               
	gpio = of_get_named_gpio(np, "lp5523,led_en_gpio", 0);
	if (gpio < 0) {
		pan_dbg("failed to get lp5523,led_en_gpio\n");
		return -ENODEV;
	}
	else{
		pdata->led_en_gpio = gpio;
		pan_dbg("lp5523,led_en_gpio=%d\n", pdata->led_en_gpio);		
	}

	/* max chan */
	rc = of_property_read_u32(np, "lp5523,max_chan", &temp_val);
	if (!rc) {
		pdata->num_channels = temp_val;
		pan_dbg("lp5523,max_chan=%d\n", pdata->num_channels);
	}
	else {
		pan_dbg("failed to get lp5523,max_chan\n");
		return rc;
	}

	/* led current */
	rc = of_property_read_u32(np, "lp5523,led_current", &led_current_val);
	if (rc)	{	
		pan_dbg("failed to get lp5523,led_current\n");
		return rc;
	}	
	pan_dbg("lp5523,led_current=%d\n", led_current_val);

	/* max current */
	rc = of_property_read_u32(np, "lp5523,max_current", &max_current_val);
	if (rc)	{
		pan_dbg("failed to get lp5523,max_current\n");
		return rc;
	}
	pan_dbg("lp5523,max_current=%d\n", max_current_val);

	/* Allocate lp5523_led_config mem*/
	pdata->led_config = kzalloc(pdata->num_channels * sizeof(struct lp5523_led_config), GFP_KERNEL);
	if (!pdata->led_config)	{
		pan_dbg("failed to allocate led config\n");
		return rc;
	}
	/*
	   prop = of_find_property(np, "lp5523,trg_touchkey-backlight", &len);	
	   len /= sizeof(u32);
	   pan_dbg("lp5523,trg_touchkey-backlight len=%d\n", len);
	   temp_tri = kzalloc(len*sizeof(u32), GFP_KERNEL);
	   rc = of_property_read_u32_array(np, "lp5523,trg_touchkey-backlight", temp_tri, len );
	   if (rc)	{
	   pan_dbg("failed to get lp5523,trg_touchkey-backlight\n");
	   return rc;
	   }

	   for (i = 0; i<len; i++ ) {
	   if ( temp_tri[i] < pdata->num_channels) {
	   pdata->led_config[temp_tri[i]].default_trigger = TRG_TOUCHKEY_BACKLIGHT;
	   pan_dbg("lp5523,trg_touchkey-backlight=%d\n", temp_tri[i]);
	   }
	   }

	   prop = of_find_property(np, "lp5523,trg_battery-charging", &len);
	   len /= sizeof(u32);
	   pan_dbg("lp5523,trg_battery-charging len=%d\n", len);
	   temp_tri = kzalloc(len*sizeof(u32), GFP_KERNEL);
	   rc = of_property_read_u32_array(np, "lp5523,trg_battery-charging", temp_tri, len );
	   if (rc)	{
	   pan_dbg("failed to get lp5523,trg_battery-charging\n");
	   return rc;
	   }

	   for (i = 0; i<len; i++ ) {
	   if ( temp_tri[i] < pdata->num_channels) {
	   pdata->led_config[temp_tri[i]].default_trigger = TRG_BATTERY_CHARGING;
	   pan_dbg("lp5523,trg_battery-charging=%d\n", temp_tri[i]);
	   }
	   }

	   prop = of_find_property(np, "lp5523,trg_battery-full", &len);
	   len /= sizeof(u32);
	   pan_dbg("lp5523,trg_battery-full len=%d\n", len);
	   temp_tri = kzalloc(len*sizeof(u32), GFP_KERNEL);
	   rc = of_property_read_u32_array(np, "lp5523,trg_battery-full", temp_tri, len );
	   if (rc)	{
	   pan_dbg("failed to get lp5523,trg_battery-full\n");
	   return rc;
	   }

	   for (i = 0; i<len; i++ ) {
	   if ( temp_tri[i] < pdata->num_channels) {
	   pdata->led_config[temp_tri[i]].default_trigger = TRG_BATTERY_FULL;
	   pan_dbg("lp5523,trg_battery-full=%d\n", temp_tri[i]);
	   }
	   }
	 */
	for ( i=0; i<pdata->num_channels; i++ )
	{
		pdata->led_config[i].chan_nr = i;
#if	defined(CONFIG_MACH_MSM8974_EF59S) || defined(CONFIG_MACH_MSM8974_EF59K) || defined(CONFIG_MACH_MSM8974_EF59L)
		if(i == 4 || i == 5){
			pdata->led_config[i].led_current = PAN_EF59_MENU_BACK_LED_BRIGHTNESS;
		}else{
			pdata->led_config[i].led_current = led_current_val;
		}
#elif defined(CONFIG_MACH_MSM8974_EF60S) || defined(CONFIG_MACH_MSM8974_EF61K) || defined(CONFIG_MACH_MSM8974_EF62L)
		if(i == 4 || i == 5){
			pdata->led_config[i].led_current = PAN_EF60_MENU_BACK_LED_BRIGHTNESS;
		}else{
			pdata->led_config[i].led_current = led_current_val;
		}
#else
		pdata->led_config[i].led_current = led_current_val;
#endif		
		pdata->led_config[i].max_current = max_current_val;
	}

	return 0;
}
#else
static int lp5523_parse_dt(struct device *dev, struct lp5523_platform_data *pdata)
{
	return -ENODEV;
}
#endif
//-- p11309


static int lp5523_write(struct i2c_client *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int lp5523_read(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		return -EIO;

	*buf = ret;
	return 0;
}

static int lp5523_detect(struct i2c_client *client)
{
	int ret;
	u8 buf;

	ret = lp5523_write(client, LP5523_REG_ENABLE, LP5523_ENABLE);
	if (ret)
		return ret;
	ret = lp5523_read(client, LP5523_REG_ENABLE, &buf);
	if (ret)
		return ret;
	if (buf == 0x40)
		return 0;
	else
		return -ENODEV;
}

static int lp5523_configure(struct i2c_client *client)
{
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	int ret = 0;
	u8 status=0;

	/* one pattern per engine setting led mux start and stop addresses */
	static const u8 pattern[][LP5523_PROGRAM_LENGTH] =  {
		{ 0x9c, 0x30, 0x9c, 0xb0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x40, 0x9c, 0xc0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x50, 0x9c, 0xd0, 0x9d, 0x80, 0xd8, 0x00, 0},
	};
	ret |= lp5523_write(client, LP5523_REG_ENABLE, LP5523_ENABLE);
	/* Chip startup time is 500 us, 1 - 2 ms gives some margin */
	usleep_range(1000, 2000);

	ret |= lp5523_write(client, LP5523_REG_CONFIG,
			LP5523_AUTO_INC | LP5523_PWR_SAVE |
			LP5523_CP_AUTO | LP5523_AUTO_CLK |
			LP5523_PWM_PWR_SAVE);

	/* turn on all leds */
	ret |= lp5523_write(client, LP5523_REG_ENABLE_LEDS_MSB, 0x01);
	//ret |= lp5523_write(client, LP5523_REG_ENABLE_LEDS_MSB, 0x00);      // D9 is not use        
	ret |= lp5523_write(client, LP5523_REG_ENABLE_LEDS_LSB, 0xff);

	/* hardcode 32 bytes of memory for each engine from program memory */
	ret |= lp5523_write(client, LP5523_REG_CH1_PROG_START, 0x00);
	ret |= lp5523_write(client, LP5523_REG_CH2_PROG_START, 0x10);
	ret |= lp5523_write(client, LP5523_REG_CH3_PROG_START, 0x20);

	/* write led mux address space for each channel */
	ret |= lp5523_load_program(&chip->engines[0], pattern[0]);
	ret |= lp5523_load_program(&chip->engines[1], pattern[1]);
	ret |= lp5523_load_program(&chip->engines[2], pattern[2]);

	if (ret) {
		dev_err(&client->dev, "could not load mux programs\n");
		return -1;
	}

	/* set all engines exec state and mode to run 00101010 */
	ret |= lp5523_write(client, LP5523_REG_ENABLE,
			(LP5523_CMD_RUN | LP5523_ENABLE));

	ret |= lp5523_write(client, LP5523_REG_OP_MODE, LP5523_CMD_RUN);

	if (ret) {
		dev_err(&client->dev, "could not start mux programs\n");
		return -1;
	}

	/* Let the programs run for couple of ms and check the engine status */
	usleep_range(3000, 6000);
	lp5523_read(client, LP5523_REG_STATUS, &status);
	status &= LP5523_ENG_STATUS_MASK;

	if (status == LP5523_ENG_STATUS_MASK) {
		dev_dbg(&client->dev, "all engines configured\n");
	} else {
		dev_info(&client->dev, "status == %x\n", status);
		dev_err(&client->dev, "cound not configure LED engine\n");
		return -1;
	}

	dev_info(&client->dev, "disabling engines\n");

	ret |= lp5523_write(client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);

	return ret;
}

static int lp5523_set_engine_mode(struct lp5523_engine *engine, u8 mode)
{
	struct lp5523_chip *chip = engine_to_lp5523(engine);
	struct i2c_client *client = chip->client;
	int ret;
	u8 engine_state;

	ret = lp5523_read(client, LP5523_REG_OP_MODE, &engine_state);
	if (ret)
		goto fail;

	engine_state &= ~(engine->engine_mask);

	/* set mode only for this engine */
	mode &= engine->engine_mask;

	engine_state |= mode;

	ret |= lp5523_write(client, LP5523_REG_OP_MODE, engine_state);
fail:
	return ret;
}

static int lp5523_load_mux(struct lp5523_engine *engine, u16 mux)
{
	struct lp5523_chip *chip = engine_to_lp5523(engine);
	struct i2c_client *client = chip->client;
	int ret = 0;

	ret |= lp5523_set_engine_mode(engine, LP5523_CMD_LOAD);

	ret |= lp5523_write(client, LP5523_REG_PROG_PAGE_SEL, engine->mux_page);
	ret |= lp5523_write(client, LP5523_REG_PROG_MEM,
			(u8)(mux >> 8));
	ret |= lp5523_write(client, LP5523_REG_PROG_MEM + 1, (u8)(mux));
	engine->led_mux = mux;

	return ret;
}

static int lp5523_load_program(struct lp5523_engine *engine, const u8 *pattern)
{
	struct lp5523_chip *chip = engine_to_lp5523(engine);
	struct i2c_client *client = chip->client;

	int ret = 0;
	ret |= lp5523_set_engine_mode(engine, LP5523_CMD_LOAD);

	ret |= lp5523_write(client, LP5523_REG_PROG_PAGE_SEL,
			engine->prog_page);
	ret |= i2c_smbus_write_i2c_block_data(client, LP5523_REG_PROG_MEM,
			LP5523_PROGRAM_LENGTH, pattern);

	return ret;
}

static int lp5523_run_program(struct lp5523_engine *engine)
{
	struct lp5523_chip *chip = engine_to_lp5523(engine);
	struct i2c_client *client = chip->client;
	int ret;

	ret = lp5523_write(client, LP5523_REG_ENABLE,
			LP5523_CMD_RUN | LP5523_ENABLE);
	if (ret)
		goto fail;

	ret = lp5523_set_engine_mode(engine, LP5523_CMD_RUN);
fail:
	return ret;
}

static int lp5523_mux_parse(const char *buf, u16 *mux, size_t len)
{
	int i;
	u16 tmp_mux = 0;
	len = len < LP5523_LEDS ? len : LP5523_LEDS;
	for (i = 0; i < len; i++) {
		switch (buf[i]) {
		case '1':
			tmp_mux |= (1 << i);
			break;
		case '0':
			break;
		case '\n':
			i = len;
			break;
		default:
			return -1;
		}
	}
	*mux = tmp_mux;

	return 0;
}

static void lp5523_mux_to_array(u16 led_mux, char *array)
{
	int i, pos = 0;
	for (i = 0; i < LP5523_LEDS; i++)
		pos += sprintf(array + pos, "%x", LED_ACTIVE(led_mux, i));

	array[pos] = '\0';
}

/*--------------------------------------------------------------*/
/*			Sysfs interface				*/
/*--------------------------------------------------------------*/

static ssize_t show_engine_leds(struct device *dev,
		struct device_attribute *attr,
		char *buf, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	char mux[LP5523_LEDS + 1];

	lp5523_mux_to_array(chip->engines[nr - 1].led_mux, mux);

	return sprintf(buf, "%s\n", mux);
}

#define show_leds(nr)							\
	static ssize_t show_engine##nr##_leds(struct device *dev,		\
			struct device_attribute *attr,		\
			char *buf)					\
{									\
	return show_engine_leds(dev, attr, buf, nr);			\
}
show_leds(1)
show_leds(2)
show_leds(3)

static ssize_t store_engine_leds(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	u16 mux = 0;
	ssize_t ret;

	if (lp5523_mux_parse(buf, &mux, len))
		return -EINVAL;

	mutex_lock(&chip->lock);
	ret = -EINVAL;
	if (chip->engines[nr - 1].mode != LP5523_CMD_LOAD)
		goto leave;

	if (lp5523_load_mux(&chip->engines[nr - 1], mux))
		goto leave;

	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}

#define store_leds(nr)						\
	static ssize_t store_engine##nr##_leds(struct device *dev,	\
			struct device_attribute *attr,	\
			const char *buf, size_t len)	\
{								\
	return store_engine_leds(dev, attr, buf, len, nr);	\
}
store_leds(1)
store_leds(2)
store_leds(3)

static ssize_t lp5523_selftest(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	int i, ret, pos = 0;
	int led = 0;
	u8 status=0, adc=0, vdd=0;

	mutex_lock(&chip->lock);

	ret = lp5523_read(chip->client, LP5523_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	/* Check that ext clock is really in use if requested */
	if ((chip->pdata) && (chip->pdata->clock_mode == LP5523_CLOCK_EXT))
		if  ((status & LP5523_EXT_CLK_USED) == 0)
			goto fail;

	/* Measure VDD (i.e. VBAT) first (channel 16 corresponds to VDD) */
	lp5523_write(chip->client, LP5523_REG_LED_TEST_CTRL,
			LP5523_EN_LEDTEST | 16);
	usleep_range(3000, 6000); /* ADC conversion time is typically 2.7 ms */
	ret = lp5523_read(chip->client, LP5523_REG_STATUS, &status);
	if (!(status & LP5523_LEDTEST_DONE))
		usleep_range(3000, 6000); /* Was not ready. Wait little bit */

	ret |= lp5523_read(chip->client, LP5523_REG_LED_TEST_ADC, &vdd);
	vdd--;	/* There may be some fluctuation in measurement */

	for (i = 0; i < LP5523_LEDS; i++) {
		/* Skip non-existing channels */
		if (chip->pdata->led_config[i].led_current == 0)
			continue;

		/* Set default current */
		lp5523_write(chip->client,
				LP5523_REG_LED_CURRENT_BASE + i,
				chip->pdata->led_config[i].led_current);

		lp5523_write(chip->client, LP5523_REG_LED_PWM_BASE + i, 0xff);
		/* let current stabilize 2 - 4ms before measurements start */
		usleep_range(2000, 4000);
		lp5523_write(chip->client,
				LP5523_REG_LED_TEST_CTRL,
				LP5523_EN_LEDTEST | i);
		/* ADC conversion time is 2.7 ms typically */
		usleep_range(3000, 6000);
		ret = lp5523_read(chip->client, LP5523_REG_STATUS, &status);
		if (!(status & LP5523_LEDTEST_DONE))
			usleep_range(3000, 6000);/* Was not ready. Wait. */
		ret |= lp5523_read(chip->client, LP5523_REG_LED_TEST_ADC, &adc);

		if (adc >= vdd || adc < LP5523_ADC_SHORTCIRC_LIM)
			pos += sprintf(buf + pos, "LED %d FAIL\n", i);

		lp5523_write(chip->client, LP5523_REG_LED_PWM_BASE + i, 0x00);

		/* Restore current */
		lp5523_write(chip->client,
				LP5523_REG_LED_CURRENT_BASE + i,
				chip->leds[led].led_current);
		led++;
	}
	if (pos == 0)
		pos = sprintf(buf, "OK\n");
	goto release_lock;
fail:
	pos = sprintf(buf, "FAIL\n");

release_lock:
	mutex_unlock(&chip->lock);

	return pos;
}

#ifdef EF60_BOOT_RAINBOW_LED
static void ef71_boot_led_work(struct work_struct *work)
{
	//check if led driver received led off event.
	if(!boot_led_off)
		return;

	// disable engine program.  
	if(lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED)){
		pan_err("EF71_LED_OFF error \n");
	}

	// sellect pattern.
	switch(boot_led_pattern_num){
	case EF71_BOOT_START_PATTERN :
		if(lp5523_load_program(&pan_led->engines[0], pan_boot_pattern[0])){
			pan_err("EF71_BOOT_START_PATTERN load error \n");
		}
		break;
	case EF71_BOOT_RAINBOW_PATTERN_1:
		if(lp5523_load_program(&pan_led->engines[0], pan_boot_pattern[1])){
			pan_err("EF71_BOOT_RAINBOW_PATTERN_1 on load  error \n");
		}
		break;
	case EF71_BOOT_RAINBOW_PATTERN_2:
		if(lp5523_load_program(&pan_led->engines[0], pan_boot_pattern[2])){
			pan_err("EF71_BOOT_RAINBOW_PATTERN_2 on load error \n");
		}
		break;
	case EF71_BOOT_RAINBOW_PATTERN_3:
		if(lp5523_load_program(&pan_led->engines[0], pan_boot_pattern[3])){
			pan_err("EF71_BOOT_RAINBOW_PATTERN_3 off load error \n");
		}
		break;
	case EF71_BOOT_RAINBOW_PATTERN_4:
		if(lp5523_load_program(&pan_led->engines[0], pan_boot_pattern[4])){
			pan_err("EF71_BOOT_RAINBOW_PATTERN_4 on load error \n");
		}
		break;
	case EF71_BOOT_RAINBOW_PATTERN_5:
		if(lp5523_load_program(&pan_led->engines[0], pan_boot_pattern[5])){
			pan_err("EF71_BOOT_RAINBOW_PATTERN_5 on load error \n");
		}
		break;
	case EF71_BOOT_RAINBOW_PATTERN_6:
		if(lp5523_load_program(&pan_led->engines[0], pan_boot_pattern[6])){
			pan_err("EF71_BOOT_RAINBOW_PATTERN_6 on load error \n");
		}	
		break;
	default :
		pan_err("ef71_boot_led_work boot_led_pattern_num is error[%d]\n",boot_led_pattern_num);
		break;    
	}

	// Pattern number is increasing for next pattern
	if(boot_led_pattern_num < EF71_BOOT_RAINBOW_PATTERN_6){
		boot_led_pattern_num++;
	}else{
		boot_led_pattern_num = EF71_BOOT_RAINBOW_PATTERN_1;
	}
	
	// engine program register set.
	if(lp5523_write(pan_led->client, LP5523_REG_ENABLE,	LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE)){
		pan_err("ef71_boot_led_work program reg enable is error\n");	  
	}

	// engine program excute.
	if( lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_RUN_ENGINE1)){
		pan_err("ef71_boot_led_work program reg mode is error\n");
	} 
	// next timer set.
	mod_timer(&boot_led_check_timer, jiffies + msecs_to_jiffies(BOOTING_RAINBOW_LED_CHECK_INTERVAL));
}
#endif


#ifdef LED_OFF_CHECK_TIMER_USAGE
// Callback function of led off check work.
static void lp5523_led_off_check_work(struct work_struct *work)
{
  int i,ret;
  // check led_off_check flag if it is enabled.
  if(!led_off_check){
    return ;
  }
  pan_dbg("lp5523_led_off_check_work\n");
//  mutex_lock(&pan_led->lock);
  ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
	if(ret)
		pan_err("EF59_LED_OFF error ret -> %d\n",ret);

	for(i=0;i<PAN_NUM_LEDS;i++){
		if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
	}
//  mutex_unlock(&pan_led->lock);
  led_off_check =0;
}
#endif
static void lp5523_set_brightness(struct led_classdev *cdev,
		enum led_brightness brightness)
{
	struct lp5523_led *led = cdev_to_led(cdev);

	pan_dbg("before work queue: channel=%d, value=%d\n",
		led->chan_nr, brightness);

	led->brightness = (u8)brightness;
#ifdef LED_OFF_CHECK_TIMER_USAGE
	//++ p11309 - 2013.12.17 for charger LED Off Error
	//led->charger_off = 0;
	if ( led->chan_nr == 8 && brightness == 9 ) {
		
		pan_dbg("%s: charger LED Light OFF\n", __func__);		
		led_off_check=1;
		mod_timer(&led_off_check_timer, jiffies + msecs_to_jiffies(LED_OFF_CHECK_INTERVAL));  

		led->charger_off = 1;
	}
	//-- p11309
  // check if led_off_check flag is enabled.
  else if(led_off_check){
    if(led->chan_nr == 8 && (brightness != 1 /*PWM_TK_ON_OFF*/ && brightness != 2 /*DIMMING_TK_ON_OFF*/ )){
      led_off_check=0; 
    }
  }	
#endif
  while(!queue_work(pan_led_single_thread_queue, &led->brightness_work)){
    flush_work(&led->brightness_work);
    pan_err("queue_work pendding\n");
  }
}



static void lp5523_led_brightness_work(struct work_struct *work)
{
	int ret,i;
	unsigned long ramp_on=0,ramp_off=0;
	u8 first_color=0,second_color=0,common_color=0;
	struct lp5523_led *led = container_of(work,
			struct lp5523_led,
			brightness_work);
	struct lp5523_chip *chip = led_to_lp5523(led);
	struct i2c_client *client = chip->client;

	mutex_lock(&chip->lock);
	if(led->chan_nr == 4 || led->chan_nr == 5){
		// menu & back tm key
		if(led->brightness){
			set_bit((int)led->chan_nr, &chip->menu_back_led_state);
		}else{
			clear_bit((int)led->chan_nr,&chip->menu_back_led_state);
		}
		//lp5523_write(client, LP5523_REG_LED_PWM_BASE + led->chan_nr,led->brightness);

	}else if(led->chan_nr == 8){
#ifdef LED_OFF_CHECK_TIMER_USAGE
//++ p11309 - 2013.12.17 for charger LED Off Error
		if ( led->charger_off == 1 ) {

			pan_dbg("%s: charger LED Light OFF\n", __func__);
			led_off_check=1;
			mod_timer(&led_off_check_timer, jiffies + msecs_to_jiffies(LED_OFF_CHECK_INTERVAL));  

			led->charger_off = 0;
		}
//-- p11309
#endif
#ifdef LED_DIMMING_IN_CHARGING
    if(is_logarithmic_dimming && (led->brightness != PWM_TK_ON_OFF && led->brightness != DIMMING_TK_ON_OFF && led->brightness != EF59_LED_OFFLINE_LED_CHECK)){
      enable_linear_dimming();
    }
#endif
		switch(led->brightness){
		case EF59_LED_OFF:
			pan_dbg("LED OFF \n");
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("EF59_LED_OFF error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				chip->leds[i].brightness=0;
				lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
				chip->leds[i].before_brightness =0;
			}
			chip->home_key_led_current_state = 0;
			chip->home_key_led_before_state = chip->home_key_led_current_state;
			boot_led_off = 0;
			break;

		case EF59_LED_ALL_OFF:
			pan_dbg("LED OFF \n");
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("EF59_LED_OFF error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				chip->leds[i].brightness=0;
				lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
				chip->leds[i].before_brightness =0;
			}
			chip->home_key_led_current_state = 0;
			chip->home_key_led_before_state = chip->home_key_led_current_state;
			chip->menu_back_led_state = 0;
			boot_led_off = 0;
			break;

		case PWM_TK_ON_OFF:
			pan_dbg("PWM_TK_ON_OFF menu -> %d, back -> %d \n",chip->leds[EF59_MENU_KEY].brightness,chip->leds[EF59_BACK_KEY].brightness);
			lp5523_write(client, LP5523_REG_LED_PWM_BASE + EF59_MENU_KEY,chip->leds[EF59_MENU_KEY].brightness);
			lp5523_write(client, LP5523_REG_LED_PWM_BASE + EF59_BACK_KEY,chip->leds[EF59_BACK_KEY].brightness);
			chip->leds[EF59_MENU_KEY].before_brightness=chip->leds[EF59_MENU_KEY].brightness;
			chip->leds[EF59_BACK_KEY].before_brightness=chip->leds[EF59_BACK_KEY].brightness;
			break;

		case DIMMING_TK_ON_OFF:

			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("DIMMING_TK_ON_OFF error ret -> %d\n",ret);

			pan_pattern[0][1]=MENU_BACK_KEY_HEX;
			if(chip->menu_back_led_state){
				pan_pattern[0][4]=DIMMING_ON_TIME;
			}
			else{
				pan_pattern[0][4]=DIMMING_OFF_TIME;
			}
			pan_dbg("DIMMING_TK_ON_OFF ON PATTEN[0]-> %d\n",(int)pan_pattern[0][0]);
			ret = lp5523_load_program(&chip->engines[2], pan_pattern[0]);
			if(ret)
				pan_err("DIMMING_TK_ON_OFF error ret -> %d\n",ret);

			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,	LP5523_CMD_RUN_ENGINE3 | LP5523_ENABLE);
			ret = lp5523_set_engine_mode(&pan_led->engines[2], LP5523_CMD_RUN_ENGINE3);
			if(ret)
				pan_err("DIMMING_TK_ON_OFF Error ret -> %d\n",ret);

			chip->leds[EF59_MENU_KEY].before_brightness=chip->leds[EF59_MENU_KEY].brightness;
			chip->leds[EF59_BACK_KEY].before_brightness=chip->leds[EF59_BACK_KEY].brightness;
			break;

		case PWM_HOMEKEY_ON_OFF:
			pan_dbg("PWM_HOMEKEY_ON_OFF. LED STATE -> %x\n",(int)chip->home_key_led_current_state);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);           // p13106 Adding EF60 series
			if(ret)
					pan_err("PWM_HOMEKEY_ON_OFF error ret -> %d\n",ret);
			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
				lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,chip->leds[i].brightness);
				chip->leds[i].before_brightness =chip->leds[i].brightness;
			}
			chip->home_key_led_before_state = chip->home_key_led_current_state;
			boot_led_off = 0;
			break;
#ifdef LED_OFF_CHECK_TIMER_USAGE		
    case DIMMING_OFF_CHECK:
      pan_dbg("DIMMING_OFF_CHECK\n");
      led_off_check=1;
      mod_timer(&led_off_check_timer, jiffies + msecs_to_jiffies(LED_OFF_CHECK_INTERVAL));  
#endif      
		case DIMMING_HOMEKEY_ON_OFF:
			if(check_dimming_channel()){
				pan_dbg("DIMMING_HOMEKEY_ON_OFF(2). LED STATE -> %x\n",(int)chip->home_key_led_current_state);

				ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
				if(ret)
					pan_err("DIMMING_HOMEKEY_ON_OFF error ret -> %d\n",ret);

				if(chip->leds[0].before_brightness != chip->leds[0].brightness){
					ret = chip->leds[0].brightness - chip->leds[0].before_brightness ;
					pan_dbg("DIMMING_HOMEKEY_ON_OFF(2) green brightness - before_brightness -> %d\n",ret);
					if(ret > 0){
						first_color|=LP5523_CMD_RUN_ENGINE1;
						pan_pattern[12][1]=(u8)0x5;
						pan_pattern[12][3]=(u8)0x00;
						pan_pattern[12][5]=ret;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[12]);
						if(ret)
							pan_err("DIMMING_HOMEKEY_ON_OFF Ramp ON error ret -> %d LINE -> %d\n",ret,__LINE__);

					}else{
						first_color|=LP5523_CMD_RUN_ENGINE1;
						pan_pattern[13][1]=(u8)0x5;
						pan_pattern[13][3]=(u8)0x00;
						pan_pattern[13][5]=(-ret);
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[13]);
						if(ret)
							pan_err("DIMMING_HOMEKEY_ON_OFF Ramp OFF error ret -> %d LINE -> %d\n",ret,__LINE__);
					}
				}

				if(chip->leds[1].before_brightness != chip->leds[1].brightness){
					ret = chip->leds[1].brightness - chip->leds[1].before_brightness ;
					pan_dbg("DIMMING_HOMEKEY_ON_OFF(2) blue brightness - before_brightness -> %d\n",ret);
					if(ret > 0){
						first_color|=LP5523_CMD_RUN_ENGINE2;
						pan_pattern[12][1]=(u8)0xa;
						pan_pattern[12][3]=(u8)0x10;
						pan_pattern[12][5]=ret;
						ret = lp5523_load_program(&chip->engines[1], pan_pattern[12]);
						if(ret)
							pan_err("DIMMING_HOMEKEY_ON_OFF Ramp ON error ret -> %d LINE -> %d\n",ret,__LINE__);

					}else{
						first_color|=LP5523_CMD_RUN_ENGINE2;
						pan_pattern[13][1]=(u8)0xa;
						pan_pattern[13][3]=(u8)0x10;
						pan_pattern[13][5]=(-ret);
						ret = lp5523_load_program(&chip->engines[1], pan_pattern[13]);
						if(ret)
							pan_err("DIMMING_HOMEKEY_ON_OFF Ramp OFF error ret -> %d LINE -> %d\n",ret,__LINE__);
					}
				}

				if(chip->leds[6].before_brightness != chip->leds[6].brightness){
					ret = chip->leds[6].brightness - chip->leds[6].before_brightness ;
					pan_dbg("DIMMING_HOMEKEY_ON_OFF(2) red brightness - before_brightness -> %d\n",ret);
					if(ret > 0){
						first_color|=LP5523_CMD_RUN_ENGINE3;
						pan_pattern[12][1]=(u8)0xc0;
						pan_pattern[12][3]=(u8)0x20;
						pan_pattern[12][5]=ret;
						ret = lp5523_load_program(&chip->engines[2], pan_pattern[12]);
						if(ret)
							pan_err("DIMMING_HOMEKEY_ON_OFF Ramp ON error ret -> %d LINE -> %d\n",ret,__LINE__);

					}else{
						first_color|=LP5523_CMD_RUN_ENGINE3;
						pan_pattern[13][1]=(u8)0xc0;
						pan_pattern[13][3]=(u8)0x20;
						pan_pattern[13][5]=(-ret);
						ret = lp5523_load_program(&chip->engines[2], pan_pattern[13]);
						if(ret)
							pan_err("DIMMING_HOMEKEY_ON_OFF Ramp OFF error ret -> %d, LINE -> %d\n",ret,__LINE__);
					}
				}
				if(first_color > 0){
					ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  first_color | LP5523_ENABLE);
					ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, first_color);
				}else{
					pan_err("DIMMING_HOMEKEY_ON_OFF error. line -> %d \n",__LINE__);
				}    

				for(i=0;i<PAN_NUM_LEDS;i++){
					if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
					chip->leds[i].before_brightness =chip->leds[i].brightness;
				}

				chip->home_key_led_before_state = chip->home_key_led_current_state;


			}else{
				pan_dbg("DIMMING_HOMEKEY_ON_OFF. LED STATE -> %x\n",(int)chip->home_key_led_current_state);
				ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
				if(ret)
					pan_err("DIMMING_HOMEKEY_ON_OFF error ret -> %d\n",ret);


				for(i=0;i<PAN_NUM_LEDS;i++){
					if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
					if(test_bit(i, &chip->home_key_led_before_state)^test_bit(i, &chip->home_key_led_current_state)){
						if(test_bit(i, &chip->home_key_led_current_state)){
							// ramp on
							set_bit(i,&ramp_on);
						}else{
							// ramp off
							set_bit(i,&ramp_off);
						}		          
					}
				}
				pan_dbg("DIMMING_HOMEKEY_ON_OFF ramp_on -> %x, ramp_off -> %x\n",(int)ramp_on,(int)ramp_off);

				if(ramp_on){
					pan_pattern[1][1]=(u8)ramp_on;
					pan_pattern[1][4]=DIMMING_ON_TIME;
					ret = lp5523_load_program(&chip->engines[0], pan_pattern[1]);
					if(ret)
						pan_err("DIMMING_HOMEKEY_ON_OFF Ramp ON error ret -> %d\n",ret);
				}
				if(ramp_off){
					pan_pattern[2][1]=(u8)ramp_off;
					pan_pattern[2][4]=DIMMING_OFF_TIME;
					ret = lp5523_load_program(&chip->engines[1], pan_pattern[2]);
					if(ret)
						pan_err("DIMMING_HOMEKEY_ON_OFF Ramp OFF error ret -> %d\n",ret);
				}

				if(ramp_on && ramp_off){
					ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  LP5523_CMD_RUN_ENGINE1|LP5523_CMD_RUN_ENGINE2 | LP5523_ENABLE);
					ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_RUN_ENGINE1|LP5523_CMD_RUN_ENGINE2);
				}else if(ramp_on){
					ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
					ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_RUN_ENGINE1);
				}else if(ramp_off){
					ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  LP5523_CMD_RUN_ENGINE2 | LP5523_ENABLE);
					ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_RUN_ENGINE2);
				}else{
				  for(i=0;i<PAN_NUM_LEDS;i++){
					  if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
					  chip->leds[i].before_brightness =chip->leds[i].brightness;
					}
					break;
				}
				//ret = lp5523_set_engine_mode(&chip->engines[0], LP5523_CMD_RUN);
				//ret = lp5523_set_engine_mode(&chip->engines[1], LP5523_CMD_RUN);

				//ret = lp5523_set_engine_mode(&chip->engines[0], LP5523_CMD_RUN_ENGINE1);
				//ret = lp5523_set_engine_mode(&chip->engines[1], LP5523_CMD_RUN_ENGINE2);
				if(ret)
					pan_err("DIMMING_HOMEKEY_ON_OFF Error ret -> %d\n",ret);

				for(i=0;i<PAN_NUM_LEDS;i++){
					if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
					chip->leds[i].before_brightness =chip->leds[i].brightness;
				}

				chip->home_key_led_before_state = chip->home_key_led_current_state;
			}
			break;

		case DIMMING_ON_COLOR_10S:
		case DIMMING_ON_COLOR_20S:
		case DIMMING_ON_COLOR_30S:

			pan_dbg("DIMMING_ON_COLOR. LED STATE -> %x\n",(int)chip->home_key_led_current_state);

			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("DIMMING_ON_WHITE_10S error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}

				if(chip->home_key_led_current_state & 0x01)
					first_color |= 0x05;
				if(chip->home_key_led_current_state & 0x02)
					first_color |= 0x0a;
				if(chip->home_key_led_current_state & 0x04)
					second_color |= 0x05;
				if(chip->home_key_led_current_state & 0x08)
					second_color |= 0x0a;
				if(chip->home_key_led_current_state & 0x40)
					first_color |= 0xc0;
				if(chip->home_key_led_current_state & 0x80)
					second_color |= 0xc0;	

				common_color = first_color & second_color;
				pan_dbg("first_color -> %x, second_color -> %x, common_color -> %x\n",first_color,second_color,common_color);
				if(second_color == 0){
					// one color and black dimming
					if(led->brightness == DIMMING_ON_COLOR_10S){
						pan_pattern[3][1]=first_color;
						pan_pattern[3][3]=0x00;
						pan_pattern[3][5]=0xff;
						pan_pattern[3][9]=0xff;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[3]);
					}else if(led->brightness == DIMMING_ON_COLOR_20S){
						pan_pattern[4][1]=first_color;
						pan_pattern[4][3]=0x00;
						pan_pattern[4][5]=0xff;
						pan_pattern[4][9]=0xff;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[4]);
					}else if(led->brightness == DIMMING_ON_COLOR_30S){
						pan_pattern[5][1]=first_color;
						pan_pattern[5][3]=0x00;
						pan_pattern[5][5]=0xff;
						pan_pattern[5][9]=0xff;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[5]);
					}else{
						break;
					}
				}else if(common_color == second_color){
					for(i=0;i<PAN_NUM_LEDS;i++){
						if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
						if(common_color & (1<<i)){
							pan_dbg("common_color & i(%d)\n",i);
							lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MAX_BRIGHTNESS);
						}
					}  		  
					if(led->brightness == DIMMING_ON_COLOR_10S){
						pan_pattern[3][1]=first_color-common_color;
						pan_pattern[3][3]=0x00;
						pan_pattern[3][5]=0xff;
						pan_pattern[3][9]=0xff;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[3]);
					}else if(led->brightness == DIMMING_ON_COLOR_20S){
						pan_pattern[4][1]=first_color-common_color;
						pan_pattern[4][3]=0x00;
						pan_pattern[4][5]=0xff;
						pan_pattern[4][9]=0xff;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[4]);
					}else if(led->brightness == DIMMING_ON_COLOR_30S){
						pan_pattern[5][1]=first_color-common_color;
						pan_pattern[5][3]=0x00;
						pan_pattern[5][5]=0xff;
						pan_pattern[5][9]=0xff;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[5]);
					}else{
						break;
					}
				}else{
					if(led->brightness == DIMMING_ON_COLOR_10S){
						pan_pattern[7][1]=first_color;
						pan_pattern[7][3]=second_color;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[7]);
					}else if(led->brightness == DIMMING_ON_COLOR_20S){
						pan_pattern[8][1]=first_color;
						pan_pattern[8][3]=second_color;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[8]);
					}else if(led->brightness == DIMMING_ON_COLOR_30S){
						pan_pattern[9][1]=first_color;
						pan_pattern[9][3]=second_color;
						ret = lp5523_load_program(&chip->engines[0], pan_pattern[9]);
					}else{
						break;
					}
				}

				if(ret)
					pan_err("DIMMING_ON_COLOR_10S error ret -> %d\n",ret);

				ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
				ret = lp5523_set_engine_mode(&chip->engines[0], LP5523_CMD_RUN_ENGINE1);
				if(ret)
					pan_err("DIMMING_ON_COLOR_10S Error ret -> %d\n",ret);
			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
				chip->leds[i].before_brightness =0;
			}
			chip->home_key_led_current_state = 0;
			chip->home_key_led_before_state = chip->home_key_led_current_state;
			break;

		case DIMMING_ON_OFF_COLOR :
			pan_dbg("DIMMING_ON_OFF_COLOR. LED STATE -> %x\n",(int)chip->home_key_led_current_state);
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("DIMMING_ON_OFF_COLOR error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}

			pan_pattern[6][1]=(u8)chip->home_key_led_current_state;
			ret = lp5523_load_program(&chip->engines[0], pan_pattern[6]);
			if(ret)
				pan_err("DIMMING_ON_OFF_COLOR error ret -> %d\n",ret);

			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
			ret = lp5523_set_engine_mode(&chip->engines[0], LP5523_CMD_RUN_ENGINE1);
			if(ret)
				pan_err("DIMMING_ON_OFF_COLOR Error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
				chip->leds[i].before_brightness =chip->leds[i].brightness;
			}
			chip->home_key_led_current_state = 0;
			chip->home_key_led_before_state = chip->home_key_led_current_state;      
			break;
#ifdef OFFLINE_CHARGER_LED_CONTROL		  
		case EF59_LED_OFFLINE_LED_CHECK :
			//ret=get_batt_status();
			//pan_err("get_batt_status ret -> %d\n",ret);
			switch(ret){
			case POWER_SUPPLY_STATUS_CHARGING:
				if(offline_led_state==1){
					// Red led is already lighting. 
				}else{
#ifdef LED_DIMMING_IN_CHARGING	
          // red dimming in offline charging.

          // disable LED IC engine.
          ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
    			if(ret)
    				pan_err("DIMMING_RED_CHARGING error in offline ret -> %d\n",ret);
    				
          // led off for dimming event.
    			for(i=0;i<PAN_NUM_LEDS;i++){
    				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
    				lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
    			}
          // register set logarithmic dimming.
          enable_logarithmic_dimming();

          // set base brightness.
          lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 6,base_brightness);
          lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 7,base_brightness);

          first_color=LP5523_CMD_RUN_ENGINE1;
          // load engine program
          if(lp5523_load_program(&pan_led->engines[0], pan_pattern[17])){
    				pan_err("DIMMING_RED_CHARGING load error in offline \n");
          }
          
          // engine program reg set.
    	    if(lp5523_write(pan_led->client, LP5523_REG_ENABLE,	first_color | LP5523_ENABLE)){
    	      pan_err("DIMMING_RED_CHARGING program reg enable is error in offline \n");	  
      	  }

      	  // engine program excute.
      	  if( lp5523_write(pan_led->client, LP5523_REG_OP_MODE, first_color)){
            pan_err("DIMMING_RED_CHARGING program reg mode is error in offline \n");
      	  }
#else
          // red pwm led in offline charging.
					for(i=0;i<PAN_NUM_LEDS;i++){
						lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
					}
					lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 6,MAX_BRIGHTNESS);
					lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 7,MAX_BRIGHTNESS);
#endif					
					offline_led_state = OFFLINE_LED_RED;          
				}
				break;
			case POWER_SUPPLY_STATUS_DISCHARGING:
			case POWER_SUPPLY_STATUS_NOT_CHARGING:
				// All led is off
				if(offline_led_state != OFFLINE_LED_ALL_OFF){
					for(i=0;i<PAN_NUM_LEDS;i++){
						lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
					}
					offline_led_state = OFFLINE_LED_ALL_OFF;
				}
				break;
			case POWER_SUPPLY_STATUS_FULL :
				if(offline_led_state == OFFLINE_LED_GREEN){
				}else{
				  // disable LED IC engine.
          ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
    			if(ret)
    				pan_err("DIMMING_RED_CHARGING error ret -> %d\n",ret);

 					for(i=0;i<PAN_NUM_LEDS;i++){
						lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
					}
					lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE ,MAX_BRIGHTNESS);
					lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 2,MAX_BRIGHTNESS);  
					offline_led_state = OFFLINE_LED_GREEN;          
				}
			default :
				break;
			}
			mod_timer(&offline_led_check_timer, jiffies + msecs_to_jiffies(OFFLINE_LEC_CHECK_INTERVAL));  
			break;
#endif
    case EF59_LED_SET_GREEN_LIGHT_VALUE_20 :    
			pan_dbg("EF59_LED_SET_GREEN_LIGHT_VALUE_20. \n");
   	  lp5523_write(pan_led->client, LP5523_REG_LED_CURRENT_BASE   ,20);
   	  lp5523_write(pan_led->client, LP5523_REG_LED_CURRENT_BASE +2,20);
		  break;
			
    case EF59_LED_SET_GREEN_LIGHT_VALUE_40 :    
			pan_dbg("EF59_LED_SET_GREEN_LIGHT_VALUE_40. \n");
   	  lp5523_write(pan_led->client, LP5523_REG_LED_CURRENT_BASE   ,40);
   	  lp5523_write(pan_led->client, LP5523_REG_LED_CURRENT_BASE +2,40);
		  break;
#ifdef LED_DIMMING_IN_CHARGING
    case DIMMING_RED_CHARGING:
      pan_dbg("DIMMING_RED_CHARGING \n");
      
      // disable LED IC engine.
      ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("DIMMING_RED_CHARGING error ret -> %d\n",ret);
				
      // led off for dimming event.
			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}
      // register set logarithmic dimming.
      enable_logarithmic_dimming();

      // set base brightness.
      lp5523_write(client, LP5523_REG_LED_PWM_BASE + 6,base_brightness);
      lp5523_write(client, LP5523_REG_LED_PWM_BASE + 7,base_brightness);

      first_color=LP5523_CMD_RUN_ENGINE1;
      // load engine program
      if(lp5523_load_program(&pan_led->engines[0], pan_pattern[17])){
				pan_err("EF60_BOOT_START_PATTERN load error \n");
      }
      
      // engine program reg set.
	    if(lp5523_write(pan_led->client, LP5523_REG_ENABLE,	first_color | LP5523_ENABLE)){
	      pan_err("DIMMING_RED_CHARGING program reg enable is error\n");	  
  	  }

  	  // engine program excute.
  	  if( lp5523_write(pan_led->client, LP5523_REG_OP_MODE, first_color)){
        pan_err("DIMMING_RED_CHARGING program reg mode is error\n");
  	  }

  	  // clear led data for next dimming event.
      for(i=0;i<PAN_NUM_LEDS;i++){
				if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
				chip->leds[i].before_brightness =chip->leds[i].brightness;
			}
			chip->home_key_led_current_state = 0;
			chip->home_key_led_before_state = chip->home_key_led_current_state;      
      break;
#endif		
    case DIMMING_ON_COLOR_TIMER:
    case DIMMING_ON_OFF_SHORT_TIMER:
      if(led->brightness == DIMMING_ON_COLOR_TIMER){
      pan_dbg("DIMMING_ON_COLOR_TIMER. \n");
      }else{
        pan_dbg("DIMMING_ON_OFF_SHORT_TIMER. \n");
      }
			ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
			if(ret)
				pan_err("DIMMING_ON_WHITE_10S error ret -> %d\n",ret);

			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
				lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
			}
			/*
			if(chip->home_key_led_current_state & 0x01)
				first_color |= 0x05;
			if(chip->home_key_led_current_state & 0x02)
				first_color |= 0x0a;
			if(chip->home_key_led_current_state & 0x40)
				first_color |= 0xc0;
      */
      if(chip->leds[2].brightness>61){
        pan_dbg(" DIMMING_ON_COLOR_TIMER. chip->leds[1].brightness is bigger than 31. value -> %d\n",chip->leds[2].brightness); 
        chip->leds[2].brightness=61;
      }  
			second_color = 	chip->leds[2].brightness;
			
			if(chip->leds[3].brightness>61){
        pan_dbg(" DIMMING_ON_COLOR_TIMER. chip->leds[3].brightness is bigger than 31. value -> %d\n",chip->leds[3].brightness); 
        chip->leds[3].brightness=61;
      }
      common_color =  chip->leds[3].brightness;
			
			pan_dbg("dimming_color -> %x, wait_dimming_on -> %x, wait_dimming_off -> %x\n",first_color,second_color,common_color);

      if(led->brightness == DIMMING_ON_COLOR_TIMER){
        for(i=0;i<3;i++){
          if(i == 0 && chip->leds[0].brightness){
    		    pan_pattern[18][1]=0x05;
    		    pan_pattern[18][3]=0x00;
    		    pan_pattern[18][5]=chip->leds[0].brightness; 
    		    first_color |= LP5523_CMD_RUN_ENGINE1;    		    
    		  }else if(i == 1 && chip->leds[1].brightness){
    		    pan_pattern[18][1]=0x0a;
    		    pan_pattern[18][3]=0x10;
    		    pan_pattern[18][5]=chip->leds[1].brightness; 
    		    first_color |= LP5523_CMD_RUN_ENGINE2;
    		  }else if(i == 2 && chip->leds[6].brightness){
            pan_pattern[18][1]=0xc0;
            pan_pattern[18][3]=0x20;
            pan_pattern[18][5]=chip->leds[6].brightness; 
    		    first_color |= LP5523_CMD_RUN_ENGINE3;
    		  }else{
    		    continue;
    		  }
    		  
    		  pan_pattern[18][8]=0xa0|(second_color>>1);
    		  pan_pattern[18][9]=((second_color&1)<<7)|0x03;
    		  pan_pattern[18][14]=0xa0|(common_color>>1);
    		  pan_pattern[18][15]=((common_color&1)<<7)|0x06;
    		  
          pan_dbg("pan_pattern[18][8] -> %x, pan_pattern[18][9] -> %x,pan_pattern[18][14] -> %x,pan_pattern[18][15] -> %x\n ",pan_pattern[18][8],pan_pattern[18][9],pan_pattern[18][14],pan_pattern[18][15]);
    		  ret = lp5523_load_program(&chip->engines[i], pan_pattern[18]);
    		  if(ret)
    		    pan_err("DIMMING_ON_COLOR_TIMER.  lp5523_load_program return value -> %d\n",ret);
   		  }  
  		}else{
  		  for(i=0;i<3;i++){
          if(i == 0 && chip->leds[0].brightness){
    		    pan_pattern[19][1]=0x05;
    		    pan_pattern[19][3]=0x00;
    		    pan_pattern[19][5]=chip->leds[0].brightness; 
    		    first_color |= LP5523_CMD_RUN_ENGINE1;    		    
    		  }else if(i == 1 && chip->leds[1].brightness){
    		    pan_pattern[19][1]=0x0a;
    		    pan_pattern[19][3]=0x10;
    		    pan_pattern[19][5]=chip->leds[1].brightness; 
    		    first_color |= LP5523_CMD_RUN_ENGINE2;
    		  }else if(i == 2 && chip->leds[6].brightness){
            pan_pattern[19][1]=0xc0;
            pan_pattern[19][3]=0x20;
            pan_pattern[19][5]=chip->leds[6].brightness; 
    		    first_color |= LP5523_CMD_RUN_ENGINE3;
    		  }else{
    		    continue;
  		    }
    		  pan_pattern[19][8]=0xa0|(second_color>>1);
    		  pan_pattern[19][9]=((second_color&1)<<7)|0x03;
    		  pan_pattern[19][14]=0xa0|(common_color>>1);
    		  pan_pattern[19][15]=((common_color&1)<<7)|0x06;
  	  
          pan_dbg("pan_pattern[19][8] -> %x, pan_pattern[19][9] -> %x,pan_pattern[19][14] -> %x,pan_pattern[19][15] -> %x\n ",pan_pattern[19][8],pan_pattern[19][9],pan_pattern[19][14],pan_pattern[19][15]);
    		  ret = lp5523_load_program(&chip->engines[i], pan_pattern[19]);
    		  if(ret)
    		    pan_err("DIMMING_ON_OFF_SHORT_TIMER.  lp5523_load_program return value -> %d\n",ret);
    		}    
  		}
	  
			ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,  first_color | LP5523_ENABLE);
			if(ret)
			  pan_err("DIMMING_ON_COLOR_TIMER.  lp5523_write return value -> %d\n",ret);
			if( first_color & LP5523_CMD_RUN_ENGINE1){  
  			ret = lp5523_set_engine_mode(&chip->engines[0], LP5523_CMD_RUN_ENGINE1);
  			if(ret)
  			  pan_err("DIMMING_ON_COLOR_TIMER.  lp5523_set_engine_mode return value -> %d\n",ret);
      }
      if( first_color & LP5523_CMD_RUN_ENGINE2){  
  			ret = lp5523_set_engine_mode(&chip->engines[1], LP5523_CMD_RUN_ENGINE2);
  			if(ret)
  			  pan_err("DIMMING_ON_COLOR_TIMER.  lp5523_set_engine_mode return value -> %d\n",ret);
      }
      if( first_color & LP5523_CMD_RUN_ENGINE3){  
  			ret = lp5523_set_engine_mode(&chip->engines[2], LP5523_CMD_RUN_ENGINE3);
  			if(ret)
  			  pan_err("DIMMING_ON_COLOR_TIMER.  lp5523_set_engine_mode return value -> %d\n",ret);
      }
			for(i=0;i<PAN_NUM_LEDS;i++){
				if(i == EF59_MENU_KEY || i == EF59_BACK_KEY) continue;
				chip->leds[i].before_brightness =0;
			}
			chip->home_key_led_current_state = 0;
			chip->home_key_led_before_state = chip->home_key_led_current_state;  
      break;
    
		default :
			break;

		}

	}else{

		if(led->brightness){
			set_bit((int)led->chan_nr, &chip->home_key_led_current_state);
		}else{
			clear_bit((int)led->chan_nr,&chip->home_key_led_current_state);
		}
	}
	pan_dbg("[LED] channel -> %d, value -> %d, home key -> %x, menu/back -> %x \n",led->chan_nr,led->brightness,(int)chip->home_key_led_current_state,(int)chip->menu_back_led_state);


	mutex_unlock(&chip->lock);
}

static int lp5523_do_store_load(struct lp5523_engine *engine,
		const char *buf, size_t len)
{
	struct lp5523_chip *chip = engine_to_lp5523(engine);
	struct i2c_client *client = chip->client;
	int  ret, nrchars, offset = 0, i = 0;
	char c[3];
	unsigned cmd;
	u8 pattern[LP5523_PROGRAM_LENGTH] = {0};

	while ((offset < len - 1) && (i < LP5523_PROGRAM_LENGTH)) {
		/* separate sscanfs because length is working only for %s */
		ret = sscanf(buf + offset, "%2s%n ", c, &nrchars);
		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto fail;
		pattern[i] = (u8)cmd;

		offset += nrchars;
		i++;
	}

	/* Each instruction is 16bit long. Check that length is even */
	if (i % 2)
		goto fail;

	mutex_lock(&chip->lock);

	if (engine->mode == LP5523_CMD_LOAD)
		ret = lp5523_load_program(engine, pattern);
	else
		ret = -EINVAL;

	mutex_unlock(&chip->lock);

	if (ret) {
		dev_err(&client->dev, "failed loading pattern\n");
		return ret;
	}

	return len;
fail:
	dev_err(&client->dev, "wrong pattern format\n");
	return -EINVAL;
}

static ssize_t store_engine_load(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	return lp5523_do_store_load(&chip->engines[nr - 1], buf, len);
}

#define store_load(nr)							\
	static ssize_t store_engine##nr##_load(struct device *dev,		\
			struct device_attribute *attr,	\
			const char *buf, size_t len)	\
{									\
	return store_engine_load(dev, attr, buf, len, nr);		\
}
store_load(1)
store_load(2)
store_load(3)

static ssize_t show_engine_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	switch (chip->engines[nr - 1].mode) {
	case LP5523_CMD_RUN:
		return sprintf(buf, "run\n");
	case LP5523_CMD_LOAD:
		return sprintf(buf, "load\n");
	case LP5523_CMD_DISABLED:
		return sprintf(buf, "disabled\n");
	default:
		return sprintf(buf, "disabled\n");
	}
}

#define show_mode(nr)							\
	static ssize_t show_engine##nr##_mode(struct device *dev,		\
			struct device_attribute *attr,	\
			char *buf)				\
{									\
	return show_engine_mode(dev, attr, buf, nr);			\
}
show_mode(1)
show_mode(2)
show_mode(3)

static ssize_t store_engine_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len, int nr)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	struct lp5523_engine *engine = &chip->engines[nr - 1];
	mutex_lock(&chip->lock);

	if (!strncmp(buf, "run", 3))
		lp5523_set_mode(engine, LP5523_CMD_RUN);
	else if (!strncmp(buf, "load", 4))
		lp5523_set_mode(engine, LP5523_CMD_LOAD);
	else if (!strncmp(buf, "disabled", 8))
		lp5523_set_mode(engine, LP5523_CMD_DISABLED);

	mutex_unlock(&chip->lock);
	return len;
}

#define store_mode(nr)							\
	static ssize_t store_engine##nr##_mode(struct device *dev,		\
			struct device_attribute *attr,	\
			const char *buf, size_t len)	\
{									\
	return store_engine_mode(dev, attr, buf, len, nr);		\
}
store_mode(1)
store_mode(2)
store_mode(3)

static ssize_t show_max_current(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5523_led *led = cdev_to_led(led_cdev);

	return sprintf(buf, "%d\n", led->max_current);
}

static ssize_t show_current(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5523_led *led = cdev_to_led(led_cdev);

	return sprintf(buf, "%d\n", led->led_current);
}

static ssize_t store_current(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lp5523_led *led = cdev_to_led(led_cdev);
	struct lp5523_chip *chip = led_to_lp5523(led);
	ssize_t ret;
	unsigned long curr;

	if (kstrtoul(buf, 0, &curr))
		return -EINVAL;

	if (curr > led->max_current)
		return -EINVAL;

	mutex_lock(&chip->lock);
	ret = lp5523_write(chip->client,
			LP5523_REG_LED_CURRENT_BASE + led->chan_nr,
			(u8)curr);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		return ret;

	led->led_current = (u8)curr;

	return len;
}

/* led class device attributes */
static DEVICE_ATTR(led_current, S_IRUGO | S_IWUSR, show_current, store_current);
static DEVICE_ATTR(max_current, S_IRUGO , show_max_current, NULL);

static struct attribute *lp5523_led_attributes[] = {
	&dev_attr_led_current.attr,
	&dev_attr_max_current.attr,
	NULL,
};

static struct attribute_group lp5523_led_attribute_group = {
	.attrs = lp5523_led_attributes
};

/* device attributes */
static DEVICE_ATTR(engine1_mode, S_IRUGO | S_IWUSR,
		show_engine1_mode, store_engine1_mode);
static DEVICE_ATTR(engine2_mode, S_IRUGO | S_IWUSR,
		show_engine2_mode, store_engine2_mode);
static DEVICE_ATTR(engine3_mode, S_IRUGO | S_IWUSR,
		show_engine3_mode, store_engine3_mode);
static DEVICE_ATTR(engine1_leds, S_IRUGO | S_IWUSR,
		show_engine1_leds, store_engine1_leds);
static DEVICE_ATTR(engine2_leds, S_IRUGO | S_IWUSR,
		show_engine2_leds, store_engine2_leds);
static DEVICE_ATTR(engine3_leds, S_IRUGO | S_IWUSR,
		show_engine3_leds, store_engine3_leds);
static DEVICE_ATTR(engine1_load, S_IWUSR, NULL, store_engine1_load);
static DEVICE_ATTR(engine2_load, S_IWUSR, NULL, store_engine2_load);
static DEVICE_ATTR(engine3_load, S_IWUSR, NULL, store_engine3_load);
static DEVICE_ATTR(selftest, S_IRUGO, lp5523_selftest, NULL);

static struct attribute *lp5523_attributes[] = {
	&dev_attr_engine1_mode.attr,
	&dev_attr_engine2_mode.attr,
	&dev_attr_engine3_mode.attr,
	&dev_attr_selftest.attr,
	&dev_attr_engine1_load.attr,
	&dev_attr_engine1_leds.attr,
	&dev_attr_engine2_load.attr,
	&dev_attr_engine2_leds.attr,
	&dev_attr_engine3_load.attr,
	&dev_attr_engine3_leds.attr,
};

static const struct attribute_group lp5523_group = {
	.attrs = lp5523_attributes,
};

/*
static int lp5523_register_sysfs(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int ret;

	ret = sysfs_create_group(&dev->kobj, &lp5523_group);
	if (ret < 0)
		return ret;

	return 0;
}
*/

static void lp5523_unregister_sysfs(struct i2c_client *client)
{
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int i;

	sysfs_remove_group(&dev->kobj, &lp5523_group);

	for (i = 0; i < chip->num_leds; i++)
		sysfs_remove_group(&chip->leds[i].cdev.dev->kobj,
				&lp5523_led_attribute_group);
}

/*--------------------------------------------------------------*/
/*			Set chip operating mode			*/
/*--------------------------------------------------------------*/
static int lp5523_set_mode(struct lp5523_engine *engine, u8 mode)
{
	int ret = 0;

	/* if in that mode already do nothing, except for run */
	if (mode == engine->mode && mode != LP5523_CMD_RUN)
		return 0;

	if (mode == LP5523_CMD_RUN) {
		ret = lp5523_run_program(engine);
	} else if (mode == LP5523_CMD_LOAD) {
		lp5523_set_engine_mode(engine, LP5523_CMD_DISABLED);
		lp5523_set_engine_mode(engine, LP5523_CMD_LOAD);
	} else if (mode == LP5523_CMD_DISABLED) {
		lp5523_set_engine_mode(engine, LP5523_CMD_DISABLED);
	}

	engine->mode = mode;

	return ret;
}

/*--------------------------------------------------------------*/
/*			Probe, Attach, Remove			*/
/*--------------------------------------------------------------*/
static int __init lp5523_init_engine(struct lp5523_engine *engine, int id)
{
	if (id < 1 || id > LP5523_ENGINES)
		return -1;
	engine->id = id;
	engine->engine_mask = LP5523_ENG_MASK_BASE >> SHIFT_MASK(id);
	engine->prog_page = id - 1;
	engine->mux_page = id + 2;

	return 0;
}

static int lp5523_init_led(struct lp5523_led *led, struct device *dev,
		int chan, struct lp5523_platform_data *pdata)
{
	char name[32];
#ifdef PAN_LED_NAME_DEBUG	
	char *name_memory;
#endif 
	int res;

	if (chan >= LP5523_LEDS)
		return -EINVAL;

	if (pdata->led_config[chan].led_current) {
		led->led_current = pdata->led_config[chan].led_current;
		led->max_current = pdata->led_config[chan].max_current;
		led->chan_nr = pdata->led_config[chan].chan_nr;

		if (led->chan_nr >= LP5523_LEDS) {
			pan_dbg("Use channel numbers between 0 and %d\n",
					LP5523_LEDS - 1);
			return -EINVAL;
		}

		snprintf(name, sizeof(name), "%s:channel%d",
				pdata->label ?: "lp5523", chan);

#ifdef PAN_LED_NAME_DEBUG
		name_memory=kzalloc(sizeof(name), GFP_KERNEL);
		memcpy(name_memory, name, strlen(name));

		led->cdev.name = name_memory;
#else
		led->cdev.name = name;
#endif
#ifdef OFFLINE_CHARGER_LED_CONTROL
		if ( strcmp(led->cdev.name, "lp5523:channel8") == 0 ){
			pan_err(" control_led is detected\n");
			control_led = led;
		}
#endif

		led->cdev.brightness_set = lp5523_set_brightness;

		//++ p11309 - 2013.05.06 for LED Trigger
		if ( pdata->led_config[chan].default_trigger )
			led->cdev.default_trigger = pdata->led_config[chan].default_trigger;
		//-- p11309

		res = led_classdev_register(dev, &led->cdev);
		if (res < 0) {
			pan_dbg("couldn't register led on channel %d\n",
					chan);
			return res;
		}
		res = sysfs_create_group(&led->cdev.dev->kobj,
				&lp5523_led_attribute_group);
		if (res < 0) {
			pan_dbg("couldn't register current attribute\n");
			led_classdev_unregister(&led->cdev);
			return res;
		}
	} else {
		led->led_current = 0;
	}
#ifdef 	LED_OFF_CHECK_TIMER_USAGE
    led->charger_off=0;
#endif
	return 0;
}

static int lp5523_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lp5523_chip		*chip;
	struct lp5523_platform_data	*pdata;
	int ret, i, led;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
#if defined (OFFLINE_CHARGER_LED_CONTROL)
	int offline_charger = 0;
	oem_pm_smem_vendor1_data_type *smem_id_vendor1_ptr;
#endif

	printk("+-----------------------------------------+\n");
	printk("|  Texas Instrument LED Driver LP5523     |\n");
	printk("+-----------------------------------------+\n");

#if defined (OFFLINE_CHARGER_LED_CONTROL)
	smem_id_vendor1_ptr =  (oem_pm_smem_vendor1_data_type*)smem_alloc(SMEM_ID_VENDOR1,sizeof(oem_pm_smem_vendor1_data_type),0,SMEM_ANY_HOST_FLAG);
	if(smem_id_vendor1_ptr->power_on_mode == 0){
		pan_err(" OFFLINE_CHARGER is enabled. LED DIMMING ON\n");
		offline_charger=1;
	}
#endif

  pan_led_single_thread_queue = create_singlethread_workqueue("pan_led_single_thread_queue");
  if(!pan_led_single_thread_queue){
    pan_err(" pan_led_single_thread_queue initialization is failed\n");
    return -1;
  }
  
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		pan_dbg("Not assigned I2C_FUNC_SMBUS_BYTE\n");
		ret = -EIO;
		return ret;
	}	

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pan_dbg("failed to allocate lp5523_chip memory\n");
		return -ENOMEM;
	}	
	pan_led=chip;

	//++ p11309 - 2013.04.25 for support device tree
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, 
				sizeof(struct lp5523_platform_data), GFP_KERNEL);
		if (!pdata) {
			pan_dbg("failed to allocate lp5523_platform_data memory.\n");
			return -ENOMEM;
		}

		ret = lp5523_parse_dt(&client->dev, pdata);
		if (ret) {
			pan_dbg("failed to parse device tree.\n");
			return ret;
		}
	}
	else {
		pdata = client->dev.platform_data;
	}	
	//-- p11309

	if (!pdata) {
		pan_dbg("no platform data\n");
		ret = -EINVAL;
		goto fail1;
	}

	mutex_init(&chip->lock);

	chip->pdata = pdata;	
	chip->client = client;
	i2c_set_clientdata(client, chip);	

	ret = lp5523_en_pin_setup(pdata->led_en_gpio);
	if (ret < 0) {
		pan_dbg("setup resources fail!\n");
		goto fail1;
	}

	lp5523_en_pin_enable(pdata->led_en_gpio, 0);
	usleep_range(1000, 2000); /* Keep enable down at least 1ms */
	lp5523_en_pin_enable(pdata->led_en_gpio, 1);
	usleep_range(1000, 2000); /* 500us abs min. */

	ret = lp5523_write(client, LP5523_REG_RESET, 0xff);
	if (ret) {
		pan_dbg("lp5523_write: LP5523 REG Reset fail. rc=%d\n", ret);
	}
	usleep_range(10000, 20000); 
	/*
	 * Exact value is not available. 10 - 20ms
	 * appears to be enough for reset.
	 */
	ret = lp5523_detect(client);
	if (ret) {
		pan_dbg("lp5523_detect fail!\n");
		goto fail2;
	}

	pan_dbg("LP5523 Programmable led chip found\n");

	/* Initialize engines */
	for (i = 0; i < ARRAY_SIZE(chip->engines); i++) {
		ret = lp5523_init_engine(&chip->engines[i], i + 1);
		if (ret) {
			pan_dbg("error initializing engine\n");
			goto fail2;
		}
	}
	ret = lp5523_configure(client);
	if (ret < 0) {
		pan_dbg("error configuring chip\n");
		goto fail2;
	}

	/* Initialize leds */
	chip->num_channels = pdata->num_channels;
	chip->num_leds = 0;
	led = 0;
	for (i = 0; i < pdata->num_channels; i++) {
		/* Do not initialize channels that are not connected */
		if (pdata->led_config[i].led_current == 0)
			continue;

		ret = lp5523_init_led(&chip->leds[led], &client->dev, i, pdata);
		if (ret) {
			pan_dbg("error initializing leds\n");
			goto fail3;
		}
		chip->num_leds++;

		chip->leds[led].id = led;
		/* Set LED current */
		ret = lp5523_write(client,
				LP5523_REG_LED_CURRENT_BASE + chip->leds[led].chan_nr,
				chip->leds[led].led_current);
		if (ret) {
			pan_dbg("lp5523_write: REG_LED_CURRENT, chan=%u, current=%u\n",
					chip->leds[led].chan_nr, chip->leds[led].led_current);
		}

		INIT_WORK(&(chip->leds[led].brightness_work),
				lp5523_led_brightness_work);

		led++;
	}

/*
	ret = lp5523_register_sysfs(client);
	if (ret) {
		pan_dbg("registering sysfs failed\n");
		goto fail3;
	}
*/
	
#ifdef PAN_LED_SYSFS_DEBUG
	ret = misc_register(&led_lp5523_io);
	if (ret) {
		printk("[LED_LP5523] led_lp5523_fops can''t register misc device\n");
	}
#endif
#ifdef LED_OFF_CHECK_TIMER_USAGE
    INIT_WORK(&led_off_check_work,lp5523_led_off_check_work);
    init_timer(&led_off_check_timer);
		led_off_check_timer.function = led_off_check_timer_func;
		led_off_check_timer.data = 0;
#endif

#if defined(CONFIG_MACH_MSM8937_EF71IE) || defined(CONFIG_MACH_MSM8937_EF71S) || defined(CONFIG_MACH_MSM8937_EF71K)
    pan_pattern[17][5] = 255 - base_brightness;
    pan_pattern[17][9] = 255 - base_brightness;
#endif

#ifdef OFFLINE_CHARGER_LED_CONTROL
	if(offline_charger){
		init_timer(&offline_led_check_timer);
		offline_led_check_timer.function = offline_led_check_timer_func;
		offline_led_check_timer.data = 0;
		mod_timer(&offline_led_check_timer, jiffies + msecs_to_jiffies(OFFLINE_LEC_CHECK_INTERVAL));
		
#ifdef LED_DIMMING_IN_CHARGING    
    ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
		if(ret)
			pan_err("DIMMING_RED_CHARGING error in offline ret -> %d\n",ret);
			
    // led off for dimming event.
		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
    // register set logarithmic dimming.
    enable_logarithmic_dimming();

    // set base brightness.
    lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 6,base_brightness);
    lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 7,base_brightness);
    
    // load engine program
    if(lp5523_load_program(&pan_led->engines[0], pan_pattern[17])){
			pan_err("DIMMING_RED_CHARGING load error in offline \n");
    }
    
    // engine program reg set.
    if(lp5523_write(pan_led->client, LP5523_REG_ENABLE,	LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE)){
      pan_err("DIMMING_RED_CHARGING program reg enable is error in offline \n");	  
	  }

	  // engine program excute.
	  if( lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_RUN_ENGINE1)){
      pan_err("DIMMING_RED_CHARGING program reg mode is error in offline \n");
	  }
#else
		// red led light 
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 6,MAX_BRIGHTNESS);
		lp5523_write(pan_led->client, LP5523_REG_LED_PWM_BASE + 7,MAX_BRIGHTNESS);
#endif
    
    // offline _led_state init. 
		offline_led_state=OFFLINE_LED_RED;
#ifdef OFFLINE_CHARGING_GREEN_CURRENT_2MA
    lp5523_write(pan_led->client, LP5523_REG_LED_CURRENT_BASE   ,20);
   	lp5523_write(pan_led->client, LP5523_REG_LED_CURRENT_BASE +2,20);
#endif
	}else{
#ifdef	EF60_BOOT_RAINBOW_LED
    pan_err("Booting Rainbow Led start.\n");

    // Rainbow LED OFF Event timer is initialized for sys vob. sys vob don't receive led off event from framework layer.
    init_timer(&offline_led_check_timer);
		offline_led_check_timer.function = booting_led_check_timer_func;
		offline_led_check_timer.data = 0;
		mod_timer(&offline_led_check_timer, jiffies + msecs_to_jiffies(BOOTING_LEC_CHECK_INTERVAL));

    // Rainbow LED Pattern change timer. Interval time is defined BOOTING_RAINBOW_LED_CHECK_INTERVAL.
		init_timer(&boot_led_check_timer);
		boot_led_check_timer.function = boot_rainbow_led_timer_func;
		boot_led_check_timer.data = 0;

		//Init Rainbow led work structure.
    INIT_WORK(&boot_led_work,ef71_boot_led_work);
    
    // boot_led_off is flag if led driver receives led off event from framework layer.
    boot_led_off = 1;
    boot_led_pattern_num=0;

    //start rainbow work.
		queue_work(pan_led_single_thread_queue, &boot_led_work);
    
#else
		pan_err("Booting Led start.\n");
		init_timer(&offline_led_check_timer);
		offline_led_check_timer.function = booting_led_check_timer_func;
		offline_led_check_timer.data = 0;
		mod_timer(&offline_led_check_timer, jiffies + msecs_to_jiffies(BOOTING_LEC_CHECK_INTERVAL));
		boot_led_off = 1;
		ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE, LP5523_CMD_DISABLED);
		if(ret)
			pan_err("Booting led error ret -> %d\n",ret);

		for(i=0;i<PAN_NUM_LEDS;i++){
			if(i==EF59_MENU_KEY || i==EF59_BACK_KEY) continue;
			lp5523_write(client, LP5523_REG_LED_PWM_BASE + i,MIN_BRIGHTNESS);
		}
		ret = lp5523_load_program(&pan_led->engines[0], pan_pattern[10]);
		if(ret)
			pan_err("Booting led red load progrma error ret -> %d\n",ret);

		ret = lp5523_write(pan_led->client, LP5523_REG_ENABLE,LP5523_CMD_RUN_ENGINE1 | LP5523_ENABLE);
		if(ret)
			pan_err("Booting led write  LP5523_REG_ENABLE error ret -> %d\n",ret);
		ret = lp5523_write(pan_led->client, LP5523_REG_OP_MODE,LP5523_CMD_RUN_ENGINE1);
		if(ret)
			pan_err("Booting led write  LP5523_REG_OP_MODE error ret -> %d\n",ret);	
#endif
	}
#endif

	pan_func_out();

	return ret;
fail3:
	for (i = 0; i < chip->num_leds; i++) {
		led_classdev_unregister(&chip->leds[i].cdev);
		cancel_work_sync(&chip->leds[i].brightness_work);
	}
fail2:
	lp5523_en_pin_enable(pdata->led_en_gpio, 0);
	lp5523_en_pin_release(pdata->led_en_gpio);
fail1:
	kfree(chip);
	return ret;
}

static int lp5523_remove(struct i2c_client *client)
{
	struct lp5523_chip *chip = i2c_get_clientdata(client);
	int i;

	lp5523_unregister_sysfs(client);

	for (i = 0; i < chip->num_leds; i++) {
#ifdef PAN_LED_NAME_DEBUG	
		kfree(&chip->leds[i].cdev.name);
#endif
		led_classdev_unregister(&chip->leds[i].cdev);
		cancel_work_sync(&chip->leds[i].brightness_work);
	}

	lp5523_en_pin_enable(chip->pdata->led_en_gpio, 0);
	lp5523_en_pin_release(chip->pdata->led_en_gpio);
	kfree(chip);
	return 0;
}

static const struct i2c_device_id lp5523_id[] = {
	{ "lp5523", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lp5523_id);

//++ p11309 - 2013.04.23 for Device Tree
static struct of_device_id lp5523_match_table[] = {
	{ .compatible = "ti,lp5523_led_driver",},
	{ },
};
//-- p11309

static struct i2c_driver lp5523_driver = {
	.driver = {
		.name	= "lp5523",
		.owner	= THIS_MODULE,

		//++ p11309 - 2013.04.23 for Device Tree
		.of_match_table = lp5523_match_table,
		//-- p11309

	},
	.probe		= lp5523_probe,
	.remove		= lp5523_remove,
	.id_table	= lp5523_id,
};

module_i2c_driver(lp5523_driver);

MODULE_AUTHOR("Mathias Nyman <mathias.nyman@nokia.com>");
MODULE_DESCRIPTION("LP5523 LED engine");
MODULE_LICENSE("GPL");

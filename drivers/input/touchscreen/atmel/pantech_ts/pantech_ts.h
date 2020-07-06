#ifndef _PANTECH_TS_POWER_
#define _PANTECH_TS_POWER_

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/io.h>
//#include <asm/mach-types.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
//#include <mach/gpio.h>
#include <linux/regulator/consumer.h>
//#include <mach/vreg.h>

/* -------------------------------------------------------------------- */
/* debug option */
/* -------------------------------------------------------------------- */
#define PAN_TOUCH_DEBUG_I2C_MASK                0x00000001
#define PAN_TOUCH_DEBUG_OPERATION_ERR_MASK      0x00000002
#define PAN_TOUCH_DEBUG_OPERATION_MASK          0x00000004
#define PAN_TOUCH_DEBUG_IOCTL_MASK              0x00000008  
#define PAN_TOUCH_DEBUG_FIRMWARE_MASK           0x00000010
#define PAN_TOUCH_DEBUG_FUNCTION_NAME_MASK      0x00000020
#define PAN_TOUCH_DEBUG_HW_MASK                 0x00000040
#define PAN_TOUCH_DEBUG_CONFIG_MASK             0x00000080
#define PAN_TOUCH_DEBUG_TOUCH_MASK              0x00000100
#define PAN_TOUCH_DEBUG_ALL_MASK                0x000001FF

static int pan_debug_state = PAN_TOUCH_DEBUG_OPERATION_ERR_MASK;
#define dbg_cr(fmt, args...)      printk("[+++ TOUCH] " fmt, ##args);
#define dbg_i2c(fmt,args...)      if(pan_debug_state & PAN_TOUCH_DEBUG_I2C_MASK) printk("[+++ TOUCH] " fmt,##args);
#define dbg_op_err(fmt,args...)   if(pan_debug_state & PAN_TOUCH_DEBUG_OPERATION_ERR_MASK) printk("[+++ TOUCH] " fmt,##args); 
#define dbg_op(fmt,args...)       if(pan_debug_state & PAN_TOUCH_DEBUG_OPERATION_MASK) printk("[+++ TOUCH] " fmt,##args);
#define dbg_ioctl(fmt,args...)    if(pan_debug_state & PAN_TOUCH_DEBUG_IOCTL_MASK) printk("[+++ TOUCH] " fmt,##args); 
#define dbg_firmw(fmt,args...)    if(pan_debug_state & PAN_TOUCH_DEBUG_FIRMWARE_MASK) printk("[+++ TOUCH] " fmt,##args); 
#define dbg_func_in()			        if(pan_debug_state & PAN_TOUCH_DEBUG_FUNCTION_NAME_MASK) printk("[+++ TOUCH] %s Func In\n", __func__);
#define dbg_func_out()			      if(pan_debug_state & PAN_TOUCH_DEBUG_FUNCTION_NAME_MASK) printk("[+++ TOUCH] %s Func Out\n", __func__)
#define dbg_hw(fmt,args...)       if(pan_debug_state & PAN_TOUCH_DEBUG_HW_MASK) printk("[+++ TOUCH] " fmt,##args); 
#define dbg_config(fmt,args...)   if(pan_debug_state & PAN_TOUCH_DEBUG_CONFIG_MASK) printk("[+++ TOUCH] " fmt,##args);
#define dbg_touch(fmt,args...)    if(pan_debug_state & (PAN_TOUCH_DEBUG_TOUCH_MASK|PAN_TOUCH_DEBUG_OPERATION_MASK)) printk("[+++ TOUCH] " fmt,##args);

#define SKY_PROCESS_CMD_KEY 	1
#define TOUCH_IO
#define TOUCH_DEV_TREE_SUPPORT

typedef enum
{
	TSC_CLEAR_ALL,
	TSC_CLEAR_EVENT,
} tsc_clear_type;

#ifdef SKY_PROCESS_CMD_KEY
typedef enum {	
	READ_TOUCH_ID = 101,
	APPLY_TOUCH_CONFIG = 501,
	DIAG_DEBUG = 502,
	RESET_TOUCH_CONFIG = 503,
	GET_TOUCH_CONFIG = 504,
	SET_TOUCH_CONFIG = 505,
	//READ_ITO_TYPE = 506,
	ATMEL_GET_REFERENCE_DATA = 508,
	RESET_TOUCH_PARAMETER = 509,
	ATMEL_GET_SELFCAP_DATA = 510,
	TOUCH_CHARGER_MODE = 701,
	TOUCH_IOCTL_READ_LASTKEY=1001,	
	TOUCH_IOCTL_DO_KEY,	
	TOUCH_IOCTL_RELEASE_KEY, 
	TOUCH_IOCTL_CLEAN,
	TOUCH_IOCTL_DEBUG,
	TOUCH_IOCTL_RESTART,
	TOUCH_IOCTL_PRESS_TOUCH,
	TOUCH_IOCTL_RELEASE_TOUCH,
	TOUCH_IOCTL_CHARGER_MODE,
	TOUCH_IOCTL_EARJACK_MODE,
	POWER_OFF,
	TOUCH_IOCTL_DELETE_ACTAREA = 2001,
	TOUCH_IOCTL_RECOVERY_ACTAREA,
	TOUCH_IOCTL_SENSOR_X = 2005,
	TOUCH_IOCTL_SENSOR_Y,
	TOUCH_IOCTL_CHECK_BASE,
	TOUCH_IOCTL_READ_IC_VERSION,
	TOUCH_IOCTL_READ_FW_VERSION,
	TOUCH_IOCTL_START_UPDATE,
	TOUCH_IOCTL_SELF_TEST,
	TOUCH_IOCTL_DIAGNOSTIC_MIN_DEBUG,
	TOUCH_IOCTL_DIAGNOSTIC_MAX_DEBUG,
	TOUCH_IOCTL_DIAG_DELTA = 2014,
//20140703_for_selftest +
	TOUCH_IOCTL_SELF_CAP_GET_ONOFF = 2020,
	TOUCH_IOCTL_SELF_CAP_SET_ONOFF = 2021,
//20140703_for_selftest -
	TOUCH_IOCTL_INIT = 3001,	
	TOUCH_IOCTL_OFF  = 3002,	
	TOUCH_IOCTL_EVENT_AUTOCAL_DISABLE  = 5001,
	TOUCH_IOCTL_DIAG_DEBUG_DELTA = 5010,
	TOUCH_IOCTL_DIAG_DEBUG_REF = 5011,
	TOUCH_IOCTL_DIAG_DEBUG_CALIBRATION	=5014,
	TOUCH_IOCTL_DIAG_DEBUG_SELF_DELTA = 5015,
	TOUCH_IOCTL_DIAG_DEBUG_SELF_REF = 5016,
	
	TOUCH_IOCTL_MULTI_TSP_OBJECT_SEL  = 6001,
	TOUCH_IOCTL_SUSPEND,
	TOUCH_IOCTL_RESUME,
//++ p11309 - 2013.07.10 for Get Touch Mode 
	TOUCH_IOCTL_MULTI_TSP_OBJECT_GET,
//++ p11309 - 2013.05.14 for gold reference T66
  TOUCH_IOCTL_HALL_IC_GPIO_GET,         //P13106 for reading hall-ic gpio state
	TOUCH_IOCTL_WIFI_DEBUG_APP_ENABLE	= 7004,
	TOUCH_IOCTL_WIFI_DEBUG_APP_DISABLE	= 7005,
//++ p11309 - 2013.07.19 Check Noise Mode shake
#ifdef PAN_CHECK_NOISE_MODE_SHAKE
	TOUCH_IOCTL_NOISE_MODE_SHAKE_CHECK_ENABLE = 7100,
	TOUCH_IOCTL_NOISE_MODE_SHAKE_CHECK_DISABLE = 7101,
	TOUCH_IOCTL_NOISE_MODE_SHAKE_CHECK_GET = 7102,
#endif
//-- p11309
//++ p11309 - 2013.07.25 Get Model Color 
	TOUCH_IOCTL_MODEL_COLOR_GET = 7200,
	TOUCH_IOCTL_TOUCH_MODE = 7201,
//++ p11309 - 2013.09.06 for Touch Pen Insertion Check
  TOUCH_IOCTL_CHECK_TOUCH_PEN = 7300,
//-- p11309

//++ p11309 - 2013.10.01 for Touch Knock On		
	TOUCH_IOCTL_KNOCKON_SET_ENABLE = 7400,
	TOUCH_IOCTL_KNOCKON_GET_ENABLE = 7401,
	TOUCH_IOCTL_KNOCKON_SET_INIT_CFG		 = 7402,
	
	TOUCH_IOCTL_KNOCKON_SET_IDLE_INTERVAL    = 7410,
	TOUCH_IOCTL_KNOCKON_SET_ACTIVE_INTERVAL  = 7411,
	TOUCH_IOCTL_KNOCKON_SET_ACTIVE_TO_IDLE   = 7412,
	TOUCH_IOCTL_KNOCKON_SET_IDLE_SYNCSPERX   = 7413,
	TOUCH_IOCTL_KNOCKON_SET_ACTIVE_SYNCSPERX = 7414,

	TOUCH_IOCTL_KNOCKON_GET_IDLE_INTERVAL    = 7420,
	TOUCH_IOCTL_KNOCKON_GET_ACTIVE_INTERVAL  = 7421,
	TOUCH_IOCTL_KNOCKON_GET_ACTIVE_TO_IDLE   = 7422,
	TOUCH_IOCTL_KNOCKON_GET_IDLE_SYNCSPERX   = 7423,
	TOUCH_IOCTL_KNOCKON_GET_ACTIVE_SYNCSPERX = 7424,
	TOUCH_IOCTL_CONTROL_KNOCKON = 7430,
//-- p11309

} TOUCH_IOCTL_CMD;
#endif

#ifdef TOUCH_IO
typedef enum  {
	IOCTL_DEBUG_SUSPEND = 0,	
	IOCTL_DEBUG_RESUME = 1,	
	IOCTL_DEBUG_GET_TOUCH_ANTITOUCH_INFO = 2,
	IOCTL_DEBUG_TCH_CH= 3,	
	IOCTL_DEBUG_ATCH_CH= 4,	
	IOCTL_GET_CALIBRATION_CNT = 5,	
} ioctl_debug_cmd;
#endif

static int diagnostic_min =0;
static int diagnostic_max =0;
#ifdef TOUCH_IO
static bool touch_diagnostic_ret = true;
#endif //TOUCH_IO

static int reference_data[MXT_MAX_CHANNEL_NUM] = {0};



/* -------------------------------------------------------------------- */
/* TSP POWER */
/* -------------------------------------------------------------------- */

enum tsp_power_pin_type {
	POWER_NOT_USED=0,
	POWER_GPIO_SETUP,	// gpio setup
	POWER_PM_REGULATOR,	// PMIC regulator setup
};

enum gpio_direction {	
	GPIO_OUTPUT_LOW=0,	// out port, default low
	GPIO_OUTPUT_HIGH,	// out port, default high
	GPIO_INPUT,			// in port
};

enum power_up_down {	
	POWER_DOWN=0,
	POWER_UP,
};

//NOT USE in dtsi (parse dt)
#define GPIO_TOUCH_RST			938 //60
#define GPIO_TOUCH_CHG			939 //61
#define GPIO_TOUCH_DVDD 		941 //63

struct tsp_power_pin_ctrl {
	int	type;
	const char *name;

	struct {
		int	num;		
		int	direction;
	} gpio;

	struct {	
		int	volt;
		struct regulator *reg;
	} regulator;
};
static int tsp_power_pin_setuped = 0;

static struct tsp_power_pin_ctrl atmel_mxt_avdd;
static struct tsp_power_pin_ctrl atmel_mxt_dvdd;
static struct tsp_power_pin_ctrl atmel_mxt_vddio;
static struct tsp_power_pin_ctrl atmel_mxt_int;
static struct tsp_power_pin_ctrl atmel_mxt_rst;

#ifdef TOUCH_DEV_TREE_SUPPORT
int rst_gpio = -1;
int iovdd_gpio = -1;
int chg_gpio = -1;
#endif

void TSP_Power_Pin_Init(void) {
	if ( tsp_power_pin_setuped == 1 ) return;

#if 0 
	atmel_mxt_avdd.type = POWER_PM_REGULATOR;
	atmel_mxt_avdd.name = "pm8994_l22";
	atmel_mxt_avdd.regulator.volt = 3300000;
	if ( atmel_mxt_avdd.regulator.reg )
		atmel_mxt_avdd.regulator.reg = NULL;
#endif 

	atmel_mxt_dvdd.type = POWER_GPIO_SETUP;
	atmel_mxt_dvdd.name = "touch_power_dvdd";


#ifdef TOUCH_DEV_TREE_SUPPORT
	atmel_mxt_dvdd.gpio.num = iovdd_gpio;
#else
	atmel_mxt_dvdd.gpio.num = GPIO_TOUCH_DVDD;
#endif
	atmel_mxt_dvdd.gpio.direction = GPIO_OUTPUT_HIGH;

	atmel_mxt_vddio.type = POWER_NOT_USED;

	atmel_mxt_int.type = POWER_GPIO_SETUP;
	atmel_mxt_int.name = "touch_int_n";
#ifdef TOUCH_DEV_TREE_SUPPORT
	atmel_mxt_int.gpio.num = chg_gpio;
#else
	atmel_mxt_int.gpio.num = GPIO_TOUCH_CHG;
#endif
	atmel_mxt_int.gpio.direction = GPIO_INPUT;	

	atmel_mxt_rst.type = POWER_GPIO_SETUP;
	atmel_mxt_rst.name = "touch_rst_n";
#ifdef TOUCH_DEV_TREE_SUPPORT
	atmel_mxt_rst.gpio.num = rst_gpio;	
#else
	atmel_mxt_rst.gpio.num = GPIO_TOUCH_RST;	
#endif
	atmel_mxt_rst.gpio.direction = GPIO_OUTPUT_LOW;

	tsp_power_pin_setuped = 1;
}

int TSP_PowerSetup(struct tsp_power_pin_ctrl pp_ctrl, int up_down) 
{
	int rc;	

	switch ( pp_ctrl.type ) {
	case POWER_NOT_USED:
		break;
	case POWER_GPIO_SETUP:			
		if ( up_down == POWER_UP ) {
			rc = gpio_request(pp_ctrl.gpio.num, pp_ctrl.name);
			if (rc) {
				gpio_free(pp_ctrl.gpio.num);				
				if (rc) {
					dbg_cr("%s: %d failed, rc=%d\n",pp_ctrl.name, pp_ctrl.gpio.num, rc);
				}
			}

			if ( pp_ctrl.gpio.direction == GPIO_INPUT ) {
				rc = gpio_direction_input(pp_ctrl.gpio.num);				
				if (rc) {
					dbg_cr("%s: %d gpio_direction_input failed, rc=%d\n",pp_ctrl.name, pp_ctrl.gpio.num, rc);
				}
			}
			else {
				rc = gpio_direction_output(pp_ctrl.gpio.num, pp_ctrl.gpio.direction);
				if (rc) {
					dbg_cr("%s: %d gpio_direction_output failed, rc=%d\n",pp_ctrl.name, pp_ctrl.gpio.num, rc);
				}
			}			
		}
		else {

			if ( pp_ctrl.gpio.direction == GPIO_OUTPUT_HIGH || pp_ctrl.gpio.direction == GPIO_OUTPUT_LOW) {				
				gpio_set_value(pp_ctrl.gpio.num, !gpio_get_value(pp_ctrl.gpio.num));
				msleep(10);
			}

			gpio_free(pp_ctrl.gpio.num);
		}		

		break;
	case POWER_PM_REGULATOR:

		if ( up_down == POWER_UP ) {

			if ( pp_ctrl.regulator.reg == NULL ) 
			{
				pp_ctrl.regulator.reg = regulator_get(NULL, pp_ctrl.name);
				if( pp_ctrl.regulator.reg == NULL ) {
					dbg_cr("%s: regulator_get failed \n", pp_ctrl.name);
					return -EINVAL;
				}
				rc = regulator_set_voltage(pp_ctrl.regulator.reg, pp_ctrl.regulator.volt, pp_ctrl.regulator.volt);
				if (rc) { 
					dbg_cr("%s: set_voltage %duV failed, rc=%d\n", pp_ctrl.name, pp_ctrl.regulator.volt, rc);
					return rc;
				}
			}

			rc = regulator_enable(pp_ctrl.regulator.reg);
			if (rc) {
				dbg_cr("%s: regulator enable failed (%d)\n", pp_ctrl.name, rc);
				return rc;
			}			
		}
		else {			
			if( pp_ctrl.regulator.reg == NULL ) {
				//dbg_cr("%s: No regulator...failed \n", pp_ctrl.name);

				pp_ctrl.regulator.reg = regulator_get(NULL, pp_ctrl.name);
				if( pp_ctrl.regulator.reg == NULL ) {
					dbg_cr("%s: regulator_get failed \n", pp_ctrl.name);
					return -EINVAL;
				}

				rc = regulator_set_voltage(pp_ctrl.regulator.reg, pp_ctrl.regulator.volt, pp_ctrl.regulator.volt);
				if (rc) { 
					dbg_cr("%s: set_voltage %duV failed, rc=%d\n", pp_ctrl.name, pp_ctrl.regulator.volt, rc);
					return rc;
				}
			}

			rc = regulator_disable(pp_ctrl.regulator.reg);
			if (rc) {
				dbg_cr("%s: regulator disable failed (%d)\n", pp_ctrl.name, rc);
				return rc;
			}

			regulator_put(pp_ctrl.regulator.reg);
			pp_ctrl.regulator.reg = NULL;
		}
		break;
	}
	return 0;
}

void TSP_reset_pin_shake(void)
{
	gpio_set_value(atmel_mxt_rst.gpio.num, GPIO_OUTPUT_LOW);	
	msleep(1);
	gpio_set_value(atmel_mxt_rst.gpio.num, GPIO_OUTPUT_HIGH);	
}

int TSP_PowerOn(void)
{
	dbg_func_in();
	TSP_Power_Pin_Init();

	TSP_PowerSetup(atmel_mxt_rst, POWER_UP);
	TSP_PowerSetup(atmel_mxt_avdd, POWER_UP);
	TSP_PowerSetup(atmel_mxt_dvdd, POWER_UP);
	TSP_PowerSetup(atmel_mxt_vddio, POWER_UP);
	TSP_reset_pin_shake();

	TSP_PowerSetup(atmel_mxt_int, POWER_UP);	
	
	msleep(100);
	dbg_func_out();
	return 0;
}

void TSP_PowerOff(void)
{
	dbg_func_in();
	TSP_Power_Pin_Init();

	TSP_PowerSetup(atmel_mxt_int, POWER_DOWN);
	TSP_PowerSetup(atmel_mxt_rst, POWER_DOWN);

	TSP_PowerSetup(atmel_mxt_avdd, POWER_DOWN);
	TSP_PowerSetup(atmel_mxt_dvdd, POWER_DOWN);
	TSP_PowerSetup(atmel_mxt_vddio, POWER_DOWN);
	
	dbg_func_out();
}

#endif /* _PANTECH_TS_POWER_ */

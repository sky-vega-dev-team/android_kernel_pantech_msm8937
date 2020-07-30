/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/fb.h>
/* -------------------------------------------------------------------- */
/* option */
/* -------------------------------------------------------------------- */

/* CHIP */
#define MXT_540S	0
#define	MXT_641T	1
#define MXT_CFG_WRITE_BIN	1
//#define MXT_CFG_EF78	1
#define MXT_CFG_EF71	1

/* FW_VERSION */
#define MXT_CURRENT_FAMILY_ID	  0xA4
#if (CONFIG_BOARD_VER == CONFIG_PT10 )
#define MXT_LATEST_FW_VERSION 0x10
#define MXT_LATEST_FW_BUILD 0xAA
#else
#define MXT_LATEST_FW_VERSION	 0x30 //0x10
#define MXT_LATEST_FW_BUILD	  0xAC //0xAA
#endif

#ifdef MXT_CFG_EF71
#if(CONFIG_BOARD_VER == CONFIG_PT10 )
#define MXT_LATEST_CONFIG_CRC 0x96A27
#else
#define MXT_LATEST_CONFIG_CRC	0x42106C //0xBE93E1 //0xE680E5 //0xD0DAFD //0xEEE02F //0x21CC3C //0xC94910 //0xD8E99C //0xC0F69E //0x92EE4A //0x31F2DA //0x4E71F4 //0x4CAA93 //0x096A27//0x9DD31D
#endif
#else
#define MXT_LATEST_CONFIG_CRC	0xA8023A
#endif
#define MXT_MAX_OBJECT	120

#define OFFLINE_CHARGER_TOUCH_DISABLE 	1

#define CFG_WRITE_OK                0u

unsigned char mxts_fw[] = {
#if(CONFIG_BOARD_VER == CONFIG_PT10 )
#include "./firmware/mxt641t/mXT641T_Production_V1.0.AA_.h"
#else
#include "./firmware/mxt641t/mXT641THIC_0x2F_3.0.AC_PROD_.h"
#endif
};

//ymlee_test
unsigned char mxts_fw2[] = {
#include "./firmware/mxt641t/mXT641THIC_0x2F_3.3.E0_PROD_.h"
};

#define MAX(a,b) (a > b ? a : b)
#define MIN(a,b) (a < b ? a : b)

/* For Calibration */
#define	TSP_PATCH	0 //1

/* For WIFI Debug */
#define TSP_ITDEV	1

/* KEY */
#ifdef MXT_CFG_EF71
#define PAN_HAVE_TOUCH_KEY		0
#define PAN_T15_KEYARRAY_ENABLE	0
#else
#define PAN_HAVE_TOUCH_KEY		1
#define PAN_T15_KEYARRAY_ENABLE	0
#endif

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7

#if MXT_540S
#define MXT_MAX_CHANNEL_NUM		540
#endif

#if MXT_641T
#define MXT_MAX_CHANNEL_NUM		640
#ifdef MXT_CFG_EF71
#define MXT_USED_CHANNEL_NUM	 (26*15) //(21*12)
#else
#define MXT_USED_CHANNEL_NUM	(32*17)
#endif
#define MXT_MAX_SELFCAP_DATA	(64*3)
#endif

#define MXT_CHANNELS_PER_PAGE	64 // 128bytes/page, 2bytes/ch
#define MXT_DELTA_MODE			0x10
#define MXT_REFERENCE_MODE		0x11
#define MXT_SELF_DELTA_MODE			0xF7
#define MXT_SELF_REFERENCE_MODE		0xF8
#define MXT_MAX_BLOCK_WRITE			256
#define MXT_WAKEUP_TIME		        25	/* msec */

#define PAN_TOUCH_KEY_REJECTION_ON_DISPLAY	0

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	u8(*read_chg) (void);
};

typedef enum
{
	TSC_EVENT_NONE,
	TSC_EVENT_WINDOW,
#if PAN_HAVE_TOUCH_KEY
	TSC_EVENT_1ST_KEY,
	TSC_EVENT_2ND_KEY,
	TSC_EVENT_3RD_KEY,
#endif
} tsc_key_mode_type;

typedef enum
{
	TOUCH_EVENT_INACTIVE = -1,
	TOUCH_EVENT_RELEASE = 0,
	TOUCH_EVENT_PRESS,
	TOUCH_EVENT_MOVE,
	TOUCH_EVENT_NOTHING,
} TOUCH_EVENT;

#if PAN_HAVE_TOUCH_KEY
#define PAN_TOUCH_KEY_COUNT		3

struct pan_touch_key_config {
	struct {
		int center;
		int margin;
		int key;
	}x[PAN_TOUCH_KEY_COUNT];

	struct {
		int center;
		int margin;		
	}y;	
};
#endif

#if PAN_T15_KEYARRAY_ENABLE
#define PAN_1ST_TOUCH_KEY_TYPE			KEY_MENU
#define PAN_2ND_TOUCH_KEY_TYPE			KEY_BACK
struct pan_touch_key_config {
  	bool  key_state;
	int   key_num;		
};
struct pan_touch_key_config mPan_KeyArray[2];

#endif


#define MAX_NUM_FINGER	10
/* Input report structure */
typedef struct
{
	uint8_t id;                     /*!< id */
	int8_t status;          	/*!< dn=1, up=0, none=-1 */
	int8_t type;
	int16_t x;                      /*!< X */
	int16_t y;                      /*!< Y */

	int8_t mode;
	int area;
} report_finger_info_t;

report_finger_info_t fingerInfo[MAX_NUM_FINGER];

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_RESERVED	4
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

enum mxt_device_state { INIT, APPMODE, BOOTLOADER, FAILED, SHUTDOWN, SUSPEND, WAKEUP_SUSPEND};

#if TSP_PATCH
static u8 patch_bin[] = {
#ifdef MXT_CFG_EF71
#include "./firmware/mxt641t/EF78/patch.h"
#else
#include "./firmware/mxt641t/patch.h"
#endif
};

struct mxt_patch {
	u8 *patch;
	u16 *stage_addr;
	u16 *tline_addr;
	u16 *trigger_addr;
	u16 *event_addr;
	u16 *src_item;
	u16 *check_cnt;
	u16 period;
	u8 stage_cnt;
	u8 tline_cnt;
	u8 trigger_cnt;
	u8 event_cnt;
	u8 option;
	u8 debug;
	u8 timer_id;
	u8 cur_stage;
	u8 cur_stage_opt;
	u8 run_stage;
	u8 start;
	u8 finger_cnt;
	u8 start_stage;
	u8 skip_test;
	u8 cal_flag; 
	u32 date;
	u32 stage_timestamp;
};

struct mxt_reportid {
	u8 type;
	u8 index;
};
#endif //TSP_PATCH

struct mxt_message {
    u8 reportid;
    u8 message[8];
};


/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	const struct mxt_platform_data *pdata;
	struct mutex    lock;
	enum mxt_device_state state;
	struct mxt_object *object_table;
	u16 mem_size;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	struct t7_config t7_cfg;
	u8 *msg_buf;
	bool t9_update_input;
	bool t100_update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 num_stylusids;
	bool flag_probe;
	int cnt_mxt_input_open_func;
	int cnt_mxt_input_close_func;
	int cnt_mxt_shutdown_func;
#if TSP_PATCH
	struct mxt_patch patch;
	struct mxt_reportid *reportids;
	bool charging_mode;
#endif
	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T37_address;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T48_reportid;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u8 T66_reportid;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	bool pre_debug_enabled;
	bool move_dbg;
	bool no_cfg_update;
	u8 batt_mode;
};

struct mxt_data *data_common =  NULL;
#define OBP_INSTANCES(o) ((u16)((o)->instances_minus_one) + 1)
#define OBP_SIZE(o) ((u16)((o)->size_minus_one) + 1)


#define MXT_OBJECT_START				0x07
#define MXT_OBJECT_SIZE					6
#define MXT_INFO_CHECKSUM_SIZE			3
#define MXT_MAX_BLOCK_WRITE				256

/* Object types */
#define MXT_GEN_MESSAGE_T5				5
#define MXT_GEN_COMMAND_T6				6
#define MXT_GEN_POWER_T7				7
#define MXT_GEN_ACQUIRE_T8				8
#define MXT_TOUCH_MULTI_T9				9
#define MXT_TOUCH_KEYARRAY_T15			15
#define MXT_SPT_COMMSCONFIG_T18			18
#define MXT_SPT_GPIOPWM_T19				19
#define MXT_PROCI_GRIPFACE_T20			20
#define MXT_PROCG_NOISE_T22				22
#define MXT_TOUCH_PROXIMITY_T23			23
#define MXT_PROCI_ONETOUCH_T24			24
#define MXT_SPT_SELFTEST_T25			25
#define MXT_PROCI_TWOTOUCH_T27			27
#define MXT_SPT_CTECONFIG_T28			28
#define MXT_DEBUG_DIAGNOSTIC_T37		37
#define MXT_SPT_USERDATA_T38			38
#define MXT_PROCI_GRIP_T40				40
#define MXT_PROCI_PALM_T41				41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_SPT_DIGITIZER_T43			43
#define MXT_SPT_MESSAGECOUNT_T44		44
#define MXT_SPT_CTECONFIG_T46			46
#define MXT_PROCI_STYLUS_T47			47
#define MXT_SPT_NOISESUPPRESSION_T48	48
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_TOUCH_PROXKEY_T52			52
#define MXT_GEN_DATASOURCE_T53			53
#define MXT_PROCI_ADAPTIVETHRESHOLD_T55	55
#define MXT_PROCI_SHIELDLESS_T56		56
#define MXT_PROCI_EXTRATOUCHSCREENDATA_T57	57
#define MXT_SPT_TIMER_T61				61
#define MXT_PROCI_ACTIVE_STYLUS_T63		63
#define MXT_PROCI_LENSBENDING_T65		65
#define MXT_SPT_GOLDENREFERENCES_T66	66
#define MXT_PROCI_PALMGESTURE_T69		69
#define MXT_SPT_DYNAMICCFGCONTROLLER_T70	70
#define MXT_SPT_DYNAMICCFGCONTAINER_T71	71
#define MXT_PROCG_NOISESUPPRESSION_T72	72
#define MXT_PROCI_GLOVEDETECTION_T78	78
#define MXT_PROCI_RETRANSMISSION_T80	80
#define MXT_PROCI_GESTURE_T84			84
#define MXT_PROCI_SYMBOLGESTUREPROCESSOR_T92	92
#define MXT_PROCI_GESTUREPROCESSOR_T93	93
#define MXT_TOUCH_MULTI_T100			100
#define MXT_SPT_TOUCHSCREENHOVER_T101	101
#define MXT_SPT_SELFCAPCBCRCONFIG_T102	102
#define MXT_PROCI_SCHNOISESUPPRESSION_T103	103
#define MXT_SPT_AUXTOUCHCONFIG_T104		104
#define MXT_SPT_DRIVENPLATEHOVERCONFIG_T105	105
#define MXT_PROCG_NOISESUPSELFCAP_T108	108
#define MXT_SPT_SELFCAPGLOBAL_T109		109
#define MXT_SPT_SELFCAPTUNINGPARAMS_T110	110
#define MXT_SPT_SELFCAPCONFIG_T111		111
#define MXT_PROCI_HOVERGRIPSUPPRESSION_T112	112
#define MXT_SPT_PROXMEASURECONFIG_T113	113

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

struct t9_range {
	u16 x;
	u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* MXT_TOUCH_MULTI_T100 field */
#define MXT_T100_XORIGIN		8
#define MXT_T100_XRANGE		13
#define MXT_T100_YORIGIN		19
#define MXT_T100_YRANGE		24

typedef enum{
	MXT_T100_TYPE_RESERVED,
	MXT_T100_TYPE_FINGER = 1,
	MXT_T100_TYPE_PASSIVE_STYLUS = 2,
	MXT_T100_TYPE_HOVERING_FINGER = 4,
	MXT_T100_TYPE_GLOVE = 5,
}mxt_touch_type;

typedef enum{
	MXT_T100_EVENT_NONE,
	MXT_T100_EVENT_MOVE,
	MXT_T100_EVENT_UNSUPPRESS,
	MXT_T100_EVENT_SUPPRESS,
	MXT_T100_EVENT_DOWN,
	MXT_T100_EVENT_UP,
	MXT_T100_EVENT_UNSUPSUP,
	MXT_T100_EVENT_UNSUPUP,
	MXT_T100_EVENT_DOWNSUP,
	MXT_T100_EVENT_DOWNUP
}mxt_event_type;

//#define MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID	24
//#define MXT_T100_SCREEN_MSG_FIRST_RPT_ID	22

/* mxt641T */
#define MXT_T100_SCREEN_MESSAGE_NUM_RPT_ID	2
#define MXT_T100_SCREEN_MSG_FIRST_RPT_ID	0

/* T15 Key array */
int mxt_t15_keys[] = { };

static unsigned long mxt_t15_keystatus;

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* T63 Stylus */
#define MXT_STYLUS_PRESS	(1 << 0)
#define MXT_STYLUS_RELEASE	(1 << 1)
#define MXT_STYLUS_MOVE		(1 << 2)
#define MXT_STYLUS_SUPPRESS	(1 << 3)

#define MXT_STYLUS_DETECT	(1 << 4)
#define MXT_STYLUS_TIP		(1 << 5)
#define MXT_STYLUS_ERASER	(1 << 6)
#define MXT_STYLUS_BARREL	(1 << 7)

#define MXT_STYLUS_PRESSURE_MASK	0x3F

/* Delay times */
#define MXT_BACKUP_TIME		25	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIME_RESUME		100	/* msec */
#define MXT_RESET_NOCHGREAD	400	/* msec */
#define MXT_FWRESET_TIME	1000	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

/*! \brief Returned by get_object_address() if object is not found. */
#define OBJECT_NOT_FOUND   				      0u

#if MXT_CFG_WRITE_BIN
#define CFG_DATA_SIZE	1898 //p11774 according to applying 1.0.AA firmware. //1855

struct mxt_cfg_info {
	u8 fw_ver;
	u8 build_ver;
	u32 cfg_crc;
	u32 cfg_len;
	const u8 *cfg_raw_data;
	struct mxt_data *data;
};

struct mxt_cfg_image {
	u8 family_id;
	u8 variant;
	u8 fw_ver;
	u8 build_ver;
	u8 reserved[3];
	__le32 cfg_crc;
	u8 data[0];
} __packed;

struct mxt_cfg_data {
	u8 type;
	u8 instance;
	u8 size;
	u8 register_val[0];
} __packed;
#endif

#if !defined(MXT_CFG_WRITE_BIN)
#define TOUCH_MODE_NORMAL         0
#define TOUCH_MODE_HIGH_SENSITIVE 1
#define TOUCH_MODE_PEN_LETTER     2
#define TOUCH_MODE_PEN_DRAW       3 
#define TOUCH_MODE_MAX_NUM	      4 
static int mTouch_mode=TOUCH_MODE_NORMAL;
#else
typedef enum{
	TOUCH_MODE_NONE = -1,
	TOUCH_MODE_NORMAL = 0,
	TOUCH_MODE_HIGH_SENSITIVE = 1,
	TOUCH_MODE_PEN_LETTER = 2,
	TOUCH_MODE_PEN_DRAW = 3,
	TOUCH_MODE_WAKEUP_SUSPEND = 4,
	TOUCH_MODE_CHARGER = 5,
	TOUCH_MODE_MAX_NUM = 6,
}touch_mode_type;
touch_mode_type mTouch_mode = TOUCH_MODE_NORMAL;
touch_mode_type preTouchMode = TOUCH_MODE_NONE;

typedef enum{
	BATTERY_PLUGGED_NONE = 0,
	BATTERY_PLUGGED_AC = 1,
	BATTERY_PLUGGED_USB = 2,
	BATTERY_PLUGGED_WIRELESS = 4,
}battery_mode_type;

#endif
static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep);
static int  mxt_suspend(struct mxt_data *data);
static int  mxt_resume(struct mxt_data *data);

static int* diag_debug(int command); 
static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset, u8 value, bool wait);
static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val);
static void mxt_start(struct mxt_data *data);
static int mxt_backup(struct mxt_data *data);
static int mxt_calibrate(struct mxt_data *data);
static int mxt_soft_reset(struct mxt_data *data, u8 value);
static ssize_t mxt_update_fw(struct device *dev);
#if MXT_CFG_WRITE_BIN
int mxt_update_cfg(struct mxt_data* data, touch_mode_type touchMode);
int mxt_update_cfg_no_backup(struct mxt_data* data, touch_mode_type touchMode);
#endif
static int mxt_write_mem(struct i2c_client *client, u16 reg, u16 len, u8* buf);
static struct mxt_object *mxt_get_object(struct mxt_data *data, u8 type);

#if TSP_PATCH
static int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, u8 val);
static int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *val);
static int mxt_read_mem(struct i2c_client *client, u16 reg, u16 len, u8* buf);
static void mxt_make_reportid_table(struct mxt_data *data);
static int mxt_patch_init(struct mxt_data *data, u8* ppatch);
#endif
void  mxt_front_test_init(void);
//20140703_for_selftest +
#if TSP_PATCH //
static int mxt_get_t8_self_cfg(struct mxt_data *data);
static int mxt_set_t8_self_cfg(struct mxt_data *data, int onoff);
#endif 
//20140703_for_selftest -

#endif /* __LINUX_ATMEL_MXT_TS_H */


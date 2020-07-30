/*
 * LP5523 LED Driver
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

#ifndef __LINUX_LP5523_H
#define __LINUX_LP5523_H

/* See Documentation/leds/leds-lp5523.txt */
#define PAN_LED_SYSFS_DEBUG
/////////////////////////
#if defined(CONFIG_MACH_MSM8974_EF59S) || defined(CONFIG_MACH_MSM8974_EF59K) || defined(CONFIG_MACH_MSM8974_EF59L) || defined(CONFIG_MACH_MSM8974_EF60S) || defined(CONFIG_MACH_MSM8974_EF61K) || defined(CONFIG_MACH_MSM8974_EF62L) || defined(CONFIG_MACH_MSM8937_EF71IE) || defined(CONFIG_MACH_MSM8937_EF71S) || defined(CONFIG_MACH_MSM8937_EF71K)
#define OFFLINE_CHARGER_LED_CONTROL
#endif

#if	defined(CONFIG_MACH_MSM8974_EF59S) || defined(CONFIG_MACH_MSM8974_EF59K) || defined(CONFIG_MACH_MSM8974_EF59L)
#define PAN_EF59_MENU_BACK_LED_BRIGHTNESS   80
#define OFFLINE_CHARGING_GREEN_CURRENT_2MA
#define LED_OFF_CHECK_TIMER_USAGE  // 2013.12.4 p13106 for last led dimming event. 
#define LED_DIMMING_IN_CHARGING    // 2013.12.4 p13106 red dimming in charging.

#elif defined(CONFIG_MACH_MSM8974_EF60S) || defined(CONFIG_MACH_MSM8974_EF61K) || defined(CONFIG_MACH_MSM8974_EF62L) || defined(CONFIG_MACH_MSM8937_EF71IE) || defined(CONFIG_MACH_MSM8937_EF71S) || defined(CONFIG_MACH_MSM8937_EF71K)
#define PAN_EF60_MENU_BACK_LED_BRIGHTNESS   60
#define OFFLINE_CHARGING_GREEN_CURRENT_2MA 
#define LED_OFF_CHECK_TIMER_USAGE  // 2013.12.4 p13106 for last led dimming event. 
#define EF60_BOOT_RAINBOW_LED      // 2013.12.4 p13106 boot rainbow led.
#define LED_DIMMING_IN_CHARGING    // 2013.12.4 p13106 red dimming in charging.
#endif

/* LP5523 REGISTRATION */
#define LP5523_REG_ENABLE			0x00
#define LP5523_REG_OP_MODE			0x01
#define LP5523_REG_RATIOMETRIC_MSB	0x02
#define LP5523_REG_RATIOMETRIC_LSB	0x03
#define LP5523_REG_ENABLE_LEDS_MSB	0x04
#define LP5523_REG_ENABLE_LEDS_LSB	0x05
#define LP5523_REG_LED_CNTRL_BASE	0x06
#define LP5523_REG_LED_PWM_BASE		0x16
#define LP5523_REG_LED_CURRENT_BASE	0x26
#define LP5523_REG_CONFIG			0x36
#define LP5523_REG_CHANNEL1_PC		0x37
#define LP5523_REG_CHANNEL2_PC		0x38
#define LP5523_REG_CHANNEL3_PC		0x39
#define LP5523_REG_STATUS			0x3a
#define LP5523_REG_GPO				0x3b
#define LP5523_REG_VARIABLE			0x3c
#define LP5523_REG_RESET			0x3d
#define LP5523_REG_TEMP_CTRL		0x3e
#define LP5523_REG_TEMP_READ		0x3f
#define LP5523_REG_TEMP_WRITE		0x40
#define LP5523_REG_LED_TEST_CTRL	0x41
#define LP5523_REG_LED_TEST_ADC		0x42
#define LP5523_REG_ENG1_VARIABLE	0x45
#define LP5523_REG_ENG2_VARIABLE	0x46
#define LP5523_REG_ENG3_VARIABLE	0x47
#define LP5523_REG_MASTER_FADER1	0x48
#define LP5523_REG_MASTER_FADER2	0x49
#define LP5523_REG_MASTER_FADER3	0x4a

#define LP5523_REG_CH1_PROG_START	0x4c
#define LP5523_REG_CH2_PROG_START	0x4d
#define LP5523_REG_CH3_PROG_START	0x4e
#define LP5523_REG_PROG_PAGE_SEL	0x4f
#define LP5523_REG_PROG_MEM			0x50

#define LP5523_ENG1_MAPPING_MSB	0x70
#define LP5523_ENG1_MAPPING_LSB	0x71
#define LP5523_ENG2_MAPPING_MSB	0x72
#define LP5523_ENG2_MAPPING_LSB	0x73
#define LP5523_ENG3_MAPPING_MSB	0x74
#define LP5523_ENG3_MAPPING_LSB	0x75
#define LP5523_GAIN_CHANGE_CONTROL 0x76




#define LP5523_CMD_LOAD				  0x15 /* 00010101 */
#define LP5523_CMD_RUN				  0x2a /* 00101010 */
#define LP5523_CMD_RUN_ENGINE1	0x20 /* 00101010 */
#define LP5523_CMD_RUN_ENGINE2  0x08
#define LP5523_CMD_RUN_ENGINE3  0x02


#define LP5523_CMD_DISABLED			0x00 /* 00000000 */

#define LP5523_ENABLE				0x40
#define LP5523_AUTO_INC				0x40
#define LP5523_PWR_SAVE				0x20
#define LP5523_PWM_PWR_SAVE			0x04
#define LP5523_CP_1					0x08
#define LP5523_CP_1_5				0x10
#define LP5523_CP_AUTO				0x18
#define LP5523_INT_CLK				0x01
#define LP5523_AUTO_CLK				0x02
#define LP5523_EN_LEDTEST			0x80
#define LP5523_LEDTEST_DONE			0x80

#define LP5523_DEFAULT_CURRENT		50 /* microAmps */
#define LP5523_PROGRAM_LENGTH		32 /* in bytes */
#define LP5523_PROGRAM_PAGES		6
#define LP5523_ADC_SHORTCIRC_LIM	80

#define LP5523_LEDS					9
#define LP5523_ENGINES				3

#define LP5523_ENG_MASK_BASE		0x30 /* 00110000 */

#define LP5523_ENG_STATUS_MASK      0x07 /* 00000111 */

#define LP5523_IRQ_FLAGS            IRQF_TRIGGER_FALLING

#define LP5523_EXT_CLK_USED			0x08

#define LED_ACTIVE(mux, led)		(!!(mux & (0x0001 << led)))
#define SHIFT_MASK(id)				(((id) - 1) * 2)

#define MAX_BRIGHTNESS        255
#define MIN_BRIGHTNESS        0
#define PAN_NUM_LEDS          8

//++ p11309 - 2013.05.06 for LED Trigger
#define TRG_TOUCHKEY_BACKLIGHT		"touchkey-backlight"
#define TRG_BATTERY_CHARGING		"battery-charging"
#define TRG_BATTERY_FULL			"battery-full"
//-- p11309
#define EF59_LED1_GREEN     0
#define EF59_LED1_BLUE      1
#define EF59_LED2_GREEN     2
#define EF59_LED2_BLUE      3
#define EF59_MENU_KEY       4
#define EF59_BACK_KEY       5
#define EF59_LED1_RED       6
#define EF59_LED2_RED       7

#define MENU_BACK_KEY_HEX   0x30
#define DIMMING_ON_TIME     0x04
#define DIMMING_OFF_TIME    0x05


struct lp5523_led_config {
	u8		chan_nr;
	u8		led_current; /* mA x10, 0 if led is not connected */
	u8		max_current;
	//++ p11309 - 0013.05.06 for LED trigger;
	const char *default_trigger;
	//-- p11309
};

#define LP5523_CLOCK_AUTO	0
#define LP5523_CLOCK_INT	1
#define LP5523_CLOCK_EXT	2

struct lp5523_platform_data {
	struct lp5523_led_config *led_config;

	//++ p11309 - 2013.04.25 for led enable pin control
	unsigned	led_en_gpio;
	//-- p11309

	u8	num_channels;
	u8	clock_mode;
	const	char *label;
};
struct lp5523_engine {
	int		id;
	u8		mode;
	u8		prog_page;
	u8		mux_page;
	u16		led_mux;
	u8		engine_mask;
};

struct lp5523_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	struct led_classdev     cdev;
	struct work_struct	brightness_work;
	u8			brightness;
#ifdef LED_OFF_CHECK_TIMER_USAGE	
//++ p11309 - 2013.12.17 for charger LED Off Error
	u8			charger_off;
//-- p11309
#endif
	u8			before_brightness;	
};

struct lp5523_chip {
	struct mutex		lock; /* Serialize control */
	struct i2c_client	*client;
	struct lp5523_engine	engines[LP5523_ENGINES];
	struct lp5523_led	leds[LP5523_LEDS];
	struct lp5523_platform_data *pdata;
	u8			num_channels;
	u8			num_leds;
	unsigned long     home_key_led_current_state;        // p13106 for Dimming
	unsigned long     home_key_led_before_state;
	unsigned long     menu_back_led_state;
};

typedef enum {	
	PAN_LED_LP5523_SET_DEFAULT_CURRENT = 100,
	PAN_LED_LP5523_SET_MAX_CURRENT = 101,
	PAN_LED_LP5523_LED_ALL_OFF = 200,
	PAN_LED_LP5523_LED_FRONT_RED,
	PAN_LED_LP5523_LED_FRONT_GREEN,
	PAN_LED_LP5523_LED_FRONT_BLUE,
	PAN_LED_LP5523_LED_REAR_RED,
	PAN_LED_LP5523_LED_REAR_GREEN,
	PAN_LED_LP5523_LED_REAR_BLUE,
	PAN_LED_LP5523_LED_MENU_BACK_LED_ON,
}PAN_LED_LP5523_IOCTL_CMD;

typedef enum {	
	EF59_LED_OFF = 0,
	PWM_TK_ON_OFF,
	DIMMING_TK_ON_OFF,
	PWM_HOMEKEY_ON_OFF,
	DIMMING_HOMEKEY_ON_OFF,
	DIMMING_ON_COLOR_10S,
	DIMMING_ON_COLOR_20S,
	DIMMING_ON_COLOR_30S,
	DIMMING_ON_OFF_COLOR,
	DIMMING_OFF_CHECK,			// 2013.12.4 p13106 for last led dimming event.
	DIMMING_ON_COLOR_TIMER, // EF59 KK for LED dimming control.
	DIMMING_ON_OFF_SHORT_TIMER,
	EF59_LED_ALL_OFF = 100,
	EF59_LED_OFFLINE_LED_CHECK,
	EF59_LED_SET_GREEN_LIGHT_VALUE_20,
	EF59_LED_SET_GREEN_LIGHT_VALUE_40,
	DIMMING_RED_CHARGING = 200, // 2013.12.4 p13106 red dimming in charging.
}PAN_LED_LP5523_BLINK_CMD;




#endif /* __LINUX_LP5523_H */

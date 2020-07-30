#ifndef __PANTECH_SYS_H
#define __PANTECH_SYS_H

#include <linux/io.h>

/*******************************************************************************
**  Must have equal vendor/pantech/system/core/include/pantech_sys.h
*******************************************************************************/

/* !! WARNING !!*/
/* you do not use below value directly */
/* you must use public macro for operation to pantech imem */

extern unsigned int *pantech_sys_imem_addr;

/*==== Pantech IMEM address ====*/

/* Qcom shred imem virtual address */
#define PANTECH_RESET_REASON_ADDR       ((volatile void *)(PANTECH_RESET_MAGIC_ADDR + 0x4))
#define PANTECH_RESET_MAGIC_ADDR          ((volatile void *)(PANTECH_TEMPORAL_MODE_ADDR + 0x4))
#define PANTECH_TEMPORAL_MODE_ADDR     ((volatile void *)PANTECH_SYS_IMEM_START_ADDR)
#define PANTECH_SYS_IMEM_START_ADDR     ((volatile void *)pantech_sys_imem_addr)

/*==== Reset Magic====*/

#define SYS_RESET_MAGIC     0xDEADEDED

/*==== Reset Reason ====*/

/* Linux Group */

#define SYS_RESET_REASON_LINUX_MASK    0x00000010
#define SYS_RESET_REASON_LINUX              0x00000011

/* FS Group */
#define SYS_RESET_REASON_USERDATA_FS               0x00000012
#define SYS_RESET_REASON_DATA_MOUNT_ERR            0x00000013
#ifdef CONFIG_PANTECH_DXHDCP_PROVISIONING //lsi@bs1 : FEATURE_PANTECH_DXHDCP_PROVISIONING secure partition
#define SYS_RESET_REASON_SECURE_FS                  0x00000014
#endif
/* Watchdog Group */
#define SYS_RESET_REASON_WATCHDOG_MASK                   0x00000020
#define SYS_RESET_REASON_WATCHDOG_NSEC_BITE           0x00000021
#define SYS_RESET_REASON_WATCHDOG_SEC_BITE             0x00000022
#define SYS_RESET_REASON_WATCHDOG_NSEC_SEC_BITE    0x00000023

/* Abnormal Group */

#define SYS_RESET_REASON_ABNORMAL_MASK    0x00000030
#define SYS_RESET_REASON_ABNORMAL              0x00000031

/* MDM Group */

#define SYS_RESET_REASON_MDM_MASK   0x00000040
#define SYS_RESET_REASON_MDM             0x00000041

/* LPASS Group */

#define SYS_RESET_REASON_LPASS_MASK    0x00000050
#define SYS_RESET_REASON_LPASS              0x00000051

/* ADSP Group */

#define SYS_RESET_REASON_ADSPS_MASK    0x00000060
#define SYS_RESET_REASON_ADSPS              0x00000061

/* WIFI Group */

#define SYS_RESET_REASON_WIFI_MASK    0x00000070
#define SYS_RESET_REASON_WIFI              0x00000071

/*RPM Group*/

#define SYS_RESET_REASON_RPM_MASK          0x00000080
#define SYS_RESET_REASON_RPM_DOGBARK    0x00000081
#define SYS_RESET_REASON_RPM_ERRFATAL    0x00000082

/* Venus Group */

#define SYS_RESET_REASON_VENUS_MASK    0x00000090
#define SYS_RESET_REASON_VENUS              0x00000091

/* Modem Group */

#define SYS_RESET_REASON_MODEM_MASK    0x000000A0
#define SYS_RESET_REASON_MODEM              0x000000A1

/* Normal Group */

#define SYS_RESET_REASON_NORMAL_MASK    0x000000F0
#define SYS_RESET_REASON_NORMAL              0x000000F1

/* Reserved Group */

/* 1011xxxx */
/* 1100xxxx */
/* 1101xxxx */
/* 1111xxxx */

/*==== Subsystem Restart Flag ====*/

#define SYS_SSR_MDM_FLAG              (0x1 << 8)
#define SYS_SSR_MDM_DUMP_FLAG    (0x1 << 9)
#define SYS_SSR_WIFI_FLAG              (0x1 << 10)
#define SYS_SSR_WIFI_DUMP_FLAG    (0x1 << 11)
#define SYS_SSR_NOTI_FLAG              (0x1 << 12)

/* Reserved Flag Bit */

/* [15:13] bit */
#define SYS_RESET_BACKLIGHT_OFF_FLAG               (0x1 << 15)

/*==== Configuration Bit ====*/
#define SYS_BACKLIGHT_ONOFF_FLAG               (0x1 << 16)
#define SYS_MDM_DUMP_ONOFF_FLAG               (0x1 << 17)
#define SYS_ERROR_RESET_OCCURED_FLAG      (0x1 << 18)

/* Reserved Configuration Bit */

/* [19] bit */

/*==== Dump Mode ====*/
/* use [31:20] bit */

#define SYS_RESET_USBDUMP_MASK    						0xA0000000 //28 ~ 31
#define SYS_RESET_RAMDUMP_MASK    						0x0B000000 //24 ~ 27
#define SYS_RESET_USERDUMP_DISABLE_MASK		0x00100000 //20 ~ 23 //p11014_wks
#define SYS_TEMPORAL_DUMP_MASK    0xCEDABAEE

/*==== Public Macro ====*/

/* determine function group */

#define IS_SYS_RESET_MAGIC \
    (PANTECH_SYS_IMEM_START_ADDR && (__raw_readl(PANTECH_RESET_MAGIC_ADDR) == SYS_RESET_MAGIC))

#define IS_SYS_USBDUMP_MODE \
    (IS_SYS_RESET_MAGIC && ((__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(0x0FFFFFFF)) == SYS_RESET_USBDUMP_MASK))

#define IS_SYS_RAMDUMP_MODE \
    (IS_SYS_RESET_MAGIC && ((__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(0xF0FFFFFF)) == SYS_RESET_RAMDUMP_MASK))

/*p11014_wks   added.... */
#define GET_SYS_USERDUMP_MODE \
	((__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(0xFF0FFFFF))>>20)

#define GET_SYS_VALUE \
	(__raw_readl(PANTECH_RESET_REASON_ADDR))
	
#define IS_SYS_ERROR_RESET \
    (IS_SYS_RESET_MAGIC && ((__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(0xFFFFFF0F)) != SYS_RESET_REASON_NORMAL_MASK))

#define IS_SYS_TEMPORAL_DUMP_MODE \
    (IS_SYS_RESET_MAGIC && (__raw_readl(PANTECH_TEMPORAL_MODE_ADDR) == SYS_TEMPORAL_DUMP_MASK))

/* set function group */

#define SET_SYS_RESET_MAGIC(x) \
    PANTECH_SYS_IMEM_START_ADDR ? __raw_writel(x, PANTECH_RESET_MAGIC_ADDR) : 0x0

#define SET_SYS_RESET_REASON(x) \
    PANTECH_SYS_IMEM_START_ADDR ? __raw_writel(((__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(0x000000FF)) | (x)), PANTECH_RESET_REASON_ADDR) : 0x0

#define SET_SYS_RESET_DUMP_MODE(mode, on) \
do { \
    if(PANTECH_SYS_IMEM_START_ADDR) { \
        if(on) { \
            __raw_writel(((__raw_readl(PANTECH_RESET_REASON_ADDR)) | (mode)), PANTECH_RESET_REASON_ADDR); \
        }else { \
            __raw_writel((__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(mode)), PANTECH_RESET_REASON_ADDR); \
        } \
    } \
} while(0)

/*p11014_wks   added.... */
#define SET_SYS_RESET_USERDUMP_MODE(mode) \
do { \
    if(PANTECH_SYS_IMEM_START_ADDR) { \
        __raw_writel(((__raw_readl(PANTECH_RESET_REASON_ADDR) & 0xFF0FFFFF) | ((mode)<<20)), PANTECH_RESET_REASON_ADDR); \
    } \
} while(0)

#define SET_SYS_RESET_IMEM_FLAG(flag, on) \
do { \
    if(PANTECH_SYS_IMEM_START_ADDR) { \
        if(on) { \
            __raw_writel((__raw_readl(PANTECH_RESET_REASON_ADDR) | (flag)), PANTECH_RESET_REASON_ADDR); \
        } else { \
            __raw_writel((__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(flag)), PANTECH_RESET_REASON_ADDR); \
        } \
    } \
} while(0)

#define SET_SYS_TEMP_DUMP_MODE(x) \
    PANTECH_SYS_IMEM_START_ADDR ? __raw_writel(x, PANTECH_TEMPORAL_MODE_ADDR) : 0x0

/* get function group */

#define GET_SYS_RESET_MAGIC \
    PANTECH_SYS_IMEM_START_ADDR? __raw_readl(PANTECH_RESET_MAGIC_ADDR) : 0x0

#define GET_SYS_RESET_REASON_GROUP \
    IS_SYS_RESET_MAGIC? (__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(0xFFFFFF0F)) : 0x0

#define GET_SYS_RESET_REASON_ERROR \
    IS_SYS_RESET_MAGIC? (__raw_readl(PANTECH_RESET_REASON_ADDR) & ~(0xFFFFFF00)) : 0x0

#define GET_SYS_RESET_REASON_ALL \
    IS_SYS_RESET_MAGIC? __raw_readl(PANTECH_RESET_REASON_ADDR) : 0x0

#define GET_SYS_RESET_IMEM_FLAG(x) \
    IS_SYS_RESET_MAGIC? ((__raw_readl(PANTECH_RESET_REASON_ADDR) & (x)) == (x)) : 0x0

/* clear function group */

#define CLEAR_SYS_RESET_REASON_AREA(x) \
    PANTECH_SYS_IMEM_START_ADDR ? __raw_writel((x), PANTECH_RESET_REASON_ADDR)

/* ioctl command (system -> kernel request) */
typedef enum {
    GET_SYS_ERROR_RESET_OCCURED_CMD = 0x1001,
} PANTECH_RESET_INFO_CMD;

/*******************************************************************************
**  Reset Reason API (Kernel Only)
*******************************************************************************/
extern void pantech_sys_imem_init(void);
extern void pantech_sys_reset_reason_set(unsigned int reason);

extern unsigned int pantech_sys_reset_backlight_flag_get(void);
extern void pantech_sys_reset_backlight_flag_set(unsigned int flag);
extern unsigned int pantech_sys_rst_is_silent_boot_mode(void);
#endif



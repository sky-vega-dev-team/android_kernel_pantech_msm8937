#if defined(FEATURE_PANTECH_ERR_CRASH_LOGGING) || defined(CONFIG_PANTECH_ERR_CRASH_LOGGING)

// ===============================================================================
// Header file include
// ===============================================================================

// ===============================================================================
// Define
// ===============================================================================

#define SSR_SET_MODEM_ENABLE(b)                 (b->ssr_set_modem_enable = (unsigned int)1)
#define SSR_SET_MODEM_DISABLE(b)                (b->ssr_set_modem_enable = (unsigned int)0)
#define SSR_SET_MODEM_DUMP_ENABLE(b)            (b->ssr_set_modem_dump_enable = (unsigned int)1)
#define SSR_SET_MODEM_DUMP_DISABLE(b)           (b->ssr_set_modem_dump_enable = (unsigned int)0)
#define SSR_SET_WIFI_ENABLE(b)                  (b->ssr_set_wifi_enable = (unsigned int)1)
#define SSR_SET_WIFI_DISABLE(b)                 (b->ssr_set_wifi_enable = (unsigned int)0)
#define SSR_SET_WIFI_DUMP_ENABLE(b)             (b->ssr_set_wifi_dump_enable = (unsigned int)1)
#define SSR_SET_WIFI_DUMP_DISABLE(b)            (b->ssr_set_wifi_dump_enable = (unsigned int)0)
#define SSR_SET_NOTIFICATION_ENABLE(b)          (b->ssr_set_notification_enable = (unsigned int)1)
#define SSR_SET_NOTIFICATION_DISABLE(b)         (b->ssr_set_notification_enable = (unsigned int)0)

#define SSR_GET_MODEM_CHECK(b)                  ((unsigned int)1 == b->ssr_set_modem_enable)
#define SSR_GET_MODEM_DUMP_CHECK(b)             ((unsigned int)1 == b->ssr_set_modem_dump_enable)
#define SSR_GET_WIFI_CHECK(b)                   ((unsigned int)1 == b->ssr_set_wifi_enable)
#define SSR_GET_WIFI_DUMP_CHECK(b)              ((unsigned int)1 == b->ssr_set_wifi_dump_enable)
#define SSR_GET_NOTIFICATION_CHECK(b)           ((unsigned int)1 == b->ssr_set_notification_enable)

#define SSR_CNT_LPASS_INCREASE(b)               (b->ssr_cnt_lpass++)
#define SSR_CNT_DSPS_INCREASE(b)                (b->ssr_cnt_dsps++)
//#define SSR_CNT_WIFI_INCREASE(b)                (b->ssr_cnt_wifi++)
#define SSR_CNT_WIFI_INCREASE(b)                (b->p_ssr_reason.ssr_reason_wifi++)
#define SSR_CNT_VENUS_INCREASE(b)               (b->ssr_cnt_venus++)
//#define SSR_CNT_MODEM_INCREASE(b)               (b->ssr_cnt_modem++)
#define SSR_CNT_MODEM_INCREASE(b)               (b->p_ssr_reason.ssr_reason_modem++)
#define SSR_CNT_MDM_INCREASE(b)                 (b->ssr_cnt_mdm++)

#define SSR_CNT_INIT(b)                         (b->ssr_cnt_lpass=b->ssr_cnt_dsps=\
                                                b->ssr_cnt_wifi=b->ssr_cnt_venus=\
                                                b->ssr_cnt_modem=b->ssr_cnt_mdm=0)

#define SSR_INFO_START_MAGIC_NUM                0xDEADDEAD
#define SSR_INFO_END_MAGIC_NUM                  0xEFBEEFBE

// sync with vendor/pantech/apps/SkyMenu/src/com/pantech/app/test_menu/apps/SetSSRMode.java
#define SSRCMD_GET_ENABLE_MODEM                 0
#define SSRCMD_SET_ENABLE_MODEM                 1
#define SSRCMD_GET_DUMP_ENABLE_MODEM            2
#define SSRCMD_SET_DUMP_ENABLE_MODEM            3
#define SSRCMD_GET_ENABLE_WIFI                  4
#define SSRCMD_SET_ENABLE_WIFI                  5
#define SSRCMD_GET_DUMP_ENABLE_WIFI             6
#define SSRCMD_SET_DUMP_ENABLE_WIFI             7

#define SSRCMD_GET_NOTIFICATION                 20
#define SSRCMD_SET_NOTIFICATION                 21
#define SSRCMD_SUBSYSTEM_LOG                    30
#define SSRCMD_SUBSYSTEM_COUNT                  31

#define SSR_NOTI_PROPERTY_NAME                  "ssr.noti.start"
#define SSR_LOG_MAX_CNT                         10

// ===============================================================================
// Type and structure definition
// ===============================================================================

typedef struct {
    unsigned int ssr_info_start_magic_num;
    unsigned int ssr_set_modem_enable;
    unsigned int ssr_set_modem_dump_enable;
    unsigned int ssr_set_wifi_enable;
    unsigned int ssr_set_wifi_dump_enable;
    unsigned int ssr_set_notification_enable;
    unsigned int ssr_cnt_lpass;
    unsigned int ssr_cnt_dsps;
    unsigned int ssr_cnt_wifi;
    unsigned int ssr_cnt_venus;
    unsigned int ssr_cnt_modem;
    unsigned int ssr_cnt_mdm;
    unsigned int silent_boot_mode;
    unsigned int backlight_off;
    unsigned int reserved3;
    unsigned int reserved4;
    unsigned int reboot_setting;
    unsigned int ssr_info_end_magic_num;
} sky_ssr_info_type;

// ===============================================================================
// Global variable
// ===============================================================================

// ===============================================================================
// External variable and funtion
// ===============================================================================

extern unsigned char ssrdump[];

// ===============================================================================
// Function declaration
// ===============================================================================


#endif

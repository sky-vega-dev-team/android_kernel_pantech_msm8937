#ifndef __PANTECH_SYS_INFO_H
#define __PANTECH_SYS_INFO_H

#define MMU_SCRIPT_BUF_SIZE 1024 
//#define MMU_SCRIPT_BUF_SIZE 512 
#define P_INFO_BUFFER_SIZE 0x80

/*******************************************************************************
**  VENDOR2 Shared memory structure for log info
*******************************************************************************/
typedef struct _pantech_log_header{
    unsigned int magic;
    unsigned int version;
    unsigned long long int klog_buf_address;
    unsigned int klog_size;
    unsigned long long int mlogcat_buf_address;
    unsigned long long int mlogcat_w_off;
    unsigned int mlogcat_size;
    unsigned long long int slogcat_buf_address;
    unsigned long long int slogcat_w_off;
    unsigned int slogcat_size;
    unsigned long long int rlogcat_buf_address;
    unsigned long long int rlogcat_w_off;
    unsigned int rlogcat_size;

    unsigned int start_phone_info_log;
    char information[P_INFO_BUFFER_SIZE];
    unsigned int end_phone_info_log;

    char mmu_cmm_script[MMU_SCRIPT_BUF_SIZE];
    int mmu_cmm_size;        

    unsigned long long int pantech_dbg_addr;
    unsigned int pantech_dbg_size;

}pantech_log_header;

/* reset reason & etc information structure */
/* !!WARNING!! - model specification, do not copy to another model directly */

#define SYSTEM_INFORMATION_START_MAGIC_NUM          0xDEADABCD
#define SYSTEM_INFORMATION_END_MAGIC_NUM             0xABCDDEAD

#define MAX_SW_RESET_LIST 10

// p16652 modify for source integration & clean.
typedef struct pantech_hw_reset_reason_type
{
    unsigned int pwr_reason_hw_reset;                  // HW reset Power On Reset
    unsigned int pwr_reason_smpl;                      // SMPL Power On Reset
    unsigned int pwr_reason_rtc_alarm;                 // RTC Alarm Power On Reset
    unsigned int pwr_reason_cable_in;                  // Cable In On Reset
    unsigned int pwr_reason_usb_chg;                   // USB CHG Power On Reset
    unsigned int pwr_reason_pon1;                      // PON1 On Reset
    unsigned int pwr_reason_cable_pwr;                 // Cable Power On Reset    
    unsigned int pwr_reason_keypad_on;                 // Keypad Power On Reset
    unsigned int pwr_reason_watch_dog;                 // WDOG Power On Reset
    unsigned int pwr_reason_reset_reason;              // Latest Power On Reason
    unsigned int pwr_reason_pre_1st_reset_reason;
    unsigned int pwr_reason_pre_2nd_reset_reason;
    unsigned int pwr_reason_total_on;                  // Total Power On Count
    unsigned int pwr_reason_reserved[5];
}p_hw_reset_reason_type;

typedef struct swr_reason_list_info
{
    unsigned int swrResetReason;
    unsigned int swrResetTime; //P11014 change the type for sync sbl1 [long int -> unsigned int]
} swr_reason_list_info_type;

typedef struct swr_reason_list
{
    swr_reason_list_info_type list[10];
    unsigned int latest_index;
} swr_reason_list_type;

typedef struct swr_reason_backup_info
{
    unsigned int swrResetOccured;
    unsigned int backlightOnOff;
} swr_reason_backup_info_type;

typedef struct pantech_sw_reset_reason_type
{
    unsigned int swr_reason_linux;
    unsigned int swr_reason_watch_dog;
    unsigned int swr_reason_abnormal;
    unsigned int swr_reason_mdm;
    unsigned int swr_reason_adsp;
    unsigned int swr_reason_wifi;
    unsigned int swr_reason_rpm;
    unsigned int swr_reason_venus;
    unsigned int swr_reason_modem;
    unsigned int swr_reason_android;
    unsigned int swr_reason_userdata_fs;
    unsigned int swr_reason_data_mount_err;     //data partition was not mounted
    unsigned int swr_reason_e2fsck_fs_crash;      //the count which e2fsck repairs /data for fs crash
    unsigned int swr_reason_e2fsck_data_mnt_err;      //the count which e2fsck repairs /data for data mount err
    unsigned int swr_reason_reserved[5];
    swr_reason_list_type swr_reset_hisroty;
    swr_reason_backup_info_type swr_backup_info;
}p_sw_reset_reason_type;

#define SSR_UEVENT_IDEN "SSR_PANTECH_EVENT="

typedef struct pantech_ssr_reset_reason_type
{
    unsigned int ssr_reason_wifi;
    unsigned int ssr_reason_modem;
    unsigned int ssr_reason_reserved[5];
 }p_ssr_reason_type;

typedef struct pantech_etc_system_info_type
{
    unsigned int sys_info_msm_manufacture_type;   // DDR Device Info  : [7:4]: Speed bin [3:0] : PVS num 
    unsigned int sys_info_ddr_manufacture_type;   // DDR Device Info  : [11:8] : DDR manufacture name, [7:0] : DDR type Info
    unsigned int sys_info_reserved[5];
}p_etc_system_info_type;

typedef struct pantech_system_information_type{
    unsigned int system_information_start_magic;           // Start Magic Number
    p_hw_reset_reason_type p_hw_reset_reason;
    p_sw_reset_reason_type p_sw_reset_reason;
    p_ssr_reason_type p_ssr_reason;
    p_etc_system_info_type p_etc_system_info;
    unsigned int system_information_end_magic;             // End Magic Number
}p_system_information;

/* dump mode information structure */

#define ERR_CRASH_DUMP_START_MAGIC_NUM          0xDEADDEAD
#define ERR_CRASH_DUMP_END_MAGIC_NUM             0xEFBEEFBE

#define SKY_DUMP_INIT   "/persist/.dumpcookie"
#define SKY_SSR_INIT      "/persist/.ssrcookie"

typedef struct{
  unsigned int err_crash_dump_start_magic_num;         // Start Magic Number
  unsigned int ram_dump_enable;
  unsigned int usb_dump_enable;
  unsigned int mdm_dump_enable;
  unsigned int temporary_dump_enable;   // by p11219  for temporary ramdump set flag in multi menu.  
  unsigned int user_dump_enable;            // p15060
  unsigned int err_crash_dump_end_magic_num;             // End Magic Number
}err_crash_dump_type;

#endif

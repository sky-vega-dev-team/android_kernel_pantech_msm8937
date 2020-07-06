/*
 * Support function for Android USB Gadget Driver
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNE General Public License for more details.
 *
 * Add comment : 2011-10-28
 * modify from Motorola base source.
 * modify by Pantech Inc. <tarial>
 *
 */

#include <linux/types.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/rwsem.h>

#ifdef CONFIG_PANTECH_PMIC_ABNORMAL
#include <linux/power/qpnp-charger.h>
#endif /* CONFIG_PANTECH_PMIC_ABNORMAL */

#include "f_pantech_android.h"

int b_support_ms_os_descriptor;
static int current_usb_status = -1;
#ifdef CONFIG_PANTECH_PMIC_ABNORMAL
static int set_charger_type_data_cable(bool type);
#endif
static int enable_android_usb_product_function(char *device_name, int cnt);
static void send_usb_event(struct android_dev *dev, int ev_type);
static void force_reenumeration(struct android_dev *dev, int dev_type);

#define MAX_DEVICE_TYPE_NUM   64
#define MAX_BUFFER_SIZE  256
#define MAX_CONFIG_SIZE 5

#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
static bool b_factory_mode_enabled;
static bool b_factory_mode_request;
static struct delayed_work factory_work;
static bool b_factory_mode_adb_onoff;
static int set_factory_adb_mode(int onoff);
static struct delayed_work factory_adb_work;
#endif

#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
static bool mdm_mode_initialized;
static bool mdm_mode_enabled;
static void (*mdm_mode_callback)(bool value);
#endif
#if defined(CONFIG_ANDROID_PANTECH_USB_MANAGER) && defined(FEATURE_PANTECH_MULTI_ETHERNET)
static struct eth_dev *pantech_eth_dev;
static struct android_usb_function *setup_net_f;
#endif

//LS2_USB tarial USB3 detect testcode
static int usb3_state_value=0;
static int configuration_id = 0;
// for multi configuration.
static char multi_config_buffer[MAX_BUFFER_SIZE];

#ifdef CONFIG_ANDROID_PANTECH_USB_CDFREE
unsigned char pantech_cdrom_enabled = (unsigned char)0;
unsigned char pantech_cdrom_only = (unsigned char)0;
#endif
#ifdef CONFIG_ANDROID_PANTECH_USB_ABNORMAL_CHARGER_INFO
extern void set_pmic_usb_configured(void);
#endif

#define FEATURE_PANTECH_MS_OS_COMPATIBLE
/* Comment : MS Extended CompatibleID OS Desc 
 * supported function list : rndis, mtp, ptp, XUSB20, BLUETUTH 
 * The mtp/ptp mode from list is only supported.
 * A single function with multi-interfaces is not supported in MCID but
 * it is possible if the composition is changed to IAD Descriptor.
 * [Change list]
 * - pantech_ext_config_desc_list : MCID descriptor definition
 * - pantech_android_vid_pid[] : IAD Class/SubClass/Prol should be defined.
 * - serial/obex function source : define FEATURE_ANDROID_PANTECH_USB_IAD
 */

#ifdef FEATURE_PANTECH_MS_OS_COMPATIBLE
/* Microsoft Extended Configuration Descriptor Header Section */
struct ms_ext_config_desc_header {
    __le32	dwLength;
    __u16	bcdVersion;
    __le16	wIndex;
    __u8	bCount;
    __u8	reserved[7];
};

/* Microsoft Extended Configuration Descriptor Function Section */
struct ms_ext_config_desc_function {
    __u8	bFirstInterfaceNumber;
    __u8	bInterfaceCount;
    __u8	compatibleID[8];
    __u8	subCompatibleID[8];
    __u8	reserved[6];
};

struct ms_ext_config_desc{
    struct ms_ext_config_desc_header header;
    struct ms_ext_config_desc_function function[9];/* max interface */ //increased for Korea CDFREE 5->9
};

struct pantech_ext_config_desc{
    u32 type;
    struct ms_ext_config_desc desc;
};

static struct pantech_ext_config_desc  *p_pantech_ext_config_desc;

/* check size
 * __constant_cpu_to_le32(sizeof(ms_ext_config_desc_header)) : 16
 * __constant_cpu_to_le32(sizeof(ms_ext_config_desc_function)) : 24 
 */
static struct pantech_ext_config_desc  pantech_ext_config_desc_list[] = 
{
    {
        /*"mtp" : 0*/
        MTP_TYPE_FLAG,
        {
            {
                .dwLength = __constant_cpu_to_le32(16 + 24), 
                .bcdVersion = __constant_cpu_to_le16(0x0100),
                .wIndex = __constant_cpu_to_le16(4),
                .bCount = __constant_cpu_to_le16(1), /* mtp only */
            },
            {
                {
                    .bFirstInterfaceNumber = 0,
                    .bInterfaceCount = 1,
                    .compatibleID = { 'M', 'T', 'P' },
                },
            }
        }	
    },
    {
        /*"ptp" : 1*/
        PTP_TYPE_FLAG,
        {
            {
                .dwLength = __constant_cpu_to_le32(16 + 24), 
                .bcdVersion = __constant_cpu_to_le16(0x0100),
                .wIndex = __constant_cpu_to_le16(4),
                .bCount = __constant_cpu_to_le16(1),
            },
            {
                {
                    .bFirstInterfaceNumber = 0,
                    .bInterfaceCount = 1,
                    .compatibleID = { 'P', 'T', 'P' },
                },
            }
        }
    },
    {
        /*"mtp,adb" : 2*/
        MTP_TYPE_FLAG | ADB_TYPE_FLAG,
        {
            {
                .dwLength = __constant_cpu_to_le32(16 + 24*2), 
                .bcdVersion = __constant_cpu_to_le16(0x0100),
                .wIndex = __constant_cpu_to_le16(4),
                .bCount = __constant_cpu_to_le16(2),/* check */
            },
            {
                {
                    .bFirstInterfaceNumber = 0,
                    .bInterfaceCount = 1,
                    .compatibleID = { 'M', 'T', 'P' },
                },
                {
                    .bFirstInterfaceNumber = 1,
                    .bInterfaceCount = 1,
                },
            }
        }
    },
    {
            /*"ptp,adb" : 3*/
            PTP_TYPE_FLAG | ADB_TYPE_FLAG,
            {
                {
                    .dwLength = __constant_cpu_to_le32( 16 + 24*2), 
                    .bcdVersion = __constant_cpu_to_le16(0x0100),
                    .wIndex = __constant_cpu_to_le16(4),
                    .bCount = __constant_cpu_to_le16(2),
                },
                {
                    {
                        .bFirstInterfaceNumber = 0,
                        .bInterfaceCount = 1,
                        .compatibleID = { 'P', 'T', 'P' },
                    },
                    {
                        .bFirstInterfaceNumber = 1,
                        .bInterfaceCount = 1,
                    },
                }
            }
    },
#ifdef CONFIG_ANDROID_PANTECH_USB_CDFREE
       {
            CDROM_TYPE_FLAG | MTP_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG,
            {
                {
                    .dwLength = __constant_cpu_to_le32( 16 + 24*7), 
                    .bcdVersion = __constant_cpu_to_le16(0x0100),
                    .wIndex = __constant_cpu_to_le16(4),
                    .bCount = __constant_cpu_to_le16(7),
                },
                {
                    {
                        .bFirstInterfaceNumber = 0, // UMS
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 1, // MTP
                        .bInterfaceCount = 1,
                        .compatibleID = { 'M', 'T', 'P' },
                    },
                    {
                        .bFirstInterfaceNumber = 2, // MODEM1
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 3, // MODEM2
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 4, // DIAG
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 5, // OBEX1
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 6, // OBEX2
                        .bInterfaceCount = 1,
                    },
                }
            }
        },
        {
            CDROM_TYPE_FLAG | MTP_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | ADB_TYPE_FLAG,
            {
                {
                    .dwLength = __constant_cpu_to_le32( 16 + 24*8), 
                    .bcdVersion = __constant_cpu_to_le16(0x0100),
                    .wIndex = __constant_cpu_to_le16(4),
                    .bCount = __constant_cpu_to_le16(8),
                },
                {
                    {
                        .bFirstInterfaceNumber = 0, // UMS
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 1, // MTP
                        .bInterfaceCount = 1,
                        .compatibleID = { 'M', 'T', 'P' },
                    },
                    {
                        .bFirstInterfaceNumber = 2, // MODEM1
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 3, // MODEM2
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 4, // DIAG
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 5, // OBEX1
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 6, // OBEX2
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 7, // ADB 
                        .bInterfaceCount = 1,
                    },
                }
            }
        },
#endif
#if 1 //LS4-USB for PST USB composition
        {
            /*"mtp,diag,serial" : 6*/
            MTP_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG,
            {
                {
                    .dwLength = __constant_cpu_to_le32( 16 + 24*4), 
                    .bcdVersion = __constant_cpu_to_le16(0x0100),
                    .wIndex = __constant_cpu_to_le16(4),
                    .bCount = __constant_cpu_to_le16(4),
                },
                {
                    {
                        .bFirstInterfaceNumber = 0, // MTP
                        .bInterfaceCount = 1,
                        .compatibleID = { 'M', 'T', 'P' },
                    },
                    {
                        .bFirstInterfaceNumber = 1, // MODEM1
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 2, // MODEM2
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 3, // DIAG
                        .bInterfaceCount = 1,
                    },
                }
            }
        },
        {
            /*"mtp,diag,serial,adb" : 7*/
            MTP_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG,
            {
                {
                    .dwLength = __constant_cpu_to_le32( 16 + 24*5), 
                    .bcdVersion = __constant_cpu_to_le16(0x0100),
                    .wIndex = __constant_cpu_to_le16(4),
                    .bCount = __constant_cpu_to_le16(5),
                },
                {
                    {
                        .bFirstInterfaceNumber = 0, // MTP
                        .bInterfaceCount = 1,
                        .compatibleID = { 'M', 'T', 'P' },
                    },
                    {
                        .bFirstInterfaceNumber = 1, // MODEM1
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 2, // MODEM2
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 3, // DIAG
                        .bInterfaceCount = 1,
                    },
                    {
                        .bFirstInterfaceNumber = 4, // ADB 
                        .bInterfaceCount = 1,
                    },
                }
            }
        },
#endif
};
#endif

/* define structure for switching mode */
struct device_pid_vid {
    char *name;
    u32 type;
    int vid;
    int pid;
    char *config_name;
    int class;
    int subclass;
    int protocol;
};

static char *pantech_android_carrier_list[CARRIER_TYPE_MAX]=
{
    "kor|",
    "verizon|",
    "atnt|",
    "japan|",
    "japan_pmc|",
    "qualcomm|"
};

static struct device_pid_vid pantech_android_vid_pid[CARRIER_TYPE_MAX][MAX_DEVICE_TYPE_NUM] = {
    /*defined(CONFIG_PANTECH_DOMESTIC) */
    {
        {"mass_storage,cdrom", 
            MSC_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0xE001, 
            "Pantech Config 1",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage", 
            MSC_TYPE_FLAG, 
            0x10A9, 0xE002,
            "Pantech Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"cdrom", 
            CDROM_TYPE_FLAG, 
            0x10A9, 0xE003, 
            "Pantech CDROM Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth", 
            ETH_TYPE_FLAG, 
            0x10A9, 0xE004,
            "Pantech Config 4",
            USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},

        {"mtp", 
            MTP_TYPE_FLAG, 
            0x10A9, 0x1105, "Pantech Config 5",
            USB_CLASS_VENDOR_SPEC, USB_SUBCLASS_VENDOR_SPEC, 0},

        {"ptp",
            PTP_TYPE_FLAG,
            0x10A9, 0x1107, "Pantech Config 22",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging", 
            CHARGING_TYPE_FLAG, 
            0x10A9, 0xE006, 
            "Pantech Config 7",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		{"midi",
			MIDI_TYPE_FLAG,
		0x10A9, 0x1104,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		
        {"accessory", 
            ACCESSORY_TYPE_FLAG , 
            0x18D1, 0x2D00,
            "Google Accessory Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,adb", 
            ACCESSORY_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D01,
            "Google Accessory ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        //LS2_USB tarial added audio_source for AOA2.0 audio device
        {"audio_source", 
            AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D02,
            "Google Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"audio_source,adb", 
            AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D03,
            "Google Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D04,
            "Google Accessory+Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source,adb", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D05,
            "Google Accessory+Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,cdrom,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0x1104, "Pantech Config 2", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104,
            "Pantech Config 21", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth,adb", 
            ETH_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104,
            "Pantech Android Composite Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mtp,adb", 
            MTP_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1105,
            "Pantech Config 6", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ptp,adb",
            PTP_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x1107,
            "Pantech Config 23",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging,adb", 
            CHARGING_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104,
            "Pantech Config 8", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		{"midi,adb",
			MIDI_TYPE_FLAG | ADB_TYPE_FLAG,
		0x10A9, 0x1104,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis", 
            RNDIS_TYPE_FLAG, 
            0x10A9, 0x1108, 
            "Pantech RNDIS Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"rndis,adb", 
            RNDIS_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1108,
            "Pantech RNDIS ADB Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"serial,diag", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1104, 
            "Pantech Config 10", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,adb", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG , 
            0x10A9, 0x1104, 
            "Pantech Config 20", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag,obex", 
            ACM_TYPE_FLAG | MTP_TYPE_FLAG | OBEX_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag,obex,adb", 
            ACM_TYPE_FLAG | ADB_TYPE_FLAG | MTP_TYPE_FLAG | OBEX_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        //LGT cdfree LS2_USB tarial
        {"cdrom,mtp,serial,diag,obex", 
            CDROM_TYPE_FLAG | ACM_TYPE_FLAG | MTP_TYPE_FLAG | OBEX_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1106, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"cdrom,mtp,serial,diag,obex,adb", 
            CDROM_TYPE_FLAG | ACM_TYPE_FLAG | ADB_TYPE_FLAG | MTP_TYPE_FLAG | OBEX_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1106, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag", 
            ACM_TYPE_FLAG | MTP_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag,adb", 
            ACM_TYPE_FLAG | ADB_TYPE_FLAG | MTP_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"ptp,serial,obex", 
            ACM_TYPE_FLAG | PTP_TYPE_FLAG | OBEX_TYPE_FLAG , 
            0x10A9, 0x1107, 
            "Pantech Config 25", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"ptp,serial,obex,adb", 
            ACM_TYPE_FLAG | ADB_TYPE_FLAG | PTP_TYPE_FLAG | OBEX_TYPE_FLAG , 
            0x10A9, 0x1107, 
            "Pantech Config 25", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 11", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex,adb", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 12", 
            USB_CLASS_COMM, 0x00, 0x00},

        //SKT, KT cdfree by LS2_USB tarial
        {"cdrom,serial,diag,obex", 
            CDROM_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 11", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"cdrom,serial,diag,obex,adb", 
            CDROM_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 12", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG, 
            0x10A9, 0x1108, 
            "Pantech Config 13", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag,adb", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1108, 
            "Pantech Config 14", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x1109, 
            "Pantech Config 15", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex,adb,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | ADB_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x1109, 
            "Pantech Config 16", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"adb", 
            ADB_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Recovery ADB only", 
            USB_CLASS_COMM, 0x00, 0x00},

        //LS2_USB QDSS enable
        {"diag,qdss",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x904A,
            "Qualcomm QDSS function driver(with DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9060,
            "Qualcomm QDSS function driver(with DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,rmnet",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x05c6, 0x9083,
            "Qualcomm QDSS function driver(with DIAG+RMNET)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,qdss,rmnet,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9084,
            "Qualcomm QDSS function driver(with DIAG+RMNET+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis,diag,qdss",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x9081,
            "Qualcomm QDSS function driver(with RNDIS+DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis,diag,qdss,adb",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9082,
            "Qualcomm QDSS function driver(with RNDIS+DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm",
            NCM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "Qualcomm ncm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm,adb",
            NCM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "Qualcomm ncm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm",
            ECM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "ecm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm,adb",
            ECM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "ecm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"default", DEFAULT_TYPE_FLAG, 0, 0, 0, 0, 0, 0},
        {}

    },/* DOMESTIC */

    /*defined(CONFIG_PANTECH_VERIZON)*/
    {
        {"mass_storage,cdrom", 
            MSC_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0xE001, 
            "Pantech Config 1",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage", 
            MSC_TYPE_FLAG, 
            0x10A9, 0xE002,
            "Pantech Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"cdrom", 
            CDROM_TYPE_FLAG, 
            0x10A9, 0xE003, 
            "Pantech CDROM Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth", 
            ETH_TYPE_FLAG, 
            0x10A9, 0xE004,
            "Pantech Config 4",
            USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},

        {"mtp", 
            MTP_TYPE_FLAG, 
            0x10A9, 0x7035, "Pantech Config 5",
            USB_CLASS_VENDOR_SPEC, USB_SUBCLASS_VENDOR_SPEC, 0},

        {"ptp",
            PTP_TYPE_FLAG,
            0x10A9, 0x7034, "Pantech Config 22",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging", 
            CHARGING_TYPE_FLAG, 
            0x10A9, 0xE006, 
            "Pantech Config 7",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		{"midi",
			MIDI_TYPE_FLAG,
			0x10A9, 0xE007,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},
			
        {"accessory", 
            ACCESSORY_TYPE_FLAG , 
            0x18D1, 0x2D00,
            "Google Accessory Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,adb", 
            ACCESSORY_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D01,
            "Google Accessory ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        //LS2_USB tarial added audio_source for AOA 2.0 audio device
        {"audio_source", 
            AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D02,
            "Google Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"audio_source,adb", 
            AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D03,
            "Google Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D04,
            "Google Accessory+Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source,adb", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D05,
            "Google Accessory+Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,cdrom,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0x7031,
            "Pantech Config 2", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x7031,
            "Pantech Config 21", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth,adb", 
            ETH_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x7031,
            "Pantech Android Composite Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mtp,adb", 
            MTP_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x7035,
            "Pantech Config 6", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ptp,adb",
            PTP_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x7034,
            "Pantech Config 23",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging,adb", 
            CHARGING_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x7031,
            "Pantech Config 8", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		{"midi,adb",
			MIDI_TYPE_FLAG | ADB_TYPE_FLAG,
			0x10A9, 0x7032,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis", 
            RNDIS_TYPE_FLAG, 
            0x10A9, 0x7032, 
            "Pantech RNDIS Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"rndis,adb", 
            RNDIS_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x7032,
            "Pantech RNDIS ADB Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"serial,diag", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x7033, 
            "Pantech Config 10", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,adb", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG , 
            0x10A9, 0x7033, 
            "Pantech Config 20", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"cdrom,serial,diag", 
            CDROM_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG, 
            0x10A9, 0x7033, 
            "Pantech Config 11", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"cdrom,serial,diag,adb", 
            CDROM_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x7033, 
            "Pantech Config 12", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG, 
            0x10A9, 0x7032, 
            "Pantech Config 13", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag,adb", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x7032, 
            "Pantech Config 14", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x7036, 
            "Pantech Config 15", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,adb,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x7037, 
            "Pantech Config 16", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"adb", 
            ADB_TYPE_FLAG, 
            0x10A9, 0x7033, 
            "Pantech Recovery ADB only", 
            USB_CLASS_COMM, 0x00, 0x00},

        //LS2_USB QDSS enable
        {"diag,qdss",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x904A,
            "Qualcomm QDSS function driver(with DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9060,
            "Qualcomm QDSS function driver(with DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,rmnet",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x05c6, 0x9083,
            "Qualcomm QDSS function driver(with DIAG+RMNET)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,qdss,rmnet,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9084,
            "Qualcomm QDSS function driver(with DIAG+RMNET+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis,diag,qdss",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x9081,
            "Qualcomm QDSS function driver(with RNDIS+DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis,diag,qdss,adb",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9082,
            "Qualcomm QDSS function driver(with RNDIS+DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm",
            NCM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "Qualcomm ncm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm,adb",
            NCM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "Qualcomm ncm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm",
            ECM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "ecm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm,adb",
            ECM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "ecm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"default", DEFAULT_TYPE_FLAG, 0, 0, 0, 0, 0, 0},
        {}

    },/* Verizon */

    /* defined(CONFIG_PANTECH_ATNT)*/
    {
        {"mass_storage,cdrom", 
            MSC_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0xE001, 
            "Pantech Config 1",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage", 
            MSC_TYPE_FLAG, 
            0x10A9, 0xE002,
            "Pantech Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"cdrom", 
            CDROM_TYPE_FLAG, 
            0x10A9, 0xE003, 
            "Pantech CDROM Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth", 
            ETH_TYPE_FLAG, 
            0x10A9, 0xE004,
            "Pantech Config 4",
            USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},

        {"mtp", 
            MTP_TYPE_FLAG, 
            0x10A9, 0x6055, "Pantech Config 5",
            USB_CLASS_VENDOR_SPEC, USB_SUBCLASS_VENDOR_SPEC, 0},

        {"ptp",
            PTP_TYPE_FLAG,
            0x10A9, 0x6054, "Pantech Config 22",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging", 
            CHARGING_TYPE_FLAG, 
            0x10A9, 0xE006, 
            "Pantech Config 7",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

		{"midi",
			MIDI_TYPE_FLAG,
			0x10A9, 0xE007,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory", 
            ACCESSORY_TYPE_FLAG , 
            0x18D1, 0x2D00,
            "Google Accessory Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,adb", 
            ACCESSORY_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D01,
            "Google Accessory ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        //LS2_USB tarial added audio_source for AOA 2.0 audio device
        {"audio_source", 
            AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D02,
            "Google Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"audio_source,adb", 
            AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D03,
            "Google Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D04,
            "Google Accessory+Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source,adb", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D05,
            "Google Accessory+Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,cdrom,adb",
            MSC_TYPE_FLAG | ADB_TYPE_FLAG | CDROM_TYPE_FLAG,
            0x10A9, 0x6050,
            "Pantech Config 2",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,adb",
            MSC_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6050,
            "Pantech Config 21",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth,adb",
            ETH_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6050,
            "Pantech Android Composite Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mtp,adb",
            MTP_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6055,
            "Pantech Config 6",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ptp,adb",
            PTP_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6054,
            "Pantech Config 23",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging,adb",
            CHARGING_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6050,
            "Pantech Config 8",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		
		{"midi,adb",
			MIDI_TYPE_FLAG | ADB_TYPE_FLAG,
			0x10A9, 0x6050,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},
	
        {"rndis",
            RNDIS_TYPE_FLAG,
            0x10A9, 0x6051,
            "Pantech RNDIS Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"rndis,adb",
            RNDIS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6051,
            "Pantech RNDIS ADB Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"serial,diag",
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG,
            0x10A9,0x6052,
            "Pantech Config 10",
            USB_CLASS_COMM,0x00, 0x00},

        {"serial,diag,adb",
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9,0x6052,
            "Pantech Config 20",
            USB_CLASS_COMM,0x00, 0x00},

        {"mass_storage,serial,diag",
            MSC_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG,
            0x10A9,0x6052,
            "Pantech Config 11",
            USB_CLASS_COMM, 0x00, 0x00},

        {"mass_storage,serial,diag,adb",
            MSC_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6052,
            "Pantech Config 12",
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG,
            0x10A9, 0x6051,
            "Pantech Config 13",
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag,adb",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6051,
            "Pantech Config 14",
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,rmnet",
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x10A9, 0x6053,
            "Pantech Config 15",
            USB_CLASS_COMM,0x00, 0x00},

        {"serial,diag,adb,rmnet",
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x10A9, 0x6053,
            "Pantech Config 16",
            USB_CLASS_COMM, 0x00, 0x00},

        {"adb",
            ADB_TYPE_FLAG,
            0x10A9, 0x6052,
            "Pantech Recovery ADB only",
            USB_CLASS_COMM, 0x00, 0x00},

        //LS2_USB QDSS enable
        {"diag,qdss",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x904A,
            "Qualcomm QDSS function driver(with DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9060,
            "Qualcomm QDSS function driver(with DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,rmnet",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x05c6, 0x9083,
            "Qualcomm QDSS function driver(with DIAG+RMNET)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,qdss,rmnet,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9084,
            "Qualcomm QDSS function driver(with DIAG+RMNET+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis,diag,qdss",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x9081,
            "Qualcomm QDSS function driver(with RNDIS+DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis,diag,qdss,adb",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9082,
            "Qualcomm QDSS function driver(with RNDIS+DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm",
            NCM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "Qualcomm ncm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm,adb",
            NCM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "Qualcomm ncm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm",
            ECM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "ecm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm,adb",
            ECM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "ecm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"default", DEFAULT_TYPE_FLAG, 0, 0, 0, 0, 0, 0},
        {}

    },
    /* defined(JAPAN) */
    {

        {"mass_storage,cdrom", 
            MSC_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0xE001, 
            "Pantech Config 1",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage", 
            MSC_TYPE_FLAG, 
            0x10A9, 0xE002,
            "Pantech Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"cdrom", 
            CDROM_TYPE_FLAG, 
            0x10A9, 0xE003, 
            "Pantech CDROM Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth", 
            ETH_TYPE_FLAG, 
            0x10A9, 0xE004,
            "Pantech Config 4",
            USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},

        {"mtp", 
            MTP_TYPE_FLAG, 
            0x10A9, 0x6059, "Pantech Config 5",
            USB_CLASS_VENDOR_SPEC, USB_SUBCLASS_VENDOR_SPEC, 0},

        {"ptp",
            PTP_TYPE_FLAG,
            0x10A9, 0x6054, "Pantech Config 22",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging", 
            CHARGING_TYPE_FLAG, 
            0x10A9, 0xE006, 
            "Pantech Config 7",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		
		{"midi",
			MIDI_TYPE_FLAG,
			0x10A9, 0xE007,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},
			
        {"accessory", 
            ACCESSORY_TYPE_FLAG , 
            0x18D1, 0x2D00,
            "Google Accessory Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,adb", 
            ACCESSORY_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D01,
            "Google Accessory ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        //LS2_USB tarial added audio_source for AOA 2.0 audio device
        {"audio_source", 
            AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D02,
            "Google Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"audio_source,adb", 
            AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D03,
            "Google Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D04,
            "Google Accessory+Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source,adb", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D05,
            "Google Accessory+Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,cdrom,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0x6056,
            "Pantech Config 2", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x6056,
            "Pantech Config 21", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth,adb", 
            ETH_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x6056,
            "Pantech Android Composite Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mtp,adb", 
            MTP_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x6059,
            "Pantech Config 6", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ptp,adb",
            PTP_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x6054,
            "Pantech Config 23",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging,adb", 
            CHARGING_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x6056,
            "Pantech Config 8", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
	
		{"midi,adb",
			MIDI_TYPE_FLAG | ADB_TYPE_FLAG,
			0x10A9, 0x6056,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis", 
            RNDIS_TYPE_FLAG, 
            0x10A9, 0x6057, 
            "Pantech RNDIS Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"rndis,adb", 
            RNDIS_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x6057,
            "Pantech RNDIS ADB Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"serial,diag", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x6058, 
            "Pantech Config 10", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,adb", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG , 
            0x10A9, 0x6058, 
            "Pantech Config 20", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mass_storage,serial,diag", 
            MSC_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG, 
            0x10A9, 0x6058, 
            "Pantech Config 11", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mass_storage,serial,diag,adb", 
            MSC_TYPE_FLAG | ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x6058, 
            "Pantech Config 12", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG, 
            0x10A9, 0x6057, 
            "Pantech Config 13", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag,adb", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x6057, 
            "Pantech Config 14", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x6058, 
            "Pantech Config 15", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,adb,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x6058, 
            "Pantech Config 16", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"adb",
            ADB_TYPE_FLAG,
            0x10A9, 0x6052,
            "Pantech Recovery ADB only",
            USB_CLASS_COMM, 0x00, 0x00},

        //LS2_USB QDSS enable
        {"diag,qdss",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x904A,
            "Qualcomm QDSS function driver(with DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9060,
            "Qualcomm QDSS function driver(with DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,rmnet",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x05c6, 0x9083,
            "Qualcomm QDSS function driver(with DIAG+RMNET)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,qdss,rmnet,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9084,
            "Qualcomm QDSS function driver(with DIAG+RMNET+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis,diag,qdss",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x9081,
            "Qualcomm QDSS function driver(with RNDIS+DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis,diag,qdss,adb",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9082,
            "Qualcomm QDSS function driver(with RNDIS+DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm",
            NCM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "Qualcomm ncm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm,adb",
            NCM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "Qualcomm ncm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm",
            ECM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "ecm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm,adb",
            ECM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "ecm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"defaut", DEFAULT_TYPE_FLAG, 0, 0, 0, 0, 0, 0},
        {}

    },
    // JAPAN_PMC
    {
        {"mass_storage,cdrom", 
            MSC_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0xE001, 
            "Pantech Config 1",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage", 
            MSC_TYPE_FLAG, 
            0x10A9, 0xE002,
            "Pantech Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"cdrom", 
            CDROM_TYPE_FLAG, 
            0x10A9, 0xE003, 
            "Pantech CDROM Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth", 
            ETH_TYPE_FLAG, 
            0x10A9, 0xE004,
            "Pantech Config 4",
            USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},

        {"mtp", 
            MTP_TYPE_FLAG, 
            0x10A9, 0x1105, "Pantech Config 5",
            USB_CLASS_VENDOR_SPEC, USB_SUBCLASS_VENDOR_SPEC, 0},

        {"ptp",
            PTP_TYPE_FLAG,
            0x10A9, 0x1104, "Pantech Config 22",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging", 
            CHARGING_TYPE_FLAG, 
            0x10A9, 0xE006, 
            "Pantech Config 7",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		
		{"midi",
			MIDI_TYPE_FLAG,
			0x10A9, 0xE007,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},
		
        {"accessory", 
            ACCESSORY_TYPE_FLAG , 
            0x18D1, 0x2D00,
            "Google Accessory Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,adb", 
            ACCESSORY_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D01,
            "Google Accessory ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        //LS2_USB tarial added audio_source for AOA2.0 audio device
        {"audio_source", 
            AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D02,
            "Google Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"audio_source,adb", 
            AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D03,
            "Google Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG , 
            0x18D1, 0x2D04,
            "Google Accessory+Audio Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"accessory,audio_source,adb", 
            ACCESSORY_TYPE_FLAG | AUDIO_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x18D1, 0x2D05,
            "Google Accessory+Audio ADB Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,cdrom,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG | CDROM_TYPE_FLAG, 
            0x10A9, 0x1104, "Pantech Config 2", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mass_storage,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104,
            "Pantech Config 21", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"eth,adb", 
            ETH_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104,
            "Pantech Android Composite Device",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"mtp,adb", 
            MTP_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1105,
            "Pantech Config 6", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ptp,adb",
            PTP_TYPE_FLAG | ADB_TYPE_FLAG,
            0x10A9, 0x1104,
            "Pantech Config 23",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"charging,adb", 
            MSC_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104,
            "Pantech Config 8", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

		{"midi",
			MIDI_TYPE_FLAG | ADB_TYPE_FLAG,
			0x10A9, 0x1104,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis", 
            RNDIS_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech RNDIS Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"rndis,adb", 
            RNDIS_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104,
            "Pantech RNDIS ADB Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"serial,diag", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1104, 
            "Pantech Config 10", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,adb", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG , 
            0x10A9, 0x1104, 
            "Pantech Config 20", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag,obex", 
            ACM_TYPE_FLAG | MTP_TYPE_FLAG | OBEX_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag,obex,adb", 
            ACM_TYPE_FLAG | ADB_TYPE_FLAG | MTP_TYPE_FLAG | OBEX_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag", 
            ACM_TYPE_FLAG | MTP_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"mtp,serial,diag,adb", 
            ACM_TYPE_FLAG | ADB_TYPE_FLAG | MTP_TYPE_FLAG | DIAG_TYPE_FLAG , 
            0x10A9, 0x1105, 
            "Pantech Config 24", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"ptp,serial,obex", 
            ACM_TYPE_FLAG | PTP_TYPE_FLAG | OBEX_TYPE_FLAG , 
            0x10A9, 0x1104, 
            "Pantech Config 25", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"ptp,serial,obex,adb", 
            ACM_TYPE_FLAG | ADB_TYPE_FLAG | PTP_TYPE_FLAG | OBEX_TYPE_FLAG , 
            0x10A9, 0x1104, 
            "Pantech Config 25", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 11", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex,adb", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 12", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 13", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"rndis,diag,adb", 
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 14", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 15", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"serial,diag,obex,adb,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | OBEX_TYPE_FLAG | ADB_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Config 16", 
            USB_CLASS_COMM, 0x00, 0x00},

        {"adb", 
            ADB_TYPE_FLAG, 
            0x10A9, 0x1104, 
            "Pantech Recovery ADB only", 
            USB_CLASS_COMM, 0x00, 0x00},

        //LS2_USB QDSS enable
        {"diag,qdss",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x904A,
            "Qualcomm QDSS function driver(with DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9060,
            "Qualcomm QDSS function driver(with DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,rmnet",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x05c6, 0x9083,
            "Qualcomm QDSS function driver(with DIAG+RMNET)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,qdss,rmnet,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9084,
            "Qualcomm QDSS function driver(with DIAG+RMNET+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis,diag,qdss",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x9081,
            "Qualcomm QDSS function driver(with RNDIS+DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"rndis,diag,qdss,adb",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9082,
            "Qualcomm QDSS function driver(with RNDIS+DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm",
            NCM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "Qualcomm ncm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm,adb",
            NCM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "Qualcomm ncm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm",
            ECM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "ecm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm,adb",
            ECM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "ecm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"default", DEFAULT_TYPE_FLAG, 0, 0, 0, 0, 0, 0},
        {}

    },
    /* defined(Qualcomm) */
    {
#ifdef CONFIG_PANTECH_USB_APQ_ONLY
        {"diag,serial,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x05C6, 0x9036, 
            "Qualcomm Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,adb,serial,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x05C6, 0x9035, 
            "Qualcomm Config 4", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
#else
	// MSM8937 Qualcomm composition
	{"diag,serial,rmnet", 
            DIAG_TYPE_FLAG | ACM_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x05C6, 0x9091, 
            "Qualcomm Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,serial,rmnet,adb", 
            DIAG_TYPE_FLAG | ACM_TYPE_FLAG | RMNET_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x05C6, 0x9091, 
            "Qualcomm Config 4", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
#if 0						
	{"diag,serial,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x05C6, 0x9026, 
            "Qualcomm Config 3", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"diag,adb,serial,rmnet", 
            ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG | RMNET_TYPE_FLAG, 
            0x05C6, 0x9025, 
            "Qualcomm Config 4", 
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
#endif	
#endif
        {"rndis", 
            RNDIS_TYPE_FLAG, 
            0x05C6, 0xf00e, 
            "Qualcomm RNDIS Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"rndis,adb", 
            RNDIS_TYPE_FLAG | ADB_TYPE_FLAG, 
            0x05C6, 0x9024,
            "Qualcomm RNDIS ADB Device",
            USB_CLASS_MISC, 0x02, 0x01},

        {"adb",
            ADB_TYPE_FLAG,
            0x05C6, 0x9024,
            "Pantech Recovery ADB only",	
            USB_CLASS_COMM, 0x00, 0x00},

        //LS2_USB QDSS enable
        {"diag,qdss",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x904A,
            "Qualcomm QDSS function driver(with DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9060,
            "Qualcomm QDSS function driver(with DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,rmnet",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG,
            0x05c6, 0x9083,
            "Qualcomm QDSS function driver(with DIAG+RMNET)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"diag,qdss,rmnet,adb",
            DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | RMNET_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9084,
            "Qualcomm QDSS function driver(with DIAG+RMNET+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis,diag,qdss",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG,
            0x05c6, 0x9081,
            "Qualcomm QDSS function driver(with RNDIS+DIAG)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},
        {"rndis,diag,qdss,adb",
            RNDIS_TYPE_FLAG | DIAG_TYPE_FLAG | QDSS_TYPE_FLAG | ADB_TYPE_FLAG,
            0x05c6, 0x9082,
            "Qualcomm QDSS function driver(with RNDIS+DIAG+ADB)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm",
            NCM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "Qualcomm ncm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ncm,adb",
            NCM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "Qualcomm ncm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm",
            ECM_TYPE_FLAG,
            0x0525, 0xA4A1,
            "ecm function driver",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

        {"ecm,adb",
            ECM_TYPE_FLAG | ADB_TYPE_FLAG,	
            0x05c6, 0x908C,
            "ecm function driver(with adb)",
            USB_CLASS_PER_INTERFACE, 0x00, 0x00},

		{"charging",
		    CHARGING_TYPE_FLAG,
	    	0x05c6, 0xF006,
		    "Qualcomm charging function driver",
		    USB_CLASS_PER_INTERFACE, 0x00, 0x00},

		{"midi",
			MIDI_TYPE_FLAG,
			0x05c6, 0x90BA,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},

		{"midi,adb",
			MIDI_TYPE_FLAG | ADB_TYPE_FLAG,
			0x05c6, 0x90BB,
			"Pantech Midi",
			USB_CLASS_PER_INTERFACE, 0x00, 0x00},
			
        {"default", DEFAULT_TYPE_FLAG, 0, 0, 0, 0, 0, 0},
        {}
    }
};

static struct device_pid_vid *mode_android_vid_pid;

// structure for mode switching operation
struct device_mode_change_dev {
    int usb_manager_on;
    int android_enabled_function_flag;
    char *pc_mode_switch_flag;
    int usb_device_cfg_flag;
    int usb_get_desc_flag;
    int usb_data_transfer_flag;
    int pst_req_mode_switch_flag;
    int usb_reconnect_flag;
    int usb_state_get_flag;
#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
    int usb_mdm_mode_flag;
#endif
    wait_queue_head_t android_enable_function_wq;
    wait_queue_head_t device_mode_change_wq;
    int g_device_type;
    atomic_t device_mode_change_excl;
};

char *pantech_usb_mode_list[MAX_USB_TYPE_NUM] = {
    "pc_mode",
    "mtp_mode",
    "ums_mode",
    "charge_mode",
    "field_mode",
    "factory_mode",
    "ptp_mode",
	"midi_mode",
    "accessory_mode",
    ""
};

static int current_mode_for_pst;

/* usb manager state enabled check for vBus session*/
static int usb_manager_state_enabled=0;

static struct device_mode_change_dev *_device_mode_change_dev;
atomic_t adb_enable_excl;

#ifdef FEATURE_PANTECH_MS_OS_COMPATIBLE
int get_ms_ext_config_desc(int type)
{
    int i = 0;
    int size = sizeof(pantech_ext_config_desc_list)/sizeof(struct pantech_ext_config_desc);

    for(i = 0; i < size; i++)		
    {
        if(pantech_ext_config_desc_list[i].type == type){
            p_pantech_ext_config_desc	= &pantech_ext_config_desc_list[i];	
            return i;
        }
    }
    printk(KERN_ERR "[PUSB]%s: not matched !!!\n", __func__);
    p_pantech_ext_config_desc = NULL;
    return -1;
}
#endif

static void get_device_pid_vid(int type, int *pid, int *vid)
{
    int i;

    *vid = 0;
    *pid = 0;
    for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
        if (mode_android_vid_pid[i].type == type) {
            *pid = mode_android_vid_pid[i].pid;
            *vid = mode_android_vid_pid[i].vid;
            break;
        }
    }
}

static int get_func_thru_type(int type)
{
    int i;

    for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
        if (mode_android_vid_pid[i].type == type)
            return i;
    }
    return -1;
}

static char *get_name_thru_type(int type)
{
    int i;

    for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
        if (mode_android_vid_pid[i].type == type)
            return mode_android_vid_pid[i].name;
    }
    return NULL;
}

int adb_enable_access(void)
{
    return atomic_read(&adb_enable_excl);
}

void usb_disconnect_cb(void)
{
    if(usb_manager_state_enabled){
        struct device_mode_change_dev *dev_mode_change =
            _device_mode_change_dev;

        if(current_usb_status == 0)
            return;

        dev_mode_change->usb_state_get_flag = -1;
        dev_mode_change->usb_data_transfer_flag = 0;
        wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
        printk(KERN_ERR "[PUSB][%s]mode[usb_disconnected]\n",__func__);
    }
    //LS2_USB tarial USB3 detection test code
    usb3_state_set(0);
}

void usb_connect_cb(void)
{
    if(usb_manager_state_enabled){
        struct device_mode_change_dev *dev_mode_change =
            _device_mode_change_dev;

        dev_mode_change->usb_state_get_flag = 1;
        dev_mode_change->usb_data_transfer_flag = 0;
        wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
        printk(KERN_ERR "[PUSB][%s]mode[usb_connected]\n",__func__);
    }

}

int usb_reconnect_cb(int type)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if(usb_manager_state_enabled){
        if(!dev_mode_change || !dev_mode_change->usb_manager_on) return 0;

        dev_mode_change->usb_reconnect_flag = 1;

        if(type){
            dev_mode_change->pc_mode_switch_flag = get_name_thru_type(type);
        }else{
            dev_mode_change->pc_mode_switch_flag = get_name_thru_type(dev_mode_change->g_device_type);
        }
        wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
        printk(KERN_ERR "[PUSB][%s]index[%s]\n",__func__, dev_mode_change->pc_mode_switch_flag);
    }
    return 1;
}

int type_switch_cb(int type)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if(type == dev_mode_change->g_device_type) {
        if(_android_dev->enabled){
            printk(KERN_ERR "[PUSB][%s]same type[0x%x]\n",__func__, type);
            return 0;
        }else{
            dev_mode_change->usb_reconnect_flag = 1;
        }
    }
    dev_mode_change->pc_mode_switch_flag = get_name_thru_type(type);
    wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
    printk(KERN_ERR "[PUSB][%s][%s]\n",__func__,dev_mode_change->pc_mode_switch_flag);
    return 1;
}

static void usb_data_transfer_callback(void)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if(!usb_manager_state_enabled)
        return;

    if (dev_mode_change->usb_data_transfer_flag == 0) {
        dev_mode_change->usb_data_transfer_flag = 1;
        dev_mode_change->usb_get_desc_flag = 1;
        wake_up_interruptible
            (&dev_mode_change->device_mode_change_wq);
    }
}

void usb_interface_enum_cb(int flag)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if(!usb_manager_state_enabled)
        return;

    dev_mode_change->usb_device_cfg_flag |= flag;
    if (dev_mode_change->usb_device_cfg_flag >=
        dev_mode_change->g_device_type)
        wake_up_interruptible
            (&dev_mode_change->device_mode_change_wq);
    printk(KERN_ERR "[PUSB][%s]current[0x%x], g_device_flag[0x%x]\n",__func__, dev_mode_change->usb_device_cfg_flag, dev_mode_change->g_device_type );
}

static inline int get_type_of_str(const char *func)
{
    int len;
    if(!func) return 0;

    len = strlen(func);

    if((len == 12) && !strcmp(func, "mass_storage")){
        return MSC_TYPE_FLAG;
    }else if((len == 3) && (!strcmp(func, "adb") || (!strcmp(func,"ffs")))){
        return ADB_TYPE_FLAG;
    }else if(((len == 3) && strcmp(func, "eth") == 0) || ((len == 6) && strcmp(func,"usbnet") == 0)){
        return ETH_TYPE_FLAG;
    }else if((len == 3) && !strcmp(func, "mtp")){
        return MTP_TYPE_FLAG;
    }else if((len == 6) && !strcmp(func, "serial")){
        return ACM_TYPE_FLAG;
    }else if((len == 5) && !strcmp(func, "cdrom")){
        return CDROM_TYPE_FLAG;
    }else if((len == 4) && !strcmp(func, "diag")){
        return DIAG_TYPE_FLAG;
    }else if((len == 5) && !strcmp(func, "rndis")){
        return RNDIS_TYPE_FLAG;
    }else if((len == 5) && !strcmp(func, "rmnet")){
        return RMNET_TYPE_FLAG;
    }else if((len == 9) && !strcmp(func, "accessory")){
        return ACCESSORY_TYPE_FLAG;
    }else if((len == 4) && !strcmp(func, "obex")){
        return OBEX_TYPE_FLAG;
    }else if((len == 3) && !strcmp(func, "ptp")){
        return PTP_TYPE_FLAG;
    }else if((len == 12) && !strcmp(func, "audio_source")) {
        return AUDIO_TYPE_FLAG;
    }else if((len == 4) && !strcmp(func, "qdss")){
        return QDSS_TYPE_FLAG;
    }else if((len == 3) && !strcmp(func, "ncm")){
        return NCM_TYPE_FLAG;
    }else if((len == 3) && !strcmp(func, "ecm")){
        return ECM_TYPE_FLAG;
    }else if((len == 8) && !strcmp(func, "charging")){
		return CHARGING_TYPE_FLAG;
    }else if((len == 4) && !strcmp(func, "midi")){
		return MIDI_TYPE_FLAG;
	}

    return 0;
}


static void android_enable_function_cb(char *functions)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;
    char *func;
    int ret;
    int type = 0;
    char *conf;
    int index = 0;
    char *ptr;

    if(!functions)
        return;

    memset(multi_config_buffer, 0x00, MAX_BUFFER_SIZE);

    //check about first config
    while(functions){

        conf = strsep(&functions, ":");
        type = 0x00;

        while(conf){
            func = strsep(&conf, ",");
            type |= get_type_of_str((const char*)func);
        }

        if(type){
#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
            if(b_factory_mode_enabled && b_factory_mode_request){
                if(type & ~(ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG) )	
                {
                    return;
                }
            }
#endif
            if(!dev_mode_change->usb_manager_on){
                if((type & ADB_TYPE_FLAG) == ADB_TYPE_FLAG)
                    atomic_set(&adb_enable_excl, 1);
                else
                    atomic_set(&adb_enable_excl, 0);
                printk(KERN_ERR "[PUSB]usb manager not ready!\n");
                return;
            }

            if(type == dev_mode_change->g_device_type) {
                if(_android_dev->enabled){
                    return;
                }else{
                    dev_mode_change->usb_reconnect_flag = 1;
                }
            }

            ptr = get_name_thru_type(type);
            if(ptr){
                if(index > 0){
                    multi_config_buffer[index++] = ':';
                }
                memcpy(&multi_config_buffer[index], ptr, strlen(ptr));
                index += strlen(ptr);
            }
        }
    }//while

    if(index){
        dev_mode_change->android_enabled_function_flag = 0;
        dev_mode_change->pc_mode_switch_flag  = multi_config_buffer;
        printk(KERN_ERR "[PUSB][%s][%s]\n",__func__,dev_mode_change->pc_mode_switch_flag);
        wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
        ret = wait_event_interruptible(dev_mode_change->android_enable_function_wq,
                                       (dev_mode_change->android_enabled_function_flag));
        if (ret < 0) {
            printk(KERN_ERR "[PUSB][%s] errno: %d\n",__func__, ret);
            return;
        }
    }
    return;
}

#ifdef CONFIG_ANDROID_PANTECH_USB_CDFREE
void pst_req_mode_switch_cb(int pst_mode)
{

    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;
    switch(pst_mode){
    case PC_MODE:
    case WINDOW_MEDIA_SYNC:
    case USB_MASS_STORAGE:
    case CHARGE_ONLY:
        break;
    default:
        printk(KERN_ERR "[PUSB][%s]pst mode change not allowed command : %d\n", __func__, pst_mode);
        return;
    }
    current_mode_for_pst = pst_mode;
    dev_mode_change->pst_req_mode_switch_flag = 1;
    wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
    printk(KERN_ERR "[PUSB][%s]pst_mode[%d]\n",__func__, pst_mode);
}
int pst_rsp_current_mode(void)
{
    return current_mode_for_pst;
}
#endif


static void pantech_usb_function_set_enabled(struct android_configuration *conf, char *name)
{
    struct android_usb_function **functions;
    struct android_usb_function *f;
    //tarial for 1013 patch
    struct android_usb_function_holder *f_holder;

    functions = _android_dev->functions;

    printk(KERN_ERR "[PUSB][%s]enabled\n", name);

    if(!strcmp(name, "mass_storage")){
        while((f = *functions++)){
            if(!strcmp(f->name, "mass_storage")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "cdrom")){
        while((f = *functions++)){
            if(!strcmp(f->name, "mass_storage")){
                if(!f->current_enabled){
                    f->current_enabled = true;
                    f_holder = kzalloc(sizeof(*f_holder),
                                       GFP_KERNEL);
                    if (!f_holder) {
                        pr_err("Failed to alloc f_holder\n");
                        return;
                    }

                    f->android_dev = _android_dev;
                    f_holder->f = f;
                    list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);

                }
                return;
            }
        }
    }

    if(!strcmp(name, "eth")){
        while((f = *functions++)){
            if(!strcmp(f->name, "usbnet")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "mtp")){
        while((f = *functions++)){
            if(!strcmp(f->name, "mtp")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "ptp")){
        while((f = *functions++)){
            if(!strcmp(f->name, "ptp")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "accessory")){
        while((f = *functions++)){
            if(!strcmp(f->name, "accessory")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "adb")){
        while((f = *functions++)){
            if(!strcmp(f->name, "adb") || !strcmp(f->name, "ffs")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "rndis")){
        while((f = *functions++)){
            if(!strcmp(f->name, "rndis")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "serial")){
        while((f = *functions++)){
            if(!strcmp(f->name, "serial")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "diag")){
        while((f = *functions++)){
            if(!strcmp(f->name, "diag")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "rmnet")){
        while((f = *functions++)){
            if(!strcmp(f->name, "rmnet")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "obex")){
        while((f = *functions++)){
            if(!strcmp(f->name, "obex")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "audio_source")){
        while((f = *functions++)){
            if(!strcmp(f->name, "audio_source")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);						
                return;
            }
        }
    }

    if(!strcmp(name, "qdss")){
        while((f = *functions++)){
            if(!strcmp(f->name, "qdss")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                isQdssEnable = true;
                return;
            }
        }
    }

    if(!strcmp(name, "ncm")){
        while((f = *functions++)){
            if(!strcmp(f->name, "ncm")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "ecm")){
        while((f = *functions++)){
            if(!strcmp(f->name, "ecm")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

    if(!strcmp(name, "charging")){
        while((f = *functions++)){
            if(!strcmp(f->name, "charging")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }
	    if(!strcmp(name, "midi")){
        while((f = *functions++)){
            if(!strcmp(f->name, "midi")){
                f->current_enabled = true;
                f_holder = kzalloc(sizeof(*f_holder),
                                   GFP_KERNEL);
                if (!f_holder) {
                    pr_err("Failed to alloc f_holder\n");
                    return;
                }

                f->android_dev = _android_dev;
                f_holder->f = f;
                list_add_tail(&f_holder->enabled_list, &conf->enabled_functions);
                return;
            }
        }
    }

}

static int enable_android_usb_product_function(char *device_name, int cnt)
{
    struct android_usb_function **functions;
    struct android_usb_function *f;
    char *name, *b;
    char buffer[MAX_BUFFER_SIZE];
    struct list_head *curr_conf;
    struct android_configuration *conf;
    char *conf_str;
    struct android_usb_function_holder *f_holder;

    if(!_android_dev) return -1;

    curr_conf = &_android_dev->configs;

    list_for_each_entry(conf, &_android_dev->configs, list_item) {	
        while (conf->enabled_functions.next !=
               &conf->enabled_functions) {
            f_holder = list_entry(conf->enabled_functions.next,
                                  typeof(*f_holder),
                                  enabled_list);
            f_holder->f->android_dev = NULL;
            list_del(&f_holder->enabled_list);
            kfree(f_holder);
        }
        INIT_LIST_HEAD(&conf->enabled_functions);
    }

    //LS2_USB Initalize QDSS flag
    isQdssEnable = false;
#ifdef CONFIG_ANDROID_PANTECH_USB_CDFREE
    pantech_set_cdrom_enabled(0, 0);
#endif
    functions = _android_dev->functions;
    while((f = *functions++)) {
        f->current_enabled = false;
        f->b_activated = false;
    }

    memset(buffer, 0x00,MAX_BUFFER_SIZE);
    memcpy(buffer, device_name, strlen(device_name));

    b = buffer;
    while (b) {
        conf_str = strsep(&b, ":");
        if (conf_str) {
            /* If the next not equal to the head , take it */
            if (curr_conf->next != &_android_dev->configs)
                conf = list_entry(curr_conf->next,
                                  struct android_configuration,
                                  list_item);
            else
                conf = alloc_android_config(_android_dev);

            curr_conf = curr_conf->next;
        }

        while (conf_str) {
            name = strsep(&conf_str, ",");
            if (name) {
                pantech_usb_function_set_enabled(conf, name);
            }
        }
    }		

    /* Free uneeded configurations if exists */
    while (curr_conf->next != &_android_dev->configs) {
        conf = list_entry(curr_conf->next,
                          struct android_configuration, list_item);
        free_android_config(_android_dev, conf);
    }
    return 0;
}

#ifdef CONFIG_PANTECH_PMIC_ABNORMAL
static int set_charger_type_data_cable(bool type)
{ 
    static bool b_noti_charger_data_cable = false;

    if(b_noti_charger_data_cable == type)
        return 0;

    b_noti_charger_data_cable = type;
    qpnp_chg_notify_charger_type(type);
    printk(KERN_ERR "[PUSB]%s - b_noti_charger(%d)\n", __func__, b_noti_charger_data_cable);

    return 1; 
}
#endif

static void send_usb_event(struct android_dev *dev, int ev_type)
{
    char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
    char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
    char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };

    if(!dev || (current_usb_status == ev_type) || ((current_usb_status == 2) && (ev_type == 1))){
        printk(KERN_ERR "%s: did not send uevent. current_usb_status[%d], ev_type[%d] \n", __func__, current_usb_status, ev_type);
        return;
    }

    if(ev_type == 0){
        kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, disconnected);
        current_usb_status = 0;
        printk(KERN_ERR "sent uevent [%s]\n", disconnected[0]);
    }else if(ev_type == 1){
        kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, connected);
        current_usb_status = 1;
        printk(KERN_ERR "sent uevent [%s]\n", connected[0]);
    }else if(ev_type == 2){
        kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, configured);
        current_usb_status = 2;
        printk(KERN_ERR "sent uevent [%s]\n", configured[0]);
    }

#ifdef CONFIG_PANTECH_PMIC_ABNORMAL
    if((ev_type == 1) || (ev_type == 2)){
        set_charger_type_data_cable(true);
    }
#endif
}

void pantech_init_device_mode_change(void)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if(!dev_mode_change) return;
    dev_mode_change->usb_device_cfg_flag = 0;
    dev_mode_change->usb_get_desc_flag = 0;
    dev_mode_change->usb_data_transfer_flag = 0;
    dev_mode_change->pc_mode_switch_flag = 0;
    dev_mode_change->usb_reconnect_flag = 0;
}

static void force_reenumeration(struct android_dev *dev, int dev_type)
{
    int vid, pid, i;
    //int gcnum;

    /* using other namespace ??? */

    get_device_pid_vid(dev_type, &pid, &vid);
    device_desc.idProduct = __constant_cpu_to_le16(pid);
    device_desc.idVendor = __constant_cpu_to_le16(vid);

    i = get_func_thru_type(dev_type);
    if (i >= 0) {
        device_desc.bDeviceClass = mode_android_vid_pid[i].class;
        device_desc.bDeviceSubClass = mode_android_vid_pid[i].subclass;
        device_desc.bDeviceProtocol = mode_android_vid_pid[i].protocol;

        if(mode_android_vid_pid[i].type & (MTP_TYPE_FLAG)){
            b_support_ms_os_descriptor = 1;
#ifdef FEATURE_PANTECH_MS_OS_COMPATIBLE
            get_ms_ext_config_desc(mode_android_vid_pid[i].type);
#endif
        }else if(mode_android_vid_pid[i].type & (PTP_TYPE_FLAG)){
            b_support_ms_os_descriptor = 2;
#ifdef FEATURE_PANTECH_MS_OS_COMPATIBLE
            get_ms_ext_config_desc(mode_android_vid_pid[i].type);
#endif
        }else{
            b_support_ms_os_descriptor = 0;
        }
    }else{
        device_desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
        device_desc.bDeviceSubClass = 0x00;
        device_desc.bDeviceProtocol = 0x00;
    }

    /*this code is changed because usb_gadget_controller_number is not defined.
      gcnum = usb_gadget_controller_number(dev->cdev->gadget);
      device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum); */
    device_desc.bcdDevice = cpu_to_le16(0x0200 + 0x90 + b_support_ms_os_descriptor);

    if (dev->cdev) {
        dev->cdev->desc.idProduct = device_desc.idProduct;
        dev->cdev->desc.idVendor = device_desc.idVendor;
        dev->cdev->desc.bcdDevice = device_desc.bcdDevice;
        dev->cdev->desc.bDeviceClass = device_desc.bDeviceClass;
        dev->cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
        dev->cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
    }
}

// sysfs operation for mode switching 
static ssize_t usb_mode_control_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct device_mode_change_dev *dev_mode_change = _device_mode_change_dev;
    int ret;
    char *none="none";

    if(dev_mode_change && dev_mode_change -> g_device_type != 0 &&
       ((ret = get_func_thru_type(dev_mode_change->g_device_type)) >= 0))
    {
        return sprintf(buf, "%s\n", mode_android_vid_pid[ret].name);
    }else{
        return sprintf(buf, "%s\n", none);
    }
}

static ssize_t usb_mode_control_store(struct device *dev, struct device_attribute *attr, const char *buff, size_t size)
{
    char buf[256], *b;

    strlcpy(buf, buff, sizeof(buf));
    b = strim(buf);
    android_enable_function_cb(b);
    return size;
}

static DEVICE_ATTR(usb_mode_control, S_IRUGO | S_IWUSR, usb_mode_control_show, usb_mode_control_store);

static ssize_t usb_manager_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", pantech_usb_mode_list[current_mode_for_pst]);
}

static ssize_t usb_manager_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    /* This function is executed by User set or usb_manage daemon set directly.
     * execution process like below 
     * 1. receive mode string by defined f_pantech_android.h
     * 2. parse mode for mode switch
     * 3. save current mode
     * 4. send to request usb_manager daemon
     */

    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;
    int i;

    /* Remove a trailing new line */
    if (size > 0 && buf[size-1] == '\n')
        ((char *) buf)[size-1] = 0;

    for (i = 0; i < MAX_USB_TYPE_NUM; i++) {
        if (pantech_usb_mode_list[i] == NULL) {
            printk(KERN_ERR "[PUSB]%s - USB mode list Found \n", __func__);
            return -EINVAL;
        }
        if (strcmp(buf, pantech_usb_mode_list[i]) == 0) { 
            break;
        }
    }

    if (i == MAX_USB_TYPE_NUM) {
        printk(KERN_ERR "[PUSB]%s - No Matching Function Found \n", __func__);
        return -EINVAL;
    }	

    current_mode_for_pst = i;
    dev_mode_change->pst_req_mode_switch_flag = 1;
    wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
    printk(KERN_ERR "[PUSB][%s]name[%s]\n", __func__, buf);

    return size;
}

static DEVICE_ATTR(usb_manager, S_IRUGO | S_IWUSR, usb_manager_show, usb_manager_store);

static ssize_t usb_charging_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    /* This function is executed by User set or usb_manage daemon set directly.
     * execution process like below 
     * 1. receive mode string by defined f_pantech_android.h
     * 2. parse mode for mode switch
     * 3. save current mode
     * 4. send to request usb_manager daemon
     */

    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    /* Remove a tailing new line */
    if (size > 0 && buf[size-1] == '\n')
        ((char *) buf)[size-1] = 0;

    if(!strcmp(buf, "serial,diag")) {
        dev_mode_change->g_device_type = (ACM_TYPE_FLAG | DIAG_TYPE_FLAG);
    } else if(!strcmp(buf, "adb")) {
        dev_mode_change->g_device_type = ADB_TYPE_FLAG;
    }

    force_reenumeration(_android_dev, dev_mode_change->g_device_type);

    return size;
}

static DEVICE_ATTR(usb_charging_mode, S_IWUSR, NULL, usb_charging_mode_store);


#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
static ssize_t factory_mode_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value;
	if(b_factory_mode_request)
		value = 1;
	else
		value = 0;

	return sprintf(buf, "%d", value);
}
static DEVICE_ATTR(factory_mode_state, S_IRUGO, factory_mode_state_show, NULL);

static ssize_t factory_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int value;
    if(b_factory_mode_enabled)
        value = 1;
    else
        value = 0;

    return sprintf(buf, "%d", value);
}

static ssize_t factory_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int value;

    sscanf(buf, "%d", &value);
    if(value){
        b_factory_mode_enabled = true;
    }else{
        b_factory_mode_enabled = false;
    }
    return size;
}

static DEVICE_ATTR(factory_mode, S_IRUGO | S_IWUSR, factory_mode_show, factory_mode_store);
static ssize_t factory_adb_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int value;
    if(adb_enable_access())
        value = 1;
    else
        value = 0;

    return sprintf(buf, "%d", value);
}

static ssize_t factory_adb_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int value;

    sscanf(buf, "%d", &value);
    if(value){
        b_factory_mode_adb_onoff = true;
    }else{
        b_factory_mode_adb_onoff = false;
    }
    schedule_delayed_work(&factory_adb_work,msecs_to_jiffies(10));
    return size;
}

static DEVICE_ATTR(factory_adb_mode, S_IRUGO | S_IWUSR, factory_adb_mode_show, factory_adb_mode_store);
#endif

#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
static ssize_t usb_mdm_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int value;
    if(mdm_mode_enabled)
        value = 1;
    else
        value = 0;

    return sprintf(buf, "%d", value);
}
static ssize_t usb_mdm_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;
    int value;
    bool b_changed = false;
    sscanf(buf, "%d", &value);
    mdm_mode_initialized = true;
    b_changed = (mdm_mode_enabled != !!value)? true:false;
    mdm_mode_enabled = !!value;
    if(mdm_mode_callback)
        mdm_mode_callback(!!value);
    if(b_changed)
    {
        if(!dev_mode_change || !dev_mode_change->usb_manager_on) 
            return size;
        dev_mode_change->usb_mdm_mode_flag = 1;
        wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
        printk(KERN_ERR "[PUSB][%s] value[%d] mdm_mode_enabled = [%d] \n",__func__, value, mdm_mode_enabled);
    }
    return size;
}
static DEVICE_ATTR(usb_mdm_mode, S_IRUGO | S_IWUSR, usb_mdm_mode_show, usb_mdm_mode_store);
#endif

static ssize_t usb3_configuration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d", usb3_state_value);
}
static DEVICE_ATTR(usb3_configuration, S_IRUGO, usb3_configuration_show, NULL);

#ifdef CONFIG_PANTECH_USB_TUNE_SIGNALING_PARAM
extern ssize_t pantech_get_slave_tune_param(char *buf);

static ssize_t pantech_slave_tune_param_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return pantech_get_slave_tune_param(buf);
}
static DEVICE_ATTR(slave_tune_param, S_IRUGO, pantech_slave_tune_param_show, NULL);
#endif

static struct attribute *android_pantech_usb_control_attrs[] = {
    &dev_attr_usb_mode_control.attr,
    &dev_attr_usb_manager.attr,
#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
    &dev_attr_factory_mode_state.attr,
    &dev_attr_factory_mode.attr,
    &dev_attr_factory_adb_mode.attr,
#endif
#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
    &dev_attr_usb_mdm_mode.attr,
#endif
    &dev_attr_usb_charging_mode.attr,
    //LS2_USB tarial USB3 detection test code
    &dev_attr_usb3_configuration.attr,
#ifdef CONFIG_PANTECH_USB_TUNE_SIGNALING_PARAM
    &dev_attr_slave_tune_param.attr,
#endif
    NULL,
};

static struct attribute_group android_pantech_usb_control_attr_grp = {
    .attrs = android_pantech_usb_control_attrs,
};

/*
 * Device is used for USB mode switch
 * misc file operation for mode switching
 */
static int device_mode_change_open(struct inode *ip, struct file *fp)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if (atomic_inc_return(&dev_mode_change->device_mode_change_excl) !=
        1) {
        atomic_dec(&dev_mode_change->device_mode_change_excl);
        return -EBUSY;
    }
    b_pantech_usb_module = true;
    mode_android_vid_pid = pantech_android_vid_pid[pantech_usb_carrier];
    return 0;
}

static int device_mode_change_release(struct inode *ip, struct file *fp)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    dev_mode_change->g_device_type = 0;	// when usb_manager killed, init device_type
    if(atomic_read(&dev_mode_change->device_mode_change_excl)){
        atomic_dec(&dev_mode_change->device_mode_change_excl);
    }

    wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
    b_pantech_usb_module = false;
    return 0;
}

void pantech_android_set_config(struct android_configuration *a_config)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;
    struct android_usb_function_holder *f_holder;
    int type;

    dev_mode_change->g_device_type  = 0;

    list_for_each_entry(f_holder, &a_config->enabled_functions, enabled_list) {
        if(f_holder){
            type = get_type_of_str((const char*)f_holder->f->name); 
#ifdef CONFIG_ANDROID_PANTECH_USB_CDFREE
            if(type == MSC_TYPE_FLAG){
                if(pantech_cdrom_enabled){
                    if(pantech_cdrom_only){
                        type = CDROM_TYPE_FLAG;
                    }else{
                        type = (MSC_TYPE_FLAG | CDROM_TYPE_FLAG);
                    }
                }
            }
#endif
            dev_mode_change->g_device_type |= type;
            f_holder->f->b_activated = true;
#if defined(CONFIG_ANDROID_PANTECH_USB_MANAGER) && defined(FEATURE_PANTECH_MULTI_ETHERNET)
            if((type == RNDIS_TYPE_FLAG) || (type == ECM_TYPE_FLAG) || (type == NCM_TYPE_FLAG))
                setup_net_f = f_holder->f;
#endif
        }
    }

    printk(KERN_ERR "[PUSB][%s] is called[0x%x]\n", __func__, dev_mode_change->g_device_type);
}

static ssize_t device_mode_change_write(struct file *file, const char __user * buffer,
                                        size_t count, loff_t *ppos)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;
    unsigned char cmd[MAX_BUFFER_SIZE + 1];
    int cnt = MAX_BUFFER_SIZE;
    int i, temp_device_type;
    int ret, result;
    struct android_configuration *conf;
    struct android_usb_function_holder *f_holder;
    bool audio_enabled = false;
    char *b;
    int len;

    if(!_android_dev){
        printk(KERN_ERR "[PUSB]%s - _android_dev not iniltialized \n", __func__);
        result = -EIO;
        goto _jump;
    }
    if (count <= 0) {
        printk(KERN_ERR "[PUSB]%s - buffer size is 0 \n", __func__);
        result =  -EFAULT;
        goto _jump;
    }

    if (cnt > count)
        cnt = count;

    if (copy_from_user(cmd, buffer, cnt)) {
        printk(KERN_ERR "[PUSB]%s -  Error Copying buffer \n", __func__);
        result =  -EFAULT;
        goto _jump;
    }

    cmd[cnt] = 0;

    if((cnt == 10) && strncmp(cmd, "manager_on", 10) == 0){
        printk(KERN_ERR "[PUSB]%s manager_on\n", __func__);
        dev_mode_change->usb_manager_on = 1;
        return count;
    }

    printk(KERN_ERR "[PUSB]%s Mode change command=[%s]\n", __func__, cmd);

#if 0
//p16102
    if(strncmp(cmd, "[LOG]", 5) == 0){
        printk(KERN_ERR "%s\n", cmd);
        return count;
    }
#endif

    if((cnt == 12) && strncmp(cmd, "disconnected", 12) == 0){
        send_usb_event(_android_dev, 0);	
        return count;
    }
    if((cnt == 9) && strncmp(cmd, "connected", 9) == 0){
        send_usb_event(_android_dev, 1);	
        return count;
    }
    if((cnt == 10) && strncmp(cmd, "configured", 10) == 0){
        send_usb_event(_android_dev, 2);
#ifdef CONFIG_ANDROID_PANTECH_USB_ABNORMAL_CHARGER_INFO
		set_pmic_usb_configured();
#endif
        return count;
    }

    if(strncmp(cmd, "carrier_mode", 12) == 0){
        char *carrier = &cmd[13];/* carrier_mode:kor*/
        int index = -1;

        if(carrier != NULL 
           && strncmp(carrier, pantech_android_carrier_list[pantech_usb_carrier],
                      strlen(pantech_android_carrier_list[pantech_usb_carrier])-1) ){
            for(i = 0; i < (int)CARRIER_TYPE_MAX; i++){
                if(strncmp(carrier, pantech_android_carrier_list[i], strlen(pantech_android_carrier_list[i])-1) == 0){
                    index = i; 
                    break;
                }
            }
            if(index >= 0) pantech_usb_carrier = index;
            mode_android_vid_pid = pantech_android_vid_pid[pantech_usb_carrier];
        }

        printk(KERN_ERR "[PUSB]%s - carrier_mode [%d]\n", __func__,pantech_usb_carrier);
        return count;
    }

    for(i = 0; i < MAX_USB_TYPE_NUM; i++) {
        if(!strcmp(cmd, pantech_usb_mode_list[i])){
            current_mode_for_pst = i;
            return count;
        }
    }

    /* USB cable detached Command */
    if (strncmp(cmd, "usb_cable_detach", 16) == 0) {
        dev_mode_change->usb_data_transfer_flag = 0;
        dev_mode_change->g_device_type = 0;
        dev_mode_change->usb_device_cfg_flag = 0;
        dev_mode_change->usb_get_desc_flag = 0;
        if(_android_dev->enabled){
            android_disable(_android_dev);
            list_for_each_entry(conf, &_android_dev->configs, list_item)
                list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                    if (f_holder->f->disable)
                        f_holder->f->disable(f_holder->f);
                }			
            _android_dev->enabled = false;
        }
        /*
         * Set the composite switch to 0 during a disconnect.
         * This is required to handle a few corner cases, where
         * enumeration has not completed, but the cable is yanked out
         */
        printk(KERN_ERR "[PUSB]%s - Handled Detach\n", __func__);
        return count;
    }

    /* USB connect/disconnect Test Commands */
    if (strncmp(cmd, "gadget_on", 9) == 0) {
        printk(KERN_ERR "[PUSB]%s - Handled Connect[%d]\n", __func__, _android_dev->enabled);
        mutex_lock(&_android_dev->mutex);
        if(!_android_dev->enabled){
            _android_dev->cdev->desc.idVendor = device_desc.idVendor;
            _android_dev->cdev->desc.idProduct = device_desc.idProduct;
            _android_dev->cdev->desc.bcdDevice = device_desc.bcdDevice;
            _android_dev->cdev->desc.bDeviceClass = device_desc.bDeviceClass;
            _android_dev->cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
            _android_dev->cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
#ifdef CONFIG_PANTECH_USB_VER_SWITCH
            /* initialize usb 2.0 and modified by enumeration step */
            _android_dev->cdev->desc.bcdUSB = cpu_to_le16(0x0200);
#endif

            list_for_each_entry(conf, &_android_dev->configs, list_item){
                conf->usb_config.iConfiguration = configuration_id;
                list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                    if (f_holder->f->enable)
                        f_holder->f->enable(f_holder->f);
                    /* Audio dock accessory is unable to enumerate device if
                     * pull-up is enabled immediately. The enumeration is
                     * reliable with 100 msec delay.
                     */
                    if(!strncmp(f_holder->f->name,"audio_source", 12))
                        audio_enabled = true;
                }
            }

            if (audio_enabled)
                msleep(100);
            android_enable(_android_dev);
            _android_dev->enabled = true;

        }
        mutex_unlock(&_android_dev->mutex);
        return count;
    }

    if (strncmp(cmd, "gadget_off", 10) == 0) {
        printk(KERN_ERR "[PUSB]%s - Handled disconnect[%d]\n", __func__, _android_dev->enabled);
        mutex_lock(&_android_dev->mutex);
        if(_android_dev->enabled){
            android_disable(_android_dev);
            list_for_each_entry(conf, &_android_dev->configs, list_item)
                list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                    if (f_holder->f->disable)
                        f_holder->f->disable(f_holder->f);
                }
            _android_dev->enabled = false;
        }
        mutex_unlock(&_android_dev->mutex);
        return count;
    }

    /*  only check about first configuration */
    b = strchr(cmd, ':');
    if(b)
        len = cnt - strlen(b);
    else
        len = cnt;

    for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
        if (mode_android_vid_pid[i].name == NULL) {
            printk(KERN_ERR "[PUSB]%s - Function Not Found \n" ,
                   __func__);
            result = count;
            goto _jump;
        }
        if (strlen(mode_android_vid_pid[i].name) != len)
            continue;
        if (strncmp(cmd, mode_android_vid_pid[i].name, len) == 0) {
            temp_device_type = mode_android_vid_pid[i].type;
            strings_dev[STRING_CONFIG_IDX].s =
                mode_android_vid_pid[i].config_name;
            break;
        }
    }

    if (i == MAX_DEVICE_TYPE_NUM) {
        printk(KERN_ERR "[PUSB]%s - No Matching Function Found \n" ,
               __func__);
        result = count;
        goto _jump;
    }

    printk(KERN_ERR "[PUSB]g_device_type[0x%x] == device_cfg_flag[0x%x], temp[0x%x]\n", 
           dev_mode_change->g_device_type, dev_mode_change->usb_device_cfg_flag, temp_device_type);

    if(dev_mode_change->g_device_type == 0){
        dev_mode_change->g_device_type = dev_mode_change->usb_device_cfg_flag;
    }

    if (temp_device_type == dev_mode_change->g_device_type) {
        result = count;
        goto _jump;
    }

    printk(KERN_ERR "[PUSB]%s - enable cmd : %s, cnt : %d, android_dev enabled[%d]\n", __func__, cmd, cnt, _android_dev->enabled);

    /****** start : force reenumeration  */
    if(_android_dev->enabled){
        android_disable(_android_dev);
        list_for_each_entry(conf, &_android_dev->configs, list_item)
            list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                if (f_holder->f->disable)
                    f_holder->f->disable(f_holder->f);
            }
    }

    ret = enable_android_usb_product_function(cmd, cnt);
    if (ret != 0) {
        printk(KERN_ERR "[PUSB]%s - Error Enabling Function \n" ,
               __func__);
        result = -EFAULT;
        goto _jump;
    }

    /* initialize adb flag */
    if((temp_device_type & ADB_TYPE_FLAG) == ADB_TYPE_FLAG){
        atomic_set(&adb_enable_excl, 1);
    }else{
        atomic_set(&adb_enable_excl, 0);
    }
#ifdef CONFIG_ANDROID_PANTECH_USB_CDFREE
    if((temp_device_type & CDROM_TYPE_FLAG) == CDROM_TYPE_FLAG){
        if((temp_device_type & MSC_TYPE_FLAG) == MSC_TYPE_FLAG){
            pantech_set_cdrom_enabled(1, 0); // ums + cdrom
        }else{
            pantech_set_cdrom_enabled(1, 1); // cdrom only
        }
    }
#endif

    dev_mode_change->g_device_type = temp_device_type;

    force_reenumeration(_android_dev, dev_mode_change->g_device_type);

    if(_android_dev->enabled){
        _android_dev->cdev->desc.idVendor = device_desc.idVendor;
        _android_dev->cdev->desc.idProduct = device_desc.idProduct;
        _android_dev->cdev->desc.bcdDevice = device_desc.bcdDevice;
        _android_dev->cdev->desc.bDeviceClass = device_desc.bDeviceClass;
        _android_dev->cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
        _android_dev->cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
#ifdef CONFIG_PANTECH_USB_VER_SWITCH
        /* initialize usb 2.0 and modified by enumeration step */
        _android_dev->cdev->desc.bcdUSB = cpu_to_le16(0x0200);
#endif

        list_for_each_entry(conf, &_android_dev->configs, list_item)
            list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                if (f_holder->f->enable)
                    f_holder->f->enable(f_holder->f);
            }
        android_enable(_android_dev);
    }
    /****** end : force reenumeration */

    printk(KERN_ERR "[PUSB]%s - Successfully enabled function - %s \n",
           __func__, cmd);

    result = count;

_jump:
    if(dev_mode_change->android_enabled_function_flag == 0){
        dev_mode_change->android_enabled_function_flag = 1;
        wake_up_interruptible(&dev_mode_change->android_enable_function_wq);
    }

    /* usb manager state enabled check for vBus session*/
    if(usb_manager_state_enabled == 0){
        usb_manager_state_enabled = 1;
    }

    return result;
}

static int event_pending(void)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if ((dev_mode_change->usb_device_cfg_flag >=
         dev_mode_change->g_device_type)
        && (dev_mode_change->g_device_type != 0))
        return 1;
    else if (dev_mode_change->usb_get_desc_flag)
        return 1;
    else if (dev_mode_change->pc_mode_switch_flag != NULL)
        return 1;
    else if (dev_mode_change->pst_req_mode_switch_flag)
        return 1;
    else if (dev_mode_change->usb_state_get_flag)
        return 1;	
#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
    else if (dev_mode_change->usb_mdm_mode_flag)
        return 1;	
#endif
    else if (!dev_mode_change->android_enabled_function_flag)
        return 1;
    else
        return 0;
}

static unsigned int device_mode_change_poll(struct file *file,
                                            struct poll_table_struct *wait)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

    if(dev_mode_change->android_enabled_function_flag == 0){
        dev_mode_change->android_enabled_function_flag = 1;
        wake_up_interruptible(&dev_mode_change->android_enable_function_wq);
    }

#if !defined (CONFIG_PANTECH_USER_BUILD)
    printk(KERN_ERR "[PUSB]mode_change_poll request\n");
#endif

    poll_wait(file, &dev_mode_change->device_mode_change_wq, wait);

    if (event_pending())
        return POLLIN | POLLRDNORM;
    else
        return 0;
}

static ssize_t device_mode_change_read(struct file *file, char *buf,
                                       size_t count, loff_t *ppos)
{
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;
    int ret, size, cnt;
    /* double check last zero */
    unsigned char no_changed[] = "none|\0";
    unsigned char usb_disconnected[] = "usb_disconnected|\0";
    unsigned char usb_connected[] = "usb_connected|\0";
    unsigned char usb_reconnect[] = "usb_reconnect|\0";
    unsigned char enumerated_str[] = "enumerated\0";
    unsigned char get_desc_str[] = "get_desc\0";
#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
    unsigned char usb_disable_str[] = "usb_disable|\0";
    unsigned char usb_enable_str[] = "usb_enable|\0";
#endif
    unsigned char modswitch_str[MAX_BUFFER_SIZE];
    unsigned char *carrier_str = NULL;

    /* Message format example:
     * carrier_str:ui_mode_change_str:type_mode_change_str:factory_mode:enumerated
     */

    if (!event_pending())
        return 0;

    /* append carrier type */ 
    carrier_str = pantech_android_carrier_list[pantech_usb_carrier];

    size = strlen(carrier_str);
    ret = copy_to_user(buf, carrier_str, size);
    cnt = size;
    buf += size;

    /* append PST usb mode state */
    if (!dev_mode_change->pst_req_mode_switch_flag) {

        if(dev_mode_change->usb_state_get_flag){
            if(dev_mode_change->usb_state_get_flag == 1){
                size = strlen(usb_connected);
                ret = copy_to_user(buf, usb_connected, size);
            }else{
                size = strlen(usb_disconnected);
                ret = copy_to_user(buf, usb_disconnected, size);
            }
            printk(KERN_ERR "[PUSB]usb_connected : %d\n", dev_mode_change->usb_state_get_flag);	
            dev_mode_change->usb_state_get_flag=0;
        }else{
            size = strlen(no_changed);
            ret = copy_to_user(buf, no_changed, size);
        }	

    }else {
        memset(modswitch_str, 0, MAX_BUFFER_SIZE);
        sprintf(modswitch_str, "%s", pantech_usb_mode_list[current_mode_for_pst]);
        strcat(modswitch_str, "|");
        size = strlen(modswitch_str);
        ret = copy_to_user(buf, modswitch_str, size);
        dev_mode_change->pst_req_mode_switch_flag = 0;
    }
    cnt += size;
    buf += size;

    /* append PC request mode */
    if (dev_mode_change->pc_mode_switch_flag == NULL) {
        size = strlen(no_changed);
        ret = copy_to_user(buf, no_changed, size);
    } else {
        memset(modswitch_str, 0, MAX_BUFFER_SIZE);
        size = strlen(dev_mode_change->pc_mode_switch_flag);
        memcpy(modswitch_str, dev_mode_change->pc_mode_switch_flag, size);
        modswitch_str[size++] = '|';
        ret = copy_to_user(buf, modswitch_str, size);
        dev_mode_change->pc_mode_switch_flag = NULL;
    }
    cnt += size;
    buf += size;	

    /* append USB usb_reconnect state */
    if (dev_mode_change->usb_reconnect_flag) {
        dev_mode_change->usb_reconnect_flag = 0;
        size = strlen(usb_reconnect);
        ret = copy_to_user(buf, usb_reconnect, size);
    } else {
        size = strlen(no_changed);
        ret = copy_to_user(buf, no_changed, size);
    }
    cnt += size;
    buf += size;	

#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
    /* USB MDM state */
    if (dev_mode_change->usb_mdm_mode_flag) {
        if(mdm_mode_enabled){
            size = strlen(usb_disable_str);
            ret += copy_to_user(buf, usb_disable_str, size);
            dev_mode_change->usb_mdm_mode_flag = 0;
            printk(KERN_ERR "[PUSB][%s] usb_mdm_mode_flag = [%d]\n",__func__, mdm_mode_enabled);
        } else {
            size = strlen(usb_enable_str);
            ret += copy_to_user(buf, usb_enable_str, size);
            dev_mode_change->usb_mdm_mode_flag = 0;
            printk(KERN_ERR "[PUSB][%s] usb_mdm_mode_flag = [%d]\n",__func__, mdm_mode_enabled);
        }
    } else {
        size = strlen(no_changed);
        ret = copy_to_user(buf, no_changed, size);
    }
    cnt += size;
    buf += size;
#endif

    /* append USB enumerated state */
    if ((dev_mode_change->usb_device_cfg_flag >=
         dev_mode_change->g_device_type)
        && (dev_mode_change->g_device_type != 0)) {
        dev_mode_change->usb_device_cfg_flag = 0;
        size = strlen(enumerated_str);
        ret += copy_to_user(buf, enumerated_str, size);
    } else {
        if (dev_mode_change->usb_get_desc_flag == 1) {
            dev_mode_change->usb_get_desc_flag = 0;
            size = strlen(get_desc_str);
            ret += copy_to_user(buf, get_desc_str, size);
        } else {			
            size = strlen(no_changed) - 1;			
            ret += copy_to_user(buf, no_changed, size);
        }
    }
    cnt += size;

    return cnt;
}

static const struct file_operations device_mode_change_fops = {
    .owner = THIS_MODULE,
    .open = device_mode_change_open,
    .write = device_mode_change_write,
    .poll = device_mode_change_poll,
    .read = device_mode_change_read,
    .release = device_mode_change_release,
};

static struct miscdevice mode_change_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "usb_device_mode",
    .fops = &device_mode_change_fops,
};

void usb3_state_set(int value)
{
    usb3_state_value = value;
}

#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
int set_factory_mode(bool onoff)
{
    static char *product_string_factory = "Android(FactoryMode)" ;
    struct android_configuration *conf;
    struct android_usb_function_holder *f_holder;

    if(b_factory_mode_request == onoff){
        return 0;
    }

    if(onoff == true){
        printk(KERN_ERR "[PUSB]factory_cable mode enable\n");
        b_factory_mode_request = true;

        if(_android_dev->enabled){
            android_disable(_android_dev);
            list_for_each_entry(conf, &_android_dev->configs, list_item)
                list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                    if (f_holder->f->disable)
                        f_holder->f->disable(f_holder->f);
                }
            _android_dev->enabled = false;
        }

        strings_dev[STRING_PRODUCT_IDX].s = product_string_factory;
        if(adb_enable_access()){
            enable_android_usb_product_function("serial,diag,adb", 0);
            _device_mode_change_dev->g_device_type = (ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG);
            force_reenumeration(_android_dev, _device_mode_change_dev->g_device_type);
        }else{
            enable_android_usb_product_function("serial,diag", 0);
            _device_mode_change_dev->g_device_type = (ACM_TYPE_FLAG | DIAG_TYPE_FLAG);
            force_reenumeration(_android_dev, _device_mode_change_dev->g_device_type);
        }

        msleep(500);
        list_for_each_entry(conf, &_android_dev->configs, list_item)
            list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                if (f_holder->f->enable)
                    f_holder->f->enable(f_holder->f);
            }
        android_enable(_android_dev);
        _android_dev->enabled = true;

    }else{
        cancel_delayed_work(&factory_work);
        b_factory_mode_request = false;
        printk(KERN_ERR "[PUSB]factory_cable mode disable\n");
        strings_dev[STRING_PRODUCT_IDX].s = product_string;
    }

    return 1;
}

static void pantech_factory_work(struct work_struct *data)
{
    set_factory_mode(true);
}

int set_factory_adb_mode(int onoff)
{
    static char *product_string_factory = "Android(FactoryMode)" ;
    struct android_configuration *conf;
    struct android_usb_function_holder *f_holder;

    printk(KERN_ERR "[PUSB]factory_adb_mode = %d\n", adb_enable_access());

    if(adb_enable_access() == onoff)
        return 0;
    atomic_set(&adb_enable_excl, onoff);

    if(onoff == 1){
        if(_android_dev->enabled){
            android_disable(_android_dev);
            list_for_each_entry(conf, &_android_dev->configs, list_item)
                list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                    if (f_holder->f->disable)
                        f_holder->f->disable(f_holder->f);
                }
            _android_dev->enabled = false;
        }
        strings_dev[STRING_PRODUCT_IDX].s = product_string_factory;
        enable_android_usb_product_function("serial,diag,adb", 0);
        _device_mode_change_dev->g_device_type = (ACM_TYPE_FLAG | DIAG_TYPE_FLAG | ADB_TYPE_FLAG);
        force_reenumeration(_android_dev, _device_mode_change_dev->g_device_type);
    }else{
        if(_android_dev->enabled){
            android_disable(_android_dev);
            list_for_each_entry(conf, &_android_dev->configs, list_item)
                list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
                    if (f_holder->f->disable)
                        f_holder->f->disable(f_holder->f);
                }
            _android_dev->enabled = false;
        }
        strings_dev[STRING_PRODUCT_IDX].s = product_string_factory;
        enable_android_usb_product_function("serial,diag", 0);
        _device_mode_change_dev->g_device_type = (ACM_TYPE_FLAG | DIAG_TYPE_FLAG);
        force_reenumeration(_android_dev, _device_mode_change_dev->g_device_type);
    }

    msleep(500);
    list_for_each_entry(conf, &_android_dev->configs, list_item)
        list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
            if (f_holder->f->enable)
                f_holder->f->enable(f_holder->f);
        }
    android_enable(_android_dev);
    _android_dev->enabled = true;

    return 1;
}

static void pantech_factory_adb_work(struct work_struct *data)
{
    set_factory_adb_mode(b_factory_mode_adb_onoff);
}
#endif

static int pantech_mode_ctrlrequest(struct usb_composite_dev *cdev,
                                    const struct usb_ctrlrequest *ctrl)
{
    int value = -EOPNOTSUPP;
    u16 wIndex = le16_to_cpu(ctrl->wIndex);
    u16 wValue = le16_to_cpu(ctrl->wValue);
    u16 wLength = le16_to_cpu(ctrl->wLength);
    struct usb_request *req = cdev->req;
    struct android_configuration *conf;
#ifdef FEATURE_PANTECH_MS_OS_COMPATIBLE
    struct device_mode_change_dev *dev_mode_change = _device_mode_change_dev;
#endif

    if((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR){
        if((ctrl->bRequest == 0x70) && (wValue == 1) && (wLength == 0) && (wIndex == 0)){
#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
            if(!b_factory_mode_enabled)
                return value;
            schedule_delayed_work(&factory_work,msecs_to_jiffies(10));
            value = 0;
            if (value >= 0) {
                int rc;
                req->zero = value < wLength;
                req->length = value;
                rc = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
                if (rc < 0)
                    printk(KERN_ERR "[PUSB]%s setup response queue error[%d]\n", __func__, rc);
            }
#endif
        }else if (ctrl->bRequest == 1 && (ctrl->bRequestType & USB_DIR_IN) && (wIndex == 4 || wIndex == 5)) {
#ifdef FEATURE_PANTECH_MS_OS_COMPATIBLE
            if(dev_mode_change && (dev_mode_change->g_device_type & (MTP_TYPE_FLAG | PTP_TYPE_FLAG))){
                if(p_pantech_ext_config_desc){
                    value = (wLength < p_pantech_ext_config_desc->desc.header.dwLength ?
                             wLength : p_pantech_ext_config_desc->desc.header.dwLength);
                    memcpy(cdev->req->buf, &p_pantech_ext_config_desc->desc, value);
                }else{
                    value = (wLength < pantech_ext_config_desc_list[0].desc.header.dwLength ?
                             wLength : pantech_ext_config_desc_list[0].desc.header.dwLength);
                    memcpy(cdev->req->buf, &pantech_ext_config_desc_list[0].desc, value);
                }
                if (value >= 0) {
                    int rc;
                    req->zero = value < wLength;
                    req->length = value;
                    rc = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
                    if (rc < 0)
                        printk(KERN_ERR "[PUSB]%s setup response queue error[%d]\n", __func__, rc);
                }
            }
#endif
        }else{ }

    }else if(ctrl->bRequest == USB_REQ_GET_DESCRIPTOR){
        usb_data_transfer_callback();
    }else if(ctrl->bRequest == USB_REQ_SET_CONFIGURATION){
        list_for_each_entry(conf, &_android_dev->configs, list_item){
            if(conf->usb_config.bConfigurationValue == wValue){
                pantech_android_set_config(conf);	
                break;
            }
        }
    }

    return value;
}

int pantech_vbus_connect(void)
{
#ifdef CONFIG_PANTECH_PMIC_ABNORMAL
    set_charger_type_data_cable(false);
#endif
    usb_connect_cb();
    return 1;
}

int pantech_vbus_disconnect(void)
{
    int ret = 0;
    struct device_mode_change_dev *dev_mode_change =
        _device_mode_change_dev;

#ifdef CONFIG_PANTECH_PMIC_ABNORMAL
    set_charger_type_data_cable(false);
#endif

#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
    ret = set_factory_mode(false);
#endif

    if(current_usb_status	!= 0){
        dev_mode_change->usb_state_get_flag = -1;
        dev_mode_change->usb_data_transfer_flag = 0;
    }

    if(ret){
        dev_mode_change->pc_mode_switch_flag = get_name_thru_type(DEFAULT_TYPE_FLAG);
        dev_mode_change->usb_reconnect_flag = 1;
    }
    wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
    printk(KERN_ERR "[PUSB][%s]type_mode[0x%x]\n",__func__, DEFAULT_TYPE_FLAG);

    return 1;
}


/* USBNET */ 
#if defined(CONFIG_PANTECH_VERIZON)
static int usbnet_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
    printk(KERN_ERR "[PUSB]%s usbnet init function!!!\n", __func__);
    return 0;
}

static void usbnet_function_cleanup(struct android_usb_function *f)
{
    printk(KERN_ERR "[PUSB]%s usbnet clean function!!!\n", __func__);
    return;
}

static int usbnet_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
    printk(KERN_ERR "[PUSB]%s!!!\n", __func__);
    usbnet_init_setup();
    return usbnet_bind_config(c);
}

static void usbnet_function_unbind_config(struct android_usb_function *f, struct usb_configuration *c)
{
    printk(KERN_ERR "[PUSB]%s!!!\n", __func__);
    misc_deregister(&usbnet_enable_device);
    usbnet_cleanup();
    return;
}

static int usbnet_function_ctrlrequest(struct android_usb_function *f,
                                       struct usb_composite_dev *cdev,
                                       const struct usb_ctrlrequest *c)
{
    return usbnet_ctrlrequest(cdev, c);
}

static struct android_usb_function usbnet_function = {
    .name		= "usbnet",
    .init		= usbnet_function_init,
    .cleanup	= usbnet_function_cleanup,
    .bind_config	= usbnet_function_bind_config,
    .unbind_config	= usbnet_function_unbind_config,
    .ctrlrequest	= usbnet_function_ctrlrequest,
};
#endif

/* PANTECH OBEX */
static int pantech_obex_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
    pr_debug("%s pantech_obex init function nothing work!!!\n", __func__);
    pantech_obex_setup();
    return 0;
}

static void pantech_obex_function_cleanup(struct android_usb_function *f)
{
    pr_debug("%s pantech_obex cleanup function nothing work!!!\n", __func__);
    pantech_obex_cleanup();
}

static int pantech_obex_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
    return pantech_obex_bind_config(c);
}

static struct android_usb_function pantech_obex_function = {
    .name		= "obex",
    .init		= pantech_obex_function_init,
    .cleanup	= pantech_obex_function_cleanup,
    .bind_config	= pantech_obex_function_bind_config,
};

#ifdef CONFIG_PANTECH_USB_BLOCKING_MDMSTATE
void get_mdm_state(char *mdm_state)
{
    char state[64];

    if(mdm_mode_enabled) {
        memset(state, 0x00, sizeof(state));
        sprintf(state, "USB_DISABLED");
    } else {
        memset(state, 0x00, sizeof(state));
        sprintf(state, "USB_ENABLED");
    }

    memcpy(mdm_state, state, strlen(state));
    pr_debug("[%s] : src=%s, dest=%s\n", __func__, state, mdm_state);
}
EXPORT_SYMBOL(get_mdm_state);

int get_pantech_mdm_state(void)
{	
//    return (mdm_mode_initialized)? mdm_mode_enabled : -EAGAIN;
	if(mdm_mode_initialized){
		printk(KERN_ERR "[PUSB][%s] mdm_mode_initialized!!\n", __func__);
		return mdm_mode_enabled;
	}
	else{
		printk(KERN_ERR "[PUSB][%s] mdm_mode_not_init!!\n", __func__);
		return -EAGAIN;
	}
}
EXPORT_SYMBOL(get_pantech_mdm_state);

void set_pantech_mdm_callback(void *cb)
{
    mdm_mode_callback = cb;
}
EXPORT_SYMBOL(set_pantech_mdm_callback);
#endif

#ifdef	CONFIG_ANDROID_PANTECH_USB_ABNORMAL_CHARGER_INFO
int get_udc_state(char *udc_state)
{
    struct android_dev *dev = _android_dev;
    struct usb_composite_dev *cdev;
    char state[20];
    int	ret = 1;
    unsigned long flags;

    memset(state, 0x00, sizeof(state));
    sprintf(state, "DISCONNECTED");

    if (!dev || !dev->cdev){
        ret = -1;
        goto out;
    }
	cdev = dev->cdev;  

    spin_lock_irqsave(&cdev->lock, flags);
    if (cdev->config) {
        memset(state, 0x00, sizeof(state));
        sprintf(state, "CONFIGURED");
    }
    else if (dev->connected) {
        memset(state, 0x00, sizeof(state));
        sprintf(state, "CONNECTED");
    }
    else {
        memset(state, 0x00, sizeof(state));
        sprintf(state, "DISCONNECTED");
    }

    spin_unlock_irqrestore(&cdev->lock, flags);

    memcpy(udc_state, state, strlen(state));
    pr_debug("%s : src=%s, dest=%s\n", __func__, state, udc_state);
out:
    return ret;
}
EXPORT_SYMBOL(get_udc_state);
#endif

#if defined(CONFIG_ANDROID_PANTECH_USB_MANAGER) && defined(FEATURE_PANTECH_MULTI_ETHERNET)
static struct eth_dev *pantech_gether_setup_name(struct android_usb_function *f,struct usb_gadget *g, u8 ethaddr[ETH_ALEN],
		const char *netname)
{
    if(!pantech_eth_dev)
    {
        pantech_eth_dev = gether_setup_name(g, ethaddr, netname);       
        setup_net_f = f;
        printk(KERN_ERR "[%s]:%s\n", __func__, netname);
    }
    return pantech_eth_dev;
}

static void pantech_gether_cleanup(struct android_usb_function *f,struct eth_dev *dev)
{
    if(!dev)
        return;

    if(pantech_eth_dev && (setup_net_f == f)){
        printk(KERN_ERR "[%s]:%s\n", __func__, pantech_eth_dev->net->name);
        gether_cleanup(pantech_eth_dev);
        pantech_eth_dev = NULL;
        setup_net_f = NULL;
    }
}
#endif

static int pantech_android_bind(struct usb_composite_dev *cdev)
{
    int id;

    id = usb_string_id(cdev);
    if (id < 0)
        return id;

    strings_dev[STRING_CONFIG_IDX].id = id;
    configuration_id = id;
    return 0;
}

static int pantech_android_probe(struct android_dev *dev)
{
    int ret;

    if(!dev){
        printk(KERN_ERR "[PUSB][%s] dev is null\n", __func__);
        return -1;
    }

    ret = sysfs_create_group(&dev->dev->kobj, &android_pantech_usb_control_attr_grp);
    if(ret < 0){
        printk(KERN_ERR "[PUSB][%s]sysfs_create_group error[%d]\n", __func__, ret);
        return -1;
    }

    return 0;
}

static int pantech_android_init(void)
{
    struct device_mode_change_dev *dev_mode_change;
    int ret;

#if defined(CONFIG_PANTECH_VERIZON)
    pantech_usb_carrier = CARRIER_VERIZON;
#elif defined(CONFIG_PANTECH_ATNT)
    pantech_usb_carrier = CARRIER_ATNT;
#elif defined(CONFIG_PANTECH_JAPAN)
    pantech_usb_carrier = CARRIER_JAPAN;
#elif defined(CONFIG_PANTECH_JAPAN_PMC)
    pantech_usb_carrier = CARRIER_JAPAN_PMC;
#else/* domestic */
    pantech_usb_carrier = CARRIER_KOR;
#endif
    mode_android_vid_pid = pantech_android_vid_pid[pantech_usb_carrier];

#ifdef CONFIG_ANDROID_PANTECH_USB_FACTORY_CABLE
    INIT_DELAYED_WORK(&factory_work, pantech_factory_work);
    INIT_DELAYED_WORK(&factory_adb_work, pantech_factory_adb_work);
#endif

    /* allocate device_mode_change dev and wait queue */
    dev_mode_change = kzalloc(sizeof(*dev_mode_change), GFP_KERNEL);
    if(!dev_mode_change){
        kfree(_android_dev);
        return -ENOMEM;
    }
    _device_mode_change_dev = dev_mode_change;
    init_waitqueue_head(&dev_mode_change->device_mode_change_wq);
    init_waitqueue_head(&dev_mode_change->android_enable_function_wq);
    dev_mode_change->pst_req_mode_switch_flag = 0;

    ret = misc_register(&mode_change_device);
    if(ret){
        printk(KERN_ERR "[PUSB][%s]:misc_register error[%d]\n", __func__, ret);
        return -1;
    }
    return 0;
}

static int pantech_android_cleanup(void)
{
    misc_deregister(&mode_change_device);
    kfree(_device_mode_change_dev);
    _device_mode_change_dev = NULL;
    return 0;
}

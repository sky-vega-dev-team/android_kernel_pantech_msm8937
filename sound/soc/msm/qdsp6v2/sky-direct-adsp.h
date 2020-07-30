/************************************************************************************************
**
**    DIRECT ADSP INTERFACE
**
**    FILE
**        sky-direct-adsp.c
**
**    DESCRIPTION
**        This file contains direct adsp service for SKY.
**
**    Copyright (c) 2012 by PANTECH Incorporated.  All Rights Reserved.
*************************************************************************************************/


/************************************************************************************************
** Includes
*************************************************************************************************/
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>
#include <asm/ioctls.h>

#include "msm-pcm-q6-v2.h"
#include "msm-pcm-routing-v2.h"
#include "q6voice.h"
/**************************************************************************************************/

/************************************************************************************************
** Message Definition
*************************************************************************************************/
//kkc 2012.08.15 - remove logs for SKY Direct ADSP Drviers.
//#define SKY_DIRECT_ADSP_MSG_LEVEL_LOW 
//#define SKY_DIRECT_ADSP_MSG_LEVEL_MED
//#define SKY_DIRECT_ADSP_MSG_LEVEL_HIGH

#ifdef SKY_DIRECT_ADSP_MSG_LEVEL_LOW
#define SKY_DIRECT_ADSP_DBG_LOW(fmt, arg...) printk(KERN_INFO "%s: " fmt "\n", __func__, ## arg)
#else
#define SKY_DIRECT_ADSP_DBG_LOW(fmt, arg...) do {} while (0)
#endif

#ifdef SKY_DIRECT_ADSP_MSG_LEVEL_MED
#define SKY_DIRECT_ADSP_DBG_MED(fmt, arg...) printk(KERN_INFO "%s: " fmt "\n", __func__, ## arg)
#else
#define SKY_DIRECT_ADSP_DBG_MED(fmt, arg...) do {} while (0)
#endif

#ifdef SKY_DIRECT_ADSP_MSG_LEVEL_HIGH
#define SKY_DIRECT_ADSP_DBG_HIGH(fmt, arg...) printk(KERN_INFO "%s: " fmt "\n", __func__, ## arg)
#else
#define SKY_DIRECT_ADSP_DBG_HIGH(fmt, arg...) do {} while (0)
#endif

#define SKY_DIRECT_ADSP_ERR(fmt, arg...) printk(KERN_ERR "%s: " fmt "\n", __func__, ## arg)
/**************************************************************************************************/

/************************************************************************************************
** Defines - must be synced with "msm-pcm-voip.c"
*************************************************************************************************/
#define VOIP_MAX_Q_LEN 10
#define VOIP_MAX_VOC_PKT_SIZE 4096
#define VOIP_MIN_VOC_PKT_SIZE 320

/* Length of the DSP frame info header added to the voc packet. */
#define DSP_FRAME_HDR_LEN 1

#define MODE_IS127		0x2
#define MODE_4GV_NB		0x3
#define MODE_4GV_WB		0x4
#define MODE_AMR		0x5
#define MODE_AMR_WB		0xD
#define MODE_PCM		0xC
#define MODE_4GV_NW		0xE
#define MODE_G711		0xA
#define MODE_G711A		0xF

enum msm_audio_g711a_frame_type {
	MVS_G711A_SPEECH_GOOD,
	MVS_G711A_SID,
	MVS_G711A_NO_DATA,
	MVS_G711A_ERASURE
};

enum msm_audio_g711a_mode {
	MVS_G711A_MODE_MULAW,
	MVS_G711A_MODE_ALAW
};

enum msm_audio_g711_mode {
	MVS_G711_MODE_MULAW,
	MVS_G711_MODE_ALAW
};

#define VOIP_MODE_MAX		MODE_G711A
#define VOIP_RATE_MAX		23850

#define IOCTL_SKY_DIRECT_ADSP_MAGIC 's'

typedef enum
{
  FUNC_SKY_DIRECT_ADSP_START = 0,
  FUNC_SKY_DIRECT_ADSP_STOP,
  FUNC_SKY_DIRECT_ADSP_PCM_WB_MODE_SET,
  FUNC_SKY_DIRECT_ADSP_PCM_NB_MODE_SET,
  FUNC_SKY_DIRECT_ADSP_AMR_WB_MODE_SET,
  FUNC_SKY_DIRECT_ADSP_AMR_NB_MODE_SET,
  FUNC_SKY_DIRECT_ADSP_MAXNR
} sky_direct_adsp_func_name_type;

#define IOCTL_SKY_DIRECT_ADSP_START                       _IO(IOCTL_SKY_DIRECT_ADSP_MAGIC, FUNC_SKY_DIRECT_ADSP_START)
#define IOCTL_SKY_DIRECT_ADSP_STOP                        _IO(IOCTL_SKY_DIRECT_ADSP_MAGIC, FUNC_SKY_DIRECT_ADSP_STOP)
#define IOCTL_SKY_DIRECT_ADSP_PCM_WB_MODE_SET             _IO(IOCTL_SKY_DIRECT_ADSP_MAGIC, FUNC_SKY_DIRECT_ADSP_PCM_WB_MODE_SET)
#define IOCTL_SKY_DIRECT_ADSP_PCM_NB_MODE_SET             _IO(IOCTL_SKY_DIRECT_ADSP_MAGIC, FUNC_SKY_DIRECT_ADSP_PCM_NB_MODE_SET)
#define IOCTL_SKY_DIRECT_ADSP_AMR_WB_MODE_SET             _IO(IOCTL_SKY_DIRECT_ADSP_MAGIC, FUNC_SKY_DIRECT_ADSP_AMR_WB_MODE_SET)
#define IOCTL_SKY_DIRECT_ADSP_AMR_NB_MODE_SET             _IO(IOCTL_SKY_DIRECT_ADSP_MAGIC, FUNC_SKY_DIRECT_ADSP_AMR_NB_MODE_SET)
#define IOCTL_SKY_DIRECT_ADSP_MAXNR  FUNC_SKY_DIRECT_ADSP_MAXNR

enum format {
	FORMAT_S16_LE = 2,
	FORMAT_SPECIAL = 31,
};
/**************************************************************************************************/

/************************************************************************************************
** Enum & Struct - must be synced with "msm-pcm-voip.c"
*************************************************************************************************/
enum amr_rate_type {
	AMR_RATE_4750, /* AMR 4.75 kbps */
	AMR_RATE_5150, /* AMR 5.15 kbps */
	AMR_RATE_5900, /* AMR 5.90 kbps */
	AMR_RATE_6700, /* AMR 6.70 kbps */
	AMR_RATE_7400, /* AMR 7.40 kbps */
	AMR_RATE_7950, /* AMR 7.95 kbps */
	AMR_RATE_10200, /* AMR 10.20 kbps */
	AMR_RATE_12200, /* AMR 12.20 kbps */
	AMR_RATE_6600, /* AMR-WB 6.60 kbps */
	AMR_RATE_8850, /* AMR-WB 8.85 kbps */
	AMR_RATE_12650, /* AMR-WB 12.65 kbps */
	AMR_RATE_14250, /* AMR-WB 14.25 kbps */
	AMR_RATE_15850, /* AMR-WB 15.85 kbps */
	AMR_RATE_18250, /* AMR-WB 18.25 kbps */
	AMR_RATE_19850, /* AMR-WB 19.85 kbps */
	AMR_RATE_23050, /* AMR-WB 23.05 kbps */
	AMR_RATE_23850, /* AMR-WB 23.85 kbps */
	AMR_RATE_UNDEF
};

enum voip_state {
	VOIP_STOPPED,
	VOIP_STARTED,
};

struct voip_frame_hdr {
	uint32_t timestamp;
	union {
		/*
		 * Bits 0-15: Frame type
		 * Bits 16-31: Frame rate
		 */
		uint32_t frame_type;
		uint32_t packet_rate;
	};
};
struct voip_frame {
	struct voip_frame_hdr frm_hdr;
	uint32_t pktlen;
	uint8_t voc_pkt[VOIP_MAX_VOC_PKT_SIZE];
};

struct voip_buf_node {
	struct list_head list;
	struct voip_frame frame;
};

struct voip_drv_info {
	enum  voip_state state;

	struct snd_pcm_substream *playback_substream;
	struct snd_pcm_substream *capture_substream;

	struct list_head in_queue;
	struct list_head free_in_queue;

	struct list_head out_queue;
	struct list_head free_out_queue;

	wait_queue_head_t out_wait;
	wait_queue_head_t in_wait;

	struct mutex lock;

	spinlock_t dsp_lock;
	spinlock_t dsp_ul_lock;

	bool voip_reset;
	uint32_t mode;
	uint32_t rate_type;
	uint32_t rate;
	uint32_t dtx_mode;

	uint8_t capture_start;
	uint8_t playback_start;

	uint8_t playback_instance;
	uint8_t capture_instance;

	unsigned int play_samp_rate;
	unsigned int cap_samp_rate;

	unsigned int pcm_size;
	unsigned int pcm_count;
	unsigned int pcm_playback_irq_pos;      /* IRQ position */
	unsigned int pcm_playback_buf_pos;      /* position in buffer */

	unsigned int pcm_capture_size;
	unsigned int pcm_capture_count;
	unsigned int pcm_capture_irq_pos;       /* IRQ position */
	unsigned int pcm_capture_buf_pos;       /* position in buffer */

	uint32_t evrc_min_rate;
	uint32_t evrc_max_rate;
};
/**************************************************************************************************/
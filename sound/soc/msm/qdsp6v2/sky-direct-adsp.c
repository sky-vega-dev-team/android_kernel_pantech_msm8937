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
#include "sky-direct-adsp.h"
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

/************************************************************************************************
** Exported symbols from "msm-pcm-voip.c"
*************************************************************************************************/
extern int msm_pcm_playback_copy(struct snd_pcm_substream *substream, int a,
                    	snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames);
extern int msm_pcm_capture_copy(struct snd_pcm_substream *substream, int a,
                    	snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames);

extern struct voip_drv_info voip_info;
extern bool bUseSKYDirectADSP;

#if 0
//navan - 2012.09.27 - for VoLTE Recording
extern char rx_temp[640];
extern /*static */bool bRecordingFlag;
extern /*static */int record_size;

//-->20130214 jhsong : volte rec tx buffer too small
extern char tx_temp[640];
extern /*static */int tx_record_size;
//<--20130214 jhsong : volte rec tx buffer too small
#endif

/************************************************************************************************
** Variables & Defines
*************************************************************************************************/
static int voip_get_media_type(uint32_t mode,
				unsigned int samp_rate,
				unsigned int *media_type);
static int voip_get_rate_type(uint32_t mode,
				uint32_t rate,
				uint32_t *rate_type);
static uint32_t MVS_MODE = MODE_PCM;
static spinlock_t pcm_read_lock;

#ifndef FEATURE_RV_VEGA_VOLTE_TICK_NOISE
#define FEATURE_RV_VEGA_VOLTE_TICK_NOISE
#endif//FEATURE_RV_VEGA_VOLTE_TICK_NOISE

/************************************************************************************************
** Functions
*************************************************************************************************/
static ssize_t sky_direct_adsp_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset)
{
    int ret = 0;
#if 0
    char rec_ind;
#endif

    if(!bUseSKYDirectADSP)
        return -1;
    
    if(voip_info.state != VOIP_STARTED)
        return -2;

    if(voip_info.capture_start != 1)
        return -3;

#if 0
    //navan - 2012.09.27 - for VoLTE Recording
    if(copy_from_user(&rec_ind, (void __user *)buffer, 1))
    {
      rec_ind = 0;
    }
    if(rec_ind == 0xff)
    {
      int rx_record_size_retVal = 0; // 20130417 jmlee add  volte recording noise fix
      if(length < record_size) return 0;
      if(record_size == 0) return 0;  // 20130417 jmlee add  volte recording noise fix

      spin_lock(&pcm_read_lock);
      if(copy_to_user((void __user *)buffer, rx_temp, record_size))
      {
        spin_unlock(&pcm_read_lock);
        return 0;
      }
      memset(rx_temp, 0x0, sizeof(rx_temp));
      //bRecordingFlag = true;
      spin_unlock(&pcm_read_lock);
	rx_record_size_retVal =  record_size;   // 20130417 jmlee add  volte recording noise fix
	record_size = 0;
      //return record_size;
      return rx_record_size_retVal;  // 20130417 jmlee add  volte recording noise fix
    }

 //-->20130214 jhsong : volte rec tx buffer too small
     if(rec_ind == 0xfe)
    {
       int tx_record_size_retVal = 0; // jmlee add
      if(length < tx_record_size) return 0;
      if(tx_record_size == 0) return 0;  // 20130417 jmlee add  volte recording noise fix

      spin_lock(&pcm_read_lock);
      if(copy_to_user((void __user *)buffer, tx_temp, tx_record_size))
      {
        spin_unlock(&pcm_read_lock);
        return 0;
      }
      memset(tx_temp, 0x0, sizeof(tx_temp));
      //bRecordingFlag = true;
      tx_record_size_retVal = tx_record_size;  // jmlee add
      tx_record_size = 0;
      spin_unlock(&pcm_read_lock);
      return tx_record_size_retVal; // jmlee add modify
    }
 //<--20130214 jhsong : volte rec tx buffer too small
#endif

    if(voip_info.mode == MODE_PCM)
    {
        ret = msm_pcm_capture_copy(voip_info.capture_substream, 0, 0, (void __user *)buffer, length/2);
    }
    else if(voip_info.mode == MODE_AMR_WB)
    {
        ret = msm_pcm_capture_copy(voip_info.capture_substream, 0, 0, (void __user *)buffer, length);    
    }
    else if(voip_info.mode == MODE_AMR)
    {
        ret = msm_pcm_capture_copy(voip_info.capture_substream, 0, 0, (void __user *)buffer, length);    
    }
    else
        return -4;
        
    if(ret != 0)
        return -5;

    return length;
}

static long sky_direct_adsp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned long flags;
    void __user *argp = (void __user *)arg;

  	int ret = 0;
	int32_t media_type = 0;
	uint32_t rate_type = 0;
    uint32_t evrc_min_rate_type = 0;
	uint32_t evrc_max_rate_type = 0;

 	SKY_DIRECT_ADSP_DBG_HIGH("sky_direct_adsp_ioctl cmd:%d, arg:%ld ", cmd, arg);

    mutex_lock(&voip_info.lock);

    if(_IOC_TYPE(cmd) != IOCTL_SKY_DIRECT_ADSP_MAGIC)
    {
        SKY_DIRECT_ADSP_ERR("[%s] invalid Magic Char [%c]\n", __func__, _IOC_TYPE(cmd));
        mutex_unlock(&voip_info.lock);
        return -EINVAL;
    }
    
    if(_IOC_NR(cmd) >= IOCTL_SKY_DIRECT_ADSP_MAXNR)
    {
        SKY_DIRECT_ADSP_ERR("[%s] invalid Magic Char [%c]\n", __func__, _IOC_TYPE(cmd));
        mutex_unlock(&voip_info.lock);
        return -EINVAL;
    }

    switch(cmd)
    {
        case IOCTL_SKY_DIRECT_ADSP_START:
            SKY_DIRECT_ADSP_DBG_LOW("sky_direct_adsp_ioctl(), start SKY Direct ADSP!!");
            bUseSKYDirectADSP = true;
            break;
        case IOCTL_SKY_DIRECT_ADSP_STOP:
            SKY_DIRECT_ADSP_DBG_LOW("sky_direct_adsp_ioctl(), stop SKY Direct ADSP!!");
            bUseSKYDirectADSP = false;            
            break;
        case IOCTL_SKY_DIRECT_ADSP_PCM_WB_MODE_SET:
            SKY_DIRECT_ADSP_DBG_LOW("sky_direct_adsp_ioctl(), set ADSP mode PCM-WB");
            MVS_MODE = MODE_PCM;
            if((voip_info.play_samp_rate != 16000 || voip_info.cap_samp_rate != 16000) 
	    	&& (voip_info.capture_substream != NULL && voip_info.playback_substream != NULL))
            {
       			voip_info.state = VOIP_STOPPED;
        		voc_end_voice_call(
        				voc_get_session_id(VOIP_SESSION_NAME));
        		
                voip_info.play_samp_rate = 16000;
                voip_info.pcm_size = snd_pcm_lib_buffer_bytes(voip_info.playback_substream);
                voip_info.pcm_count = snd_pcm_lib_period_bytes(voip_info.playback_substream);
                voip_info.pcm_playback_irq_pos = 0;
                voip_info.pcm_playback_buf_pos = 0;

                voip_info.cap_samp_rate = 16000;
                voip_info.pcm_capture_size  = snd_pcm_lib_buffer_bytes(voip_info.capture_substream);
                voip_info.pcm_capture_count = snd_pcm_lib_period_bytes(voip_info.capture_substream);
                voip_info.pcm_capture_irq_pos = 0;
                voip_info.pcm_capture_buf_pos = 0;
                    
        		ret = voip_get_rate_type(voip_info.mode,
        					voip_info.rate,
        					&rate_type);
                
        		if (ret < 0) {
        			SKY_DIRECT_ADSP_DBG_HIGH("fail at getting rate_type, set ADSP to default WB 23.85k rate\n");
                    voip_info.rate = 23850;
                    rate_type = AMR_RATE_23850 - AMR_RATE_6600;
        		}

        		voip_info.rate_type = rate_type;
            	ret = voip_get_media_type(voip_info.mode,
            					voip_info.play_samp_rate,
                				&media_type);
                
            	if (ret < 0) {
            		SKY_DIRECT_ADSP_DBG_HIGH("fail at getting media_type\n");
            		ret = -EINVAL;
            	}
                
            	pr_debug(" media_type=%d, rate_type=%d\n", media_type,
            		rate_type);
                
            	if ((voip_info.play_samp_rate == 16000) &&
            				(voip_info.cap_samp_rate == 16000))
                    voc_config_vocoder(media_type, rate_type,
                            voip_info.dtx_mode,
                            evrc_min_rate_type,
                            evrc_max_rate_type);
            	else {
            		SKY_DIRECT_ADSP_DBG_HIGH("%s: Invalid rate playback %d, capture %d\n",
            			 __func__, voip_info.play_samp_rate,
            			 voip_info.cap_samp_rate);
            		ret = -EINVAL;
            	}
                
            	voc_start_voice_call(voc_get_session_id(VOIP_SESSION_NAME));

            	voip_info.state = VOIP_STARTED;
            }
            break;
        case IOCTL_SKY_DIRECT_ADSP_PCM_NB_MODE_SET:
            SKY_DIRECT_ADSP_DBG_LOW("sky_direct_adsp_ioctl(), set ADSP mode PCM-NB");
            MVS_MODE = MODE_PCM;
            if((voip_info.play_samp_rate != 8000 || voip_info.cap_samp_rate != 8000)
	    	&& (voip_info.capture_substream != NULL && voip_info.playback_substream != NULL))
            {
       			voip_info.state = VOIP_STOPPED;
        		voc_end_voice_call(
        				voc_get_session_id(VOIP_SESSION_NAME));

                voip_info.play_samp_rate = 8000;
                voip_info.pcm_size = snd_pcm_lib_buffer_bytes(voip_info.playback_substream);
                voip_info.pcm_count = snd_pcm_lib_period_bytes(voip_info.playback_substream);
                voip_info.pcm_playback_irq_pos = 0;
                voip_info.pcm_playback_buf_pos = 0;

                voip_info.cap_samp_rate = 8000;
                voip_info.pcm_capture_size  = snd_pcm_lib_buffer_bytes(voip_info.capture_substream);
                voip_info.pcm_capture_count = snd_pcm_lib_period_bytes(voip_info.capture_substream);
                voip_info.pcm_capture_irq_pos = 0;
                voip_info.pcm_capture_buf_pos = 0;
                    
        		ret = voip_get_rate_type(voip_info.mode,
        					voip_info.rate,
        					&rate_type);
                
        		if (ret < 0) {
        			SKY_DIRECT_ADSP_DBG_HIGH("fail at getting rate_type, set ADSP to default WB 23.85k rate\n");
                    voip_info.rate = 23850;
                    rate_type = AMR_RATE_23850 - AMR_RATE_6600;
        		}

        		voip_info.rate_type = rate_type;
            	ret = voip_get_media_type(voip_info.mode,
            					voip_info.play_samp_rate,
                				&media_type);
                
            	if (ret < 0) {
            		SKY_DIRECT_ADSP_DBG_HIGH("fail at getting media_type\n");
            		ret = -EINVAL;
            	}
                
            	pr_debug(" media_type=%d, rate_type=%d\n", media_type,
            		rate_type);
                
            	if ((voip_info.play_samp_rate == 8000) &&
            				(voip_info.cap_samp_rate == 8000))
                    voc_config_vocoder(media_type, rate_type,
                            voip_info.dtx_mode,
                            evrc_min_rate_type,
                            evrc_max_rate_type);
            	else {
            		SKY_DIRECT_ADSP_DBG_HIGH("%s: Invalid rate playback %d, capture %d\n",
            			 __func__, voip_info.play_samp_rate,
            			 voip_info.cap_samp_rate);
            		ret = -EINVAL;
            	}
                
            	voc_start_voice_call(voc_get_session_id(VOIP_SESSION_NAME));

            	voip_info.state = VOIP_STARTED;
            }
            break;
        case IOCTL_SKY_DIRECT_ADSP_AMR_WB_MODE_SET:
            SKY_DIRECT_ADSP_DBG_HIGH("sky_direct_adsp_ioctl(), set ADSP mode AMR-WB");
            MVS_MODE = MODE_AMR_WB;

            if(copy_from_user(&flags, argp, sizeof(flags)))
            {
                mutex_unlock(&voip_info.lock);
                return -EFAULT;
            }
            else
            {
                voip_info.rate = (uint32_t)flags;
            }

            if(MVS_MODE != voip_info.mode)
            {
                // TODO:  implement restart ADSP routine with new AMR-WB mode setting
                if(voip_info.state == VOIP_STARTED)
                {
                    //stop ADSP
           			voip_info.state = VOIP_STOPPED;
            		voc_end_voice_call(
            				voc_get_session_id(VOIP_SESSION_NAME));
            		//voc_register_mvs_cb(NULL, NULL, voip_info);

                    //restart ASDP
                    voip_info.mode = MODE_AMR_WB;

            		ret = voip_get_rate_type(voip_info.mode,
            					voip_info.rate,
            					&rate_type);
                    
            		if (ret < 0) {
            			pr_err("fail at getting rate_type, set ADSP to default WB 23.85k rate\n");
                        voip_info.rate = 23850;
                        rate_type = AMR_RATE_23850 - AMR_RATE_6600;
            		}

            		voip_info.rate_type = rate_type;
                	ret = voip_get_media_type(voip_info.mode,
                					voip_info.play_samp_rate,
                					&media_type);
                    
                	if (ret < 0) {
                		pr_err("fail at getting media_type\n");
                		ret = -EINVAL;
                	}
                    
                	pr_debug(" media_type=%d, rate_type=%d\n", media_type,
                		rate_type);
                    
                	if ((voip_info.play_samp_rate == 16000) &&
                				(voip_info.cap_samp_rate == 16000))
                        voc_config_vocoder(media_type, rate_type,
                                voip_info.dtx_mode,
                                evrc_min_rate_type,
                                evrc_max_rate_type);
                	else {
                		pr_debug("%s: Invalid rate playback %d, capture %d\n",
                			 __func__, voip_info.play_samp_rate,
                			 voip_info.cap_samp_rate);
                		ret = -EINVAL;
                	}
//                	voc_register_mvs_cb(voip_process_ul_pkt,
//                				voip_process_dl_pkt, voip_info);
                	voc_start_voice_call(voc_get_session_id(VOIP_SESSION_NAME));

                	voip_info.state = VOIP_STARTED;
                }
                else
                {
                    SKY_DIRECT_ADSP_ERR("sky_direct_adsp_ioctl() failed, voip_info.state : %d",voip_info.state);        
                }
            }
            break;
        case IOCTL_SKY_DIRECT_ADSP_AMR_NB_MODE_SET:
            SKY_DIRECT_ADSP_DBG_HIGH("sky_direct_adsp_ioctl(), set ADSP mode AMR-NB");
            MVS_MODE = MODE_AMR;
            
            if(copy_from_user(&flags, argp, sizeof(flags)))
            {
                mutex_unlock(&voip_info.lock);
                return -EFAULT;
            }
            else
            {
                voip_info.rate = (uint32_t)flags;
            }

            if(MVS_MODE != voip_info.mode)
            {
                // TODO:  implement restart ADSP routine with new AMR-NB mode setting
                if(voip_info.state == VOIP_STARTED)
                {
                    //stop ADSP
           			voip_info.state = VOIP_STOPPED;
            		voc_end_voice_call(
            				voc_get_session_id(VOIP_SESSION_NAME));
            		//voc_register_mvs_cb(NULL, NULL, voip_info);

                    //restart ASDP
                    voip_info.mode = MODE_AMR;

            		ret = voip_get_rate_type(voip_info.mode,
            					voip_info.rate,
            					&rate_type);
                    
            		if (ret < 0) {
            			pr_err("fail at getting rate_type, set ADSP to default NB 12.2k rate\n");
                        voip_info.rate = 12200;
                        rate_type = AMR_RATE_12200;
            		}
                    
               		voip_info.rate_type = rate_type;
                	ret = voip_get_media_type(voip_info.mode,
                					voip_info.play_samp_rate,
                					&media_type);
                    
                	if (ret < 0) {
                		pr_err("fail at getting media_type\n");
                		ret = -EINVAL;
                	}
                    
                	pr_debug(" media_type=%d, rate_type=%d\n", media_type,
                		rate_type);
                    
                	if ((voip_info.play_samp_rate == 8000) &&
                				(voip_info.cap_samp_rate == 8000))
                        voc_config_vocoder(media_type, rate_type,
                                voip_info.dtx_mode,
                                evrc_min_rate_type,
                                evrc_max_rate_type);
                	else {
                		pr_debug("%s: Invalid rate playback %d, capture %d\n",
                			 __func__, voip_info.play_samp_rate,
                			 voip_info.cap_samp_rate);
                		ret = -EINVAL;
                	}
//                	voc_register_mvs_cb(voip_process_ul_pkt,
//                				voip_process_dl_pkt, voip_info);
                	voc_start_voice_call(voc_get_session_id(VOIP_SESSION_NAME));

                	voip_info.state = VOIP_STARTED;
                }
                else
                {
                    SKY_DIRECT_ADSP_ERR("sky_direct_adsp_ioctl() failed, voip_info.state : %d",voip_info.state);        
                }
            }
            break;
        default:
            SKY_DIRECT_ADSP_DBG_HIGH("sky_direct_adsp_ioctl(), set ADSP mode PCM");
            MVS_MODE = MODE_PCM;
            break;
    }

    mutex_unlock(&voip_info.lock);

    return ret;
}

#ifdef CONFIG_COMPAT
static long sky_direct_adsp_ioctl_compat(struct file *file1, unsigned int cmd, unsigned long arg)
{
	return sky_direct_adsp_ioctl(file1, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int voip_get_rate_type(uint32_t mode, uint32_t rate,
				 uint32_t *rate_type)
{
	int ret = 0;

	switch (mode) {
	case MODE_AMR: {
		switch (rate) {
		case 4750:
			*rate_type = AMR_RATE_4750;
			break;
		case 5150:
			*rate_type = AMR_RATE_5150;
			break;
		case 5900:
			*rate_type = AMR_RATE_5900;
			break;
		case 6700:
			*rate_type = AMR_RATE_6700;
			break;
		case 7400:
			*rate_type = AMR_RATE_7400;
			break;
		case 7950:
			*rate_type = AMR_RATE_7950;
			break;
		case 10200:
			*rate_type = AMR_RATE_10200;
			break;
		case 12200:
			*rate_type = AMR_RATE_12200;
			break;
		default:
			pr_err("wrong rate for AMR NB.\n");
			ret = -EINVAL;
			break;
		}
		break;
	}
	case MODE_AMR_WB: {
		switch (rate) {
		case 6600:
			*rate_type = AMR_RATE_6600 - AMR_RATE_6600;
			break;
		case 8850:
			*rate_type = AMR_RATE_8850 - AMR_RATE_6600;
			break;
		case 12650:
			*rate_type = AMR_RATE_12650 - AMR_RATE_6600;
			break;
		case 14250:
			*rate_type = AMR_RATE_14250 - AMR_RATE_6600;
			break;
		case 15850:
			*rate_type = AMR_RATE_15850 - AMR_RATE_6600;
			break;
		case 18250:
			*rate_type = AMR_RATE_18250 - AMR_RATE_6600;
			break;
		case 19850:
			*rate_type = AMR_RATE_19850 - AMR_RATE_6600;
			break;
		case 23050:
			*rate_type = AMR_RATE_23050 - AMR_RATE_6600;
			break;
		case 23850:
			*rate_type = AMR_RATE_23850 - AMR_RATE_6600;
			break;
		default:
			pr_err("wrong rate for AMR_WB.\n");
			ret = -EINVAL;
			break;
		}
		break;
	}
	case MODE_PCM: {
		*rate_type = 0;
		break;
	}
	case MODE_IS127:
	case MODE_4GV_NB:
	case MODE_4GV_WB: {
		switch (rate) {
		case VOC_0_RATE:
		case VOC_8_RATE:
		case VOC_4_RATE:
		case VOC_2_RATE:
		case VOC_1_RATE:
			*rate_type = rate;
			break;
		default:
			pr_err("wrong rate for IS127/4GV_NB/WB.\n");
			ret = -EINVAL;
			break;
		}
		break;
	}
	case MODE_4GV_NW: {
		switch (rate) {
		case VOC_0_RATE:
		case VOC_8_RATE:
		case VOC_4_RATE:
		case VOC_2_RATE:
		case VOC_1_RATE:
		case VOC_8_RATE_NC:
			*rate_type = rate;
			break;
		default:
			pr_err("wrong rate for 4GV_NW.\n");
			ret = -EINVAL;
			break;
		}
		break;
	}
	default:
		pr_err("wrong mode type.\n");
		ret = -EINVAL;
	}
	pr_debug("%s, mode=%d, rate=%u, rate_type=%d\n",
		__func__, mode, rate, *rate_type);
	return ret;
}

static int voip_get_media_type(uint32_t mode,
				unsigned int samp_rate,
				unsigned int *media_type)
{
	int ret = 0;

	pr_debug("%s: mode=%d, samp_rate=%d\n", __func__,
		mode, samp_rate);
	switch (mode) {
	case MODE_AMR:
		*media_type = VSS_MEDIA_ID_AMR_NB_MODEM;
		break;
	case MODE_AMR_WB:
		*media_type = VSS_MEDIA_ID_AMR_WB_MODEM;
		break;
	case MODE_IS127: /* EVRC-A */
		*media_type = VSS_MEDIA_ID_EVRC_MODEM;
		break;
	case MODE_4GV_NB: /* EVRC-B */
		*media_type = VSS_MEDIA_ID_4GV_NB_MODEM;
		break;
	case MODE_4GV_WB: /* EVRC-WB */
		*media_type = VSS_MEDIA_ID_4GV_WB_MODEM;
		break;
	case MODE_4GV_NW: /* EVRC-NW */
		*media_type = VSS_MEDIA_ID_4GV_NW_MODEM;
		break;
	default:
		pr_debug(" input mode is not supported\n");
		ret = -EINVAL;
	}

	pr_debug("%s: media_type is 0x%x\n", __func__, *media_type);

	return ret;
}

static struct miscdevice sky_direct_adsp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sky_direct_adsp",
	.fops = &sky_direct_adsp_fops
};

static int __init sky_direct_adsp_init(void)
{
    int rc=0;
  	spin_lock_init(&pcm_read_lock);
    SKY_DIRECT_ADSP_DBG_HIGH("sky_direct_adsp_misc_device_init");
    rc = misc_register(&sky_direct_adsp_misc);
    if(rc<0)    SKY_DIRECT_ADSP_ERR("sky_direct_adsp_misc_device_init rc= %d ", rc);
    return rc;
}
static void __exit sky_direct_adsp_exit(void)
{
    SKY_DIRECT_ADSP_DBG_HIGH("sky_direct_adsp_misc_device_deinit");
    misc_deregister(&sky_direct_adsp_misc);
}

module_init(sky_direct_adsp_init);
module_exit(sky_direct_adsp_exit);

MODULE_DESCRIPTION("MSM MVS Voip driver");
MODULE_LICENSE("GPL v2");

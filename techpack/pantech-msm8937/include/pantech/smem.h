/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Data the Pantech bootloader shares in SMEM, as msm-3.18 laid it out in
 * soc/qcom/smem.h (CONFIG_PANTECH_PMIC_SHARED_DATA).
 */
#ifndef _PANTECH_SMEM_H
#define _PANTECH_SMEM_H

#include <linux/err.h>
#include <linux/soc/qcom/smem.h>
#include <linux/types.h>

/* SMEM_ID_VENDOR1 in the msm-3.18 SMEM item list */
#define PANTECH_SMEM_ID_VENDOR1	135

typedef struct {
	uint32_t power_on_reason;
	uint32_t factory_dummy_adc;
	uint8_t  factory_dummy_present;
	uint32_t battery_id;
	uint32_t battery_id_adc;
	uint32_t power_on_mode;		/* 0: offline charging */
	uint32_t silent_boot_mode;
	uint32_t backlight_off;
} oem_pm_smem_vendor1_data_type;

/* NULL when the item is missing or too small */
static inline oem_pm_smem_vendor1_data_type *pantech_smem_vendor1(void)
{
	oem_pm_smem_vendor1_data_type *p;
	size_t size = 0;

	p = qcom_smem_get(QCOM_SMEM_HOST_ANY, PANTECH_SMEM_ID_VENDOR1, &size);
	if (IS_ERR_OR_NULL(p) || size < sizeof(*p))
		return NULL;

	return p;
}

#endif /* _PANTECH_SMEM_H */

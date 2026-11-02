/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pantech key codes. msm-3.18 added them to uapi/linux/input.h, in a
 * range that is still unused in msm-4.19.
 */
#ifndef _PANTECH_INPUT_H
#define _PANTECH_INPUT_H

#include <linux/input.h>

#ifndef KEY_PAN_WHEEL
#define KEY_PAN_WHEEL	250	/* wheel click */
#endif

#ifndef KEY_VT_CALL
#define KEY_VT_CALL	252
#endif

#endif /* _PANTECH_INPUT_H */

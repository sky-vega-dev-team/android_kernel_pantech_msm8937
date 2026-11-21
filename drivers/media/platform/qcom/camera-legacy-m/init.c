// SPDX-License-Identifier: GPL-2.0-only
/*
 * In msm-4.19 several camera stacks shared one kernel and this flag picked
 * the legacy one at boot. Here it is the only stack.
 */
#include <linux/types.h>

bool camera_legacy_m_enable = true;

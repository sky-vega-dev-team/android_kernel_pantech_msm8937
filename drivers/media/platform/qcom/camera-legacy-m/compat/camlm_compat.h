/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Glue for building the msm-4.19 camera_v2 stack on a mainline kernel.
 * Included before every source file of the stack.
 */
#ifndef __CAMLM_COMPAT_H
#define __CAMLM_COMPAT_H

#include <linux/types.h>
#include <linux/time_types.h>
#include <linux/time32.h>
#include <linux/time64.h>

/*
 * The uapi structures still carry struct timeval, which the kernel no
 * longer defines: __kernel_old_timeval has the same layout. The compat
 * side used compat_timeval, now old_timeval32.
 */
#define timeval		__kernel_old_timeval
#define compat_timeval	old_timeval32

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/timekeeping.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <media/v4l2-dev.h>

/* Renamed or removed kernel interfaces */
#define kzfree			kfree_sensitive
#define strlcpy			strscpy
#define del_timer		timer_delete
#define del_timer_sync		timer_delete_sync
#define VFL_TYPE_GRABBER	VFL_TYPE_VIDEO
#define timespec		timespec64

static inline void do_gettimeofday(struct __kernel_old_timeval *tv)
{
	struct timespec64 ts;

	ktime_get_real_ts64(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
}

static inline void get_monotonic_boottime(struct timespec64 *ts)
{
	ktime_get_boottime_ts64(ts);
}

static inline void getnstimeofday(struct timespec64 *ts)
{
	ktime_get_real_ts64(ts);
}

/* The struct gpio arrays of the legacy GPIO API */
struct gpio {
	unsigned int gpio;
	unsigned long flags;
	const char *label;
};

static inline int gpio_request_array(const struct gpio *array, size_t num)
{
	size_t i;
	int err;

	for (i = 0; i < num; i++) {
		err = gpio_request_one(array[i].gpio, array[i].flags,
				       array[i].label);
		if (err) {
			while (i--)
				gpio_free(array[i].gpio);
			return err;
		}
	}
	return 0;
}

static inline void gpio_free_array(const struct gpio *array, size_t num)
{
	while (num--)
		gpio_free(array[num].gpio);
}

/* Raw formats of the Qualcomm videodev2.h */
#ifndef V4L2_PIX_FMT_SBGGR10DPCM6
#define V4L2_PIX_FMT_SBGGR10DPCM6 v4l2_fourcc('b', 'B', 'A', '6')
#define V4L2_PIX_FMT_SGBRG10DPCM6 v4l2_fourcc('b', 'G', 'A', '6')
#define V4L2_PIX_FMT_SGRBG10DPCM6 v4l2_fourcc('B', 'D', '1', '6')
#define V4L2_PIX_FMT_SRGGB10DPCM6 v4l2_fourcc('b', 'R', 'A', '6')
#endif

#endif

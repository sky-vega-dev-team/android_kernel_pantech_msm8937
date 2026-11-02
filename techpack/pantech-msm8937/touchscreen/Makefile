# SPDX-License-Identifier: GPL-2.0-only

# mxt_ts.c includes the rest of the driver
obj-$(CONFIG_TOUCHSCREEN_ATMEL_MXT641T_PANTECH) += atmel_mxt641t/mxt_ts.o

obj-$(CONFIG_TOUCHSCREEN_FT8716_PANTECH) += ft8716/fts_ts.o
ft8716/fts_ts-y := ft8716/focaltech_core.o ft8716/focaltech_ctl.o \
	ft8716/focaltech_ex_fun.o ft8716/focaltech_flash.o \
	ft8716/focaltech_gesture.o ft8716/focaltech_esd_protection.o

# SPDX-License-Identifier: GPL-2.0-only
#
# Qualcomm camera_v2 stack (msm-3.18 "Marshmallow" flavour), as the vendor
# camera daemon of the EF71 expects it. Taken from msm-4.19
# techpack/camera-legacy-m.

CAMLM := $(src)

subdir-ccflags-y += -I$(CAMLM) \
		    -I$(CAMLM)/compat -I$(CAMLM)/sensor -I$(CAMLM)/pproc \
		    -I$(CAMLM)/msm_vb2 -I$(CAMLM)/camera -I$(CAMLM)/jpeg_10 \
		    -I$(CAMLM)/fd -I$(CAMLM)/common
subdir-ccflags-y += -include $(CAMLM)/compat/camlm_compat.h
ccflags-y += $(subdir-ccflags-y)

obj-y += compat/of_gpio.o compat/gdsc_regulator.o compat/ion_compat.o

obj-y += init.o
obj-y += common/
obj-y += msm.o
obj-y += camera/
obj-y += msm_vb2/
obj-y += sensor/
obj-y += pproc/
obj-y += isp/
obj-y += ispif/
obj-$(CONFIG_MSMB_JPEG) += jpeg_10/
obj-y += msm_buf_mgr/
obj-$(CONFIG_MSM_FD) += fd/

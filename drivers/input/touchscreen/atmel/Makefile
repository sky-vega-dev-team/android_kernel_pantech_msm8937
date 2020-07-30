#
# Makefile for the MSM8992 Atmel mXT641T TOUCH drivers.
#


obj-y += mxt_ts.o
#obj-m := mxt_ts.o

ANDROID_ROOT_DIR:=../../../../../
KDIR := $(ANDROID_ROOT_DIR)out/target/product/msm8937_64/obj/KERNEL_OBJ/

#KBUILD_CFLAGS = -W -Wall -nostdinc $(DEFINES) $(INCLUDES)
PWD := $(shell pwd)
export ARCH:=arm64

export CROSS_COMPILE:=$(ANDROID_ROOT_DIR)../prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-

#KCFLAGS=-mno-android

#EXTRA_CFLAGS += -include ../../../../../../../BOARD_REV.h

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	rm -f -r *.o *.ko .*cmd *.mod.c .tmp* core *.i modules.order Module.symvers modules.builtin

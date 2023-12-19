HOST_ARCH       ?= $(shell uname -m | sed -e s/arm.*/arm/ -e s/aarch64.*/arm64/)
ARCH            ?= $(shell uname -m | sed -e s/arm.*/arm/ -e s/aarch64.*/arm64/)
PWD 			      ?= ${shell pwd}
BUILD_DIR       ?= $(shell pwd)
_BUILD_DIR      ?= $(shell readlink -f $(BUILD_DIR))

ifdef KERNELRELEASE
	KERNELRELEASE := $(KERNELRELEASE)
else
	KERNELRELEASE ?= $(shell uname -r)
endif

ifdef KERNEL_SRC
  KERNEL_SRC_DIR  := $(KERNEL_SRC)
else
  KERNEL_SRC_DIR  ?= /lib/modules/$(shell uname -r)/build
endif

ifeq ($(ARCH), arm)
 ifneq ($(HOST_ARCH), arm)
   CROSS_COMPILE  ?= arm-linux-gnueabihf-
 endif
endif
ifeq ($(ARCH), arm64)
 ifneq ($(HOST_ARCH), arm64)
   CROSS_COMPILE  ?= aarch64-linux-gnu-
 endif
endif

obj-m := sensing-max9296.o sensing-max9295.o sensing-ar0233-gw5200.o

.PHONY: all
all: modules

modules:
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) M=$(BUILD_DIR) src=$(PWD) modules

dtbo:
	dtc -O dtb -o sensing-ar0233-gw5200-gmsl-device-tree-overlay.dtbo -@ sensing-ar0233-gw5200-gmsl-device-tree-overlay.dts
	#cp -rf sensing-ar0233-gw5200-gmsl-device-tree-overlay.dtbo /boot

clean:
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) M=$(BUILD_DIR) src=$(PWD) clean
	rm -f sensing-ar0233-gw5200-gmsl-device-tree-overlay.dtbo

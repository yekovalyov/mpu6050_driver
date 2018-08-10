#
# mpu6050 accelerometer & gyroscope
#
ifneq ($(KERNELRELEASE),)

obj-m := mpu6050.o

else


ifeq ($(KERNELDIR),)
ifeq ($(OPI_KERNELDIR),)

#$(warning Path to kernel tree - KERNELDIR or OPI_KERNELDIR variable is not defined!)

endif
endif

KERNELDIR ?= $(OPI_KERNELDIR)

export ARCH = arm
export CROSS_COMPILE ?= arm-linux-gnueabihf-


.PHONY: all clean dtb info


all:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) modules

info:
	@echo ARCH=$(ARCH)
	@echo KERNELDIR=$(KERNELDIR)
	@echo OPI_KERNELDIR=$(OPI_KERNELDIR)
	@echo CROSS_COMPILE=$(CROSS_COMPILE)


clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) clean

dtb:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) dtbs


endif

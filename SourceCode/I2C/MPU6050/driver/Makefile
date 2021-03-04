# This is your kernel source directory
# Please enter the source path 
KERNELDIR := 
CURRENT_PATH := $(shell pwd)

obj-m := i2c_mpu6050.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean

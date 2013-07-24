#
# Makefile for Invensense MPU6000 device.
#

obj-m := inv-mpu6000.o
inv-mpu6000-objs := inv_mpu_core.o

KDIR := /mnt/scratch/repos/linux

all:
	make -C $(KDIR) M=$(PWD)

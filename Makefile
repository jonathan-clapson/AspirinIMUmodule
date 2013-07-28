#
# Makefile for Invensense MPU6050 device.
#

obj-m := inv-mpu60x0.o
inv-mpu60x0-objs := inv_mpu_core_spi.o mpu60x0/inv_mpu_ring.o mpu60x0/inv_mpu_trigger.o

KDIR := $(KERNEL_SRC)

all:
	make -C $(KDIR) M=$(PWD)

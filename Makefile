obj-m := module.o
KVERSION := $(shell uname -r)
KDIR := /home/jonathan/repos/linux-rpi-3.9.y

all:
	make -C $(KDIR) M=$(PWD)

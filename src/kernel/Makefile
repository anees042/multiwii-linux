#  This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version. see <http://www.gnu.org/licenses/>
#
# Makefile for the multiwii kernel module




DRIVERNAME=mi2c

ifneq ($(KERNELRELEASE),)
	obj-m += mi2c.o
	mi2c-objs := mi2c-char.o mi2c-i2c.o  
else
    PWD := $(shell pwd)


all-kernel:
ifeq ($(strip $(KERNELDIR)),)
	$(error "KERNELDIR is undefined!")
else
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules 
endif


all:
	all-kernel




install:all-kernel
	cp mi2c.ko $(MOD_INSTALL_DIR)

clean:
	rm -rf   *~ *.ko *.o *.mod.c modules.order Module.symvers .${DRIVERNAME}* .tmp_versions

endif


#
# File Name: Makefile
#
# Description: Makefile to build kernel hplib utility module.
#
# Copyright (C) 2014 Texas Instruments, Incorporated
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation version 2.
#
# This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
# whether express or implied; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#

obj-m := temperature_kmodule.o

KVERSION ?= $(shell uname -r)
KERNEL_SRC  ?= /lib/modules/$(KVERSION)/build

default all:
	make -C $(KERNEL_SRC) M=$(PWD) modules 

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean

modules_install: 
	make -C $(KERNEL_SRC) M=$(PWD) modules_install

install_ipk: default all
	install -m 644 -d $(DESTDIR)/lib/modules/$(KVERSION)/extra
	sudo mv temperature_kmodule.ko $(DESTDIR)/lib/modules/$(KVERSION)/extra 


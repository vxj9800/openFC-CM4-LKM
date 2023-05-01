
# To build modules outside of the kernel tree, we run "make"
# in the kernel source tree; the Makefile these then includes this
# Makefile once again.
# This conditional selects whether we are being included from the
# kernel Makefile or not.
ifeq ($(KERNELRELEASE),)

    # Assume the source tree is where the running kernel was built
    # You should set KERNELDIR in the environment if it's elsewhere
    KERNELDIR ?= ~/WSL2-Linux-Kernel/
    # The current directory is passed to sub-makes as argument
    # However, a link is made to actual directory in user's home
    # directory to deal with a case where module folder path
    # conatins whitespaces.
    PWD := ~/openFC-CM4-LKM
    ACTPWD := $(shell pwd)
    MKLNK := $(shell ln -s -T "$(ACTPWD)/" $(PWD))

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

.PHONY: modules modules_install clean

else
    # called from kernel build system: just declare what our modules are
    obj-m := usbDriver.o
endif

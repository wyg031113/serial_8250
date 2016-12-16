tar:=serial
ifneq ($(KERNELRELEASE),)
obj-m=$(tar).o
else
	KDIR=/lib/modules/`uname -r`/build
	PWD=$(shell pwd)
all:$(tar).c
	make -C $(KDIR) M=$(PWD) modules
clean:
	rm -rf *.o *.ko .*.cmd *.mod.c *.order *.unsigned .tmp_versions
endif

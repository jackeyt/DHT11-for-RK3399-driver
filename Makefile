ROOTFS_DIR = /home/pi/

ifeq ($(KERNELRELEASE), )

KERNEL_DIR = /home/pi/t4_develop/t4_kernel
CUR_DIR = $(shell pwd)
APP_NAME = dht11_test
CC = aarch64-linux-gnu-gcc
all :
	make -C  $(KERNEL_DIR) M=$(CUR_DIR) modules
	$(CC) $(APP_NAME).c  -o $(APP_NAME)
	
clean :
	make -C  $(KERNEL_DIR) M=$(CUR_DIR) clean
	rm -rf $(APP_NAME)
	
install:
	cp -raf *.ko   $(ROOTFS_DIR)
	cp -raf  $(APP_NAME)  /usr/local/bin

else

obj-m += dht11_drv.o
#obj-m += math.o
endif
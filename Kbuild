obj-m = opiservo.o

KERNEL_TREE := /home/orangepi/OrangePi/xunlong_github/OrangePi-Kernel/linux-3.4.113/

module:
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) modules

clean:
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) clean


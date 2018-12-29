KERNEL_TREE := /home/orangepi/OrangePi/xunlong_github/OrangePi-Kernel/linux-3.4.113/
INSTALL_PATH := /lib/modules/$(shell /bin/uname -r)/kernel/drivers/misc/opiservo
#CROSS_OPTS := CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- ARCH=arm
CROSS_OPTS :=

.PHONY: all install install_autostart uninstall
all:	opiservo.ko

opiservo.ko:	opiservo.c
	@[ -d ${KERNEL_TREE} ] || { echo "Edit Makefile to set KERNEL_TREE to point at your kernel"; exit 1; }
	@[ -e ${KERNEL_TREE}/Module.symvers ] || { echo "KERNEL_TREE/Module.symvers does not exist, you need to configure and compile your kernel"; exit 1; }
	make -C ${KERNEL_TREE} ${CROSS_OPTS} M=$(PWD) modules

install: opiservo.ko
	@sudo cp $(PWD)/udev_scripts/opiservo /lib/udev
	@sudo cp $(PWD)/udev_scripts/20-opiservo.rules /etc/udev/rules.d
	@sudo chmod +x /lib/udev/opiservo
	@echo "OPIServo udev rules complete."

reload: opiservo.ko
	@sudo rmmod opiservo
	@sudo insmod opiservo.ko
	@echo "OPIServo reload."

install_autostart: install
	@echo "Enabling OPIServo autostart on boot."
	@sudo mkdir -p $(INSTALL_PATH)
	@sudo cp $(PWD)/opiservo.ko $(INSTALL_PATH)
	@if ! grep opiservo /etc/modules > /dev/null 2>&1; then sudo sed -i '$$a\opiservo' /etc/modules; fi
	@sudo depmod -a
	@echo "OPIServo will now auto start on next boot."
	@echo "The following commands will start and stop the driver:"
	@echo "	modprobe opiservo"
	@echo "	modprobe -r opiservo"

uninstall:
	@modprobe -r opiservo
	@sudo rm -f /lib/udev/opiservo
	@sudo rm -f /etc/udev/rules.d/20-opiservo.rules
	@sudo rm -f $(INSTALL_PATH)/opiservo.ko
	@sudo depmod -a
	
clean:
	make -C ${KERNEL_TREE} ${CROSS_OPTS} M=$(PWD) clean


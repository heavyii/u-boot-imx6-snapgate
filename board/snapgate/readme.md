
for 1Gb RAM:

	make snapgate_config

for 512Mb RAM:

	make snapgate512m_config

# compile

	export ARCH=arm
	export CROSS_COMPILE=arm-none-linux-gnueabi-
	make snapgate_config
	make -j4

# install

	dev=sdb
	sudo dd if=u-boot.imx of=/dev/$dev bs=1k seek=1

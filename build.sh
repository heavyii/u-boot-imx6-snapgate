#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-
make snapgate_config
make -j4

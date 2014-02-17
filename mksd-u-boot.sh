#!/bin/sh
dev=$1
set -x
sudo dd if=SPL of=/dev/$dev bs=1k seek=1 oflag=dsync
sudo dd if=u-boot.img of=/dev/$dev bs=1k seek=69 oflag=dsync
sudo sync


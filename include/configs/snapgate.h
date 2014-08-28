/*
 * Copyright (C) 2014 DLRC.
 *
 * Configuration settings for the snapgate.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <asm/sizes.h>

#include "imx6_spl.h"

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define MACH_TYPE_SNAPGATE		4412
#define CONFIG_MACH_TYPE		MACH_TYPE_SNAPGATE

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#define CONFIG_SYS_NO_FLASH
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR


#define CONFIG_BOOTDELAY		1

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x10800000
#define CONFIG_SYS_TEXT_BASE		0x17800000

/* fuse */
#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* OCOTP Configs */
#define CONFIG_CMD_IMXOTP
#ifdef CONFIG_CMD_IMXOTP
#define CONFIG_IMX_OTP
#define IMX_OTP_BASE			OCOTP_BASE_ADDR
#define IMX_OTP_ADDR_MAX		0x7F
#define IMX_OTP_DATA_ERROR_VAL		0xBADABADA
#define IMX_OTPWRITE_ENABLED
#endif

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* Ethernet Configuration */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		0

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE (1 << 20)
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_HDMI

#if defined(CONFIG_MX6DL) || defined(CONFIG_MX6S)
#define CONFIG_DEFAULT_FDT_FILE		"imx6dl-snapgate.dtb"
#elif defined(CONFIG_MX6Q)
#define CONFIG_DEFAULT_FDT_FILE		"imx6q-snapgate.dtb"
#else
#define CONFIG_DEFAULT_FDT_FILE		"imx6q-snapgate.dtb"
#endif

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
	"if run loadbootenv; then " \
		"echo Loaded environment from ${bootenv};" \
		"run importbootenv;" \
	"fi;" \
	"if test -n $uenvcmd; then " \
		"echo Running uenvcmd ...;" \
		"run uenvcmd;" \
	"fi;" \
	"if run loadramdisk; then " \
		"setenv rootdevice ${ramdisk_dev}; " \
		"setenv bootsys \'bootm ${loadaddr} ${initrdaddr}\'; " \
	"fi;" \
	"if run loaduimage; then " \
		"run setbootargs; " \
		"if run loadfdt; then " \
			"run bootsys_fdt ; " \
		"fi; " \
		"run bootsys ; " \
	"fi;" \
	"fi;"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootargs_base=console=ttymxc0,115200\0" \
	"setbootargs=run setbase; run setvideo; run setopts; run setplatform\0" \
	"setbase=setenv bootargs ${bootargs_base} ${rootdevice}\0" \
	"setvideo=setenv bootargs ${bootargs} ${video_mode}\0" \
	"setplatform=if test -n $expansion; then setenv bootargs " \
		"$bootargs expansion=$expansion;fi;if test -n $baseboard;" \
		" then setenv bootargs $bootargs baseboard=$baseboard;fi\0" \
	"setopts=setenv bootargs ${bootargs} ${optargs}\0" \
	"setvideo=setenv bootargs ${bootargs} ${video_mode}\0" \
	"fdt_addr=0x11000000\0" \
	"bootsys=bootm ${loadaddr}\0 " \
	"bootsys_fdt=echo boot fdt...; \
		bootm ${loadaddr} - ${fdt_addr} \0 " \
	"initrdaddr=0x13000000\0" \
	"rootdevice=root=/dev/mmcblk0p2 rootwait rw rootfstype=ext3\0" \
	"mmcdev=0\0" \
	"bootenv=boot/uEnv.txt\0" \
	"fdt_file=imx6dl-snapgate.dtb\0" \
	"bootcmd="CONFIG_BOOTCOMMAND"\0" \
	"loadbootenv=fatload mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment...; " \
		"env import -t $loadaddr $filesize\0" \
	"loadfdt=fatload mmc ${mmcdev} ${fdt_addr} boot/${fdt_file}\0" \
	"loaduimage=fatload mmc ${mmcdev} ${loadaddr} boot/uImage\0" \
	"loaduimage_raw=mmc read ${loadaddr} 0x800 0x4000\0 "	\
	"loadramdisk=fatload mmc ${mmcdev} ${initrdaddr} boot/uramdisk.img\0" \
	"splashimage=0x10800000\0"				\
	"splashimage_mmc_init_block=0x410\0"			\
	"splashimage_mmc_blkcnt=0x3F0\0"			\
	"splashimage_file_name=boot/out.bmp.gz\0"		\
        "splashpos=m,m\0"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "DLRC#"
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		512

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       32
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)


/* SPI FLASH */

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_CMD_SPI
#define CONFIG_MXC_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO

#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   (IMX_GPIO_NR(4, 9)<<8)
#define CONFIG_SF_DEFAULT_SPEED 20000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)

#define CONFIG_ENV_SPI_BUS		0
#define CONFIG_ENV_SPI_CS		(IMX_GPIO_NR(4, 9)<<8)
#define CONFIG_ENV_SPI_MAX_HZ	20000000
#define CONFIG_ENV_SPI_MODE		(SPI_MODE_0)
#endif

/* environment  */
#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif



#endif			       /* __CONFIG_H * */

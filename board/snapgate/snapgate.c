/*
 * Copyright (C) 2014 DLRC.
 *
 * Author: ruisheng <ruishenglin@126.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <ipu_pixfmt.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <fat.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |             \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |             \
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define USDHC3_CD_GPIO		IMX_GPIO_NR(3, 16)
#define EMMC_RESET_GPIO		IMX_GPIO_NR(3, 23)
#define LED_GPIO		IMX_GPIO_NR(7, 13)
#define LAN8720_RESET_GPIO IMX_GPIO_NR(6, 15)

int dram_init(void)
{
#if defined(CONFIG_DDR_MB)
	u32 sdram_size;
	gd->ram_size = (u32)CONFIG_DDR_MB * 1024 * 1024;
#else
	uint cpurev, imxtype;
	u32 sdram_size;

	cpurev = get_cpu_rev();
	imxtype = (cpurev & 0xFF000) >> 12;
			
	switch (imxtype){
	case MXC_CPU_MX6SOLO:
		sdram_size = 1u * 1024 * 1024 * 1024;
		break;
	case MXC_CPU_MX6Q:
		sdram_size = 2u * 1024 * 1024 * 1024;
		break;
	case MXC_CPU_MX6DL:
	default:
		sdram_size = 1u * 1024 * 1024 * 1024;;
		break;	
	}
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, sdram_size);
#endif
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* SOM MicroSD Card Detect */
	MX6_PAD_EIM_D16__GPIO_3_16     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__USDHC4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__USDHC4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__USDHC4_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__USDHC4_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__USDHC4_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__USDHC4_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__USDHC4_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__USDHC4_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__USDHC4_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__USDHC4_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* EMMC reset */
	MX6_PAD_EIM_D23__GPIO_3_23     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const fec_pads[] = {
    MX6_PAD_ENET_MDIO__ENET_MDIO        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_MDC__ENET_MDC        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_CRS_DV__ENET_RX_EN        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT     |
MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_RX_ER__ENET_RX_ER        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_TX_EN__ENET_TX_EN        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_RXD0__ENET_RDATA_0        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_RXD1__ENET_RDATA_1        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_TXD0__ENET_TDATA_0        | MUX_PAD_CTRL(ENET_PAD_CTRL),
    MX6_PAD_ENET_TXD1__ENET_TDATA_1        | MUX_PAD_CTRL(ENET_PAD_CTRL),

    /* RESET PIN */
    MX6_PAD_NANDF_CS2__GPIO_6_15     | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_usdhc4(void)
{
	imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
}

static void setup_iomux_usdhc3(void)
{
	imx_iomux_v3_setup_multiple_pads(usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	/*
	 * Following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD4 emmc
	 * mmc1                    SD3 micro SD
	 */
	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 1:
			setup_iomux_usdhc4();
			usdhc_cfg[index].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			usdhc_cfg[index].max_bus_width = 8;
			{
				/* Reset AR8031 PHY */
				gpio_direction_output(EMMC_RESET_GPIO, 0);
				udelay(500);
				gpio_set_value(EMMC_RESET_GPIO, 1);
			}
			break;
		case 0:
			setup_iomux_usdhc3();
			usdhc_cfg[index].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			usdhc_cfg[index].max_bus_width = 4;
			gpio_direction_input(USDHC3_CD_GPIO);
			break;

		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}

#if defined(CONFIG_VIDEO_IPUV3)
static struct fb_videomode const hdmi = {
	.name           = "HDMI",
	.refresh        = 60,
	.xres           = 1024,
	.yres           = 768,
	.pixclock       = 15385,
	.left_margin    = 220,
	.right_margin   = 40,
	.upper_margin   = 21,
	.lower_margin   = 7,
	.hsync_len      = 60,
	.vsync_len      = 10,
	.sync           = FB_SYNC_EXT,
	.vmode          = FB_VMODE_NONINTERLACED
};

int board_video_skip(void)
{
	int ret;

	ret = ipuv3_fb_init(&hdmi, 0, IPU_PIX_FMT_RGB24);

	if (ret)
		printf("HDMI cannot be configured: %d\n", ret);

	imx_enable_hdmi_phy();

	return ret;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);
}
#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_SPLASH_SCREEN

int splash_screen_prepare(void)
{
        ulong addr;
        char *s;
        __maybe_unused  int n, err;
        __maybe_unused  ulong offset = 0, size = 0, count = 0;
	__maybe_unused  struct mmc *mmc = NULL;
	__maybe_unused  struct block_dev_desc_t *sata = NULL;

	printf("%s\n",__FUNCTION__);
        s = getenv("splashimage");
        if (s != NULL) {
                addr = simple_strtoul(s, NULL, 16);
		mmc = find_mmc_device(0);

		if (!mmc){
			printf ("splash:Error Reading MMC.\n");
		}

		mmc_init(mmc);

#ifdef CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION
		if((s = getenv("splashimage_file_name")) != NULL) {
			err = fat_register_device(&mmc->block_dev,
	  			CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION);
			if (!err) {
				if (file_fat_read(s, (ulong *)addr, 0) > 0) {
					printf("splash:loading %s from MMC FAT...\n", s);
					return 0;
				}
			}
		}
#endif
		printf("splash: reading from MMC RAW\n");
		if((s = getenv("splashimage_mmc_init_block")) != NULL)
			offset = simple_strtoul (s, NULL, 16);
		if (!offset){
			printf ("splashimage_mmc_init_block is illegal! \n");
		}
		if((s = getenv("splashimage_mmc_blkcnt")) != NULL)
			size = simple_strtoul (s, NULL, 16);
		if (!size){
			printf ("splashimage_mmc_blkcnt is illegal! \n");
		}
		n = mmc->block_dev.block_read(0, offset, size, (ulong *)addr);
		if (n == 0)
			printf("splash: mmc blk read err \n");
        }
        return -1;
}
#endif



#ifdef CONFIG_FEC_MXC

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec_pads, ARRAY_SIZE(fec_pads));
}

static int board_enable_fec_anatop_clock(void)
{
    unsigned int reg = 0;
    int timeout = 100000;

    struct anatop_regs __iomem *anatop =
        (struct anatop_regs __iomem *)ANATOP_BASE_ADDR;

    reg = readl(&anatop->pll_enet);
    if ((reg & BM_ANADIG_PLL_ENET_POWERDOWN) ||
        (!(reg & BM_ANADIG_PLL_ENET_LOCK))) {
        reg &= ~BM_ANADIG_PLL_ENET_POWERDOWN;
        writel(reg, &anatop->pll_enet);
        while (timeout--) {
            if (readl(&anatop->pll_enet) & BM_ANADIG_PLL_ENET_LOCK)
                break;
        }
        if (timeout < 0)
            return -ETIMEDOUT;
    }

    /* Enable FEC clock */
    reg |= BM_ANADIG_PLL_ENET_ENABLE;
    reg &= ~BM_ANADIG_PLL_ENET_BYPASS;
    writel(reg, &anatop->pll_enet);

    return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_fec();

	/* Reset LAN8720 PHY */
	gpio_direction_output(LAN8720_RESET_GPIO, 0);
	udelay(500);
	gpio_set_value(LAN8720_RESET_GPIO, 1);

	ret = cpu_eth_init(bis);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		return ret;
	}

	return 0;
}

static int setup_fec(void)
{
	int ret;
	u32 reg;
	/*
	 * get enet tx reference clk from internal clock from anatop
	 * GPR1[21] = 1
	 */
	reg = readl(IOMUXC_BASE_ADDR + 0x4);
	reg |= (0x01 << 21); /* External clock chosen */
	writel(reg, IOMUXC_BASE_ADDR + 0x4);

	ret = board_enable_fec_anatop_clock();
	if (ret)
		return ret;

	return 0;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

#ifdef CONFIG_MXC_SPI

static iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_KEY_COL0__ECSPI1_SCLK   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	/* SPI1 SS0 */
	MX6_PAD_KEY_ROW1__GPIO_4_9 | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void setup_iomux_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
}

static void setup_spi(void)
{
	setup_iomux_spi();
}
#endif

static void setup_led(void)
{
	printf("light led\n");
	imx_iomux_v3_setup_pad(MX6_PAD_GPIO_18__GPIO_7_13 | MUX_PAD_CTRL(NO_PAD_CTRL));
	gpio_direction_output(LED_GPIO, 1);
	gpio_direction_output(LED_GPIO, 0);
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	setup_led();
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef	CONFIG_FEC_MXC
	setup_fec();
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: snapgate\n");

	return 0;
}

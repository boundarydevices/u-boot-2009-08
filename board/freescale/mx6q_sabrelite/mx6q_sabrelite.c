/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6.h>
#include <asm/arch/mx6_pins.h>
#include <asm/arch/mx6dl_pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <miiphy.h>

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

#ifdef CONFIG_ARCH_MMU
#include <asm/mmu.h>
#include <asm/arch/mmu.h>
#endif

#ifdef CONFIG_CMD_CLOCK
#include <asm/clock.h>
#endif

#ifdef CONFIG_IMX_ECSPI
#include <imx_spi.h>
#endif

#ifdef CONFIG_ANDROID_RECOVERY
#include "../common/recovery.h"
#include <part.h>
#include <ext2fs.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <ubi_uboot.h>
#include <jffs2/load_kernel.h>
#endif
#include <micrel.h>

#define IMX_GPIO_NR(port, offset) (((port - 1) << 5) | offset)
#include "pads.h"
#define FOR_SOLO
#include "pads.h"

int cpu_is_mx6q(void) {
	u32 cpu_type = readl(ANATOP_BASE_ADDR + 0x280);

	cpu_type >>= 16;
	if (cpu_type == 0x60)
		return 0;	//this is a soloLite
	cpu_type = readl(ANATOP_BASE_ADDR + 0x260);
	cpu_type >>= 16;
	if (cpu_type == 0x63) {
		return 1;	//this is a mx6Q
	}
	//0x61 is a mx6dl or solo
	return 0;
}

DECLARE_GLOBAL_DATA_PTR;

static u32 system_rev;
static enum boot_device boot_dev;

static inline void setup_boot_device(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;

	switch (bt_mem_ctl) {
	case 0x0:
		if (bt_mem_type)
			boot_dev = ONE_NAND_BOOT;
		else
			boot_dev = WEIM_NOR_BOOT;
		break;
	case 0x2:
		boot_dev = SATA_BOOT;
		break;
	case 0x3:
		if (bt_mem_type)
			boot_dev = SPI_NOR_BOOT;
		else
			boot_dev = I2C_BOOT;
		break;
	case 0x4:
	case 0x5:
		boot_dev = SD_BOOT;
		break;
	case 0x6:
	case 0x7:
		boot_dev = MMC_BOOT;
		break;
	case 0x8 ... 0xf:
		boot_dev = NAND_BOOT;
		break;
	default:
		boot_dev = UNKNOWN_BOOT;
		break;
	}
}

enum boot_device get_boot_device(void)
{
	return boot_dev;
}

u32 get_board_rev(void)
{

	system_rev = 0x63000;

	return system_rev;
}

#ifdef CONFIG_ARCH_MMU
void board_mmu_init(void)
{
	unsigned long ttb_base = PHYS_SDRAM_1 + 0x4000;
	unsigned long i;

	/*
	 * Set the TTB register
	 */
	asm volatile ("mcr  p15,0,%0,c2,c0,0" : : "r" (ttb_base) /*: */);

	/*
	 * Set the Domain Access Control Register
	 */
	i = ARM_ACCESS_DACR_DEFAULT;
	asm volatile ("mcr  p15,0,%0,c3,c0,0" : : "r" (i) /*: */);

	/*
	 * First clear all TT entries - ie Set them to Faulting
	 */
	memset((void *)ttb_base, 0, ARM_FIRST_LEVEL_PAGE_TABLE_SIZE);
	/* Actual   Virtual  Size   Attributes          Function */
	/* Base     Base     MB     cached? buffered?  access permissions */
	/* xxx00000 xxx00000 */
	X_ARM_MMU_SECTION(0x000, 0x000, 0x001, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW);	/* ROM, 1M */
	X_ARM_MMU_SECTION(0x001, 0x001, 0x008, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW);	/* 8M */
	X_ARM_MMU_SECTION(0x009, 0x009, 0x001, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW);	/* IRAM */
	X_ARM_MMU_SECTION(0x00A, 0x00A, 0x0F6, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW);	/* 246M */
	/* 2 GB memory starting at 0x10000000, only map 1.875 GB */
	X_ARM_MMU_SECTION(0x100, 0x100, 0x780,
			  ARM_CACHEABLE, ARM_BUFFERABLE, ARM_ACCESS_PERM_RW_RW);
	/* uncached alias of the same 1.875 GB memory */
	X_ARM_MMU_SECTION(0x100, 0x880, 0x780,
			  ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			  ARM_ACCESS_PERM_RW_RW);

	/* Enable MMU */
	MMU_ON();
}
#endif

static const unsigned char col_lookup[] = {9, 10, 11, 8, 12, 9, 9, 9};
static const unsigned char bank_lookup[] = {3, 2};

int dram_init(void)
{
	unsigned mdctl = readl(MMDC_P0_BASE_ADDR + 0x000);
	unsigned mdmisc = readl(MMDC_P0_BASE_ADDR + 0x018);
	int bits = 11 + 0 + 0 + 1; 	/* row+col+bank+width */
	bits += (mdctl >> 24) & 7;	/* row */
	bits += col_lookup[(mdctl >> 20) & 7];	/* col */
	bits += bank_lookup[(mdmisc >> 5) & 1];	/* bank */
	bits += (mdctl >> 16) & 3;	/* width */
	bits += (mdctl >> 30) & 1;	/* cs1 enabled*/
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = 1 << bits;
	return 0;
}

#ifdef CONFIG_IMX_ECSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	switch (dev->slave.cs) {
	case 0:
		/* PMIC */
		dev->base = ECSPI1_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_HIGH;
		dev->ss = 0;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
		break;
	case 1:
		/* SPI-NOR */
		dev->base = ECSPI1_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 1;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
		break;
	default:
		printf("Invalid Bus ID!\n");
	}

	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev)
{
}
#endif

#ifdef CONFIG_NET_MULTI
int board_eth_init(bd_t *bis)
{
	int rc = -ENODEV;

	return rc;
}
#endif

#ifdef CONFIG_CMD_MMC

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR, 1, 1, 1},
	{USDHC4_BASE_ADDR, 1, 1, 1},
};

#ifdef CONFIG_DYNAMIC_MMC_DEVNO
int get_mmc_env_devno(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	return (soc_sbmr & 0x00001800) >> 11;
}
#endif


int usdhc_gpio_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; index++) {
		switch (index) {
		case 0:
		case 1:
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

int board_mmc_init(bd_t *bis)
{
	if (!usdhc_gpio_init(bis))
		return 0;
	else
		return -1;
}

/* For DDR mode operation, provide target delay parameter for each SD port.
 * Use cfg->esdhc_base to distinguish the SD port #. The delay for each port
 * is dependent on signal layout for that particular port.  If the following
 * CONFIG is not defined, then the default target delay value will be used.
 */
#ifdef CONFIG_GET_DDR_TARGET_DELAY
u32 get_ddr_delay(struct fsl_esdhc_cfg *cfg)
{
	/* No delay required  */
	return 0;
}
#endif

#endif


#ifdef CONFIG_CMD_SATA
int setup_sata(void)
{
	unsigned gpr;
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	gpr = readl(&iomuxc_regs->gpr[13]);
	gpr &= ~IOMUXC_GPR13_SATA_MASK;
	gpr |= IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB |
			IOMUXC_GPR13_SATA_PHY_7_SATA2M |
			IOMUXC_GPR13_SATA_SPEED_3G |
			(3 << IOMUXC_GPR13_SATA_PHY_6_SHIFT) |
			IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED |
			IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16 |
			IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB |
			IOMUXC_GPR13_SATA_PHY_2_TX_1P104V |
			IOMUXC_GPR13_SATA_PHY_1_SLOW;
	writel(gpr, &iomuxc_regs->gpr[13]);

	return 0;
}
#endif

int board_init(void)
{
	int mx6q = cpu_is_mx6q();

#ifdef CONFIG_MFG
/* MFG firmware need reset usb to avoid host crash firstly */
#define USBCMD 0x140
	int val = readl(OTG_BASE_ADDR + USBCMD);
	val &= ~0x1;		/*RS bit */
	writel(val, OTG_BASE_ADDR + USBCMD);
#endif
	mxc_iomux_v3_init((void *)IOMUXC_BASE_ADDR);
	setup_boot_device();
	/* Disable wl1271 For Nitrogen6w */
	gpio_direction_input(IMX_GPIO_NR(6, 14));
	gpio_direction_output(IMX_GPIO_NR(6, 15), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 16), 0);
	if (mx6q)
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_pads, ARRAY_SIZE(mx6q_sabrelite_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6solo_sabrelite_pads, ARRAY_SIZE(mx6solo_sabrelite_pads));
	clk_config_cko1(8000000);

	/* board id for linux */
	gd->bd->bi_arch_number = MACH_TYPE_MX6Q_SABRELITE;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

#ifdef CONFIG_I2C_MXC
	{
		struct i2c_pads_info *pi =
				mx6q ? mx6q_i2c_pad_info : mx6solo_i2c_pad_info;
		setup_i2c(0, CONFIG_SYS_I2C1_SPEED, 0x7f, &pi[0]);
		setup_i2c(1, CONFIG_SYS_I2C2_SPEED, 0x7f, &pi[1]);
		setup_i2c(2, CONFIG_SYS_I2C3_SPEED, 0x7f, &pi[2]);
	}
#endif
#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	return 0;
}

#ifdef CONFIG_ANDROID_RECOVERY
struct reco_envs supported_reco_envs[BOOT_DEV_NUM] = {
	{
		.cmd = NULL,
		.args = NULL,
	},
	{
		.cmd = NULL,
		.args = NULL,
	},
	{
		.cmd = NULL,
		.args = NULL,
	},
	{
		.cmd = NULL,
		.args = NULL,
	},
	{
		.cmd = CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC,
		.args = CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC,
	},
	{
		.cmd = CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC,
		.args = CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC,
	},
	{
		.cmd = CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC,
		.args = CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC,
	},
	{
		.cmd = CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC,
		.args = CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC,
	},
	{
		.cmd = NULL,
		.args = NULL,
	},
};

int check_recovery_cmd_file(void)
{
	int button_pressed = 0;
	int recovery_mode = 0;

	recovery_mode = check_and_clean_recovery_flag();

	/* Check Recovery Combo Button press or not. */
	gpio_direction_input(IMX_GPIO_NR(4, 5));

	if (!gpio_get_value(IMX_GPIO_NR(4, 5))) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
	}

	return recovery_mode || button_pressed;
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}

static int phy_write(char *devname, unsigned char addr, unsigned char reg,
		     unsigned short value)
{
	int ret = miiphy_write(devname, addr, reg, value);
	if (ret)
		printf("Error writing to %s PHY addr=%02x reg=%02x\n", devname,
		       addr, reg);

	return ret;
}

static int phy_read(char *devname, unsigned char addr, unsigned char reg)
{
	unsigned short value = 0;
	int ret = miiphy_read(devname, addr, reg, &value);
	if (ret) {
		printf("Error reading from %s PHY addr=%02x reg=%02x\n", devname,
		       addr, reg);
		return ret;
	}
	return value;
}

int board_phy_config(char *phydev, int phy_addr)
{
	int val = phy_read(phydev, phy_addr, PHY_PHYIDR2) & 0xfff0;
	if (val != 0x1620) {
		/* 0x161x - ksz9021 */
#if 0
		/* To advertise only 10 Mbs */
		phy_write(phydev, phy_addr, 0x4, 0x61);
		phy_write(phydev, phy_addr, 0x9, 0x0c00);
		/* enable master mode, force phy to 100Mbps */
		phy_write(phydev, phy_addr, 0x9, 0x1c00);
#endif
		/* min rx data delay */
		ksz9021_phy_extended_write(phydev, phy_addr,
				MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
		/* min tx data delay */
		ksz9021_phy_extended_write(phydev, phy_addr,
				MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
		/* max rx/tx clock delay, min rx/tx control */
		ksz9021_phy_extended_write(phydev, phy_addr,
				MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
		ksz9021_config(phydev, phy_addr);
	} else {
		ksz9031_config(phydev, phy_addr);
	}
	return 0;
}

int mx6_rgmii_rework(char *phydev, int phy_addr)
{
	board_phy_config(phydev, phy_addr);
	return 0;
}

void enet_board_init(void)
{
	int mx6q = cpu_is_mx6q();
	/* Sabrelite phy reset: gpio3-23 */
	gpio_direction_output(IMX_GPIO_NR(3, 23), 0);
	/* Nitrogen6w phy reset: gpio1-27 */
	gpio_direction_output(IMX_GPIO_NR(1, 27), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 30), (CONFIG_FEC0_PHY_ADDR >> 2));
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
	if (mx6q)
		mxc_iomux_v3_setup_multiple_pads(mx6q_enet_pads,
				ARRAY_SIZE(mx6q_enet_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6solo_enet_pads,
				ARRAY_SIZE(mx6solo_enet_pads));
	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);

	udelay(500);
	/* Sabrelite phy reset: gpio3-23 */
	gpio_direction_output(IMX_GPIO_NR(3, 23), 1);
	/* Nitrogen6w phy reset: gpio1-27 */
	gpio_direction_output(IMX_GPIO_NR(1, 27), 1);
	if (mx6q)
		mxc_iomux_v3_setup_multiple_pads(mx6q_enet_pads_final,
				ARRAY_SIZE(mx6q_enet_pads_final));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6solo_enet_pads_final,
				ARRAY_SIZE(mx6solo_enet_pads_final));
}

int checkboard(void)
{
	printf("Board: %s-SABRELITE:[ ", cpu_is_mx6q() ? "MX6Q" : "MX6 SOLO");

	switch (__REG(SRC_BASE_ADDR + 0x8)) {
	case 0x0001:
		printf("POR");
		break;
	case 0x0009:
		printf("RST");
		break;
	case 0x0010:
	case 0x0011:
		printf("WDOG");
		break;
	default:
		printf("unknown");
	}
	printf("]\n");

	printf("Boot Device: ");
	switch (get_boot_device()) {
	case WEIM_NOR_BOOT:
		printf("NOR\n");
		break;
	case ONE_NAND_BOOT:
		printf("ONE NAND\n");
		break;
	case PATA_BOOT:
		printf("PATA\n");
		break;
	case SATA_BOOT:
		printf("SATA\n");
		break;
	case I2C_BOOT:
		printf("I2C\n");
		break;
	case SPI_NOR_BOOT:
		printf("SPI NOR\n");
		break;
	case SD_BOOT:
		printf("SD\n");
		break;
	case MMC_BOOT:
		printf("MMC\n");
		break;
	case NAND_BOOT:
		printf("NAND\n");
		break;
	case UNKNOWN_BOOT:
	default:
		printf("UNKNOWN\n");
		break;
	}
	return 0;
}

#ifdef CONFIG_IMX_UDC

void udc_pins_setting(void)
{
	/* set USB_OTG_PWR to 0 */
	gpio_direction_output(IMX_GPIO_NR(3, 22), 0);

	mxc_iomux_set_gpr_register(1, 13, 1, 1);
}
#endif

int do_hdmidet(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	u8 reg = readb(HDMI_PHY_STAT0);
	return (HDMI_PHY_STAT0_HPD == (reg&HDMI_PHY_STAT0_HPD))
		? 0 : 1 ;
}

U_BOOT_CMD(hdmidet, 1, 1, do_hdmidet,
		"detect HDMI",
		""
);

/*
 * Copyright (C) 2007, Guennadi Liakhovetski <lg@denx.de>
 *
 * (C) Copyright 2009-2010 Freescale Semiconductor, Inc.
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
#include <asm/arch/mx51.h>
#include <asm/arch/mx51_pins.h>
#include <asm/arch/iomux.h>
#include <asm/errno.h>
#include <i2c.h>
#include "board-nitrogen.h"
#ifdef CONFIG_IMX_ECSPI
#include <imx_spi.h>
#include <asm/arch/imx_spi_pmic.h>
#endif

#include <asm/errno.h>

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

#ifdef CONFIG_ARCH_MMU
#include <asm/mmu.h>
#include <asm/arch/mmu.h>
#endif

#ifdef CONFIG_GET_FEC_MAC_ADDR_FROM_IIM
#include <asm/imx_iim.h>
#endif

#ifdef CONFIG_ANDROID_RECOVERY
#include "../../freescale/common/recovery.h"
#include <part.h>
#include <ext2fs.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <ubi_uboot.h>
#include <jffs2/load_kernel.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

static enum boot_device boot_dev;
u32	mx51_io_base_addr;

#define GPIO_DR 0
#define GPIO_DIR 4
#define GPIO_PSR 8
#define GPIO_NUMBER(port, offset) (((port - 1) << 5) | offset)
unsigned gp_base[] = {GPIO1_BASE_ADDR, GPIO2_BASE_ADDR, GPIO3_BASE_ADDR, GPIO4_BASE_ADDR};
void Set_GPIO_output_val(unsigned gp, unsigned val)
{
	unsigned base = gp_base[gp >> 5];
	unsigned mask = (1 << (gp & 0x1f));
	unsigned reg = readl(base + GPIO_DR);
	if (val & 1)
		reg |= mask;	/* set high */
	else
		reg &= ~mask;	/* clear low */
	writel(reg, base + GPIO_DR);

	reg = readl(base + GPIO_DIR);
	reg |= mask;		/* configure GPIO line as output */
	writel(reg, base + GPIO_DIR);
}

void Set_GPIO_input(unsigned gp)
{
	unsigned base = gp_base[gp >> 5];
	unsigned mask = (1 << (gp & 0x1f));
	unsigned reg;
	reg = readl(base + GPIO_DIR);
	reg &= ~mask;		/* configure GPIO line as input */
	writel(reg, base + GPIO_DIR);
}

unsigned gpio_get_value(unsigned gp)
{
	unsigned base = gp_base[gp >> 5];
	unsigned reg = readl(base + GPIO_PSR);
	return (reg >> (gp & 0x1f)) & 1;
}


unsigned get_srev(void);

u32 get_board_rev(void)
{
	unsigned srev = get_srev();
	return 0x51010 + srev;
}

static inline void setup_boot_device(void)
{
	uint *fis_addr = (uint *)IRAM_BASE_ADDR;

	switch (*fis_addr) {
	case NAND_FLASH_BOOT:
		boot_dev = NAND_BOOT;
		break;
	case SPI_NOR_FLASH_BOOT:
		boot_dev = SPI_NOR_BOOT;
		break;
	case MMC_FLASH_BOOT:
		boot_dev = MMC_BOOT;
		break;
	default:
		{
			uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
			uint bt_mem_ctl = soc_sbmr & 0x00000003;
			uint bt_mem_type = (soc_sbmr & 0x00000180) >> 7;

			switch (bt_mem_ctl) {
			case 0x3:
				if (bt_mem_type == 0)
					boot_dev = MMC_BOOT;
				else if (bt_mem_type == 3)
					boot_dev = SPI_NOR_BOOT;
				else
					boot_dev = UNKNOWN_BOOT;
				break;
			case 0x1:
				boot_dev = NAND_BOOT;
				break;
			default:
				boot_dev = UNKNOWN_BOOT;
			}
		}
		break;
	}
}

enum boot_device get_boot_device(void)
{
	return boot_dev;
}


#ifdef CONFIG_ARCH_MMU
void board_mmu_init(unsigned long ttb_base)
{
	unsigned long i;

	/*
	* Set the TTB register
	*/
	asm volatile ("mcr p15, 0, %0, c2, c0, 0" : : "r"(ttb_base) /*:*/);

	/*
	* Set the Domain Access Control Register
	*/
	i = ARM_ACCESS_DACR_DEFAULT;
	asm volatile ("mcr p15, 0, %0, c3, c0, 0" : : "r"(i) /*:*/);

	/*
	* First clear all TT entries - ie Set them to Faulting
	*/
	memset((void *)ttb_base, 0, ARM_FIRST_LEVEL_PAGE_TABLE_SIZE);
	/* Actual   Virtual  Size   Attributes          Function */
	/* Base     Base     MB     cached? buffered?  access permissions */
	/* xxx00000 xxx00000 */
	X_ARM_MMU_SECTION(0x000, 0x200, 0x1,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* ROM */
	X_ARM_MMU_SECTION(0x1FF, 0x1FF, 0x001,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* IRAM */
	X_ARM_MMU_SECTION(0x300, 0x300, 0x100,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* GPU */
	X_ARM_MMU_SECTION(0x400, 0x400, 0x200,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* IPUv3D */
	X_ARM_MMU_SECTION(0x600, 0x600, 0x300,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* periperals */
	X_ARM_MMU_SECTION(0x900, 0x000, 0x1FF,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* SDRAM */
	X_ARM_MMU_SECTION(0x900, 0x900, 0x200,
			ARM_CACHEABLE, ARM_BUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* SDRAM */
	X_ARM_MMU_SECTION(0x900, 0xE00, 0x200,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* SDRAM 0:128M*/
	X_ARM_MMU_SECTION(0xB80, 0xB80, 0x10,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* CS1 EIM control*/
	X_ARM_MMU_SECTION(0xCC0, 0xCC0, 0x040,
			ARM_UNCACHEABLE, ARM_UNBUFFERABLE,
			ARM_ACCESS_PERM_RW_RW); /* CS4/5/NAND Flash buffer */

	/* Workaround for arm errata #709718 */
	/* Setup PRRR so device is always mapped to non-shared */
	asm volatile ("mrc p15, 0, %0, c10, c2, 0" : "=r"(i) : /*:*/);
	i &= (~(3 << 0x10));
	asm volatile ("mcr p15, 0, %0, c10, c2, 0" : : "r"(i) /*:*/);

	/* Enable MMU */
	MMU_ON();
}
#endif

#define ESDCTL1 0x83fd9008
int dram_init(void)
{
	unsigned shift;
	shift = readl(ESDCTL1) >> 31;
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = (256 << 20) << shift;
	return 0;
}

/*
 * continued fraction
 *      2  1  2  1  2
 * 0 1  2  3  8 11 30
 * 1 0  1  1  3  4 11
 */
static void get_best_ratio(unsigned *pnum, unsigned *pdenom, unsigned max)
{
	unsigned a = *pnum;
	unsigned b = *pdenom;
	unsigned c;
	unsigned n[] = {0, 1};
	unsigned d[] = {1, 0};
	unsigned whole;
	unsigned i = 1;
	while (b) {
		i ^= 1;
		whole = a / b;
		n[i] += (n[i ^ 1] * whole);
		d[i] += (d[i ^ 1] * whole);
//		printf("cf=%i n=%i d=%i\n", whole, n[i], d[i]);
		if ((n[i] | d[i]) > max) {
			i ^= 1;
			break;
		}
		c = a - (b * whole);
		a = b;
		b = c;
	}
	*pnum = n[i];
	*pdenom = d[i];
}

//#define DEBUG		//if enabled, also enable in start.S
#ifdef DEBUG
void TransmitX(char ch);
#define debug_putc(ch) TransmitX(ch)
#else
#define debug_putc(ch)
#endif

unsigned get_uart_base(void)
{
#ifdef CONFIG_UART_BASE_ADDR
	return CONFIG_UART_BASE_ADDR;
#else
	return UART1_BASE_ADDR;
#endif
}

#define UTXD		0x0040
#define UFCR		0x0090
#define USR2		0x0098
#define UBIR		0x00a4
#define UBMR		0x00a8

void setup_uart(void)
{
	u32 uart = get_uart_base();
	u32 clk_src = mxc_get_clock(MXC_UART_CLK);
	u32 mult, div;
	unsigned int pad = PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
			 PAD_CTL_PUE_PULL | PAD_CTL_DRV_HIGH;
#define BAUDRATE 115200
	writel(0x4 << 7, uart + UFCR);	/* divide input clock by 2 */
	mult = BAUDRATE * 2 * 16;	/* 16 samples/clock */
	div = clk_src;
//	writel(611 - 1, uart + UBIR);
//	writel(11022 - 1, uart + UBMR);
	get_best_ratio(&mult, &div, 0x10000);
	writel(mult - 1, uart + UBIR);
	writel(div - 1, uart + UBMR);

	mxc_request_iomux(MX51_PIN_UART1_RXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_RXD, pad | PAD_CTL_SRE_FAST);
	mxc_request_iomux(MX51_PIN_UART1_TXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_TXD, pad | PAD_CTL_SRE_FAST);
	mxc_request_iomux(MX51_PIN_UART1_RTS, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_RTS, pad);
	mxc_request_iomux(MX51_PIN_UART1_CTS, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_UART1_CTS, pad);

	debug_putc('x');
//	printf("setup_uart clk_src=%i mult=%i div=%i\n", clk_src, mult, div);
}

void setup_nfc(void)
{
	/* Enable NFC IOMUX */
	mxc_request_iomux(MX51_PIN_NANDF_CS0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS4, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS5, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS6, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_NANDF_CS7, IOMUX_CONFIG_ALT0);
}

static void setup_expio(void)
{
	u32 reg;
	/* CS5 setup */
	mxc_request_iomux(MX51_PIN_EIM_CS5, IOMUX_CONFIG_ALT0);
	writel(0x00410089, WEIM_BASE_ADDR + 0x78 + CSGCR1);
	writel(0x00000002, WEIM_BASE_ADDR + 0x78 + CSGCR2);
	/* RWSC=50, RADVA=2, RADVN=6, OEA=0, OEN=0, RCSA=0, RCSN=0 */
	writel(0x32260000, WEIM_BASE_ADDR + 0x78 + CSRCR1);
	/* APR = 0 */
	writel(0x00000000, WEIM_BASE_ADDR + 0x78 + CSRCR2);
	/* WAL=0, WBED=1, WWSC=50, WADVA=2, WADVN=6, WEA=0, WEN=0,
	 * WCSA=0, WCSN=0
	 */
	writel(0x72080F00, WEIM_BASE_ADDR + 0x78 + CSWCR1);
	if ((readw(CS5_BASE_ADDR + PBC_ID_AAAA) == 0xAAAA) &&
	    (readw(CS5_BASE_ADDR + PBC_ID_5555) == 0x5555)) {
		if (get_srev() < SREV2_0) {
			reg = readl(CCM_BASE_ADDR + CLKCTL_CBCDR);
			reg = (reg & (~0x70000)) | 0x30000;
			writel(reg, CCM_BASE_ADDR + CLKCTL_CBCDR);
			/* make sure divider effective */
			while (readl(CCM_BASE_ADDR + CLKCTL_CDHIPR) != 0)
				;
			writel(0x0, CCM_BASE_ADDR + CLKCTL_CCDR);
		}
		mx51_io_base_addr = CS5_BASE_ADDR;
	} else {
		/* CS1 */
		writel(0x00410089, WEIM_BASE_ADDR + 0x18 + CSGCR1);
		writel(0x00000002, WEIM_BASE_ADDR + 0x18 + CSGCR2);
		/*  RWSC=50, RADVA=2, RADVN=6, OEA=0, OEN=0, RCSA=0, RCSN=0 */
		writel(0x32260000, WEIM_BASE_ADDR + 0x18 + CSRCR1);
		/* APR=0 */
		writel(0x00000000, WEIM_BASE_ADDR + 0x18 + CSRCR2);
		/* WAL=0, WBED=1, WWSC=50, WADVA=2, WADVN=6, WEA=0,
		 * WEN=0, WCSA=0, WCSN=0
		 */
		writel(0x72080F00, WEIM_BASE_ADDR + 0x18 + CSWCR1);
		mx51_io_base_addr = CS1_BASE_ADDR;
	}

	/* Reset interrupt status reg */
	writew(0x1F, mx51_io_base_addr + PBC_INT_REST);
	writew(0x00, mx51_io_base_addr + PBC_INT_REST);
	writew(0xFFFF, mx51_io_base_addr + PBC_INT_MASK);

	/* Reset the XUART and Ethernet controllers */
	reg = readw(mx51_io_base_addr + PBC_SW_RESET);
	reg |= 0x9;
	writew(reg, mx51_io_base_addr + PBC_SW_RESET);
	reg &= ~0x9;
	writew(reg, mx51_io_base_addr + PBC_SW_RESET);
}

#ifdef CONFIG_IMX_ECSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	dev->base = CSPI1_BASE_ADDR;
	dev->freq = 54000000;
	dev->fifo_sz = 64 * 4;
	dev->us_delay = 0;
	switch (dev->slave.cs) {
	case 0:
		/* pmic */
		dev->ss_pol = IMX_SPI_ACTIVE_HIGH;
		dev->ss = 0;
		break;
	case 1:
		/* spi_nor */
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 1;
		break;
	default:
		printf("Invalid Bus ID! \n");
		break;
	}
	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev, int active)
{
	if (dev->ss == 0) {
		Set_GPIO_output_val(GPIO_NUMBER(4, 24), active ? 1 : 0);
	} else if (dev->ss == 1) {
		Set_GPIO_output_val(GPIO_NUMBER(4, 25), active ? 0 : 1);
	}
}
#endif

#ifdef CONFIG_MXC_FEC

#ifdef CONFIG_GET_FEC_MAC_ADDR_FROM_IIM

int fec_get_mac_addr(unsigned char *mac)
{
	u32 *iim1_mac_base =
		(u32 *)(IIM_BASE_ADDR + IIM_BANK_AREA_1_OFFSET +
			CONFIG_IIM_MAC_ADDR_OFFSET);
	int i;

	for (i = 0; i < 6; ++i, ++iim1_mac_base)
		mac[i] = (u8)readl(iim1_mac_base);

	return 0;
}
#endif

unsigned short fec_setup_pins[] = {
	/** MDIO, EIM_EB2, alt 3, IOMUXC_FEC_FEC_MDI_SELECT_INPUT */
	0x3, 0x0d4,  0x01f5, 0x468,  0x0, 0x954,
	/** RDATA[1], EIM_EB3, alt 3, IOMUXC_FEC_FEC_RDATA_1_SELECT_INPUT */
	0x3, 0x0d8,  0x0180, 0x46c,  0x0, 0x95c,
	/** RDATA[2], EIM_CS2, alt 3, IOMUXC_FEC_FEC_RDATA_2_SELECT_INPUT */
	0x3, 0x0e8,  0x0180, 0x47c,  0x0, 0x960,
	/** RDATA[3], EIM_CS3, alt 3, IOMUXC_FEC_FEC_RDATA_3_SELECT_INPUT */
	0x3, 0x0ec,  0x0180, 0x480,  0x0, 0x964,
	/** FEC RX_ER, EIM_CS4, alt 3, IOMUXC_FEC_FEC_RX_ER_SELECT_INPUT */
	0x3, 0x0f0,  0x0180, 0x484,  0x0, 0x970,
	/** CRS, EIM_CS5, alt 3, IOMUXC_FEC_FEC_CRS_SELECT_INPUT */
	0x3, 0x0f4,  0x0180, 0x488,  0x0, 0x950,
	/** COL, NANDF_RB2, alt 1, IOMUXC_FEC_FEC_COL_SELECT_INPUT */
	0x1, 0x124,  0x2180, 0x500,  0x0, 0x94c,
	/** RX_CLK, NANDF_RB3, alt 1, IOMUXC_FEC_FEC_RX_CLK_SELECT_INPUT */
	0x1, 0x128,  0x2180, 0x504,  0x0, 0x968,
	/* TX_ER, NANDF_CS2 */
	0x2, 0x138,  0x2004, 0x520,
	/** MDC, NANDF_CS3, alt 2 */
	0x2, 0x13c,  0x2004, 0x524,
	/** TDATA[1], NANDF_CS4 */
	0x2, 0x140,  0x2004, 0x528,
	/** TDATA[2], NANDF_CS5 */
	0x2, 0x144,  0x2004, 0x52c,
	/** TDATA[3], NANDF_CS6 */
	0x2, 0x148,  0x2004, 0x530,
	/** TX_EN, NANDF_CS7 */
	0x1, 0x14c,  0x2004, 0x534,
	/** TX_CLK, NANDF_RDY_INT, alt 1, IOMUXC_FEC_FEC_TX_CLK_SELECT_INPUT */
	0x1, 0x150,  0x2180, 0x538,  0x0, 0x974,
	/** RX_DV, NANDF_D11, alt 2, IOMUXC_FEC_FEC_RX_DV_SELECT_INPUT */
	0x2, 0x164,  0x2180, 0x54c,  0x0, 0x96c,
	/** RDATA[0], NANDF_D9, alt 2, IOMUXC_FEC_FEC_RDATA_0_SELECT_INPUT */
	0x2, 0x16c,  0x2180, 0x554,  0x0, 0x958,
	/** TDATA[0], NANDF_D8 */
	0x2, 0x170,  0x2004, 0x558,
	0,0
};
#endif

unsigned short gpio_pins[] = {
	/*
	 * Outputs: gpio3 pins 3,4,5,6
	 * Inputs: gpio3 pins 7,8,9,16
	 */
	0x3, 0x108, 0x0024, 0x4e4, 0x0, 0x980,	/* NANDF_WE_B - gpio3[3], G_IN_3_SELECT_INPUT */
	0x3, 0x10c, 0x0024, 0x4e8, 0x0, 0x984,	/* NANDF_RE_B - gpio3[4], G_IN_4_SELECT_INPUT */
	0x3, 0x110, 0x0004, 0x4ec, 0x0, 0x988,	/* NANDF_ALE - gpio3[5], G_IN_5_SELECT_INPUT */
	0x3, 0x114, 0x0004, 0x4f0, 0x0, 0x98c,	/* NANDF_CLE - gpio3[6], G_IN_6_SELECT_INPUT */
	0x3, 0x118, 0x20e4, 0x4f4, 0x0, 0x990,	/* NANDF_WP_B - gpio3[7], G_IN_7_SELECT_INPUT */
	0x3, 0x11c, 0x20e4, 0x4f8, 0x0, 0x994,	/* NANDF_RB0 - gpio3[8], G_IN_8_SELECT_INPUT */
	0x3, 0x120, 0x20e4, 0x4fc,		/* NANDF_RB1 - gpio3[9] */
	0x3, 0x130, 0x20e4, 0x518,		/* NANDF_CS0 - gpio3[16] */
	0x1, 0x060, 0x0000, 0x3f4,		/* EIM_D17 - gpio2[1] */
	0,0
};

static void setup_pins(unsigned short * pins)
{
	unsigned int base = IOMUXC_BASE_ADDR;
	unsigned val;
	unsigned offset;
	for (;;) {
		val = *pins++;
		offset = *pins++;
		if (!offset)
			break;
		writel(val, base + offset);
	}
}

static void setup_gpios(void)
{
	Set_GPIO_output_val(GPIO_NUMBER(3, 3), 1);	/* Gpio connector pin 5 */
	Set_GPIO_output_val(GPIO_NUMBER(3, 4), 1);	/* Gpio connector pin 4 */
	Set_GPIO_output_val(GPIO_NUMBER(3, 5), 1);	/* Gpio connector pin 3 */
	Set_GPIO_output_val(GPIO_NUMBER(3, 6), 1);	/* Gpio connector pin 2 */
	Set_GPIO_input(GPIO_NUMBER(3, 7));		/* Gpio connector pin 9 */
	Set_GPIO_input(GPIO_NUMBER(3, 8));		/* Gpio connector pin 8 */
	Set_GPIO_input(GPIO_NUMBER(3, 9));		/* Gpio connector pin 7 */
	Set_GPIO_input(GPIO_NUMBER(3, 16));		/* Gpio connector pin 6 */
	Set_GPIO_input(GPIO_NUMBER(2, 1));		/* Pic16F616 interrupt */
	setup_pins(gpio_pins);
}

#ifdef CONFIG_I2C_MXC
static void setup_i2c(unsigned int module_base)
{
	switch (module_base) {
	case I2C1_BASE_ADDR:
		writel(0x14, IOMUXC_BASE_ADDR + 0x5c); /* i2c1 SDA, EIM_D16 */
		writel(0x10d, IOMUXC_BASE_ADDR + 0x3f0);
		writel(0x0, IOMUXC_BASE_ADDR + 0x9B4);

		writel(0x14, IOMUXC_BASE_ADDR + 0x68); /* i2c2 SCL, EIM_D19 */
		writel(0x10d, IOMUXC_BASE_ADDR + 0x3fc);
		writel(0x0, IOMUXC_BASE_ADDR + 0x9B0);
		break;
	case I2C2_BASE_ADDR:
		/* dummy here*/
		break;
	default:
		printf("Invalid I2C base: 0x%x\n", module_base);
		break;
	}
}

int toggle_i2c(void *toggle_data)
{
	unsigned i;
	unsigned sda_gp = GPIO_NUMBER(2, 0);
	unsigned scl_gp = GPIO_NUMBER(2, 3);
	Set_GPIO_input(sda_gp);
	Set_GPIO_input(scl_gp);
	//gpio mode
	writel(0x11, IOMUXC_BASE_ADDR + 0x5c); /* i2c1 SDA, EIM_D16 */
	writel(0x11, IOMUXC_BASE_ADDR + 0x68); /* i2c2 SCL, EIM_D19 */

	printf("%s sda=%i scl=%i sda.gp=0x%x scl.gp=0x%x\n", __func__, gpio_get_value(sda_gp), gpio_get_value(scl_gp), sda_gp, scl_gp);
	/* Send high and low on the SCL line */
	for (i = 0; i < 9; i++) {
		Set_GPIO_output_val(scl_gp, 0);
		udelay(20);
		Set_GPIO_input(scl_gp);
		udelay(10);
		printf("%s sda=%i scl=%i\n", __func__, gpio_get_value(sda_gp), gpio_get_value(scl_gp));
		udelay(10);
	}
	//back to I2C mode
	writel(0x14, IOMUXC_BASE_ADDR + 0x5c); /* i2c1 SDA, EIM_D16 */
	writel(0x14, IOMUXC_BASE_ADDR + 0x68); /* i2c2 SCL, EIM_D19 */
	return 0;
}

static void setup_core_voltage_i2c(void)
{
	unsigned int reg;
	unsigned char buf[1] = { 0 };

	puts("PMIC Mode: linear\n");

	writel(CONFIG_SYS_ARM_PODF, CCM_BASE_ADDR + CLKCTL_CACRR);
	reg = readl(GPIO2_BASE_ADDR + 0x0);
	reg &= ~0x4000;  /* Lower reset line */
	writel(reg, GPIO2_BASE_ADDR + 0x0);

	reg = readl(GPIO2_BASE_ADDR + 0x4);
	reg |= 0x4000;  /* configure GPIO lines as output */
	writel(reg, GPIO2_BASE_ADDR + 0x4);

	/* Reset the ethernet controller over GPIO */
	writel(0x1, IOMUXC_BASE_ADDR + 0x0AC);

	/*Configure LDO4*/
	i2c_read(0x34, 0x12, 1, buf, 1);
	buf[0] = buf[0] | 0x40;
	if (i2c_write(0x34, 0x12, 1, buf, 1)) {
		puts("write to PMIC 0x12 failed!\n");
		return;
	}
	i2c_read(0x34, 0x12, 1, buf, 1);
	printf("PMIC 0x12: 0x%x \n", buf[0]);

	i2c_read(0x34, 0x10, 1, buf, 1);
	buf[0] = buf[0] | 0x40;
	if (i2c_write(0x34, 0x10, 1, buf, 1)) {
		puts("write to PMIC 0x10 failed!\n");
		return;
	}
	i2c_read(0x34, 0x10, 1, buf, 1);
	printf("PMIC 0x10: 0x%x \n", buf[0]);

	udelay(500);

	reg = readl(GPIO2_BASE_ADDR + 0x0);
	reg |= 0x4000;
	writel(reg, GPIO2_BASE_ADDR + 0x0);
}
#endif

#ifdef CONFIG_IMX_ECSPI
static void setup_core_voltage_spi(void)
{
	struct spi_slave *slave;
	unsigned int val;

	puts("PMIC Mode: SPI\n");

#define REV_ATLAS_LITE_1_0         0x8
#define REV_ATLAS_LITE_1_1         0x9
#define REV_ATLAS_LITE_2_0         0x10
#define REV_ATLAS_LITE_2_1         0x11

	slave = spi_pmic_probe();

	/* Write needed to Power Gate 2 register */
	val = pmic_reg(slave, 34, 0, 0);
	val &= ~0x10000;
	pmic_reg(slave, 34, val, 1);

	/* Write needed to update Charger 0 */
	pmic_reg(slave, 48, 0x0023807f, 1);

	/* power up the system first */
	pmic_reg(slave, 34, 0x00200000, 1);

// Macro to convert voltage specified in mV to PMIC voltage code
#define mV_TO_CODE(mV)	(((mV) - 600) / 25)

	if (get_srev() <= SREV2_0) {
		/* Set core voltage to 1.1V */
		val = pmic_reg(slave, 24, 0, 0);
		val = (val & (~0x1f)) | mV_TO_CODE(1100);
		pmic_reg(slave, 24, val, 1);

		/* Setup VCC (SW2) to 1.25 */
		val = pmic_reg(slave, 25, 0, 0);
		val = (val & (~0x1f)) | mV_TO_CODE(1250);
		pmic_reg(slave, 25, val, 1);

		/* Setup 1V2_DIG1 (SW3) to 1.275 */
		val = pmic_reg(slave, 26, 0, 0);
		val = (val & (~0x1f)) | mV_TO_CODE(1275);
		pmic_reg(slave, 26, val, 1);
		udelay(50);
		/* Raise the core frequency to 800MHz */
		writel(CONFIG_SYS_ARM_PODF, CCM_BASE_ADDR + CLKCTL_CACRR);
	} else {
		/* TO 3.0 */
		/* Set core voltage to 1.1V */
		val = pmic_reg(slave, 24, 0, 0);
		val = (val & (~0x1f)) | mV_TO_CODE(1100);
		pmic_reg(slave, 24, val, 1);

		/* Setup VCC (SW2) to 1.225 */
		val = pmic_reg(slave, 25, 0, 0);
		val = (val & (~0x1f)) | mV_TO_CODE(1225);
		pmic_reg(slave, 25, val, 1);

		/* Setup 1V2_DIG1 (SW3) to 1.2 */
		val = pmic_reg(slave, 26, 0, 0);
		val = (val & (~0x1f)) | mV_TO_CODE(1200);
		pmic_reg(slave, 26, val, 1);
	}

	if (((pmic_reg(slave, 7, 0, 0) & 0x1f) < REV_ATLAS_LITE_2_0) ||
		(((pmic_reg(slave, 7, 0, 0) >> 9) & 0x3) == 0)) {
		/* Set switchers in PWM mode for Atlas 2.0 and lower */
		/* Setup the switcher mode for SW1 & SW2*/
		val = pmic_reg(slave, 28, 0, 0);
		val = (val & (~0x3c0f)) | 0x1405;
		pmic_reg(slave, 28, val, 1);

		/* Setup the switcher mode for SW3 & SW4 */
		val = pmic_reg(slave, 29, 0, 0);
		val = (val & (~0xf0f)) | 0x505;
		pmic_reg(slave, 29, val, 1);
	} else {
		/* Set switchers in Auto in NORMAL mode & STANDBY mode for Atlas 2.0a */
		/* Setup the switcher mode for SW1 & SW2*/
		val = pmic_reg(slave, 28, 0, 0);
		val = (val & (~0x3c0f)) | 0x2008;
		pmic_reg(slave, 28, val, 1);

		/* Setup the switcher mode for SW3 & SW4 */
		val = pmic_reg(slave, 29, 0, 0);
		val = (val & (~0xf0f)) | 0x808;
		pmic_reg(slave, 29, val, 1);
	}

	/* Set VDIG to 1.65V, VGEN3 to 1.8V, VCAM to 2.5V */
	val = pmic_reg(slave, 30, 0, 0);
	val &= ~0x34030;
	val |= 0x10020;
	pmic_reg(slave, 30, val, 1);

	/* Set VVIDEO to 2.775V, VAUDIO to 3V, VSD to 3.15V */
	val = pmic_reg(slave, 31, 0, 0);
	val &= ~0x1fc;
	val |= 0x1f4;
	pmic_reg(slave, 31, val, 1);

	/* Configure VGEN3 and VCAM regulators to use external PNP */
	val = 0x208;
	pmic_reg(slave, 33, val, 1);
	udelay(200);
	Set_GPIO_output_val(GPIO_NUMBER(2, 14), 0); /* lower reset line  */

	/* Reset the ethernet controller over GPIO */
	writel(0x1, IOMUXC_BASE_ADDR + 0x0ac);

	/* Enable VGEN3, VCAM, VAUDIO, VVIDEO, VSD regulators */
	val = 0x49249;
	pmic_reg(slave, 33, val, 1);

	udelay(500);

	Set_GPIO_output_val(GPIO_NUMBER(2, 14), 1); /* raise reset line */

	spi_pmic_free(slave);
}
#endif

static void setup_pmic(void)
{
	/*
	 * eCSPI1 pads (for PMIC)
	 */
	unsigned int pad = PAD_CTL_HYS_ENABLE | PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST ;
	Set_GPIO_output_val(GPIO_NUMBER(4, 24), 0);	/* SS0 high active */
	Set_GPIO_output_val(GPIO_NUMBER(4, 25), 1);	/* SS1 low active */
	mxc_request_iomux(MX51_PIN_CSPI1_MOSI,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_CSPI1_MOSI, pad);
	mxc_request_iomux(MX51_PIN_CSPI1_MISO,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_CSPI1_MISO,pad);
	mxc_request_iomux(MX51_PIN_CSPI1_SS0,IOMUX_CONFIG_ALT3);
	mxc_iomux_set_pad(MX51_PIN_CSPI1_SS0,pad);
	mxc_request_iomux(MX51_PIN_CSPI1_SS1,IOMUX_CONFIG_ALT3);
	mxc_iomux_set_pad(MX51_PIN_CSPI1_SS1,pad);
	mxc_request_iomux(MX51_PIN_CSPI1_RDY,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_CSPI1_RDY, PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_LOW);
	mxc_request_iomux(MX51_PIN_CSPI1_SCLK,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_CSPI1_SCLK,pad);
}

void init_display_pins(void)
{
	unsigned int pad = PAD_CTL_HYS_NONE | PAD_CTL_DRV_MEDIUM | PAD_CTL_SRE_FAST ;
	mxc_request_iomux(MX51_PIN_DISP1_DAT0,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT0,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT1,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT1,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT2,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT2,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT3,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT3,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT4,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT4,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT5,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT5,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT6,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT6,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT7,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT7,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT8,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT8,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT9,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT9,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT10,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT10,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT11,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT11,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT12,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT12,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT13,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT13,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT14,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT14,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT15,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT15,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT16,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT16,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT17,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT17,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT18,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT18,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT19,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT19,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT20,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT20,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT21,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT21,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT22,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT22,pad);
	mxc_request_iomux(MX51_PIN_DISP1_DAT23,IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX51_PIN_DISP1_DAT23,pad);

	mxc_request_iomux(MX51_PIN_DI1_PIN2,IOMUX_CONFIG_ALT0);	//Hsync
	mxc_iomux_set_pad(MX51_PIN_DI1_PIN2,pad);
	mxc_request_iomux(MX51_PIN_DI1_PIN3,IOMUX_CONFIG_ALT0);	//Vsync
	mxc_iomux_set_pad(MX51_PIN_DI1_PIN3,pad);
	mxc_iomux_set_pad(MX51_PIN_DI1_DISP_CLK,pad);		//PCLK
	mxc_iomux_set_pad(MX51_PIN_DI1_PIN15,pad);		//DRDY - (DE) or (OE) 

	mxc_request_iomux(MX51_PIN_DI1_PIN11,IOMUX_CONFIG_ALT1);
	mxc_request_iomux(MX51_PIN_DI1_PIN12,IOMUX_CONFIG_ALT1);
	mxc_request_iomux(MX51_PIN_DI1_PIN13,IOMUX_CONFIG_ALT1);
	mxc_request_iomux(MX51_PIN_DI1_D0_CS,IOMUX_CONFIG_ALT1);
	mxc_request_iomux(MX51_PIN_DI1_D1_CS,IOMUX_CONFIG_ALT4);
	mxc_request_iomux(MX51_PIN_DI_GP4,IOMUX_CONFIG_ALT4);
}

#ifdef CONFIG_NET_MULTI

int board_eth_init(bd_t *bis)
{
	int rc = -ENODEV;

	return rc;
}
#endif

#ifdef CONFIG_CMD_MMC

struct fsl_esdhc_cfg esdhc_cfg[2] = {
	{MMC_SDHC1_BASE_ADDR, 1, 1},
	{MMC_SDHC2_BASE_ADDR, 1, 1},
};

#ifdef CONFIG_DYNAMIC_MMC_DEVNO
int get_mmc_env_devno(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	return (soc_sbmr & 0x00180000) ? 1 : 0;
}
#endif

int esdhc_gpio_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	for (index = 0; index < CONFIG_SYS_FSL_ESDHC_NUM;
		++index) {
		switch (index) {
		case 0:
			mxc_request_iomux(MX51_PIN_SD1_CMD,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_CLK,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

			mxc_request_iomux(MX51_PIN_SD1_DATA0,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_DATA1,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_DATA2,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD1_DATA3,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_iomux_set_pad(MX51_PIN_SD1_CMD,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_CLK,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_NONE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA0,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA1,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA2,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD1_DATA3,
					PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
					PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PD |
					PAD_CTL_PUE_PULL |
					PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
			break;
		case 1:
			mxc_request_iomux(MX51_PIN_SD2_CMD,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_CLK,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

			mxc_request_iomux(MX51_PIN_SD2_DATA0,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_DATA1,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_DATA2,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_request_iomux(MX51_PIN_SD2_DATA3,
					IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
			mxc_iomux_set_pad(MX51_PIN_SD2_CMD,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_CLK,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA0,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA1,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA2,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			mxc_iomux_set_pad(MX51_PIN_SD2_DATA3,
					PAD_CTL_DRV_MAX | PAD_CTL_22K_PU |
					PAD_CTL_SRE_FAST);
			break;
		default:
			printf("Warning: you configured more ESDHC controller"
				"(%d) as supported by the board(2)\n",
				CONFIG_SYS_FSL_ESDHC_NUM);
			return status;
			break;
		}
		status |= fsl_esdhc_initialize(bis, &esdhc_cfg[index]);
	}

	return status;
}

int board_mmc_init(bd_t *bis)
{
	if (!esdhc_gpio_init(bis))
		return 0;
	else
		return -1;
}

#endif

#if defined(CONFIG_MXC_KPD)
int setup_mxc_kpd(void)
{
	mxc_request_iomux(MX51_PIN_KEY_COL0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL3, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL4, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_COL5, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW0, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW1, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW2, IOMUX_CONFIG_ALT0);
	mxc_request_iomux(MX51_PIN_KEY_ROW3, IOMUX_CONFIG_ALT0);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC

int sdhc_init(void)
{
	u32 interface_esdhc = 0;
	s32 status = 0;

	interface_esdhc = (readl(SRC_BASE_ADDR + 0x4) & (0x00180000)) >> 19;

	switch (interface_esdhc) {
	case 0:

		esdhc_base_pointer = (volatile u32 *)MMC_SDHC1_BASE_ADDR;

		mxc_request_iomux(MX51_PIN_SD1_CMD,
			  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_CLK,
			  IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);

		mxc_request_iomux(MX51_PIN_SD1_DATA0,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_DATA1,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_DATA2,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_request_iomux(MX51_PIN_SD1_DATA3,
				IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION);
		mxc_iomux_set_pad(MX51_PIN_SD1_CMD,
				PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				PAD_CTL_PUE_PULL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_CLK,
				PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				PAD_CTL_HYS_NONE | PAD_CTL_47K_PU |
				PAD_CTL_PUE_PULL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA0,
				PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				PAD_CTL_PUE_PULL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA1,
				PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				PAD_CTL_PUE_PULL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA2,
				PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				PAD_CTL_HYS_ENABLE | PAD_CTL_47K_PU |
				PAD_CTL_PUE_PULL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX51_PIN_SD1_DATA3,
				PAD_CTL_DRV_MAX | PAD_CTL_DRV_VOT_HIGH |
				PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PD |
				PAD_CTL_PUE_PULL |
				PAD_CTL_PKE_ENABLE | PAD_CTL_SRE_FAST);
		break;
	case 1:
		status = 1;
		break;
	case 2:
		status = 1;
		break;
	case 3:
		status = 1;
		break;
	default:
		status = 1;
		break;
	}

	return status = 1;
}

#endif

extern void setup_display(void);

int board_init(void)
{
#ifdef CONFIG_MFG
/* MFG firmware need reset usb to avoid host crash firstly */
#define USBCMD 0x140
	int val = readl(OTG_BASE_ADDR + USBCMD);
	val &= ~0x1; /*RS bit*/
	writel(val, OTG_BASE_ADDR + USBCMD);
#endif
	setup_uart();
	debug("%s a", __func__);
	setup_gpios();
	debug_putc('b');
	setup_boot_device();
	debug_putc('c');

	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;	/* board id for linux */
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	debug_putc('e');
	setup_nfc();
	debug_putc('f');
	setup_expio();
	debug_putc('g');
#ifdef CONFIG_MXC_FEC
	setup_pins(fec_setup_pins);
#endif
#ifdef CONFIG_I2C_MXC
	setup_i2c(I2C1_BASE_ADDR);
#endif
	debug_putc('h');

	setup_pmic();
	debug_putc('i');
	debug_putc('\n');
	return 0;
}

extern void mxc_fec_eth_set_mac_addr(const unsigned char *macaddr);

int parse_mac(char const *cmac, unsigned char *mac)
{
	int i = 0;
	char *end;
	int ret = (cmac) ? 0 : -1;

	do {
		mac[i++] = cmac ? simple_strtoul(cmac, &end, 16) : 0;
		if (i == 6)
			break;
		if (cmac) {
			cmac = (*end) ? end+1 : end;
			if ((*end != '-') && (*end != ':'))
				ret = -1;
		}
	} while (1);
	return ret;
}

static int get_rom_mac(unsigned char *mac)
{
	char *cmac = getenv("ethaddr");
	if (cmac) {
		if (!parse_mac(cmac, mac)) {
			mxc_fec_eth_set_mac_addr(mac);
			return 0;
		}
	}
	return -1;
}

int misc_init_r(void)
{
	unsigned char macAddr[6];
	if (!get_rom_mac(macAddr)) {
		printf( "Mac address %02x:%02x:%02x:%02x:%02x:%02x\n",
				macAddr[0], macAddr[1], macAddr[2],
				macAddr[3], macAddr[4], macAddr[5] );
	} else
		printf( "No mac address assigned\n" );
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
	 .cmd = CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC,
	 .args = CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC,
	 },
};

int check_recovery_cmd_file(void)
{
	disk_partition_t info;
	ulong part_length;
	int filelen;

	switch (get_boot_device()) {
	case MMC_BOOT:
		{
			block_dev_desc_t *dev_desc = NULL;
			struct mmc *mmc = find_mmc_device(0);

			dev_desc = get_dev("mmc", 0);

			if (NULL == dev_desc) {
				puts("** Block device MMC 0 not supported\n");
				return 0;
			}

			mmc_init(mmc);

			if (get_partition_info(dev_desc,
					CONFIG_ANDROID_CACHE_PARTITION_MMC,
					&info)) {
				printf("** Bad partition %d **\n",
					CONFIG_ANDROID_CACHE_PARTITION_MMC);
				return 0;
			}

			part_length = ext2fs_set_blk_dev(dev_desc,
							CONFIG_ANDROID_CACHE_PARTITION_MMC);
			if (part_length == 0) {
				printf("** Bad partition - mmc 0:%d **\n",
					CONFIG_ANDROID_CACHE_PARTITION_MMC);
				ext2fs_close();
				return 0;
			}

			if (!ext2fs_mount(part_length)) {
				printf("** Bad ext2 partition or disk - mmc 0:%d **\n",
					CONFIG_ANDROID_CACHE_PARTITION_MMC);
				ext2fs_close();
				return 0;
			}

			filelen = ext2fs_open(CONFIG_ANDROID_RECOVERY_CMD_FILE);

			ext2fs_close();
		}
		break;
	case NAND_BOOT:
		return 0;
		break;
	case SPI_NOR_BOOT:
		return 0;
		break;
	case UNKNOWN_BOOT:
	default:
		return 0;
		break;
	}

	return (filelen > 0) ? 1 : 0;

}
#endif
void bus_i2c_init(void *base, int speed, int unused,
		int (*toggle_fn)(void *p), void *toggle_data);

#ifdef BOARD_LATE_INIT
int board_late_init(void)
{
	/* GPIO3[5-6] are used to enable/disable I2C interface to HDMI transmitter */
	/* set them as GPIOs */
	mxc_request_iomux(MX51_PIN_NANDF_ALE, IOMUX_CONFIG_ALT3);
	mxc_request_iomux(MX51_PIN_NANDF_CLE, IOMUX_CONFIG_ALT3);
	mxc_request_iomux(MX51_PIN_DISPB2_SER_DIN, IOMUX_CONFIG_ALT4); /* GPIO3[5] */
	__REG(IOMUXC_GPIO3_IPP_IND_G_IN_5_SELECT_INPUT) = 1 ; /* GPIO3[5] is connected to DISPB2_SER_DIN */

	Set_GPIO_output_val(GPIO_NUMBER(3, 5), 0);
	Set_GPIO_output_val(GPIO_NUMBER(3, 6), 1);
	
	/* GPIO1:2 is PWM0 and controls the backlight. Set it as GPIO and high */
	mxc_request_iomux(MX51_PIN_GPIO1_2, IOMUX_CONFIG_ALT0);
	Set_GPIO_output_val(GPIO_NUMBER(1, 2), 1);

	setup_display();

#ifdef CONFIG_I2C_MXC
	bus_i2c_init((void *)I2C1_BASE_ADDR, CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE,
			toggle_i2c, NULL);
//	if (!i2c_probe(0x34))
//		setup_core_voltage_i2c();
//	else
#endif
#ifdef CONFIG_IMX_ECSPI
		setup_core_voltage_spi();
#endif

	return 0;
}
#endif

int checkboard(void)
{
	unsigned srev = get_srev();
	printf("Board: MX51 Nitrogen %d.%d [", (srev >> 4) + 1, srev & 0x0f);

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
	case NAND_BOOT:
		printf("NAND\n");
		break;
	case SPI_NOR_BOOT:
		printf("SPI NOR\n");
		break;
	case MMC_BOOT:
		printf("MMC\n");
		break;
	case UNKNOWN_BOOT:
	default:
		printf("UNKNOWN\n");
		break;
	}
	return 0;
}


/*
 * Translate the virtual address of ram space to physical address
 * It is dependent on the implementation of mmu_init
 */
unsigned long iomem_to_phys(unsigned long virt)
{
	if (virt < (PHYS_SDRAM_1_SIZE - 0x100000))
		return (unsigned long)(virt + PHYS_SDRAM_1);

	if (virt >= 0xE0000000)
		return (unsigned long)((virt - 0xE0000000) + PHYS_SDRAM_1);

	return (unsigned long)virt;
}

/*
 * Remap the physical address of ram space to uncacheable virtual address space
 * It is dependent on the implementation of hal_mmu_init
 */
void __iounmap(void *addr)
{
	return;
}

void *__ioremap(unsigned long offset, size_t size, unsigned long flags)
{
	if (1 == flags) {
		/* 0xE0000000~0xFFFFFFFF is uncacheable
		meory space which is mapped to SDRAM */
		if (offset >= PHYS_SDRAM_1 &&
			offset < (PHYS_SDRAM_1 + PHYS_SDRAM_1_SIZE))
			return (void *)((offset - PHYS_SDRAM_1) + 0xE0000000);
		else
			return NULL;
	} else
		return (void *)offset;
}


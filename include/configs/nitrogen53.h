/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the MX53-EVK Freescale board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/mx53.h>
#define CONFIG_MACH_TYPE	MACH_TYPE_MX53_NITROGEN
#define CONFIG_BOARD_NAME	"MX53-Nitrogen"

 /* High Level Configuration Options */
#define CONFIG_ARMV7		1	/* This is armv7 Cortex-A8 CPU core */
#define CONFIG_MXC_GPIO
#define CONFIG_MXC		1
#define CONFIG_MX53
#define CONFIG_VIDEO_IMX5X	1
#define CONFIG_FLASH_HEADER	1
#define CONFIG_FLASH_HEADER_OFFSET 0x400
#define CONFIG_MX53_CLK32	32768

#define CONFIG_MX53_HCLK_FREQ	24000000
#define CONFIG_SYS_PLL1_FREQ	1000
#define CONFIG_SYS_PLL2_FREQ    400
#define CONFIG_SYS_AHB_PODF     1
#define CONFIG_SYS_AXIA_PODF    1
#define CONFIG_SYS_AXIB_PODF    2

#define CONFIG_ARCH_CPU_INIT
#define CONFIG_ARCH_MMU

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_SYS_64BIT_VSPRINTF

#define BOARD_LATE_INIT
/*
 * Disabled for now due to build problems under Debian and a significant
 * increase in the final file size: 144260 vs. 109536 Bytes.
 */

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_REVISION_TAG		1
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		((CONFIG_ENV_SIZE + (2 * 1024 * 1024) + 0x3fff) & ~0x3fff)
/* size in bytes reserved for initial data */
#define CONFIG_SYS_GBL_DATA_SIZE	128

#define CONFIG_PWM2_PERIOD	0xf4	/* 32768 Hz */
#define CONFIG_PWM2_DUTY	128	/* 128 out of 256 */

/*
 * Hardware drivers
 */
#define CONFIG_MXC_UART 1
#define CONFIG_UART_BASE_ADDR   UART2_BASE_ADDR
#define CONFIG_TFP410_LDO10
#define CONFIG_TFP410_BUS ((void *)I2C2_BASE_ADDR)

/*
 * Android support Configs
 */

/* Android fastboot configs */
#define CONFIG_USB_DEVICE
#define CONFIG_IMX_UDC
#define CONFIG_FASTBOOT
#define CONFIG_FASTBOOT_STORAGE_EMMC_SATA
#define CONFIG_FASTBOOT_STORAGE_SF
//#define CONFIG_FASTBOOT_STORAGE_NAND
#define CONFIG_FASTBOOT_VENDOR_ID      0x18d1
#define CONFIG_FASTBOOT_PRODUCT_ID     0x0c01
#define CONFIG_FASTBOOT_BCD_DEVICE     0x0311
#define CONFIG_FASTBOOT_MANUFACTURER_STR  "Freescale"
#define CONFIG_FASTBOOT_PRODUCT_NAME_STR "i.mx53 Nit"
#define CONFIG_FASTBOOT_CONFIGURATION_STR  "Android fastboot"
#define CONFIG_FASTBOOT_INTERFACE_STR         "Android fastboot"
#define CONFIG_FASTBOOT_SERIAL_NUM   "12345"
#define CONFIG_FASTBOOT_TRANSFER_BUF 0x80000000
#define CONFIG_FASTBOOT_TRANSFER_BUF_SIZE 0x09400000 /* 148M byte */

#define CONFIG_ANDROID_BOOT_PARTITION_MMC 1
#define CONFIG_ANDROID_RECOVERY_PARTITION_MMC 2
#define CONFIG_ANDROID_CACHE_PARTITION_MMC 6
#define CONFIG_ANDROID_RECOVERY
#define CONFIG_ANDROID_RECOVERY_BOOTARGS_MMC NULL
#define CONFIG_ANDROID_RECOVERY_BOOTCMD_MMC  \
        "booti mmc1 recovery"
#define CONFIG_ANDROID_RECOVERY_CMD_FILE "/recovery/command"

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

/***********************************************************
 * Command definition
 ***********************************************************/

#include <config_cmd_default.h>

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_DNS
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_NET_RETRY_COUNT	100
//#define CONFIG_NET_MULTI 1
#define CONFIG_CMD_SPI
#define CONFIG_CMD_SF
#define CONFIG_CMD_MMC
#define CONFIG_CMD_IIM
#define CONFIG_CMD_I2C

#define CONFIG_LCDPANEL		1
#define CONFIG_CMD_LCDPANEL	1
#define CONFIG_LCD_MULTI 	1
#define CONFIG_CMD_BMP		1

#define CONFIG_CMD_CLOCK
#define CONFIG_REF_CLK_FREQ CONFIG_MX53_HCLK_FREQ

#define CONFIG_CMD_SATA
#define CONFIG_CMD_BMODE

///////////
/*
 * FUSE Configs
 * */
#ifdef CONFIG_CMD_IIM
	#define CONFIG_IMX_IIM
	#define IMX_IIM_BASE    IIM_BASE_ADDR
	#define CONFIG_IIM_MAC_BANK	1
	#define CONFIG_IIM_MAC_ROW	9
#endif

/*
 * SPI Configs
 * */
#ifdef CONFIG_CMD_SF
	#define CONFIG_FSL_SF		1
#if 1
	#define CONFIG_SPI_FLASH       1
	#define CONFIG_SPI_FLASH_ATMEL 1
	#define CONFIG_SPI_FLASH_STMICRO 1
	#define CONFIG_SPI_FLASH_SST	1
#else
	#define CONFIG_SPI_FLASH_IMX_ATMEL	1
#endif
	#define CONFIG_SPI_FLASH_CS	1
	#define CONFIG_IMX_ECSPI
	#define IMX_CSPI_VER_2_3        1

	#define MAX_SPI_BYTES		(64 * 4)
#endif

/*
 * MMC Configs
 * */
#ifdef CONFIG_CMD_MMC
	#define CONFIG_MMC				1
	#define CONFIG_MMC_BASE				0
	#define CONFIG_FSL_MMC				1
	#define CONFIG_GENERIC_MMC
	#define CONFIG_IMX_MMC
	#define CONFIG_SYS_FSL_ESDHC_NUM	2
	#define CONFIG_SYS_FSL_ESDHC_ADDR       0
	#define CONFIG_SYS_MMC_ENV_DEV	0
	#define CONFIG_DOS_PARTITION	1
	#define CONFIG_CMD_FAT		1
	#define CONFIG_CMD_EXT2		1
	#define CONFIG_DYNAMIC_MMC_DEVNO
//	#define CONFIG_BOOT_PARTITION_ACCESS
#endif

/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
	#define CONFIG_DWC_AHSATA
	#define CONFIG_SYS_SATA_MAX_DEVICE	1
	#define CONFIG_DWC_AHSATA_PORT_ID	0
	#define CONFIG_DWC_AHSATA_BASE_ADDR 	SATA_BASE_ADDR
	#define CONFIG_LBA48
	#define CONFIG_LIBATA
#endif

/*
 * I2C Configs
 */
#ifdef CONFIG_CMD_I2C
	#define CONFIG_I2C_MXC          1
	#define CONFIG_SYS_I2C1_SPEED	400000
	#define CONFIG_SYS_I2C2_SPEED	100000
	#define CONFIG_SYS_I2C3_SPEED	100000
	#define CONFIG_SYS_I2C_SPEED	CONFIG_SYS_I2C1_SPEED
	#define CONFIG_SYS_I2C_SLAVE	0xfe
	#define CONFIG_I2C_MULTI_BUS
#endif

/*
 * Eth Configs
 */
#define CONFIG_HAS_ETH1
#define CONFIG_NET_MULTI 1
#define CONFIG_MXC_FEC
#define CONFIG_MII
#define CONFIG_MII_GASKET
#define CONFIG_MII_GASKET_MODE_MII

//#define CONFIG_GET_FEC_MAC_ADDR_FROM_IIM
#define CONFIG_IIM_MAC_ADDR_OFFSET      0x24

#define CONFIG_FEC0_IOBASE	FEC_BASE_ADDR
#define CONFIG_FEC0_PINMUX	-1
#define CONFIG_FEC0_PHY_ADDR	0x05
//#define CONFIG_DISCOVER_PHY
#define CONFIG_FEC0_MIIBASE 	-1

#define CONFIG_MISC_INIT_R

#define CONFIG_CMD_ENV

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	1

#define CONFIG_PRIME	"FEC0"

#define CONFIG_LOADADDR		0x70800000	/* loadaddr env var */
#define CONFIG_RD_LOADADDR	(CONFIG_LOADADDR + 0x300000)

#define	CONFIG_EXTRA_ENV_SETTINGS	\
	"ethprime=FEC0\0"		\
	"upgradeu=fatload mmc 0 70008000 n53_upgrade && source 70008000\0" \
	"clearenv=sf probe 1 && sf erase 0x5f000 0x1000 && echo 'environment reset to factory defaults';\0" \

#define CONFIG_ARP_TIMEOUT	200UL

#define CONFIG_BOOTCOMMAND	"fatload mmc 0 70008000 nitrogen53_bootscript* && source 70008000 ; errmsg=\"Error running bootscript!\" ; lecho $errmsg ; echo $errmsg ;"

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */
#define CONFIG_SYS_PROMPT		"U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		256	/* Console I/O Buffer Size */
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE /* Boot Argument Buffer Size */

#define CONFIG_SYS_MEMTEST_START	0x80000000	/* memtest works on */
#define CONFIG_SYS_MEMTEST_END		0xa2000000
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_SYS_MEMTEST_SCRATCH	0x71000000

#undef	CONFIG_SYS_CLKS_IN_HZ		/* everything, incl board info, in Hz */

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_SYS_HZ				1000

#define CONFIG_CMDLINE_EDITING	1
#define CONFIG_SYS_HUSH_PARSER		1	/* Use the HUSH parser		*/
#ifdef	CONFIG_SYS_HUSH_PARSER
#define	CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#endif

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128 * 1024)	/* regular stack */

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1
#define PHYS_SDRAM_1		CSD0_BASE_ADDR
#define PHYS_SDRAM_1_SIZE	(1u * 1024 * 1024 * 1024)	/* GE board has 1G */
#define iomem_valid_addr(addr, size) \
	((addr >= PHYS_SDRAM_1) && (addr <= (PHYS_SDRAM_1 + PHYS_SDRAM_1_SIZE)))

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CONFIG_SYS_NO_FLASH

/* Monitor at beginning of flash */
/*  */
#define CONFIG_FSL_ENV_IN_SF		/* check here 1st */
#define CONFIG_FSL_ENV_IN_SF_FIRST
#define CONFIG_FSL_ENV_IN_MMC		/* check here next */


#if defined(CONFIG_FSL_ENV_IN_MMC)
	#define CONFIG_ENV_IS_IN_MMC		1
	#define CONFIG_ENV_MMC_OFFSET		(14 * 512)
	#define CONFIG_ENV_MMC_SIZE		(1 * 1024)
	#define CONFIG_ENV_MMC_SECT_SIZE	CONFIG_ENV_MMC_SIZE
#endif

#if defined(CONFIG_FSL_ENV_IN_SF)
	#define CONFIG_ENV_IS_IN_SPI_FLASH	1
	#define CONFIG_ENV_SPI_CS		1
	#define CONFIG_ENV_SF_SIZE		(4 * 1024)
	#define CONFIG_ENV_SF_OFFSET		((384 * 1024)-CONFIG_ENV_SF_SIZE)
	#define CONFIG_ENV_SF_SECT_SIZE		CONFIG_ENV_SF_SIZE
#endif

#if defined(CONFIG_FSL_ENV_IN_SF)
	/* choose the greater of the 2 sizes */
	#define CONFIG_ENV_SECT_SIZE	CONFIG_ENV_SF_SECT_SIZE
	#define CONFIG_ENV_SIZE		CONFIG_ENV_SF_SIZE
#else
#if defined(CONFIG_FSL_ENV_IN_MMC)
	#define CONFIG_ENV_SECT_SIZE    CONFIG_ENV_MMC_SECT_SIZE
	#define CONFIG_ENV_SIZE		CONFIG_ENV_MMC_SIZE
#else
	#define CONFIG_ENV_IS_NOWHERE	1
	#define CONFIG_ENV_SECT_SIZE    (128 * 1024)
	#define CONFIG_ENV_SIZE		CONFIG_ENV_SECT_SIZE
#endif
#endif

#define CONFIG_CMD_XMODEM
#define CONFIG_HW_WATCHDOG
#define CONFIG_CMD_ECHO
#define CONFIG_CMD_SOURCE
#define CONFIG_CMD_RUN

#define DA90_I2C_BUS ((void *)I2C1_BASE_ADDR)
#define DA90_I2C_ADDR 0x48
#define CONFIG_CMD_DA90_I2C_PMIC
#define N53_I2C_CONNECTOR_BUFFER_ENABLE		GPIO_NUMBER(3, 10)
#define PIN_N53_I2C_CONNECTOR_BUFFER		MX53_PIN_EIM_DA10
#endif				/* __CONFIG_H */

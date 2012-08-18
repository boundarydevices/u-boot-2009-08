/*
 * (C) Copyright 2007
 * Sascha Hauer, Pengutronix
 *
 * (C) Copyright 2009 Freescale Semiconductor, Inc.
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
#include <asm/arch/imx-regs.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/imx-common/boot_mode.h>

void boot_mode_apply(unsigned cfg_val)
{
	writel(cfg_val, &((struct srtc_regs *)SRTC_BASE_ADDR)->lpgr);
}
/*
 * cfg_val will be used for
 * Boot_cfg3[7:0]:Boot_cfg2[7:0]:Boot_cfg1[7:0]
 *
 * If bit 28 of LPGR is set upon watchdog reset,
 * bits[25:0] of LPGR will move to SBMR.
 */
const struct boot_mode soc_boot_modes[] = {
	{"normal",	MAKE_CFGVAL(0x00, 0x00, 0x00, 0x00)},
	/* usb or serial download */
	{"usb",		MAKE_CFGVAL(0x00, 0x00, 0x00, 0x13)},
	{"sata",	MAKE_CFGVAL(0x28, 0x00, 0x00, 0x12)},
	{"escpi1:0",	MAKE_CFGVAL(0x38, 0x20, 0x00, 0x12)},
	{"escpi1:1",	MAKE_CFGVAL(0x38, 0x20, 0x04, 0x12)},
	{"escpi1:2",	MAKE_CFGVAL(0x38, 0x20, 0x08, 0x12)},
	{"escpi1:3",	MAKE_CFGVAL(0x38, 0x20, 0x0c, 0x12)},
	/* 4 bit bus width */
	{"esdhc1",	MAKE_CFGVAL(0x40, 0x20, 0x00, 0x12)},
	{"esdhc2",	MAKE_CFGVAL(0x40, 0x20, 0x08, 0x12)},
	{"esdhc3",	MAKE_CFGVAL(0x40, 0x20, 0x10, 0x12)},
	{"esdhc4",	MAKE_CFGVAL(0x40, 0x20, 0x18, 0x12)},
	{NULL,		0},
};

/*
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

#ifndef __ASM_ARCH_MX5_IMX_REGS_H__
#define __ASM_ARCH_MX5_IMX_REGS_H__

struct srtc_regs {
	u32	lpscmr;		/* 0x00 */
	u32	lpsclr;		/* 0x04 */
	u32	lpsar;		/* 0x08 */
	u32	lpsmcr;		/* 0x0c */
	u32	lpcr;		/* 0x10 */
	u32	lpsr;		/* 0x14 */
	u32	lppdr;		/* 0x18 */
	u32	lpgr;		/* 0x1c */
	u32	hpcmr;		/* 0x20 */
	u32	hpclr;		/* 0x24 */
	u32	hpamr;		/* 0x28 */
	u32	hpalr;		/* 0x2c */
	u32	hpcr;		/* 0x30 */
	u32	hpisr;		/* 0x34 */
	u32	hpienr;		/* 0x38 */
};

#endif				/* __ASM_ARCH_MX5_IMX_REGS_H__ */

/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_MX6_IMX_REGS_H__
#define __ASM_ARCH_MX6_IMX_REGS_H__

struct src_regs {
	u32	scr;		/* 0x00 */
	u32	sbmr1;		/* 0x04 */
	u32	srsr;		/* 0x08 */
	u32	reserved1;	/* 0x0c */
	u32	reserved2;	/* 0x10 */
	u32	sisr;		/* 0x14 */
	u32	simr;		/* 0x18 */
	u32	sbmr2;		/* 0x1c */
	u32	gpr1;		/* 0x20 */
	u32	gpr2;		/* 0x24 */
	u32	gpr3;		/* 0x28 */
	u32	gpr4;		/* 0x2c */
	u32	gpr5;		/* 0x30 */
	u32	gpr6;		/* 0x34 */
	u32	gpr7;		/* 0x38 */
	u32	gpr8;		/* 0x3c */
	u32	gpr9;		/* 0x40 */
	u32	gpr10;		/* 0x44 */
};

#endif /* __ASM_ARCH_MX6_IMX_REGS_H__ */

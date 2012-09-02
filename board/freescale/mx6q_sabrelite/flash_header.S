/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
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

#include <config.h>
#include <asm/arch/mx6.h>

#ifdef	CONFIG_FLASH_HEADER
#ifndef CONFIG_FLASH_HEADER_OFFSET
# error "Must define the offset of flash header"
#endif

#define CPU_2_BE_32(l) \
       ((((l) << 24) & 0xff000000) | \
	(((l) <<  8) & 0x00ff0000) | \
	(((l) >>  8) & 0x0000ff00) | \
	(((l) >> 24) & 0x000000ff))

#define MAKE_TAG(tag, len, v) CPU_2_BE_32( ((tag) << 24) | (((len) & 0xffff) << 8) | ((v) & 0xff) )
#define MXC_DCD_ITEM(addr, val)	.word CPU_2_BE_32(addr), CPU_2_BE_32(val)

#ifdef CONFIG_MX6_SOLO
#define iMX6Q 0
#else
#define iMX6Q 1
#endif

#define M(a, b)	(iMX6Q * a + (iMX6Q ^ 1) * b)

#define IOM_GPR4		0x010
#define IOM_GPR6		0x018
#define IOM_GPR7		0x01c

#define IOM_DRAM_DQM0		M(0x5ac, 0x470)
#define IOM_DRAM_DQM1		M(0x5b4, 0x474)
#define IOM_DRAM_DQM2		M(0x528, 0x478)
#define IOM_DRAM_DQM3		M(0x520, 0x47c)
#define IOM_DRAM_DQM4		M(0x514, 0x480)
#define IOM_DRAM_DQM5		M(0x510, 0x484)
#define IOM_DRAM_DQM6		M(0x5bc, 0x488)
#define IOM_DRAM_DQM7		M(0x5c4, 0x48c)

#define IOM_DRAM_CAS		M(0x56c, 0x464)
#define IOM_DRAM_RAS		M(0x578, 0x490)
#define IOM_DRAM_RESET		M(0x57c, 0x494)
#define IOM_DRAM_SDCLK_0	M(0x588, 0x4ac)
#define IOM_DRAM_SDCLK_1	M(0x594, 0x4b0)
#define IOM_DRAM_SDBA2		M(0x58c, 0x4a0)
#define IOM_DRAM_SDCKE0		M(0x590, 0x4a4)
#define IOM_DRAM_SDCKE1		M(0x598, 0x4a8)
#define IOM_DRAM_SDODT0		M(0x59c, 0x4b4)
#define IOM_DRAM_SDODT1		M(0x5a0, 0x4b8)

#define IOM_DRAM_SDQS0		M(0x5a8, 0x4bc)
#define IOM_DRAM_SDQS1		M(0x5b0, 0x4c0)
#define IOM_DRAM_SDQS2		M(0x524, 0x4c4)
#define IOM_DRAM_SDQS3		M(0x51c, 0x4c8)
#define IOM_DRAM_SDQS4		M(0x518, 0x4cc)
#define IOM_DRAM_SDQS5		M(0x50c, 0x4d0)
#define IOM_DRAM_SDQS6		M(0x5b8, 0x4d4)
#define IOM_DRAM_SDQS7		M(0x5c0, 0x4d8)

#define IOM_GRP_B0DS		M(0x784, 0x764)
#define IOM_GRP_B1DS		M(0x788, 0x770)
#define IOM_GRP_B2DS		M(0x794, 0x778)
#define IOM_GRP_B3DS		M(0x79c, 0x77c)
#define IOM_GRP_B4DS		M(0x7a0, 0x780)
#define IOM_GRP_B5DS		M(0x7a4, 0x784)
#define IOM_GRP_B6DS		M(0x7a8, 0x78c)
#define IOM_GRP_B7DS		M(0x748, 0x748)
#define IOM_GRP_ADDDS		M(0x74c, 0x74c)
#define IOM_DDRMODE_CTL		M(0x750, 0x750)
#define IOM_GRP_DDRPKE		M(0x758, 0x754)
#define IOM_GRP_DDRMODE		M(0x774, 0x760)
#define IOM_GRP_CTLDS		M(0x78c, 0x76c)
#define IOM_GRP_DDR_TYPE	M(0x798, 0x774)

.section ".text.flasheader", "x"
	b	_start
	.org	CONFIG_FLASH_HEADER_OFFSET

ivt_header:	  .word MAKE_TAG(0xD1, 0x20, 0x40) /* 0x402000D1, Tag=0xD1, Len=0x0020, Ver=0x40 */
app_code_jump_v:  .word _start
reserv1:          .word 0x0
dcd_ptr:          .word dcd_hdr
boot_data_ptr:	  .word boot_data
self_ptr:         .word ivt_header
app_code_csf:     .word 0x0
reserv2:          .word 0x0

boot_data:        .word TEXT_BASE
image_len:        .word _end_of_copy  - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET
plugin:           .word 0x0

dcd_hdr:	  .word MAKE_TAG(0xD2, hdr_end - ., 0x40)		/* Tag=0xD2, Len, Ver=0x40 */
		  .word MAKE_TAG(0xCC, cmd_end1 - ., 4)		/* Tag=0xCC, Len, Param=0x04 */

#define PAD_SDQS		M(0x30, 0x38)
#define PAD_DQM			M(0x00020030, 0x00020030)

#define _IOM IOMUXC_BASE_ADDR
/* DCD */
MXC_DCD_ITEM(_IOM + IOM_GRP_DDR_TYPE, 0x000C0000)	/* ddr3 mode */
MXC_DCD_ITEM(_IOM + IOM_GRP_DDRPKE, 0x00000000)	/* disable ddr pullups */
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDCLK_0, 0x00020030)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDCLK_1, 0x00020030)

MXC_DCD_ITEM(_IOM + IOM_DRAM_CAS, 0x00020030)
MXC_DCD_ITEM(_IOM + IOM_DRAM_RAS, 0x00020030)
MXC_DCD_ITEM(_IOM + IOM_GRP_ADDDS, 0x00000030)

MXC_DCD_ITEM(_IOM + IOM_DRAM_RESET, 0x000e0030)	/* DDR3 mode */
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDCKE0, 0x00003000)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDCKE1, 0x00003000)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDBA2, 0x00000000)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDODT0, 0x00003030)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDODT1, 0x00003030)
/* 40 Ohm drive strength for cs0/1,sdba2,cke0/1,sdwe */
MXC_DCD_ITEM(_IOM + IOM_GRP_CTLDS, 0x00000030)
/* (differential input) */
MXC_DCD_ITEM(_IOM + IOM_DDRMODE_CTL, 0x00020000)

MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS0, PAD_SDQS)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS1, PAD_SDQS)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS2, PAD_SDQS)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS3, PAD_SDQS)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS4, PAD_SDQS)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS5, PAD_SDQS)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS6, PAD_SDQS)
MXC_DCD_ITEM(_IOM + IOM_DRAM_SDQS7, PAD_SDQS)

/* (differential input) */
MXC_DCD_ITEM(_IOM + IOM_GRP_DDRMODE, 0x00020000)
MXC_DCD_ITEM(_IOM + IOM_GRP_B0DS, 0x00000030)
MXC_DCD_ITEM(_IOM + IOM_GRP_B1DS, 0x00000030)
MXC_DCD_ITEM(_IOM + IOM_GRP_B2DS, 0x00000030)
MXC_DCD_ITEM(_IOM + IOM_GRP_B3DS, 0x00000030)
MXC_DCD_ITEM(_IOM + IOM_GRP_B4DS, 0x00000030)
MXC_DCD_ITEM(_IOM + IOM_GRP_B5DS, 0x00000030)
MXC_DCD_ITEM(_IOM + IOM_GRP_B6DS, 0x00000030)
MXC_DCD_ITEM(_IOM + IOM_GRP_B7DS, 0x00000030)

MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM0, PAD_DQM)
MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM1, PAD_DQM)
MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM2, PAD_DQM)
MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM3, PAD_DQM)
MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM4, PAD_DQM)
MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM5, PAD_DQM)
MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM6, PAD_DQM)
MXC_DCD_ITEM(_IOM + IOM_DRAM_DQM7, PAD_DQM)

/* MPZQHWCTRL */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x800, M(0xA1380003, 0xa1390003))
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x800, M(0xA1380003, 0xa1390003))

/* MPWLDECTRL0/1 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x80c, M(0x001F001F, 0x0040003c))
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x810, M(0x001F001F, 0x0032003e))
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x80c, M(0x00440044, 0x0040003c))
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x810, M(0x00440044, 0x0032003e))

/* MPDGCTRL0/1 DQS GATE*/
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x83c, M(0x434B0350, 0x42350231))
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x840, M(0x034C0359, 0x021a0218))
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x83c, M(0x434B0350, 0x42350231))
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x840, M(0x03650348, 0x021a0218))

/* MPRDDLCTL */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x848, M(0x4436383B, 0x4b4b4e49))
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x848, M(0x39393341, 0x4b4b4e49))
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x850, M(0x35373933, 0x3f3f3035))
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x850, M(0x48254A36, 0x3f3f3035))

/* Read data bit delay */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x81c, 0x33333333)	/* Read DQ Byte0 Delay */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x820, 0x33333333)	/* Read DQ Byte1 Delay */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x824, 0x33333333)	/* Read DQ Byte2 Delay */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x828, 0x33333333)	/* Read DQ Byte3 Delay */
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x81c, 0x33333333)	/* Read DQ Byte0 Delay */
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x820, 0x33333333)	/* Read DQ Byte1 Delay */
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x824, 0x33333333)	/* Read DQ Byte2 Delay */
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x828, 0x33333333)	/* Read DQ Byte3 Delay */

/* MPMUR0 - Complete calibration by forced measurement */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x8b8, 0x00000800)
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x8b8, 0x00000800)
/* MDPDC - CKE pulse width = 3 cycles. CKSRE = 6 cycles, CKSRX = 6 cycles */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x004, M(0x00020036, 0x0002002d))
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x008, M(0x09444040, 0x00333030))
/*
 * MDCFG0, tRFC=0x56 clocks, tXS=0x5b clocks
 * tXP=4 clocks, tXPDLL=13 clocks
 * tFAW=24 clocks, cas=8 cycles
 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x00c, M(0x555A7975, 0x696d5323))
/*
 * MDCFG1, tRDC=8, tRP=8, tRC=27,tRAS=20,tRPA=tRP+1,tWR=8
 * tMRD=4, tCWL=6
 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x010, M(0xFF538E64, 0xb66e8c63))
/*
 * MDCFG2,tDLLK=512,tRTP=4,tWTR=4,tRRD=4
 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x014, M(0x01FF00DB, 0x01ff00db))
/*
 * MDMISC, mirroring, interleaved (row/bank/col) FIXME
 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x018, M(0x00081740, 0x00001740))

/*
 * MDSCR, con_req
 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x01c, 0x00008000)
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x02c, 0x000026D2)	/* MDRWD */

MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x030, M(0x005B0E21, 0x006d0e21))	/* MDOR */

#if (iMX6Q==1)
#define CONFIG_DDR_64BIT
#else
#define CONFIG_DDR_32BIT
#endif

#ifdef CONFIG_DDR_64BIT
//last address is  (1<<28 (base) + 1<<30  - 1)  / (1<<25) = 1<<3 + 1<<5 - 1 = 8 + 0x20 -1 = 0x27
/* MDASP, CS0_END */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x040, 0x00000027)
/* MDCTL, CS0 enable, CS1 disabled, row=14, col=10, burst=8,  */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x000, 0x831A0000)	/* width=64bit */
//row+col+bank+width=14+10+3+3=30 = 1G
#endif
#ifdef CONFIG_DDR_32BIT
//last address is  (1<<28 (base) + 1<<29  - 1)  / (1<<25) = 1<<3 + 1<<4 - 1 = 8 + 0x10 -1 = 0x17
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x040, 0x00000017)
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x000, 0x83190000)	/* width=32bit */
//row+col+bank+width=14+10+3+2=29 = 512M
#endif
#ifdef CONFIG_DDR_16BIT
//last address is  (1<<28 (base) + 1<<28  - 1)  / (1<<25) = 1<<3 + 1<<3 - 1 = 8 + 8 -1 = 0x0f
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x040, 0x0000000f)
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x000, 0x83180000)	/* width=16bit */
//row+col+bank+width=14+10+3+1=28 = 256M
#endif

/* MDSCR, con_req, LOAD MR2, CS0, A3,A10 set (CAS Write=6), RZQ/2 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x01c, M(0x04088032, 0x04008032))
/* LOAD MR3, CS0 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x01c, 0x00008033)
/* LOAD MR1, CS0, A1,A6 set Rtt=RZQ/2, ODI=RZQ/7 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x01c, M(0x00428031, 0x00048031))
/* LOAD MR0, CS0, A6,A8,A11 set CAS=8, WR=8, DLL reset */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x01c, M(0x09408030, 0x07208030))
/* ZQ calibrate, CS0 */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x01c, 0x04008040)
/* MDREF,  32KHz refresh, 4 refeshes each */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x020, 0x00005800)
/* MPODTCTRL */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x818, 0x00022227)
MXC_DCD_ITEM(MMDC_P1_BASE_ADDR + 0x818, 0x00022227)

MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x004, M(0x00025576, 0x0002556d))
/* MAPSR, 1024 cycles idle before self-refresh */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x404, 0x00011006)
/* MDSCR, enable ddr */
MXC_DCD_ITEM(MMDC_P0_BASE_ADDR + 0x01c, 0x00000000)

/* enable AXI cache for VDOA/VPU/IPU */
MXC_DCD_ITEM(_IOM + IOM_GPR4, 0xf00000ff)
/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
MXC_DCD_ITEM(_IOM + IOM_GPR6, 0x007f007f)
MXC_DCD_ITEM(_IOM + IOM_GPR7, 0x007f007f)
cmd_end1:
hdr_end:
#endif

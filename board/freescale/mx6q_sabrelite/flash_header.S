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


/* mx6 duallite and solo have same offsets */
/*
 * Bits 31:30 :
 *  0 : 29:2 absolute address, 1:0 # of data values, followed by 0-3 data values in (mx6q, mx6dl, mx6_sololite), 0 means repeat last data value
 *  1 : iomuxc relative address, 9:0 mx6q offset, 19:10 mx6dl offset, 29:20 mx6_sololite offset, 0 offset means skip if this processor
 *          previous data repeated
 *  2 : same as 1, but single data value follows
 *  3 : same as 1. but 3 data values follow
 */
#define MA1 (1 << 30)
#define MA2 (2 << 30)
#define MA3 (3 << 30)
#define MA(mx6q, mx6dl, mx6solo_lite)	(((mx6q >> 2) & 0x3ff) | (((mx6dl >> 2) & 0x3ff) << 10) | (((mx6solo_lite >> 2) & 0x3ff) << 20))


#define IOM_GPR4		MA(0x010, 0x010, 0x010)
#define IOM_GPR6		MA(0x018, 0x018, 0x018)
#define IOM_GPR7		MA(0x01c, 0x01c, 0x01c)

#define IOM_DRAM_DQM0		MA(0x5ac, 0x470, 0x0)
#define IOM_DRAM_DQM1		MA(0x5b4, 0x474, 0x0)
#define IOM_DRAM_DQM2		MA(0x528, 0x478, 0x0)
#define IOM_DRAM_DQM3		MA(0x520, 0x47c, 0x0)
#define IOM_DRAM_DQM4		MA(0x514, 0x480, 0x0)
#define IOM_DRAM_DQM5		MA(0x510, 0x484, 0x0)
#define IOM_DRAM_DQM6		MA(0x5bc, 0x488, 0x0)
#define IOM_DRAM_DQM7		MA(0x5c4, 0x48c, 0x0)

#define IOM_DRAM_CAS		MA(0x56c, 0x464, 0x0)
#define IOM_DRAM_RAS		MA(0x578, 0x490, 0x0)
#define IOM_DRAM_RESET		MA(0x57c, 0x494, 0x0)
#define IOM_DRAM_SDCLK_0	MA(0x588, 0x4ac, 0x0)
#define IOM_DRAM_SDCLK_1	MA(0x594, 0x4b0, 0x0)
#define IOM_DRAM_SDBA2		MA(0x58c, 0x4a0, 0x0)
#define IOM_DRAM_SDCKE0		MA(0x590, 0x4a4, 0x0)
#define IOM_DRAM_SDCKE1		MA(0x598, 0x4a8, 0x0)
#define IOM_DRAM_SDODT0		MA(0x59c, 0x4b4, 0x0)
#define IOM_DRAM_SDODT1		MA(0x5a0, 0x4b8, 0x0)

#define IOM_DRAM_SDQS0		MA(0x5a8, 0x4bc, 0x0)
#define IOM_DRAM_SDQS1		MA(0x5b0, 0x4c0, 0x0)
#define IOM_DRAM_SDQS2		MA(0x524, 0x4c4, 0x0)
#define IOM_DRAM_SDQS3		MA(0x51c, 0x4c8, 0x0)
#define IOM_DRAM_SDQS4		MA(0x518, 0x4cc, 0x0)
#define IOM_DRAM_SDQS5		MA(0x50c, 0x4d0, 0x0)
#define IOM_DRAM_SDQS6		MA(0x5b8, 0x4d4, 0x0)
#define IOM_DRAM_SDQS7		MA(0x5c0, 0x4d8, 0x0)

#define IOM_GRP_B0DS		MA(0x784, 0x764, 0x0)
#define IOM_GRP_B1DS		MA(0x788, 0x770, 0x0)
#define IOM_GRP_B2DS		MA(0x794, 0x778, 0x0)
#define IOM_GRP_B3DS		MA(0x79c, 0x77c, 0x0)
#define IOM_GRP_B4DS		MA(0x7a0, 0x780, 0x0)
#define IOM_GRP_B5DS		MA(0x7a4, 0x784, 0x0)
#define IOM_GRP_B6DS		MA(0x7a8, 0x78c, 0x0)
#define IOM_GRP_B7DS		MA(0x748, 0x748, 0x0)
#define IOM_GRP_ADDDS		MA(0x74c, 0x74c, 0x0)
#define IOM_DDRMODE_CTL		MA(0x750, 0x750, 0x0)
#define IOM_GRP_DDRPKE		MA(0x758, 0x754, 0x0)
#define IOM_GRP_DDRMODE		MA(0x774, 0x760, 0x0)
#define IOM_GRP_CTLDS		MA(0x78c, 0x76c, 0x0)
#define IOM_GRP_DDR_TYPE	MA(0x798, 0x774, 0x0)

#define MMDC_P0		MMDC_P0_BASE_ADDR	/* 0x021b0000 */
#define MMDC_P1		MMDC_P1_BASE_ADDR	/* 0x021b4000 */
#define PAD_SDQS	0x30, 0x38
#define PAD_DQM		0x00020030

#define HAB_RVT_ENTRY		0x98
#define HAB_RVT_FAIL_SAFE_VECT	0xbc
#define HAB_RVT_LOAD_DATA	0xc8
#define IRAM_FREE_START		0x00907000

#define M(a, b)	(\mx6q * a + (\mx6q ^ 1) * b)
	.set	prev1, 0
	.set	prev2, 0
	.set	prev3, 0

	.macro	write_val	addr:req, val1:req, val2, val3
	.set	_val1, \val1
	.ifb \val2
	.set	_val2, _val1
	.else
	.set	_val2, \val2
	.endif
	.ifb \val3
	.set	_val3, _val2
	.else
	.set	_val3, \val3
	.endif
	.if	((\addr) & 3)
	.err	"address must be aligned"
	.endif

	.if	((_val1 == prev1) && (_val2 == prev2) && (_val3 == prev3))
	.word	\addr
	.else
	.if	((_val1 == _val2) && (_val2 == _val3))
	.word	\addr | 1, _val1
	.elseif	(_val2 == _val3)
	.word	\addr | 2, _val1, _val2
	.else
	.word	\addr | 3, _val1, _val2, _val3
	.endif
	.set	prev1, _val1
	.set	prev2, _val2
	.set	prev3, _val3
	.endif
	.endm

	.macro	iomuxc_val	addr:req, val1:req, val2, val3
	.set	_val1, \val1
	.ifb \val2
	.set	_val2, _val1
	.else
	.set	_val2, \val2
	.endif
	.ifb \val3
	.set	_val3, _val2
	.else
	.set	_val3, \val3
	.endif

	.if	((_val1 == prev1) && (_val2 == prev2) && (_val3 == prev3))
	.word	\addr | MA1
	.else
	.if	((_val1 == _val2) && (_val2 == _val3))
	.word	\addr | MA2, _val1
	.else
	.word	\addr | MA3, _val1, _val2, _val3
	.endif
	.set	prev1, _val1
	.set	prev2, _val2
	.set	prev3, _val3
	.endif
	.endm

.section ".text.flasheader", "x"
origin:
	b	_start
	.org	CONFIG_FLASH_HEADER_OFFSET

/* First IVT to copy the plugin that initializes the system into OCRAM */
/* 0x402000D1, Tag=0xD1, Len=0x0020, Ver=0x40 */
ivt_header:	.long MAKE_TAG(0xD1, 0x20, 0x40)
/* Plugin entry point, address after the second IVT table */
app_code_jump_v: .long IRAM_FREE_START + (plugin_start - origin)
reserv1:	.long 0x0
dcd_ptr:	.long 0x0
/*0x00907420*/
boot_data_ptr:	.long IRAM_FREE_START + (boot_data - origin)
self_ptr:	.long IRAM_FREE_START + (ivt_header - origin)
app_code_csf:	.long 0x0
reserv2:	.long 0x0

boot_data:	.long IRAM_FREE_START
image_len:	.long plugin_end - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET
plugin:		.long 0x1  /* Enable plugin flag */

	.align	6	//64 byte alignment
/* Second IVT to give entry point into the bootloader copied to DDR */
/* 0x402000D1, Tag=0xD1, Len=0x0020, Ver=0x40 */
ivt2_header:	.long MAKE_TAG(0xD1, 0x20, 0x40)
app2_code_jump_v: .long _start       /* Entry point for uboot */
reserv3:	.long 0x0
dcd2_ptr:	.long 0x0
boot_data2_ptr:	.long boot_data2
self_ptr2:	.long ivt2_header
app_code_csf2:	.long 0x0
reserv4:	.long 0x0

boot_data2:	.long TEXT_BASE
image_len2:	.long _end_of_copy - TEXT_BASE + CONFIG_FLASH_HEADER_OFFSET
plugin2:	.long 0x0

/*
 * plugin_start(void **start, size_t *bytes, UINT32 *ivt_offset)
 */
plugin_start:
/* Save the return address and the function arguments */
	push	{r0-r8, lr}

/* r0-r2 must not be 0 and must be 4 byte aligned */
	tst	r0, r0
	tstne	r1, r1
	tstne	r2, r2
	mov	r3, r0
	orreq	r3, r3, #1
	orr	r3, r3, r1
	orr	r3, r3, r2
	ldr	r0, plugin
	tst	r0, r0
	orreq	r3, r3, #1

#define rTable	r2
#define rFlag	r3
#define	rIomux	r4
#define rVal0	r5
#define rVal1	r6
#define rVal2	r7
#define rCPU	r8	/* 22 - mx6q, 12 - mx6dl, 2 sololite */
	mov	rCPU, #2

	ldr	r1, =ANATOP_BASE_ADDR
	ldr	r0, [r1, #0x280]
	mov	r0, r0, LSR #16
	cmp	r0, #0x60
	movne	rCPU, #12
	ldrne	r0, [r1, #0x260]
	movne	r0, r0, LSR #16
	cmpne	r0, #0x61
	movne	rCPU, #22

	mov	rVal0, #0
	mov	rVal1, #0
	mov	rVal2, #0
	ldr	rIomux, =IOMUXC_BASE_ADDR
	adr	rTable, mx6_table
	b	3f

1:	movs	r0, r1, LSR #30
	beq	2f
	mov	r1, r1, LSL rCPU
	movs	r1, r1, LSR #32-10
	addne	r1, rIomux, r1, LSL #2
	cmp	r0, #3
	subne	r0, r0, #1
	orr	r1, r1, r0

2:	ands	r0, r1, #3
	bic	r1, r1, #3
	ldrne	rVal0, [rTable], #4
	movne	rVal1, rVal0
	movne	rVal2, rVal0
	subnes	r0, r0, #1
	ldrne	rVal1, [rTable], #4
	movne	rVal2, rVal1
	subnes	r0, r0, #1
	ldrne	rVal2, [rTable], #4

	mov	r0, rVal0
	cmp	rCPU, #12
	moveq	r0, rVal1
	cmp	rCPU, #2
	moveq	r0, rVal2
	cmp	r1, #0
	strne	r0, [r1]
3:	ldr	r1, [rTable], #4
	cmp	r1, #0
	bne	1b

	ands	rFlag, rFlag, #3
	bne	4f		/* Branch if not called as plugin */

	ldr	r0, boot_data2
	ldr	r1, image_len2
	mov	r2, r0
	mov	r3, r1
	mov	r4, #0
	push	{r0-r4}
	mov	r0, #HAB_RVT_LOAD_DATA
	ldr	r4, [r0]
	mov	r0, sp
	add	r1, sp, #4
	add	r2, sp, #8
	blx	r4

	pop	{r4, lr}
	add	sp, sp, #12
	pop	{r0-r3}
/*
 * Before returning to ROM, we need to fill the return values arguments
 * to our function.
 * plugin_start(void **start, size_t *bytes, UINT32 *ivt_offset)
 */

	str	r4, [r0]
	str	lr, [r1]
	mov	r4, #ivt2_header - origin
	str	r4, [r2]
	mov	r0, #1
	pop	{r4-r8, pc}

/* Not called as plugin */
4:	popne	{r0-r8, lr}
	mov	r0, #HAB_RVT_ENTRY
	ldr	lr, [r0]
	blx	lr
	mov	r0, #HAB_RVT_FAIL_SAFE_VECT
	ldr	lr, [r0]
	blx	lr

	.ltorg
mx6_table:
/* ddr3 mode */
	iomuxc_val	IOM_GRP_DDR_TYPE, 0x000C0000
/* disable ddr pullups */
	iomuxc_val	IOM_GRP_DDRPKE, 0x00000000
	iomuxc_val	IOM_DRAM_SDCLK_0, 0x00020030
	iomuxc_val	IOM_DRAM_SDCLK_1, 0x00020030

	iomuxc_val	IOM_DRAM_CAS, 0x00020030
	iomuxc_val	IOM_DRAM_RAS, 0x00020030
	iomuxc_val	IOM_GRP_ADDDS, 0x00000030

	iomuxc_val	IOM_DRAM_RESET, 0x000e0030
	iomuxc_val	IOM_DRAM_SDCKE0, 0x00003000
	iomuxc_val	IOM_DRAM_SDCKE1, 0x00003000
	iomuxc_val	IOM_DRAM_SDBA2, 0x00000000
	iomuxc_val	IOM_DRAM_SDODT0, 0x00003030
	iomuxc_val	IOM_DRAM_SDODT1, 0x00003030
/* 40 Ohm drive strength for cs0/1,sdba2,cke0/1,sdwe */
	iomuxc_val	IOM_GRP_CTLDS, 0x00000030
/* (differential input) */
	iomuxc_val	IOM_DDRMODE_CTL, 0x00020000

	iomuxc_val	IOM_DRAM_SDQS0, PAD_SDQS
	iomuxc_val	IOM_DRAM_SDQS1, PAD_SDQS
	iomuxc_val	IOM_DRAM_SDQS2, PAD_SDQS
	iomuxc_val	IOM_DRAM_SDQS3, PAD_SDQS
	iomuxc_val	IOM_DRAM_SDQS4, PAD_SDQS
	iomuxc_val	IOM_DRAM_SDQS5, PAD_SDQS
	iomuxc_val	IOM_DRAM_SDQS6, PAD_SDQS
	iomuxc_val	IOM_DRAM_SDQS7, PAD_SDQS

/* (differential input) */
	iomuxc_val	IOM_GRP_DDRMODE, 0x00020000
	iomuxc_val	IOM_GRP_B0DS, 0x00000030
	iomuxc_val	IOM_GRP_B1DS, 0x00000030
	iomuxc_val	IOM_GRP_B2DS, 0x00000030
	iomuxc_val	IOM_GRP_B3DS, 0x00000030
	iomuxc_val	IOM_GRP_B4DS, 0x00000030
	iomuxc_val	IOM_GRP_B5DS, 0x00000030
	iomuxc_val	IOM_GRP_B6DS, 0x00000030
	iomuxc_val	IOM_GRP_B7DS, 0x00000030

	iomuxc_val	IOM_DRAM_DQM0, PAD_DQM
	iomuxc_val	IOM_DRAM_DQM1, PAD_DQM
	iomuxc_val	IOM_DRAM_DQM2, PAD_DQM
	iomuxc_val	IOM_DRAM_DQM3, PAD_DQM
	iomuxc_val	IOM_DRAM_DQM4, PAD_DQM
	iomuxc_val	IOM_DRAM_DQM5, PAD_DQM
	iomuxc_val	IOM_DRAM_DQM6, PAD_DQM
	iomuxc_val	IOM_DRAM_DQM7, PAD_DQM

/* MPZQHWCTRL */
	write_val	MMDC_P0 + 0x800, 0xA1380003, 0xa1390003
	write_val	MMDC_P1 + 0x800, 0xA1380003, 0xa1390003

/* MPWLDECTRL0/1 */
	write_val	MMDC_P0 + 0x80c, 0x001F001F, 0x0040003c
	write_val	MMDC_P0 + 0x810, 0x001F001F, 0x0032003e
	write_val	MMDC_P1 + 0x80c, 0x00440044, 0x0040003c
	write_val	MMDC_P1 + 0x810, 0x00440044, 0x0032003e

/* MPDGCTRL0/1 DQS GATE*/
	write_val	MMDC_P0 + 0x83c, 0x434B0350, 0x42350231
	write_val	MMDC_P1 + 0x83c, 0x434B0350, 0x42350231
	write_val	MMDC_P0 + 0x840, 0x034C0359, 0x021a0218
	write_val	MMDC_P1 + 0x840, 0x03650348, 0x021a0218

/* MPRDDLCTL */
	write_val	MMDC_P0 + 0x848, 0x4436383B, 0x4b4b4e49
	write_val	MMDC_P1 + 0x848, 0x39393341, 0x4b4b4e49
	write_val	MMDC_P0 + 0x850, 0x35373933, 0x3f3f3035
	write_val	MMDC_P1 + 0x850, 0x48254A36, 0x3f3f3035

/* Read data DQ Byte0-3 delay */
	write_val	MMDC_P0 + 0x81c, 0x33333333
	write_val	MMDC_P0 + 0x820, 0x33333333
	write_val	MMDC_P0 + 0x824, 0x33333333
	write_val	MMDC_P0 + 0x828, 0x33333333
	write_val	MMDC_P1 + 0x81c, 0x33333333
	write_val	MMDC_P1 + 0x820, 0x33333333
	write_val	MMDC_P1 + 0x824, 0x33333333
	write_val	MMDC_P1 + 0x828, 0x33333333

/* MPMUR0 - Complete calibration by forced measurement */
	write_val	MMDC_P0 + 0x8b8, 0x00000800
	write_val	MMDC_P1 + 0x8b8, 0x00000800
/* MDPDC - CKE pulse width = 3 cycles. CKSRE = 6 cycles, CKSRX = 6 cycles */
	write_val	MMDC_P0 + 0x004, 0x00020036, 0x0002002d
	write_val	MMDC_P0 + 0x008, 0x09444040, 0x00333030
/*
 * MDCFG0, tRFC=0x56 clocks, tXS=0x5b clocks
 * tXP=4 clocks, tXPDLL=13 clocks
 * tFAW=24 clocks, cas=8 cycles
 */
	write_val	MMDC_P0 + 0x00c, 0x555A7975, 0x696d5323
/*
 * MDCFG1, tRDC=8, tRP=8, tRC=27,tRAS=20,tRPA=tRP+1,tWR=8
 * tMRD=4, tCWL=6
 */
	write_val	MMDC_P0 + 0x010, 0xFF538E64, 0xb66e8c63
/*
 * MDCFG2,tDLLK=512,tRTP=4,tWTR=4,tRRD=4
 */
	write_val	MMDC_P0 + 0x014, 0x01FF00DB, 0x01ff00db
/*
 * MDMISC, mirroring, interleaved (row/bank/col) FIXME
 */
	write_val	MMDC_P0 + 0x018, 0x00081740, 0x00001740

/*
 * MDSCR, con_req
 */
	write_val	MMDC_P0 + 0x01c, 0x00008000
/* MDRWD */
	write_val	MMDC_P0 + 0x02c, 0x000026D2
/* MDOR */
	write_val	MMDC_P0 + 0x030, 0x005B0E21, 0x006d0e21


/*
 * Mx6Q - 64 bit wide ddr
 * last address is  (1<<28 (base) + 1<<30  - 1)  / (1<<25) =
 * 	1<<3 + 1<<5 - 1 = 8 + 0x20 -1 = 0x27
 */
/*
 * MX6SOLO - 32 bit wide ddr
 * last address is  (1<<28 (base) + 1<<29  - 1)  / (1<<25) =
 *	1<<3 + 1<<4 - 1 = 8 + 0x10 -1 = 0x17
 */
/*
 * 16 bit wide ddr
 * last address is  (1<<28 (base) + 1<<28  - 1)  / (1<<25) =
 *	1<<3 + 1<<3 - 1 = 8 + 8 -1 = 0x0f
 */
#ifdef CONFIG_DDR_16BIT
	write_val	MMDC_P0 + 0x040, 0x0000000f
/* width=16bit */
	write_val	MMDC_P0 + 0x000, 0x83180000
//16 bit wide ddr : row+col+bank+width=14+10+3+1=28 = 256M
#else
/* MDASP, CS0_END */
	write_val	MMDC_P0 + 0x040, 0x00000027, 0x00000017
/* MDCTL, CS0 enable, CS1 disabled, row=14, col=10, burst=8, width=64/32bit */
	write_val	MMDC_P0 + 0x000, 0x831A0000, 0x83190000
//mx6q    : row+col+bank+width=14+10+3+3=30 = 1G
//mx6solo : row+col+bank+width=14+10+3+2=29 = 512M
#endif

/* MDSCR, con_req, LOAD MR2, CS0, A3,A10 set (CAS Write=6), RZQ/2 */
	write_val	MMDC_P0 + 0x01c, 0x04088032, 0x04008032
/* LOAD MR3, CS0 */
	write_val	MMDC_P0 + 0x01c, 0x00008033
/* LOAD MR1, CS0, A1,A6 set Rtt=RZQ/2, ODI=RZQ/7 */
	write_val	MMDC_P0 + 0x01c, 0x00428031, 0x00048031
/* LOAD MR0, CS0, A6,A8,A11 set CAS=8, WR=8, DLL reset */
	write_val	MMDC_P0 + 0x01c, 0x09408030, 0x07208030
/* ZQ calibrate, CS0 */
	write_val	MMDC_P0 + 0x01c, 0x04008040
/* MDREF,  32KHz refresh, 4 refeshes each */
	write_val	MMDC_P0 + 0x020, 0x00005800
/* MPODTCTRL */
	write_val	MMDC_P0 + 0x818, 0x00022227
	write_val	MMDC_P1 + 0x818, 0x00022227

	write_val	MMDC_P0 + 0x004, 0x00025576, 0x0002556d
/* MAPSR, 1024 cycles idle before self-refresh */
	write_val	MMDC_P0 + 0x404, 0x00011006
/* MDSCR, enable ddr */
	write_val	MMDC_P0 + 0x01c, 0x00000000

/* enable AXI cache for VDOA/VPU/IPU */
	iomuxc_val	IOM_GPR4, 0xf00000ff
/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
	iomuxc_val	IOM_GPR6, 0x007f007f
	iomuxc_val	IOM_GPR7, 0x007f007f
	.word	0	/* end marker */
plugin_end:
#endif

/*
 * Copyright 2008 - 2009 (C) Wind River Systems, Inc.
 * Tom Rix <Tom.Rix@windriver.com>
 *
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc.
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
 *
 * Part of the rx_handler were copied from the Android project.
 * Specifically rx command parsing in the  usb_rx_data_complete
 * function of the file bootable/bootloader/legacy/usbloader/usbloader.c
 *
 * The logical naming of flash comes from the Android project
 * Thse structures and functions that look like fastboot_flash_*
 * They come from bootable/bootloader/legacy/libboot/flash.c
 *
 * This is their Copyright:
 *
 * Copyright (C) 2008 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <common.h>
#include <asm/byteorder.h>
#include <command.h>
#include <fastboot.h>


/* Use do_reset for fastboot's 'reboot' command */
extern int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

/* Forward decl */
static int tx_handler(struct cmd_fastboot_interface *pi);
static int rx_handler(struct cmd_fastboot_interface *pi);
static void reset_handler(struct cmd_fastboot_interface *pi);

static struct cmd_fastboot_interface interface = {
	.rx_handler            = rx_handler,
	.reset_handler         = reset_handler,
	.product_name          = NULL,
	.serial_no             = NULL,
	.nand_block_size       = 0,
	.transfer_buffer       = (unsigned char *)0xffffffff,
	.transfer_buffer_size  = 0,
};

/* To support the Android-style naming of flash */
#define MAX_PTN		    48

static fastboot_ptentry ptable[MAX_PTN];
static unsigned int pcount;
static int static_pcount = -1;

static void reset_handler(struct cmd_fastboot_interface *pi)
{
	/* If there was a download going on, bail */
	pi->download_size = 0;
	pi->download_bytes = 0;
	pi->download_bytes_unpadded = 0;
	pi->download_error = 0;
	pi->continue_booting = 0;
	pi->upload_size = 0;
	pi->upload_bytes = 0;
	pi->upload_error = 0;
}

static int tx_handler(struct cmd_fastboot_interface *pi)
{
	if (pi->upload_size) {

		int bytes_written;
		bytes_written = fastboot_tx(pi->transfer_buffer +
					    pi->upload_bytes, pi->upload_size -
					    pi->upload_bytes);
		if (bytes_written > 0) {

			pi->upload_bytes += bytes_written;
			/* Check if this is the last */
			if (pi->upload_bytes == pi->upload_size) {

				/* Reset upload */
				pi->upload_size = 0;
				pi->upload_bytes = 0;
				pi->upload_error = 0;
			}
		}
	}
	return pi->upload_error;
}

struct cmd_entry {
	char *cmd_name;
	int (*fn)(struct cmd_fastboot_interface *pi,
			char *response, const char *cmdbuf);
};

/* reboot
   Reboot the board. */
int cmd_reboot(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	sprintf(response, "OKAY");
	fastboot_tx_status(response, strlen(response));
	udelay(1000000); /* 1 sec */

	do_reset(NULL, 0, 0, NULL);

	/* This code is unreachable,
	   leave it to make the compiler happy */
	return 0;
}

extern const char version_string[];

/* getvar
   Get common fastboot variables
   Board has a chance to handle other variables */
int cmd_getvar(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	strcpy(response, "OKAY");

	if (!strcmp(cmdbuf, "version")) {
		strncpy(response + 4, FASTBOOT_VERSION, 60);
	} else if (!strcmp(cmdbuf, "version-bootloader")) {
		strncpy(response + 4, version_string, 60);
	} else if (!strcmp(cmdbuf, "product")) {
		if (pi->product_name)
			strncpy(response + 4, pi->product_name, 60);
	} else if (!strcmp(cmdbuf, "serialno")) {
		if (pi->serial_no)
			strncpy(response + 4, pi->serial_no, 60);
	} else if (!strcmp(cmdbuf, "downloadsize")) {
		if (pi->transfer_buffer_size)
			sprintf(response + 4, "0x%x",
					pi->transfer_buffer_size);
	} else {
		fastboot_getvar(cmdbuf, response + 4);
	}
	return 0;
}

/*
 * Erase a register flash partition
 * Board has to set up flash partitions
 */
int cmd_erase(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	struct fastboot_ptentry *ptn;
	int ret = -1;

	ptn = fastboot_flash_find_ptn(cmdbuf);
	if (ptn == 0)
		sprintf(response, "FAILpartition does not exist");
	else if (ptn->dev_ops->erase)
		ret = ptn->dev_ops->erase(ptn, pi, response);
	else
		printf("erase command not supported for partition\n");
	return ret;
}

/*
 * download something ..
 * What happens to it depends on the next command after data
 */
int cmd_download(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	/* save the size */
	pi->download_size = simple_strtoul(cmdbuf, NULL, 16);
	/* Reset the bytes count, now it is safe */
	pi->download_bytes = 0;
	/* Reset error */
	pi->download_error = 0;

	if (0 == pi->download_size) {
		/* bad user input */
		sprintf(response, "FAILdata invalid size");
	} else if (pi->download_size > pi->transfer_buffer_size) {
		/* set to 0 because this is an error */
		pi->download_size = 0;
		sprintf(response, "FAILdata too large");
	} else {
		/* The default case, the transfer fits
		   completely in the interface buffer */
		printf("Starting download of %d bytes\n", pi->download_size);
		sprintf(response, "DATA%08x", pi->download_size);
	}
	return 0;
}

int do_booti(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

/*
 * boot what was downloaded
 *
 * WARNING WARNING WARNING
 *
 * This is not what you expect.
 * The fastboot client does its own packaging of the
 * kernel.  The layout is defined in the android header
 * file bootimage.h.  This layeout is copied, looks like this,
 *
 ** +-----------------+
 ** | boot header     | 1 page
 ** +-----------------+
 ** | kernel          | n pages
 ** +-----------------+
 ** | ramdisk         | m pages
 ** +-----------------+
 ** | second stage    | o pages
 ** +-----------------+
 **
 *
 * We only care about the kernel.
 * So we have to jump past a page.
 *
 * What is a page size ?
 * The fastboot client uses 2048
 *
 * The is the default value of
 *
 * CFG_FASTBOOT_MKBOOTIMAGE_PAGE_SIZE
 *
 */
int cmd_boot(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	if ((pi->download_bytes) && (CFG_FASTBOOT_MKBOOTIMAGE_PAGE_SIZE <
			pi->download_bytes)) {
		char start[32];
		char *booti_args[4] = {"booti",  NULL, "boot", NULL};

		/*
		 * Use this later to determine if a command line was passed
		 * for the kernel.
		 */
//		struct fastboot_boot_img_hdr *fb_hdr =
//				(struct fastboot_boot_img_hdr *) pi->transfer_buffer;

		/* Skip the mkbootimage header */
		/* image_header_t *hdr = */
		/* 	(image_header_t *) */
		/* 	pi.transfer_buffer[CFG_FASTBOOT_MKBOOTIMAGE_PAGE_SIZE]; */

		booti_args[1] = start;
		sprintf(start, "0x%x", (unsigned int)pi->transfer_buffer);

		/* Execution should jump to kernel so send the response
		   now and wait a bit.  */
		sprintf(response, "OKAY");
		fastboot_tx_status(response, strlen(response));

		printf("Booting kernel...\n");


		/* Reserve for further use, this can
		 * be more convient for developer. */
		/* if (strlen ((char *) &fb_hdr->cmdline[0])) */
		/* 	set_env("bootargs", (char *) &fb_hdr->cmdline[0]); */

		/* boot the boot.img */
		do_booti(NULL, 0, 3, booti_args);
	}
	sprintf(response, "FAILinvalid boot image");
	return 0;
}

/*
 * Flash what was downloaded
 */
int cmd_flash(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	if (pi->download_bytes) {
		struct fastboot_ptentry *ptn;
		struct fastboot_ptentry ptn_copy;
		int offset = 0;
		int partition = 0;
		int seg = 0;
		const char *buf = cmdbuf;
		const char *cmdend = buf;
		const char *p;
		char *endp;

		while (*buf && (*buf != ':')) {
			if (*buf == 'p') {
				buf++;
				p = buf;
				if ((*p >= '0') && (*p <= '9')) {
					while (*buf && (*buf != ':'))
						buf++;
					partition = simple_strtoul(p, &endp, 10);
					if (endp == buf)
						break;
					partition = 0;
				}
			 } else {
				 buf++;
			 }
			cmdend = buf;
		}
		if (*buf == ':') {
			p = buf + 1;
			if (!strncmp(p, "mbr", 3)) {
				seg = FB_SEG_MBR;
				buf += 4;
			} else if (!strncmp(p, "bootloader", 10)) {
				seg = FB_SEG_BOOTLOADER;
				buf += 11;
			} else if (!strncmp(p, "env", 3)) {
				seg = FB_SEG_ENV;
				buf += 4;
			}
		}
		if (*buf == ':') {
			buf++;
			p = buf;
			while (*buf)
				buf++;
			offset = simple_strtoul(p, &endp, 10);
		}
		if (*buf) {
			printf("unexpected '%s'\n", buf);
			sprintf(response, "FAILsyntax");
			return 0;
		}
		ptn = fastboot_flash_find_ptn_len(cmdbuf, cmdend - cmdbuf);
		if (!ptn) {
			printf("device:'%s' does not exist\n", cmdbuf);
			sprintf(response, "FAILdevice does not exist");
			return 0;
		}
		ptn_copy = *ptn;
		if (partition || seg) {
			if (!ptn_copy.dev_ops->partion_info) {
				printf("partion_info:'%s' not supported\n",
					cmdbuf);
				sprintf(response, "FAILpartition_info");
				return 0;
			}
			if (ptn_copy.dev_ops->partion_info(&ptn_copy,
					seg, partition)) {
				printf("Partition:'%s' does not exist\n",
					cmdbuf);
				sprintf(response,
					"FAILpartition does not exist");
				return 0;
			}
		}
		if (ptn_copy.length <= offset) {
			printf("Partition:'%s' too small\n", cmdbuf);
			sprintf(response, "FAILpartition too small");
			return 0;
		}
		ptn_copy.length -= offset;
		ptn_copy.start += offset;
		printf("dev=%s name='%s' start=%u(0x%x) len=%u(0x%x)\n",
				ptn_copy.dev_ops->name, ptn_copy.name,
				ptn_copy.start, ptn_copy.start,
				ptn_copy.length, ptn_copy.length);
		if (ptn_copy.dev_ops->flash)
			return ptn_copy.dev_ops->flash(&ptn_copy, pi, response);
		printf("Partition:'%s' does not support flash\n", ptn->name);
		sprintf(response, "FAILflash not supported");
	} else {
		sprintf(response, "FAILno image downloaded");
	}
	return 0;
}

/*
 * continue, Stop doing fastboot
 */
int cmd_continue(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	sprintf(response, "OKAY");
	pi->continue_booting = 1;
	return 0;
}

/*
 * Upload just the data in a partition
 */
int upload(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf, unsigned is_raw)
{
	/* This is how much the user is expecting */
	unsigned int user_size;
	/*
	 * This is the maximum size needed for
	 * this partition
	 */
	unsigned int size;
	/* This is the length of the data */
	unsigned int length;
	/*
	 * Used to check previous write of
	 * the parition
	 */
	char env_ptn_length_var[128];
	char *env_ptn_length_val;
	unsigned int delim_index, len;
	struct fastboot_ptentry *ptn;

	/* Scan to the next ':' to find when the size starts */
	len = strlen(cmdbuf);
	for (delim_index = 0; delim_index < len; delim_index++) {
		if (cmdbuf[delim_index] == ':')
			break;
	}

	ptn = fastboot_flash_find_ptn_len(cmdbuf, delim_index);
	if (ptn == 0) {
		sprintf(response, "FAILpartition does not exist");
		return 0;
	}
	 if (!ptn->dev_ops->upload) {
		 printf("Partition:'%s' does not support upload\n", ptn->name);
		 sprintf(response, "FAILupload not supported");
		 return 0;
	 }
	user_size = 0;
	if (delim_index < len)
		user_size = simple_strtoul(cmdbuf + delim_index +
				 1, NULL, 16);

	/* Make sure output is padded to block size */
	length = ptn->length;
	sprintf(env_ptn_length_var, "%s_nand_size", ptn->name);
	env_ptn_length_val = getenv(env_ptn_length_var);
	if (env_ptn_length_val) {
		length = simple_strtoul(env_ptn_length_val, NULL, 16);
		/* Catch possible problems */
		if (!length)
			length = ptn->length;
	}

	size = length / pi->nand_block_size;
	size *= pi->nand_block_size;
	if (length % pi->nand_block_size)
		size += pi->nand_block_size;

	if (is_raw)
		size += (size /
			 pi->nand_block_size) *
			pi->nand_oob_size;

	if (size > pi->transfer_buffer_size) {

		sprintf(response, "FAILdata too large");

	} else if (user_size == 0) {

		/* Send the data response */
		sprintf(response, "DATA%08x", size);

	} else if (user_size != size) {
		/* This is the wrong size */
		sprintf(response, "FAIL");
	} else {
		return ptn->dev_ops->upload(ptn, pi, response, size, is_raw);
	}
	return 0;
}

int cmd_upload(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	return upload(pi, response, cmdbuf, 0);
}

int cmd_uploadraw(struct cmd_fastboot_interface *pi,
		char *response, const char *cmdbuf)
{
	return upload(pi, response, cmdbuf, 1);
}

struct cmd_entry cmd_list[] = {
	{"reboot", cmd_reboot},
	{"getvar:", cmd_getvar},
	{"erase:", cmd_erase},
	{"download:", cmd_download},
	{"boot", cmd_boot},
	{"flash:", cmd_flash},
	{"continue", cmd_continue},
	{"upload:", cmd_upload},
	{"uploadraw:", cmd_uploadraw},
	{NULL, NULL}
};

static int rx_handler(struct cmd_fastboot_interface *pi)
{
	int ret = 1;

	/* Use 65 instead of 64
	   null gets dropped
	   strcpy's need the extra byte */
	char response[65];

	if (pi->download_size) {
		/* Something to download */
		/* Handle possible overflow */
		int db = pi->download_bytes;
		int dot_freq = pi->nand_block_size << 5;
		unsigned int transfer_size = pi->download_size - pi->download_bytes;
		int buffer_size = fastboot_get_rx_buffer(pi->transfer_buffer +
				pi->download_bytes, transfer_size);

		if (buffer_size > transfer_size) {
			/* drop remaining bytes */
			fastboot_get_rx_buffer(NULL, buffer_size);
			printf("Error, more data received than expected 0x%x\n", buffer_size);
		} else {
			transfer_size = buffer_size;
		}
		pi->download_bytes += transfer_size;

		/* Check if transfer is done */
		if (pi->download_bytes >= pi->download_size) {
			/* Reset global transfer variable,
				   Keep pi->download_bytes because it will be
				   used in the next possible flashing command */
			pi->download_size = 0;

			if (pi->download_error) {
				/* There was an earlier error */
				sprintf(response, "ERROR");
			} else {
				/* Everything has transferred,
					   send the OK response */
				sprintf(response, "OKAY");
			}
			fastboot_tx_status(response, strlen(response));

			printf("\ndownloading of %d bytes finished\n",
					pi->download_bytes);
			return 0;
		}

		/* Provide some feedback */
		db -= db % dot_freq;
		for (;;)  {
			db += dot_freq;
			if (db > pi->download_bytes)
				break;
			if (pi->download_error)
				printf("X");
			else
				printf(".");
			if (0 == (db % (80 * dot_freq)))
				printf("\n");
		}
		ret = 0;
	} else {
		/* A command */

		/* Cast to make compiler happy with string functions */
		struct cmd_entry *p = cmd_list;
		char buffer[0x201];
		const char *cmdbuf = buffer;
		int buffer_size = fastboot_get_rx_buffer(buffer, sizeof(buffer) - 1);

		if (buffer_size > sizeof(buffer) - 1) {
			/* drop remaining bytes */
			fastboot_get_rx_buffer(NULL, buffer_size);
			printf("Error, more cmd received than expected 0x%x\n", buffer_size);
			buffer_size = sizeof(buffer) - 1;
		}
		buffer[buffer_size] = 0;
		printf("cmdbuf: %s\n", cmdbuf);

		/* Generic failed response */
		sprintf(response, "FAIL");
		for (;;) {
			int len = strlen(p->cmd_name);
			if (memcmp(cmdbuf, p->cmd_name, len) == 0) {
				response[ARRAY_SIZE(response) - 1] = 0;
				ret = p->fn(pi, response, cmdbuf + len);
				break;
			}
			p++;
			if (!p->cmd_name) {
				printf("unknown command: %s\n", cmdbuf);
				break;
			}
		}
		fastboot_tx_status(response, strlen(response));
	} /* End of command */

	return ret;
}

static int check_against_static_partition(struct fastboot_ptentry *ptn)
{
	struct fastboot_ptentry *c;
	int i;

	if (!ptn->length)
		return 0;

	for (i = 0; i < static_pcount; i++) {
		c = fastboot_flash_get_ptn((unsigned int) i);
		if (c->dev_ops != ptn->dev_ops)
			continue;
		if (c->dev_index != ptn->dev_index)
			continue;
		if ((ptn->start >= c->start) &&
		    (ptn->start < c->start + c->length))
			return 0;

		if ((ptn->start + ptn->length > c->start) &&
		    (ptn->start + ptn->length <= c->start + c->length))
			return 0;

		if (!strcmp(ptn->name, c->name))
			return 0;
	}
	return 1;
}

static unsigned long long memparse(const char *ptr, const char **retptr)
{
	char *endptr;	/* local pointer to end of parsed string */

	unsigned long ret = simple_strtoul(ptr, &endptr, 0);

	switch (*endptr) {
	case 'M':
	case 'm':
		ret <<= 10;
	case 'K':
	case 'k':
		ret <<= 10;
		endptr++;
	default:
		break;
	}

	if (retptr)
		*retptr = endptr;

	return ret;
}

static int add_partition(const char *s, const char **retptr,
		const struct dev_partition_ops *dev_ops, unsigned dev_index)
{
	struct fastboot_ptentry part;
	int delim = 0;
	int name_len;
	const char *name;
	unsigned long offset = 0;
	unsigned int flags = 0;
	unsigned long size = memparse(s, &s);

	if (0 == size) {
		printf("Error:FASTBOOT size of parition is 0\n");
		return 1;
	}

	/* check for offset */
	if (*s == '@') {
		s++;
		offset = memparse(s, &s);
	} else {
		printf("Error:FASTBOOT offset of parition is not given\n");
		return 1;
	}

	/* now look for name */
	if (*s == '(')
		delim = ')';

	if (delim) {
		char *p;

		name = ++s;
		p = strchr((const char *)name, delim);
		if (!p) {
			printf("Error:FASTBOOT no closing %c found in partition name\n", delim);
			return 1;
		}
		name_len = p - name;
		s = p + 1;
	} else {
		printf("Error:FASTBOOT no partition name for \'%s\'\n", s);
		return 1;
	}

	/* test for options */
	while (1) {
		if (strncmp(s, "i", 1) == 0) {
			flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_I;
			s += 1;
		} else if (strncmp(s, "yaffs", 5) == 0) {
			/* yaffs */
			flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_YAFFS;
			s += 5;
		} else if (strncmp(s, "swecc", 5) == 0) {
			/* swecc */
			flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_SW_ECC;
			s += 5;
		} else if (strncmp(s, "hwecc", 5) == 0) {
			/* hwecc */
			flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_HW_ECC;
			s += 5;
		} else {
			break;
		}
		if (strncmp(s, "|", 1) == 0)
			s += 1;
	}

	/* enter this partition (offset will be calculated later if it is zero at this point) */
	part.length = size;
	part.start = offset;
	part.flags = flags;
	part.dev_ops = dev_ops;
	part.dev_index = dev_index;

	if (name_len >= sizeof(part.name)) {
		printf("Error:FASTBOOT partition name is too long\n");
		return 1;
	}
	strncpy(&part.name[0], name, name_len);
	/* name is not null terminated */
	part.name[name_len] = '\0';


	/* Check if this overlaps a static partition */
	if (check_against_static_partition(&part)) {
		printf("Adding: %s, offset 0x%8.8x, size 0x%8.8x, flags 0x%8.8x\n",
		       part.name, part.start, part.length, part.flags);
		fastboot_flash_add_ptn(&part);
	}

	/* return (updated) pointer command line string */
	*retptr = s;

	/* return partition table */
	return 0;
}

void parse_partitions(const char *s, const struct dev_partition_ops *dev_ops,
		unsigned dev_index)
{
	unsigned len = strlen(s);
	const char *e = s + len;

	printf("Fastboot: Adding partitions from environment\n");
	while (s < e) {
		if (add_partition(s, &s, dev_ops, dev_index)) {
			printf("Error:Fastboot: Abort adding partitions\n");
			/* reset back to static */
			pcount = static_pcount;
			break;
		}
		/* Skip a bunch of delimiters */
		while (s < e) {
			if ((' ' == *s) ||
					('\t' == *s) ||
					('\n' == *s) ||
					('\r' == *s) ||
					(',' == *s)) {
				s++;
			} else {
				break;
			}
		}
	}
}

struct fastboot_partition_vars
{
	char *name;
	const struct dev_partition_ops *dev_ops;
	unsigned dev_index;
};

#ifdef CONFIG_FASTBOOT_STORAGE_NAND
#define DEFAULT_OPS &part_nand_ops
#else
#define DEFAULT_OPS &part_mmc_ops
#endif

const struct fastboot_partition_vars vars[] = {
	{"fbparts", DEFAULT_OPS},
#ifdef CONFIG_FASTBOOT_STORAGE_STORAGE_EMMC_SATA
	{"fbparts_mmc0", &part_mmc_ops, 0},
	{"fbparts_mmc1", &part_mmc_ops, 1},
	{"fbparts_sata", &part_sata_ops, 0},
#endif
#ifdef CONFIG_FASTBOOT_STORAGE_STORAGE_SF
	{"fbparts_sf", &part_sf_ops, 0},
#endif
	{NULL, NULL}
};

void add_environment_partitions(void)
{
	const struct fastboot_partition_vars *fbvars = vars;

	while (fbvars->name) {
		char *env = getenv(fbvars->name);
		if (env)
			parse_partitions(env, fbvars->dev_ops, fbvars->dev_index);
		fbvars++;
	}
}

int do_fastboot (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int ret = 1;
	int check_timeout = 0;
	uint64_t timeout_endtime = 0;
	uint64_t timeout_ticks = 1000;
	long timeout_seconds = -1;
	int continue_from_disconnect = 0;

	/*
	 * Place the runtime partitions at the end of the
	 * static paritions.  First save the start off so
	 * it can be saved from run to run.
	 */
	if (static_pcount >= 0) {
		/* Reset */
		pcount = static_pcount;
	} else {
		/* Save */
		static_pcount = pcount;
	}
	add_environment_partitions();

	/* Time out */
	if (2 == argc) {
		long try_seconds;
		char *try_seconds_end;
		/* Check for timeout */
		try_seconds = simple_strtol(argv[1],
					    &try_seconds_end, 10);
		if ((try_seconds_end != argv[1]) &&
		    (try_seconds >= 0)) {
			check_timeout = 1;
			timeout_seconds = try_seconds;
			printf("Fastboot inactivity timeout %ld seconds\n", timeout_seconds);
		}
	}

	do {
		continue_from_disconnect = 0;
		reset_handler(&interface);
		/* Initialize the board specific support */
		if (0 == fastboot_init(&interface)) {

			int poll_status;

			/* If we got this far, we are a success */
			ret = 0;
			printf("fastboot initialized\n");

			timeout_endtime = get_timer(0);
			timeout_endtime += timeout_ticks;

			while (1) {
				uint64_t current_time = 0;

				poll_status = fastboot_poll();
				if (1 == check_timeout)
					current_time = get_timer(0);

				if (FASTBOOT_ERROR == poll_status) {
					/* Error */
					break;
				} else if (FASTBOOT_DISCONNECT == poll_status) {
					/* beak, cleanup and re-init */
					printf("Fastboot disconnect detected\n");
					continue_from_disconnect = 1;
					break;
				} else if ((1 == check_timeout) &&
					   (FASTBOOT_INACTIVE == poll_status)) {

					/* No activity */
					if (current_time >= timeout_endtime) {
						printf("Fastboot inactivity detected\n");
						break;
					}
				} else {
					/* Something happened */
					if (1 == check_timeout) {
						/* Update the timeout endtime */
						timeout_endtime = current_time;
						timeout_endtime += timeout_ticks;
					}
				}

				/* Check if the user wanted to terminate with ^C */
				if ((FASTBOOT_OK != poll_status) &&
				    (ctrlc())) {
					printf("Fastboot ended by user %x %x\n", interface.download_bytes, interface.download_size);
					continue_from_disconnect = 0;
					break;
				}

				/*
				 * Check if the fastboot client wanted to
				 * continue booting
				 */
				if (interface.continue_booting) {
					printf("Fastboot ended by client\n");
					break;
				}

				/* Check if there is something to upload */
				tx_handler(&interface);
			}
		}

		/* Reset the board specific support */
		fastboot_shutdown();

		/* restart the loop if a disconnect was detected */
	} while (continue_from_disconnect);

	return ret;
}

U_BOOT_CMD(
	fastboot,	2,	1,	do_fastboot,
	"fastboot- use USB Fastboot protocol\n",
	"[inactive timeout]\n"
	"    - Run as a fastboot usb device.\n"
	"    - The optional inactive timeout is the decimal seconds before\n"
	"    - the normal console resumes\n"
);


void fastboot_flash_dump_ptn(void)
{
	unsigned int n;
	for (n = 0; n < pcount; n++) {
		fastboot_ptentry *ptn = ptable + n;
		if (ptn->length)
			printf("dev=%s ptn %d name='%s' start=0x%x len=0x%x\n",
					ptn->dev_ops->name, n, ptn->name, ptn->start, ptn->length);
	}
}


fastboot_ptentry *fastboot_flash_find_ptn_len(const char *name, int match_len)
{
	unsigned int n;

	for (n = 0; n < pcount; n++) {
		/* Make sure a substring is not accepted */
		if (match_len == strlen(ptable[n].name)) {
			if (0 == strncmp(ptable[n].name, name, match_len))
				return ptable + n;
		}
	}
	return NULL;
}

fastboot_ptentry *fastboot_flash_find_ptn(const char *name)
{
	fastboot_ptentry *p = fastboot_flash_find_ptn_len(name, strlen(name));
	if (!p) {
		printf("can't find partition: %s, dump the partition table\n", name);
		fastboot_flash_dump_ptn();
	}
	return p;
}

fastboot_ptentry *fastboot_flash_find(const char *name)
{
	return fastboot_flash_find_ptn_len(name, strlen(name));
}

fastboot_ptentry *fastboot_flash_get_ptn(unsigned int n)
{
	if (n < pcount)
		return ptable + n;
	else
		return 0;
}

/*
 * Android style flash utilties
 */
void fastboot_flash_add_ptn(fastboot_ptentry *ptn)
{
	fastboot_ptentry *p = fastboot_flash_find(ptn->name);
	if (!p) {
		if (pcount < MAX_PTN) {
			p = ptable + pcount;
			pcount++;
		}
	}
	if (p) {
		if (ptn->length)
			memcpy(p, ptn, sizeof(fastboot_ptentry));
		else {
			fastboot_ptentry *pend = ptable + pcount;
			fastboot_ptentry *pstart = p + 1;
			unsigned cnt = (char *)pend - (char *)pstart;

			if (cnt)
				memcpy(p, pstart, cnt);
			pcount--;
		}
	}
}

void fastboot_clear_device(int dev_index, const struct dev_partition_ops *dev_ops)
{
	fastboot_ptentry *ptn = ptable;
	fastboot_ptentry *pend = ptable + pcount;

	while (ptn < pend) {
		if ((ptn->dev_index == dev_index) && (ptn->dev_ops == dev_ops)) {
			fastboot_ptentry *pstart = ptn + 1;
			unsigned cnt = (char *)pend - (char *)pstart;

			if (cnt)
				memcpy(ptn, pstart, cnt);
			pcount--;
			pend--;
		} else {
			ptn++;
		}
	}
}

unsigned int fastboot_flash_get_ptn_count(void)
{
	return pcount;
}

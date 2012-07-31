/*
 * Copyright (C) 2012 Boundary Devices, Inc.
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
#include <asm/errno.h>
#include <malloc.h>
#include <fastboot.h>
#include <spi_flash.h>

extern int do_spi_flash(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

static int sf_flash(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response)
{
	char s_address[16], s_offset[16], s_length[16];
	char *sf_erase[] = {"sf", "erase", NULL, NULL};
	char *sf_write[] = {"sf", "write", NULL, NULL, NULL};
	int aligned_offset;
	int length = pi->download_bytes;
	unsigned start, end;

	if (length > ptn->length) {
		printf("Image too large for the partition\n");
		sprintf(response, "FAILimage too large for partition");
		return -1;
	}
/*
 * sample commands sent
 * sf probe
 * sf erase 0 0x40000
 * # two steps to prevent bricking on power failure
 * sf write 0x72000400 0x800 $filesize
 * sf write 0x72000000 0x400 0x400
 */
	printf("Erasing '%s'\n", ptn->name);
	/* offset */
	aligned_offset = ptn->start & ~0xfff;
	sprintf(s_offset, "0x%x", aligned_offset);
	/* byte count */
	sprintf(s_length, "0x%x", length + ptn->start - aligned_offset);
	sf_erase[2] = s_offset;
	sf_erase[3] = s_length;
	if (do_spi_flash(NULL, 0, ARRAY_SIZE(sf_erase), sf_erase)) {
		printf("Erasing '%s' FAILED!\n", ptn->name);
		sprintf(response, "FAIL:erase");
		return 0;
	}

	sf_write[2] = s_address;
	sf_write[3] = s_offset;
	sf_write[4] = s_length;
	printf("Writing '%s'\n", ptn->name);
	start = ptn->start;
	end = start + length;
	if ((0x400 >= start) && (0x800 < end)) {
		/* two step write to prevent bricking on power failure */
		unsigned new_start = 0x800;
		sprintf(s_address, "%p", pi->transfer_buffer + new_start - start);
		sprintf(s_offset, "0x%x", new_start);
		sprintf(s_length, "0x%x", end - new_start);

		if (do_spi_flash(NULL, 0, ARRAY_SIZE(sf_write), sf_write)) {
			printf("Writing '%s' FAILED!\n", ptn->name);
			sprintf(response, "FAIL:write");
			return 0;
		}
		length = new_start - start;
	}
	sprintf(s_address, "%p", pi->transfer_buffer);
	sprintf(s_offset, "0x%x", start);
	sprintf(s_length, "0x%x", length);

	if (do_spi_flash(NULL, 0, ARRAY_SIZE(sf_write), sf_write)) {
		printf("Writing '%s' FAILED!\n", ptn->name);
		sprintf(response, "FAIL:write");
		return 0;
	}
	printf("Writing '%s' DONE!\n", ptn->name);
	sprintf(response, "OKAY");
	return 0;
}

static int sf_partition_info(struct fastboot_ptentry *ptn, int seg, int partition)
{
	switch (seg) {
	case FB_SEG_MBR:
		ptn->start = 0;
		ptn->length = 0x400;
		break;
	case FB_SEG_BOOTLOADER:
		ptn->start = 0x400;
		ptn->length = CONFIG_ENV_SF_OFFSET- ptn->start;
		break;
#ifdef CONFIG_ENV_SF_SIZE
	case FB_SEG_ENV:
		ptn->start = CONFIG_ENV_SF_OFFSET;
		ptn->length = CONFIG_ENV_SF_SIZE;
		break;
#endif
	default:
		return -ENODEV;
	}
	return 0;
}

const struct dev_partition_ops part_sf_ops = {
	.name = "spi",
	.flash = sf_flash,
	.partion_info = sf_partition_info,
};

int get_spi_flash_size(void);

int fastboot_init_sf_ptable(void)
{
	fastboot_ptentry fb_part;
	int size = get_spi_flash_size();

	if (!size) {
		printf("Probing spi flash FAILED!\n");
		return 0;
	}
	fb_part.boot_partition = 0;
	fb_part.dev_index = 0;
	fb_part.dev_ops = &part_sf_ops;

	strcpy(fb_part.name, "spi");
	fb_part.start = 0x0;
	fb_part.length = size;
	fastboot_flash_add_ptn(&fb_part);
	return 0;
}

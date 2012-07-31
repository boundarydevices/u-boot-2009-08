/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
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
#include <fastboot.h>
#include <malloc.h>
#include <mmc.h>
#include <sata.h>
#include <usb/imx_udc.h>
#include <usbdevice.h>

#define MMC_SATA_BLOCK_SIZE 512

/*
 * imx family android layout
 * mbr -  0 ~ 0x3FF byte
 * bootloader - 0x400 ~ 0xFFFFF byte
 * kernel - 0x100000 ~ 5FFFFF byte
 * uramedisk - 0x600000 ~ 0x6FFFFF  supposing 1M temporarily
 * SYSTEM partition - /dev/mmcblk0p2  or /dev/sda2
 * RECOVERY parittion - dev/mmcblk0p4 or /dev/sda4
 */

extern int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
#if defined(CONFIG_CMD_SATA)
extern int do_sata(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
#endif

int add_mmc(int dev_index, const struct dev_partition_ops *dev_ops);

static int mmc_flash(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response)
{
	char source[32], dest[32];
	char length[32], slot_no[32];
	char part_no[32];
	unsigned int temp;
	char *mmc_write[] = {"mmc", "write", NULL, NULL, NULL};
	char *mmc_dev[] = {"mmc", "dev", NULL, NULL};
	/*
	 * mmc boot partition: -1 means no partition,
	 * 0 user part., 1 boot part.
	 * default is no partition,
	 * for emmc default user part
	 */
	int partition_id = -1;
	struct mmc *mmc = find_mmc_device(ptn->dev_index);

	if (!mmc)
		return -ENODEV;
	if (mmc_init(mmc))
		printf("MMC card init failed!\n");

	/* multiple boot paritions for eMMC 4.3 later */
	if (mmc->part_config != MMCPART_NOAVAILABLE) {
		partition_id = ptn->boot_partition;
	}

	if (pi->download_bytes > ptn->length * MMC_SATA_BLOCK_SIZE) {
		printf("Image too large for the partition\n");
		sprintf(response, "FAILimage too large for partition");
		return -1;
	}
	printf("writing to partition '%s'\n", ptn->name);

	mmc_dev[2] = slot_no;
	mmc_dev[3] = part_no;
	mmc_write[2] = source;
	mmc_write[3] = dest;
	mmc_write[4] = length;

	sprintf(slot_no, "%d", ptn->dev_index);
	sprintf(source, "%p", pi->transfer_buffer);
	/* partition no */
	sprintf(part_no, "%d", partition_id);
	/* block offset */
	sprintf(dest, "0x%x", ptn->start);
	/* block count */
	temp = (pi->download_bytes + MMC_SATA_BLOCK_SIZE - 1) /
			MMC_SATA_BLOCK_SIZE;
	sprintf(length, "0x%x", temp);

	printf("Initializing '%s'\n", ptn->name);
	if (do_mmcops(NULL, 0, 4, mmc_dev))
		sprintf(response, "FAIL:Init of MMC card");
	else
		sprintf(response, "OKAY");

	printf("Writing '%s'\n", ptn->name);
	if (do_mmcops(NULL, 0, 5, mmc_write)) {
		printf("Writing '%s' FAILED!\n", ptn->name);
		sprintf(response, "FAIL: Write partition");
	} else {
		printf("Writing '%s' DONE!\n", ptn->name);
		sprintf(response, "OKAY");
	}
	return 0;
}

static int sata_flash(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response)
{
	char source[32], dest[32];
	char length[32];
	unsigned int temp;
	int ret = -1;
	char *sata_write[5] = {"sata", "write", NULL, NULL,
			NULL};

	if (pi->download_bytes > ptn->length * MMC_SATA_BLOCK_SIZE) {
		printf("Image too large for the partition\n");
		sprintf(response, "FAILimage too large for partition");
		return -1;
	}

	sata_write[2] = source;
	sata_write[3] = dest;
	sata_write[4] = length;

	sprintf(source, "%p", pi->transfer_buffer);
	/* block offset */
	sprintf(dest, "0x%x", ptn->start);
	/* block count */
	temp = (pi->download_bytes + MMC_SATA_BLOCK_SIZE - 1) /
			MMC_SATA_BLOCK_SIZE;
	sprintf(length, "0x%x", temp);
	if (do_sata(NULL, 0, 5, sata_write)) {
		printf("Writing '%s' FAILED!\n", ptn->name);
		sprintf(response, "FAIL: Write partition");
	} else {
		printf("Writing '%s' DONE!\n", ptn->name);
		sprintf(response, "OKAY");
		ret = 0;
	}
	return ret; /* End of sata download */
}

static unsigned get_low_parition_start(block_dev_desc_t *dev_desc)
{
	int i;
	unsigned min = 0xffffffff;

	for (i = 0; i < 8; i++) {
		disk_partition_t info;
		if (get_partition_info(dev_desc, i, &info))
			continue;
//		printf("%d: %08lx %08lx\n", i, info.start, info.size);
		if (min > info.start)
			min = info.start;
	}
	return min;
}

static int set_partition_info(block_dev_desc_t *dev_desc,
		struct fastboot_ptentry *ptn, int partition)
{
	disk_partition_t info;
	ptn->start = 0;
	ptn->length = 0;
	if (get_partition_info(dev_desc, partition, &info))
		return -ENODEV;
	ptn->start = info.start;
	ptn->length = info.size;
	return 0;
}

static int partition_info(block_dev_desc_t *dev_desc,
		struct fastboot_ptentry *ptn, int seg, int partition)
{
	int blksz = 512;

	if (dev_desc && dev_desc->blksz)
		blksz = dev_desc->blksz;

	switch (seg) {
	case FB_SEG_MBR:
		ptn->start = 0;
		ptn->length = 1;
		break;
	case FB_SEG_BOOTLOADER:
		ptn->start = 0x400 / blksz;
		ptn->length = get_low_parition_start(dev_desc) - ptn->start;
		ptn->boot_partition = 1;
		break;
	case FB_SEG_ENV:
		ptn->start = CONFIG_ENV_MMC_OFFSET;
		ptn->length = CONFIG_ENV_MMC_SIZE;
		break;
	default:
		return set_partition_info(dev_desc, ptn, partition);
	}
	return 0;
}

static int mmc_partition_info(struct fastboot_ptentry *ptn, int seg, int partition)
{
	struct mmc *mmc;
	block_dev_desc_t *dev_desc = get_dev("mmc", ptn->dev_index);

	if (!dev_desc)
		return -ENODEV;
	mmc = find_mmc_device(ptn->dev_index);
	if (!mmc)
		return -ENODEV;
	if (mmc_init(mmc))
		printf("MMC card init failed!\n");
	return partition_info(dev_desc, ptn, seg, partition);
}

static int sata_partition_info(struct fastboot_ptentry *ptn, int seg, int partition)
{
	block_dev_desc_t *dev_desc = sata_get_dev(ptn->dev_index);
	if (!dev_desc)
		return -ENODEV;
	return partition_info(dev_desc, ptn, seg, partition);
}

static void add_fb_device(unsigned dev_index,
		const struct dev_partition_ops *dev_ops)
{
	fastboot_ptentry fb_part;

	fastboot_clear_device(dev_index, dev_ops);
	sprintf(fb_part.name, "%s%d", dev_ops->name, dev_index);
	fb_part.dev_index = dev_index;
	fb_part.dev_ops = dev_ops;
	fb_part.boot_partition = 0;

	fb_part.start = 0;
	fb_part.length = 0xffffffff;
	fastboot_flash_add_ptn(&fb_part);
}

#ifdef CONFIG_CMD_SATA
static int sata_initialized;
const struct dev_partition_ops part_sata_ops = {
	.name = "sata",
	.flash = sata_flash,
	.partion_info = sata_partition_info,
};
#endif

const struct dev_partition_ops part_mmc_ops = {
	.name = "mmc",
	.flash = mmc_flash,
	.partion_info = mmc_partition_info,
};

int fastboot_init_mmc_sata_ptable(void)
{
	int dev_index;

	for (dev_index = 0; dev_index < 4; dev_index++) {
		struct mmc *mmc;
		block_dev_desc_t *dev_desc = get_dev("mmc", dev_index);
		if (!dev_desc)
			break;
		mmc = find_mmc_device(dev_index);
		if (!mmc)
			break;
		add_fb_device(dev_index, &part_mmc_ops);
	}
#ifdef CONFIG_CMD_SATA
	if (!sata_initialized) {
		if (sata_initialize())
			return 0;
		sata_initialized = 1;
	}
	for (dev_index = 0; dev_index < CONFIG_SYS_SATA_MAX_DEVICE; dev_index++) {
		block_dev_desc_t *dev_desc = sata_get_dev(dev_index);
		if (!dev_desc)
			break;
		add_fb_device(dev_index, &part_sata_ops);
	}
#endif
	return 0;
}

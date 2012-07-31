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
#include <environment.h>
#include <fastboot.h>
#include <nand.h>

/* Use do_nand for fastboot's flash commands */
extern int do_nand(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
int do_saveenv(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);
int do_setenv(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

static void set_env(char *var, char *val)
{
	char *setenv[4]  = { "setenv", NULL, NULL, NULL, };

	setenv[1] = var;
	setenv[2] = val;

	do_setenv(NULL, 0, 3, setenv);
}

static void save_env(struct fastboot_ptentry *ptn,
		     char *var, char *val)
{
	char start[32], length[32];
	char ecc_type[32];

	char *lock[5]    = { "nand", "lock",   NULL, NULL, NULL, };
	char *unlock[5]  = { "nand", "unlock", NULL, NULL, NULL, };
	char *ecc[4]     = { "nand", "ecc",    NULL, NULL, };
	char *saveenv[2] = { "setenv", NULL, };

	lock[2] = unlock[2] = start;
	lock[3] = unlock[3] = length;

	set_env(var, val);

	/* Some flashing requires the nand's ecc to be set */
	ecc[2] = ecc_type;
	if ((ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_HW_ECC) &&
	    (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_SW_ECC))	{
		/* Both can not be true */
		printf("Warning can not do hw and sw ecc for partition '%s'\n",
			 ptn->name);
		printf("Ignoring these flags\n");
	} else if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_HW_ECC) {
		sprintf(ecc_type, "hw");
		do_nand(NULL, 0, 3, ecc);
	} else if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_SW_ECC) {
		sprintf(ecc_type, "sw");
		do_nand(NULL, 0, 3, ecc);
	}
	sprintf(start, "0x%x", ptn->start);
	sprintf(length, "0x%x", ptn->length);

	/* This could be a problem is there is an outstanding lock */
	do_nand(NULL, 0, 4, unlock);
	do_saveenv(NULL, 0, 1, saveenv);
	do_nand(NULL, 0, 4, lock);
}

static void save_block_values(struct fastboot_ptentry *ptn,
			      unsigned int offset,
			      unsigned int size)
{
	struct fastboot_ptentry *env_ptn;

	char var[64], val[32];
	char start[32], length[32];
	char ecc_type[32];

	char *lock[5]    = { "nand", "lock",   NULL, NULL, NULL, };
	char *unlock[5]  = { "nand", "unlock", NULL, NULL, NULL, };
	char *ecc[4]     = { "nand", "ecc",    NULL, NULL, };
	char *setenv[4]  = { "setenv", NULL, NULL, NULL, };
	char *saveenv[2] = { "setenv", NULL, };

	setenv[1] = var;
	setenv[2] = val;
	lock[2] = unlock[2] = start;
	lock[3] = unlock[3] = length;

	printf("saving it..\n");

	if (size == 0) {
		/* The error case, where the variables are being unset */

		sprintf(var, "%s_nand_offset", ptn->name);
		val[0] = 0;
		do_setenv(NULL, 0, 3, setenv);

		sprintf(var, "%s_nand_size", ptn->name);
		val[0] = 0;
		do_setenv(NULL, 0, 3, setenv);
	} else {
		/* Normal case */

		sprintf(var, "%s_nand_offset", ptn->name);
		sprintf(val, "0x%x", offset);

		printf("%s %s %s\n", setenv[0], setenv[1], setenv[2]);

		do_setenv(NULL, 0, 3, setenv);

		sprintf(var, "%s_nand_size", ptn->name);

		sprintf(val, "0x%x", size);

		printf("%s %s %s\n", setenv[0], setenv[1], setenv[2]);

		do_setenv(NULL, 0, 3, setenv);
	}


	/* Warning :
	   The environment is assumed to be in a partition named 'enviroment'.
	   It is very possible that your board stores the enviroment
	   someplace else. */
	env_ptn = fastboot_flash_find_ptn("environment");

	if (env_ptn) {
		/* Some flashing requires the nand's ecc to be set */
		ecc[2] = ecc_type;
		if ((env_ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_HW_ECC) &&
		    (env_ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_SW_ECC)) {
			/* Both can not be true */
			printf("Warning can not do hw and sw ecc for \
				partition '%s'\n", ptn->name);
			printf("Ignoring these flags\n");
		} else if (env_ptn->flags &
			    FASTBOOT_PTENTRY_FLAGS_WRITE_HW_ECC) {
			sprintf(ecc_type, "hw");
			do_nand(NULL, 0, 3, ecc);
		} else if (env_ptn->flags &
			    FASTBOOT_PTENTRY_FLAGS_WRITE_SW_ECC) {
			sprintf(ecc_type, "sw");
			do_nand(NULL, 0, 3, ecc);
		}

		sprintf(start, "0x%x", env_ptn->start);
		sprintf(length, "0x%x", env_ptn->length);

		/* This could be a problem is there is an outstanding lock */
		do_nand(NULL, 0, 4, unlock);
	}

	do_saveenv(NULL, 0, 1, saveenv);

	if (env_ptn)
		do_nand(NULL, 0, 4, lock);
}

/* When save = 0, just parse.  The input is unchanged
   When save = 1, parse and do the save.  The input is changed */
static int parse_env(void *ptn, struct cmd_fastboot_interface *pi, char *err_string, int save, int debug)
{
	int ret = 1;
	unsigned int sets = 0;
	unsigned int comment_start = 0;
	char *var = NULL;
	char *var_end = NULL;
	char *val = NULL;
	char *val_end = NULL;
	unsigned int i;

	char *buff = (char *)pi->transfer_buffer;
	unsigned int size = pi->download_bytes_unpadded;

	/* The input does not have to be null terminated.
	   This will cause a problem in the corner case
	   where the last line does not have a new line.
	   Put a null after the end of the input.

	   WARNING : Input buffer is assumed to be bigger
	   than the size of the input */
	if (save)
		buff[size] = 0;

	for (i = 0; i < size; i++) {

		if (NULL == var) {

			/*
			 * Check for comments, comment ok only on
			 * mostly empty lines
			 */
			if (buff[i] == '#')
				comment_start = 1;

			if (comment_start) {
				if  ((buff[i] == '\r') ||
				     (buff[i] == '\n')) {
					comment_start = 0;
				}
			} else {
				if (!((buff[i] == ' ') ||
				      (buff[i] == '\t') ||
				      (buff[i] == '\r') ||
				      (buff[i] == '\n'))) {
					/*
					 * Normal whitespace before the
					 * variable
					 */
					var = &buff[i];
				}
			}

		} else if (((NULL == var_end) || (NULL == val)) &&
			   ((buff[i] == '\r') || (buff[i] == '\n'))) {

			/* This is the case when a variable
			   is unset. */

			if (save) {
				/* Set the var end to null so the
				   normal string routines will work

				   WARNING : This changes the input */
				buff[i] = '\0';

				save_env(ptn, var, val);

				if (debug)
					printf("Unsetting %s\n", var);
			}

			/* Clear the variable so state is parse is back
			   to initial. */
			var = NULL;
			var_end = NULL;
			sets++;
		} else if (NULL == var_end) {
			if ((buff[i] == ' ') ||
			    (buff[i] == '\t'))
				var_end = &buff[i];
		} else if (NULL == val) {
			if (!((buff[i] == ' ') ||
			      (buff[i] == '\t')))
				val = &buff[i];
		} else if (NULL == val_end) {
			if ((buff[i] == '\r') ||
			    (buff[i] == '\n')) {
				/* look for escaped cr or ln */
				if ('\\' == buff[i - 1]) {
					/* check for dos */
					if ((buff[i] == '\r') &&
					    (buff[i+1] == '\n'))
						buff[i + 1] = ' ';
					buff[i - 1] = buff[i] = ' ';
				} else {
					val_end = &buff[i];
				}
			}
		} else {
			sprintf(err_string, "Internal Error");

			if (debug)
				printf("Internal error at %s %d\n",
				       __FILE__, __LINE__);
			return 1;
		}
		/* Check if a var / val pair is ready */
		if (NULL != val_end) {
			if (save) {
				/* Set the end's with nulls so
				   normal string routines will
				   work.

				   WARNING : This changes the input */
				*var_end = '\0';
				*val_end = '\0';

				save_env(ptn, var, val);

				if (debug)
					printf("Setting %s %s\n", var, val);
			}

			/* Clear the variable so state is parse is back
			   to initial. */
			var = NULL;
			var_end = NULL;
			val = NULL;
			val_end = NULL;

			sets++;
		}
	}

	/* Corner case
	   Check for the case that no newline at end of the input */
	if ((NULL != var) &&
	    (NULL == val_end)) {
		if (save) {
			/* case of val / val pair */
			if (var_end)
				*var_end = '\0';
			/* else case handled by setting 0 past
			   the end of buffer.
			   Similar for val_end being null */
			save_env(ptn, var, val);

			if (debug) {
				if (var_end)
					printf("Trailing Setting %s %s\n", var, val);
				else
					printf("Trailing Unsetting %s\n", var);
			}
		}
		sets++;
	}
	/* Did we set anything ? */
	if (0 == sets)
		sprintf(err_string, "No variables set");
	else
		ret = 0;

	return ret;
}

static int saveenv_to_ptn(struct fastboot_ptentry *ptn,
		struct cmd_fastboot_interface *pi, char *err_string)
{
	int ret = 1;
	int save = 0;
	int debug = 0;

	/* err_string is only 32 bytes
	   Initialize with a generic error message. */
	sprintf(err_string, "%s", "Unknown Error");

	/* Parse the input twice.
	   Only save to the enviroment if the entire input if correct */
	save = 0;
	if (0 == parse_env(ptn, pi, err_string, save, debug)) {
		save = 1;
		ret = parse_env(ptn, pi, err_string, save, debug);
	}
	return ret;
}

static void set_ptn_ecc(struct fastboot_ptentry *ptn)
{
	char ecc_type[32];
	char *ecc[4] = {"nand", "ecc", NULL, NULL, };

	/* Some flashing requires the nand's ecc to be set */
	ecc[2] = ecc_type;
	if ((ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_HW_ECC) &&
	    (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_SW_ECC)) {
		/* Both can not be true */
		printf("Warning can not do hw and sw ecc for partition '%s'\n",
		       ptn->name);
		printf("Ignoring these flags\n");
	} else if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_HW_ECC) {
		sprintf(ecc_type, "hw");
		do_nand(NULL, 0, 3, ecc);
	} else if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_SW_ECC) {
		sprintf(ecc_type, "sw");
		do_nand(NULL, 0, 3, ecc);
	}
}

static int write_to_ptn(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi)
{
	int ret = 1;
	char start[32], length[32];
	char wstart[32], wlength[32], addr[32];
	char write_type[32];
	int repeat, repeat_max;

	char *lock[5]   = { "nand", "lock",   NULL, NULL, NULL, };
	char *unlock[5] = { "nand", "unlock", NULL, NULL, NULL,	};
	char *write[6]  = { "nand", "write",  NULL, NULL, NULL, NULL, };
	char *erase[5]  = { "nand", "erase",  NULL, NULL, NULL, };

	lock[2] = unlock[2] = erase[2] = start;
	lock[3] = unlock[3] = erase[3] = length;

	write[1] = write_type;
	write[2] = addr;
	write[3] = wstart;
	write[4] = wlength;

	printf("flashing '%s'\n", ptn->name);

	/* Which flavor of write to use */
	if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_I)
		sprintf(write_type, "write.i");
#ifdef CFG_NAND_YAFFS_WRITE
	else if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_YAFFS)
		sprintf(write_type, "write.yaffs");
#endif
	else
		sprintf(write_type, "write");

	set_ptn_ecc(ptn);

	/* Some flashing requires writing the same data in multiple,
	   consecutive flash partitions */
	repeat_max = 1;
	if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_REPEAT_MASK) {
		if (ptn->flags &
		    FASTBOOT_PTENTRY_FLAGS_WRITE_CONTIGUOUS_BLOCK) {
			printf("Warning can not do both 'contiguous block' and 'repeat' writes for for partition '%s'\n", ptn->name);
			printf("Ignoring repeat flag\n");
		} else {
			repeat_max = ptn->flags &
				FASTBOOT_PTENTRY_FLAGS_REPEAT_MASK;
		}
	}

	/* Unlock the whole partition instead of trying to
	   manage special cases */
	sprintf(length, "0x%x", ptn->length * repeat_max);

	for (repeat = 0; repeat < repeat_max; repeat++) {
		sprintf(start, "0x%x", ptn->start + (repeat * ptn->length));

		do_nand(NULL, 0, 4, unlock);
		do_nand(NULL, 0, 4, erase);

		if ((ptn->flags &
		     FASTBOOT_PTENTRY_FLAGS_WRITE_NEXT_GOOD_BLOCK) &&
		    (ptn->flags &
		     FASTBOOT_PTENTRY_FLAGS_WRITE_CONTIGUOUS_BLOCK)) {
			/* Both can not be true */
			printf("Warning can not do 'next good block' and \
				'contiguous block' for partition '%s'\n",
				ptn->name);
			printf("Ignoring these flags\n");
		} else if (ptn->flags &
			   FASTBOOT_PTENTRY_FLAGS_WRITE_NEXT_GOOD_BLOCK) {
			/* Keep writing until you get a good block
			   transfer_buffer should already be aligned */
			if (pi->nand_block_size) {
				unsigned int blocks = pi->download_bytes /
					pi->nand_block_size;
				unsigned int i = 0;
				unsigned int offset = 0;

				sprintf(wlength, "0x%x",
					pi->nand_block_size);
				while (i < blocks) {
					/* Check for overflow */
					if (offset >= ptn->length)
						break;

					/* download's address only advance
					   if last write was successful */
					sprintf(addr, "%p",
						pi->transfer_buffer +
						(i * pi->nand_block_size));

					/* nand's address always advances */
					sprintf(wstart, "0x%x",
						ptn->start + (repeat * ptn->length) + offset);

					ret = do_nand(NULL, 0, 5, write);
					if (ret)
						break;
					else
						i++;

					/* Go to next nand block */
					offset += pi->nand_block_size;
				}
			} else {
				printf("Warning nand block size can not be 0 \
					when using 'next good block' for \
					partition '%s'\n", ptn->name);
				printf("Ignoring write request\n");
			}
		} else if (ptn->flags &
			 FASTBOOT_PTENTRY_FLAGS_WRITE_CONTIGUOUS_BLOCK) {
			/* Keep writing until you get a good block
			   transfer_buffer should already be aligned */
			if (pi->nand_block_size) {
				if (0 == nand_curr_device) {
					nand_info_t *nand;
					unsigned long off;
					unsigned int ok_start;

					nand = &nand_info[nand_curr_device];

					printf("\nDevice %d bad blocks:\n",
					       nand_curr_device);

					/* Initialize the ok_start to the
					   start of the partition
					   Then try to find a block large
					   enough for the download */
					ok_start = ptn->start;

					/* It is assumed that the start and
					   length are multiples of block size */
					for (off = ptn->start;
					     off < ptn->start + ptn->length;
					     off += nand->erasesize) {
						if (nand_block_isbad(nand, off)) {
							/* Reset the ok_start
							   to the next block */
							ok_start = off +
								nand->erasesize;
						}

						/* Check if we have enough
						   blocks */
						if ((ok_start - off) >=
						    pi->download_bytes)
							break;
					}

					/* Check if there is enough space */
					if (ok_start + pi->download_bytes <=
					    ptn->start + ptn->length) {
						sprintf(addr,    "%p", pi->transfer_buffer);
						sprintf(wstart,  "0x%x", ok_start);
						sprintf(wlength, "0x%x", pi->download_bytes);

						ret = do_nand(NULL, 0, 5, write);

						/* Save the results into an
						   environment variable on the
						   format
						   ptn_name + 'offset'
						   ptn_name + 'size'  */
						if (ret) {
							/* failed */
							save_block_values(ptn, 0, 0);
						} else {
							/* success */
							save_block_values(ptn, ok_start, pi->download_bytes);
						}
					} else {
						printf("Error could not find enough contiguous space in partition '%s' \n", ptn->name);
						printf("Ignoring write request\n");
					}
				} else {
					/* TBD : Generalize flash handling */
					printf("Error only handling 1 NAND per board");
					printf("Ignoring write request\n");
				}
			} else {
				printf("Warning nand block size can not be 0 \
					when using 'continuous block' for \
					partition '%s'\n", ptn->name);
				printf("Ignoring write request\n");
			}
		} else {
			/* Normal case */
			sprintf(addr,    "%p", pi->transfer_buffer);
			sprintf(wstart,  "0x%x", ptn->start +
				(repeat * ptn->length));
			sprintf(wlength, "0x%x", pi->download_bytes);
#ifdef CFG_NAND_YAFFS_WRITE
			if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_WRITE_YAFFS)
				sprintf(wlength, "0x%x",
					pi->download_bytes_unpadded);
#endif

			ret = do_nand(NULL, 0, 5, write);

			if (0 == repeat) {
				if (ret) /* failed */
					save_block_values(ptn, 0, 0);
				else     /* success */
					save_block_values(ptn, ptn->start,
							  pi->download_bytes);
			}
		}

		do_nand(NULL, 0, 4, lock);

		if (ret)
			break;
	}

	return ret;
}

/*
 * Pad to block length
 *  In most cases, padding the download to be block aligned is correct. The
 *  exception is when the following flash writes to the oob area.  This happens
 *  when the image is a YAFFS image.  Since we do not know what the download is
 *  until it is flashed, go ahead and pad it, but save the true size in case if
 *  should have been unpadded
 */
void pad_nand(struct cmd_fastboot_interface *pi)
{
	pi->download_bytes_unpadded = pi->download_bytes;
	if (pi->nand_block_size) {
		if (pi->download_bytes %
		    pi->nand_block_size) {
			unsigned int pad = pi->nand_block_size - (pi->download_bytes % pi->nand_block_size);
			unsigned int i;

			for (i = 0; i < pad; i++) {
				if (pi->download_bytes >= pi->transfer_buffer_size)
					break;

				pi->transfer_buffer[pi->download_bytes] = 0;
				pi->download_bytes++;
			}
		}
	}
}

static int flash_env_nand(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response)
{
	pad_nand(pi);
	if (pi->download_bytes > ptn->length) {
		sprintf(response, "FAILimage too large for partition");
		/* TODO : Improve check for yaffs write */
		return -1;
	}
	/*
	 * Since the response can only be 64 bytes,
	 * there is no point in having a large error message.
	 */
	char err_string[32];
	if (saveenv_to_ptn(ptn, pi, &err_string[0])) {
		printf("savenv '%s' failed : %s\n", ptn->name, err_string);
		sprintf(response, "FAIL%s", err_string);
	} else {
		printf("partition '%s' saveenv-ed\n", ptn->name);
		sprintf(response, "OKAY");
	}
}

static int flash_nand(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response)
{
	pad_nand(pi);
	if (pi->download_bytes > ptn->length) {
		sprintf(response, "FAILimage too large for partition");
		/* TODO : Improve check for yaffs write */
		return -1;
	}
	if (write_to_ptn(ptn, pi)) {
		printf("flashing '%s' failed\n", ptn->name);
		sprintf(response, "FAILfailed to flash partition");
	} else {
		printf("partition '%s' flashed\n", ptn->name);
		sprintf(response, "OKAY");
	}
	return 0;
}

static int erase_nand(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response)
{
	char start[32], length[32];
	int status, repeat, repeat_max;

	printf("erasing '%s'\n", ptn->name);

	char *lock[5]   = { "nand", "lock",   NULL, NULL, NULL, };
	char *unlock[5] = { "nand", "unlock", NULL, NULL, NULL,	};
	char *erase[5]  = { "nand", "erase",  NULL, NULL, NULL, };

	lock[2] = unlock[2] = erase[2] = start;
	lock[3] = unlock[3] = erase[3] = length;

	repeat_max = 1;
	if (ptn->flags & FASTBOOT_PTENTRY_FLAGS_REPEAT_MASK)
		repeat_max = ptn->flags & FASTBOOT_PTENTRY_FLAGS_REPEAT_MASK;

	sprintf(length, "0x%x", ptn->length);
	for (repeat = 0; repeat < repeat_max; repeat++) {
		sprintf(start, "0x%x", ptn->start + (repeat * ptn->length));

		do_nand(NULL, 0, 4, unlock);
		status = do_nand(NULL, 0, 4, erase);
		do_nand(NULL, 0, 4, lock);

		if (status)
			break;
	}

	if (status) {
		sprintf(response, "FAILfailed to erase partition");
	} else {
		printf("partition '%s' erased\n", ptn->name);
		sprintf(response, "OKAY");
	}
	return 0;
}

int upload_nand(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response, unsigned size, unsigned is_raw)
{
	/*
	 * This is where the transfer
	 * buffer is populated
	 */
	char start[32], length[32], type[32], addr[32];
	unsigned char *buf = pi->transfer_buffer;
	char *read[] = { "nand", NULL, NULL, NULL, NULL, NULL};

	/*
	 * Setting upload_size causes
	 * transfer to happen in main loop
	 */
	pi->upload_size = size;
	pi->upload_bytes = 0;
	pi->upload_error = 0;

	/*
	 * Poison the transfer buffer, 0xff
	 * is erase value of nand
	 */
	memset(buf, 0xff, pi->upload_size);

	/* Which flavor of read to use */
	sprintf(type, is_raw ? "read.raw" : "read.i");

	sprintf(addr, "%p", pi->transfer_buffer);
	sprintf(start, "0x%x", ptn->start);
	sprintf(length, "0x%x", pi->upload_size);

	read[1] = type;
	read[2] = addr;
	read[3] = start;
	read[4] = length;

	set_ptn_ecc(ptn);

	do_nand(NULL, 0, 5, read);

	/* Send the data response */
	sprintf(response, "DATA%08x", size);
	return 0;
}

const struct dev_partition_ops part_nand_ops = {
	.name = "nand",
	.flash = flash_nand,
	.erase = erase_nand,
	.upload = upload_nand,
};

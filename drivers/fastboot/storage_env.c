/* Use do_saveenv to permenantly save data */
int do_saveenv(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

/*
 * This is a saveenv of downloaded data
 */
int flash_env(struct fastboot_ptentry *ptn, struct cmd_fastboot_interface *pi, char *response)
{
	unsigned int i = 0;
	unsigned max = ENV_SIZE;
	/*
	 * Env file is expected with a NULL delimeter between
	 * env variables So replace New line Feeds (0x0a) with
	 * NULL (0x00)
	 */
	printf("Goto write env, flags=0x%x\n", ptn->flags);
	for (i = 0; i < pi->download_bytes; i++) {
		if (pi->transfer_buffer[i] == 0x0a)
			pi->transfer_buffer[i] = 0x00;
	}
	memset(env_ptr->data, 0, max);
	if (max > pi->download_bytes)
		max = pi->download_bytes;
	memcpy(env_ptr->data, pi->transfer_buffer, max);
	do_saveenv(NULL, 0, 1, NULL);
	printf("saveenv to '%s' DONE!\n", ptn->name);
	sprintf(response, "OKAY");
	return 0;
}

/* drivers/misc/apanic_handle_ram.c
 *
 * Handle a kernel panic notification and generate a panic dump in the Android
 * "apanic" style.  Store the result into a fixed memory location to be read
 * after a warm reset by the "apanic_report_ram" or by the bootloader.
 *
 * Copyright (c) 2010, Motorola.
 *
 * Authors:
 *	Russ W. Knize <russ.knize@motorola.com>
 *
 * Derived from:
 *   drivers/misc/apanic.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: San Mehat <san@android.com>
 *
 * Other parts from:
 *   drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/preempt.h>
#include <asm/io.h>

#include <linux/apanic.h>


/* Grab some external hooks */
extern void ram_console_enable_console(int);
extern int log_buf_copy(char *dest, int idx, int len);
extern void log_buf_clear(void);
extern void apanic_ram_dump(void*, int);


struct apanic_data {
	uint8_t	*buffer;
	size_t	buffer_size;
};

static struct apanic_data drv_ctx;

static int in_panic = 0;


static int apanic(struct notifier_block *this, unsigned long event,
			void *ptr)
{
	struct apanic_data *ctx = &drv_ctx;
	struct panic_header *hdr = (struct panic_header *)ctx->buffer;
	int console_offset = 0;
	int console_len = 0;
	int threads_offset = 0;
	int threads_len = 0;

	if (in_panic)
		return NOTIFY_DONE;
	in_panic = 1;
#ifdef CONFIG_PREEMPT
	/* Ensure that cond_resched() won't try to preempt anybody */
	add_preempt_count(PREEMPT_ACTIVE);
#endif
	touch_softlockup_watchdog();

	if (!ctx->buffer)
		goto out;

	if (hdr->magic == PANIC_MAGIC) {
		printk(KERN_EMERG "panic memory block in use!\n");
		goto out;
	}

	memset(ctx->buffer, 0, ctx->buffer_size);

	console_offset = sizeof(struct panic_header);
	console_len = (ctx->buffer_size - sizeof(struct panic_header)) / 2;

	/*
	 * Write out the console
	 */
	console_len = log_buf_copy(ctx->buffer, 0, console_len);
	if (console_len < 0) {
		printk(KERN_EMERG "apanic: error writing console to panic log (%d)\n",
		       console_len);
		console_len = 0;
	}

	/*
	 * Write out all threads
	 */
	threads_offset = console_offset + console_len;
	threads_len = ctx->buffer_size - console_len - sizeof(struct panic_header);
#if 0
	ram_console_enable_console(0);

	log_buf_clear();
	show_state_filter(0);
	threads_len = log_buf_copy(ctx->buffer, 0, threads_len);
	if (threads_len < 0) {
		printk(KERN_EMERG "apanic: error writing threads to panic log (%d)\n",
		       threads_len);
		threads_len = 0;
	}
#endif
	/*
	 * Finally write the panic header
	 */
	memset(hdr, 0, sizeof(struct panic_header));
	hdr->magic = PANIC_MAGIC;
	hdr->version = PHDR_VERSION;

	hdr->console_offset = console_offset;
	hdr->console_length = console_len;

	hdr->threads_offset = threads_offset;
	hdr->threads_length = threads_len;

	printk(KERN_EMERG "apanic: panic dump successfully wrote %d bytes to RAM\n",
	       sizeof(struct panic_header) + console_len + threads_len);

	apanic_ram_dump(ctx->buffer, 16);

 out:
#ifdef CONFIG_PREEMPT
	sub_preempt_count(PREEMPT_ACTIVE);
#endif
	in_panic = 0;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= apanic,
};

static int panic_dbg_get(void *data, u64 *val)
{
	apanic(NULL, 0, NULL);
	return 0;
}

static int panic_dbg_set(void *data, u64 val)
{
	BUG();
	return -1;
}

DEFINE_SIMPLE_ATTRIBUTE(panic_dbg_fops, panic_dbg_get, panic_dbg_set, "%llu\n");

static int apanic_handle_ram_probe(struct platform_device *pdev)
{
	struct apanic_data *ctx = &drv_ctx;
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;

	if (res == NULL || pdev->num_resources != 1 ||
	    !(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "apanic_handle_ram: invalid resource, %p %d flags "
		       "%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}
	buffer_size = res->end - res->start + 1;
	start = res->start;
	buffer_size = (buffer_size + PAGE_SIZE - 1) & PAGE_MASK;
	start &= PAGE_MASK;
	printk(KERN_INFO "apanic_handle_ram: got buffer at %x, size %x\n",
	       start, buffer_size);
	ctx->buffer = ioremap_wc(res->start, buffer_size);
	if (ctx->buffer == NULL) {
		printk(KERN_ERR "apanic_handle_ram: failed to map memory\n");
		return -ENOMEM;
	}
	ctx->buffer_size = buffer_size;

	apanic_ram_dump(ctx->buffer, 16);

	return 0;
}

static struct platform_driver apanic_handle_ram_driver = {
	.probe = apanic_handle_ram_probe,
	.driver		= {
		.name	= "apanic_handle_ram",
	},
};

int __init apanic_handle_ram_init(void)
{
	int result = 0;

	memset(&drv_ctx, 0, sizeof(drv_ctx));
	result = platform_driver_register(&apanic_handle_ram_driver);
	if (result == 0) {
		atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
		debugfs_create_file("apanic", 0644, NULL, NULL, &panic_dbg_fops);
		printk(KERN_INFO "Android kernel panic handler initialized\n"L);
	}

	return result;
}

module_init(apanic_handle_ram_init);

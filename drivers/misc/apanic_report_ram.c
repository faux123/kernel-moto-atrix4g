/* drivers/misc/apanic_report_ram.c
 *
 * Publish into user space a panic dump stored in RAM by the "apanic_handle_ram"
 * driver.  It is exposed via two entries in /proc in the style that Android's
 * "init" process is expecting.  This is implemented as a separate driver so
 * that a hybrid approach can be used.
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
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <asm/io.h>

#include <linux/apanic.h>


#define PROC_APANIC_CONSOLE 1
#define PROC_APANIC_THREADS 2


struct apanic_data {
	uint8_t			*buffer;
	size_t			buffer_size;
	struct panic_header	curr;
	struct proc_dir_entry	*apanic_console;
	struct proc_dir_entry	*apanic_threads;
};

static struct apanic_data drv_ctx;
static struct work_struct proc_removal_work;
static DEFINE_MUTEX(drv_mutex);

void apanic_ram_dump(void* base, int rows)
{
	static char *dump_base = NULL;
	char *p;
	int i;

	if (base)
		dump_base = base;

	p = dump_base;
	if (!p)
		return;

	for (i = 0; i < rows; i++) {
		printk("apanic_ram_dump: %08x   "
		       "%02X %02X %02X %02X  %02X %02X %02X %02X  "
		       "%02X %02X %02X %02X  %02X %02X %02X %02X\n",
			p,
			p[0],  p[1],  p[2],  p[3],  p[4],  p[5],  p[6],  p[7],
			p[8],  p[9],  p[10], p[11], p[12], p[13], p[14], p[15]);
		p += 16;
	}
}

static void apanic_ram_erase(void)
{
	struct apanic_data *ctx = &drv_ctx;

	memset(ctx->buffer, 0, ctx->buffer_size);
}

static int apanic_proc_read(char *buffer, char **start, off_t offset,
			    int count, int *peof, void *dat)
{
	struct apanic_data *ctx = &drv_ctx;
	size_t file_length;
	off_t file_offset;

	if (!count)
		return 0;

	mutex_lock(&drv_mutex);

	switch ((int) dat) {
	case PROC_APANIC_CONSOLE:
		file_length = ctx->curr.console_length;
		file_offset = ctx->curr.console_offset;
		break;
	case PROC_APANIC_THREADS:
		file_length = ctx->curr.threads_length;
		file_offset = ctx->curr.threads_offset;
		break;
	default:
		pr_err("bad apanic source (%d)\n", (int) dat);
		mutex_unlock(&drv_mutex);
		return -EINVAL;
	}

	if (offset >= file_length) {
		mutex_unlock(&drv_mutex);
		return 0;
	}

	if ((offset + count) > file_length)
		count = file_length - offset;

	memcpy(buffer, ctx->buffer, count);

	*start = count;		// WTF?

	if ((offset + count) == file_length)
		*peof = 1;

	mutex_unlock(&drv_mutex);
	return count;
}

static void apanic_remove_proc_work(struct work_struct *work)
{
	struct apanic_data *ctx = &drv_ctx;

	mutex_lock(&drv_mutex);
	memset(&ctx->curr, 0, sizeof(struct panic_header));
	apanic_ram_erase();
	if (ctx->apanic_console) {
		remove_proc_entry("apanic_console", NULL);
		ctx->apanic_console = NULL;
	}
	if (ctx->apanic_threads) {
		remove_proc_entry("apanic_threads", NULL);
		ctx->apanic_threads = NULL;
	}
	mutex_unlock(&drv_mutex);
}

static int apanic_proc_write(struct file *file, const char __user *buffer,
				unsigned long count, void *data)
{
	schedule_work(&proc_removal_work);
	return count;
}

static int apanic_report(void)
{
	struct apanic_data *ctx = &drv_ctx;
	struct panic_header *hdr = (struct panic_header *)ctx->buffer;
	int    proc_entry_created = 0;

	if (hdr->magic != PANIC_MAGIC) {
		printk(KERN_INFO "apanic: no panic data available\n");
		apanic_ram_erase();
		return 0;
	}

	if (hdr->version != PHDR_VERSION) {
		printk(KERN_INFO "apanic: header version mismatch (%d != %d)\n",
		       hdr->version, PHDR_VERSION);
		apanic_ram_erase();
		return -1;
	}

	memcpy(&ctx->curr, hdr, sizeof(struct panic_header));

	printk(KERN_INFO "apanic: c(%u, %u) t(%u, %u)\n",
	       hdr->console_offset, hdr->console_length,
	       hdr->threads_offset, hdr->threads_length);

	if (hdr->console_length) {
		ctx->apanic_console = create_proc_entry("apanic_console",
						      S_IFREG | S_IRUGO, NULL);
		if (!ctx->apanic_console)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			ctx->apanic_console->read_proc = apanic_proc_read;
			ctx->apanic_console->write_proc = apanic_proc_write;
			ctx->apanic_console->size = hdr->console_length;
			ctx->apanic_console->data = (void *) PROC_APANIC_CONSOLE;
			proc_entry_created = 1;
		}
	}

	if (hdr->threads_length) {
		ctx->apanic_threads = create_proc_entry("apanic_threads",
						       S_IFREG | S_IRUGO, NULL);
		if (!ctx->apanic_threads)
			printk(KERN_ERR "%s: failed creating procfile\n",
			       __func__);
		else {
			ctx->apanic_threads->read_proc = apanic_proc_read;
			ctx->apanic_threads->write_proc = apanic_proc_write;
			ctx->apanic_threads->size = hdr->threads_length;
			ctx->apanic_threads->data = (void *) PROC_APANIC_THREADS;
			proc_entry_created = 1;
		}
	}

	if (!proc_entry_created) {
		apanic_ram_erase();
		return -1;
	}

	return 0;
}

static int apanic_report_ram_probe(struct platform_device *pdev)
{
	struct apanic_data *ctx = &drv_ctx;
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;

	if (res == NULL || pdev->num_resources != 1 ||
	    !(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "apanic_report_ram: invalid resource, %p %d flags "
		       "%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}
	buffer_size = res->end - res->start + 1;
	start = res->start;
	buffer_size = (buffer_size + PAGE_SIZE - 1) & PAGE_MASK;
	start &= PAGE_MASK;
	printk(KERN_INFO "apanic_report_ram: got buffer at %x, size %x\n",
	       start, buffer_size);
	ctx->buffer = ioremap_wc(res->start, buffer_size);
	if (ctx->buffer == NULL) {
		printk(KERN_ERR "apanic_report_ram: failed to map memory\n");
		return -ENOMEM;
	}
	ctx->buffer_size = buffer_size;

	apanic_ram_dump(ctx->buffer, 16);

	return apanic_report();
}

static struct platform_driver apanic_report_ram_driver = {
	.probe = apanic_report_ram_probe,
	.driver		= {
		.name	= "apanic_report_ram",
	},
};

int __init apanic_report_ram_init(void)
{
	int result = 0;

	memset(&drv_ctx, 0, sizeof(drv_ctx));
	result = platform_driver_register(&apanic_report_ram_driver);
	if (result == 0) {
		INIT_WORK(&proc_removal_work, apanic_remove_proc_work);
		printk(KERN_INFO "Android kernel panic reporter initialized\n");
	}

	return result;
}

module_init(apanic_report_ram_init);

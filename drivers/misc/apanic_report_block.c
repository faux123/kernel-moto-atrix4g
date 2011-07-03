/* drivers/misc/apanic_report_block.c
 *
 * Publish into user space a panic dump stored in a block device.  It is
 * exposed via two entries in /proc in the style that Android's "init" process
 * is expecting.  Because this driver can't use the MTD registration interface,
 * panic block detection is triggered by user space, which also provides the
 * name of the block device via a write to "/proc/apanic.  This driver should
 * probably just be replaced by a user space app that does everything.
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
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/bio.h>

#include <linux/apanic.h>


#define DRVNAME "apanic_report_block: "

#define PROC_APANIC_CONSOLE 1
#define PROC_APANIC_THREADS 2


struct apanic_data {
	char                   devpath[256];
	struct panic_header    curr;
	void		      *bounce;
	struct proc_dir_entry *apanic_trigger;
	struct proc_dir_entry *apanic_console;
	struct proc_dir_entry *apanic_threads;

};

static struct apanic_data drv_ctx;
static struct work_struct proc_removal_work;
/* avoid collision of proc interface operation and proc removal */
static DEFINE_MUTEX(drv_mutex);

#define APANIC_INVALID_OFFSET 0xFFFFFFFF

static void mmc_bio_complete(struct bio *bio, int err)
{
	complete((struct completion *)bio->bi_private);
}

static int apanic_proc_read(char *buffer, char **start, off_t offset,
                            int count, int *peof, void *dat)
{
	int i, index = 0;
	int err;
	int start_sect;
	int end_sect;
	size_t file_length;
	off_t file_offset;
	struct apanic_data *ctx = &drv_ctx;
	struct block_device *bdev;
	struct bio bio;
	struct bio_vec bio_vec;
	struct completion complete;
	struct page *page;

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

	if ((offset + count) > file_length) {
		mutex_unlock(&drv_mutex);
		return 0;
	}

	bdev = lookup_bdev(ctx->devpath);
	if (IS_ERR(bdev)) {
		printk(KERN_ERR DRVNAME "failed to look up device %s (%ld)\n",
		       ctx->devpath, PTR_ERR(bdev));
		return -1;
	}
	err = blkdev_get(bdev, FMODE_READ);
	if (err) {
		printk(KERN_ERR DRVNAME "failed to open device %s (%d)\n",
		       ctx->devpath, err);
		return err;
	}
	page = virt_to_page(ctx->bounce);

	start_sect = (file_offset +  offset) / 512;
	end_sect = (file_offset + offset + count - 1) / 512;

	for (i = start_sect; i <= end_sect; i++) {
		bio_init(&bio);
		bio.bi_io_vec = &bio_vec;
		bio_vec.bv_page = page;
		bio_vec.bv_len = 512;
		bio_vec.bv_offset = 0;
		bio.bi_vcnt = 1;
		bio.bi_idx = 0;
		bio.bi_size = 512;
		bio.bi_bdev = bdev;
		bio.bi_sector = i;
		init_completion(&complete);
		bio.bi_private = &complete;
		bio.bi_end_io = mmc_bio_complete;
		submit_bio(READ, &bio);
		wait_for_completion(&complete);
		if (!test_bit(BIO_UPTODATE, &bio.bi_flags)) {
			err = -EIO;
			goto out_blkdev;
		}

		if ((i == start_sect) && ((file_offset + offset) % 512 != 0)) {
			/* first sect, may be the only sect */
			memcpy(buffer, ctx->bounce + (file_offset + offset)
				% 512, min((unsigned long)count,
				(unsigned long)
				(512 - (file_offset + offset) % 512)));
			index += min((unsigned long)count, (unsigned long)
				(512 - (file_offset + offset) % 512));
		} else if ((i == end_sect) && ((file_offset + offset + count)
			% 512 != 0)) {
			/* last sect */
			memcpy(buffer + index, ctx->bounce, (file_offset +
				offset + count) % 512);
		} else {
			/* middle sect */
			memcpy(buffer + index, ctx->bounce, 512);
			index += 512;
		}
	}

	*start = (char *)count;

	if ((offset + count) == file_length)
		*peof = 1;

	mutex_unlock(&drv_mutex);
	err = count;

out_blkdev:
	blkdev_put(bdev, FMODE_READ);

	mutex_unlock(&drv_mutex);
	return err;
}

static void mmc_panic_erase(void)
{
	int i = 0;
	int err;
	struct apanic_data *ctx = &drv_ctx;
	struct block_device *bdev;
	struct bio bio;
	struct bio_vec bio_vec;
	struct completion complete;
	struct page *page;

	bdev = lookup_bdev(ctx->devpath);
	if (IS_ERR(bdev)) {
		printk(KERN_ERR DRVNAME "failed to look up device %s (%ld)\n",
		       ctx->devpath, PTR_ERR(bdev));
		return;
	}
	err = blkdev_get(bdev, FMODE_WRITE);
	if (err) {
		printk(KERN_ERR DRVNAME "failed to open device %s (%d)\n",
		       ctx->devpath, err);
		return;
	}
	page = virt_to_page(ctx->bounce);
	memset(ctx->bounce, 0, PAGE_SIZE);

	while (i < bdev->bd_part->nr_sects) {
		bio_init(&bio);
		bio.bi_io_vec = &bio_vec;
		bio_vec.bv_offset = 0;
		bio_vec.bv_page = page;
		bio.bi_vcnt = 1;
		bio.bi_idx = 0;
		bio.bi_sector = i;
		if (bdev->bd_part->nr_sects - i >= 8) {
			bio_vec.bv_len = PAGE_SIZE;
			bio.bi_size = PAGE_SIZE;
			i += 8;
		} else {
			bio_vec.bv_len = (bdev->bd_part->nr_sects - i) * 512;
			bio.bi_size = (bdev->bd_part->nr_sects - i) * 512;
			i = bdev->bd_part->nr_sects;
		}
		bio.bi_bdev = bdev;
		init_completion(&complete);
		bio.bi_private = &complete;
		bio.bi_end_io = mmc_bio_complete;
		submit_bio(WRITE, &bio);
		wait_for_completion(&complete);
	}
	blkdev_put(bdev, FMODE_WRITE);

	return;
}

static void apanic_remove_proc_work(struct work_struct *work)
{
	struct apanic_data *ctx = &drv_ctx;

	mutex_lock(&drv_mutex);
	mmc_panic_erase();
	memset(&ctx->curr, 0, sizeof(struct panic_header));
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

static int apanic_trigger_check(struct file *file, const char __user *devpath,
                                unsigned long count, void *data)
{
	struct apanic_data *ctx = &drv_ctx;
	struct panic_header *hdr = ctx->bounce;
	struct block_device *bdev;
	struct bio bio;
	struct bio_vec bio_vec;
	struct completion complete;
	struct page *page;
	int err = 0;

	bdev = lookup_bdev(devpath);
	if (IS_ERR(bdev)) {
		printk(KERN_ERR DRVNAME "failed to look up device %s (%ld)\n",
		       devpath, PTR_ERR(bdev));
		return -1;
	}
	err = blkdev_get(bdev, FMODE_READ);
	if (err) {
		printk(KERN_ERR DRVNAME "failed to open device %s (%d)\n",
		       devpath, err);
		return err;
	}

	strncpy(ctx->devpath, devpath, sizeof(ctx->devpath));
	page = virt_to_page(ctx->bounce);

	bio_init(&bio);
	bio.bi_io_vec = &bio_vec;
	bio_vec.bv_page = page;
	bio_vec.bv_len = PAGE_SIZE;
	bio_vec.bv_offset = 0;
	bio.bi_vcnt = 1;
	bio.bi_idx = 0;
	bio.bi_size = PAGE_SIZE;
	bio.bi_bdev = bdev;
	bio.bi_sector = 0;
	init_completion(&complete);
	bio.bi_private = &complete;
	bio.bi_end_io = mmc_bio_complete;
	submit_bio(READ, &bio);
	wait_for_completion(&complete);

	blkdev_put(bdev, FMODE_READ);
	printk(KERN_ERR DRVNAME "using block device '%s'\n", devpath);

	if (hdr->magic != PANIC_MAGIC) {
		printk(KERN_INFO DRVNAME "no panic data available\n");
		return -1;
	}

	if (hdr->version != PHDR_VERSION) {
		printk(KERN_INFO DRVNAME "version mismatch (%d != %d)\n",
		       hdr->version, PHDR_VERSION);
		return -1;
	}

	memcpy(&ctx->curr, hdr, sizeof(struct panic_header));

	printk(KERN_INFO DRVNAME "c(%u, %u) t(%u, %u)\n",
	       hdr->console_offset, hdr->console_length,
	       hdr->threads_offset, hdr->threads_length);

	if (hdr->console_length) {
		ctx->apanic_console = create_proc_entry("apanic_console",
						      S_IFREG | S_IRUGO, NULL);
		if (!ctx->apanic_console)
			printk(KERN_ERR DRVNAME "failed creating procfile\n");
		else {
			ctx->apanic_console->read_proc = apanic_proc_read;
			ctx->apanic_console->write_proc = apanic_proc_write;
			ctx->apanic_console->size = hdr->console_length;
			ctx->apanic_console->data = (void *)PROC_APANIC_CONSOLE;
		}
	}

	if (hdr->threads_length) {
		ctx->apanic_threads = create_proc_entry("apanic_threads",
						       S_IFREG | S_IRUGO, NULL);
		if (!ctx->apanic_threads)
			printk(KERN_ERR DRVNAME "failed creating procfile\n");
		else {
			ctx->apanic_threads->read_proc = apanic_proc_read;
			ctx->apanic_threads->write_proc = apanic_proc_write;
			ctx->apanic_threads->size = hdr->threads_length;
			ctx->apanic_threads->data = (void *)PROC_APANIC_THREADS;
		}
	}

	return err;
}

int __init apanic_report_block_init(void)
{
	int result = 0;

	memset(&drv_ctx, 0, sizeof(drv_ctx));

	drv_ctx.apanic_trigger = create_proc_entry("apanic",
						   S_IFREG | S_IRUGO, NULL);
	if (!drv_ctx.apanic_trigger)
		printk(KERN_ERR "%s: failed creating procfile\n",
			   __func__);
	else {
		drv_ctx.apanic_trigger->read_proc = NULL;
		drv_ctx.apanic_trigger->write_proc = apanic_trigger_check;
		drv_ctx.apanic_trigger->size = 1;
		drv_ctx.apanic_trigger->data = NULL;

		drv_ctx.bounce = (void *)__get_free_page(GFP_KERNEL);
		INIT_WORK(&proc_removal_work, apanic_remove_proc_work);
		printk(KERN_INFO DRVNAME "kernel panic reporter initialized\n");
	}

	return result;
}

module_init(apanic_report_block_init);

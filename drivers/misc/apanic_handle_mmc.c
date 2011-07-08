/* drivers/misc/apanic_handle_mmc.c
 *
 * Handle a kernel panic notification and generate a panic dump in the Android
 * "apanic" style.  Store the result into a partition on an MMC-type device to
 * be read after reset by "apanic_report_block".
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
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/nmi.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/rtc.h>
#include <linux/console.h>
#include <linux/preempt.h>
#include <linux/mmc/mmc_simple.h>
#include <linux/apanic.h>

#include <mach/apanic.h>


#define DRVNAME "apanic_handle_mmc: "

#define THREADS_PER_PASS 20	/* threads to dump to log_buf at a time */


struct apanic_data {
	struct apanic_mmc_platform_data	*dev;
	void				*bounce;
	struct proc_dir_entry		*proc_annotate;
	char				*annotation;
};

static struct apanic_data drv_ctx;
static int in_panic;
static int did_panic;

extern int log_buf_copy(char *dest, int idx, int len);
extern void log_buf_clear(void);

/*
 * Writes the contents of the console to the specified offset in mmc.
 * Returns number of bytes written
 */
static int apanic_write_console_mmc(unsigned int offset)
{
	struct apanic_data *ctx = &drv_ctx;
	int saved_oip;
	int total_len = 0;
	int log_len, write_len;
	unsigned int last_chunk = 0;

	while (!last_chunk) {
		saved_oip = oops_in_progress;
		oops_in_progress = 1;
		log_len = log_buf_copy(ctx->bounce, total_len, PAGE_SIZE);
		if (log_len < 0)
			break;

		if (log_len != PAGE_SIZE)
			last_chunk = log_len;

		oops_in_progress = saved_oip;
		if (log_len <= 0)
			break;
		if (log_len != PAGE_SIZE)
			memset(ctx->bounce + log_len, ' ', PAGE_SIZE - log_len);

		write_len = mmc_simple_write(ctx->bounce, log_len, offset);
		if (write_len <= 0) {
			printk(KERN_EMERG DRVNAME
			       "write failed (%d)\n", write_len);
			return total_len;
		}

		if (!last_chunk)
			total_len += write_len;
		else
			total_len += last_chunk;
		offset += write_len;
	}
	return total_len;
}

static int apanic_mmc(struct notifier_block *this, unsigned long event,
			void *ptr)
{
	struct apanic_data *ctx = &drv_ctx;
	struct panic_header *hdr;
	int console_offset = 0;
	int console_len = 0;
	int threads = 0;
	int threads_offset = 0;
	int threads_len = 0;
	int rc;
	struct timespec now;
	struct timespec uptime;
	struct rtc_time rtc_timestamp;
	struct console *con;
	struct task_struct *g, *p;

	if (in_panic)
		return NOTIFY_DONE;
	in_panic = 1;

#ifdef CONFIG_PREEMPT
	/* Ensure that cond_resched() won't try to preempt anybody */
	add_preempt_count(PREEMPT_ACTIVE);
#endif
	touch_softlockup_watchdog();

	if (!ctx->dev)
		goto out;

	if (mmc_simple_init(ctx->dev->id,
	                    ctx->dev->start_sector,
			    ctx->dev->sectors,
			    ctx->dev->sector_size)) {
		printk(KERN_ERR DRVNAME "unable to initialize MMC device\n");
		goto out;
	}

	memset(ctx->bounce, 0, PAGE_SIZE);
	hdr = (struct panic_header *)ctx->bounce;

	/*
	 * Read the first block and check for the panic header
	 */
	rc = mmc_simple_read(ctx->bounce, ctx->dev->sector_size, 0);
	if (did_panic || (rc == ctx->dev->sector_size &&
	                  hdr->magic == PANIC_MAGIC)) {
		printk(KERN_EMERG DRVNAME "panic partition in use\n");
		goto out;
	}

	/*
	 * Add timestamp to displays current UTC time and uptime (in seconds).
	 */
	now = current_kernel_time();
	rtc_time_to_tm((unsigned long)now.tv_sec, &rtc_timestamp);
	do_posix_clock_monotonic_gettime(&uptime);
	printk(KERN_EMERG "Timestamp = %lu.%03lu\n",
			(unsigned long)now.tv_sec,
			(unsigned long)(now.tv_nsec / 1000000));
	printk(KERN_EMERG "Current Time = "
			"%02d-%02d %02d:%02d:%lu.%03lu, "
			"Uptime = %lu.%03lu seconds\n",
			rtc_timestamp.tm_mon + 1, rtc_timestamp.tm_mday,
			rtc_timestamp.tm_hour, rtc_timestamp.tm_min,
			(unsigned long)rtc_timestamp.tm_sec,
			(unsigned long)(now.tv_nsec / 1000000),
			(unsigned long)uptime.tv_sec,
			(unsigned long)(uptime.tv_nsec/USEC_PER_SEC));
	if (ctx->annotation)
		printk(KERN_EMERG "%s\n", ctx->annotation);


	/*
	 * Write out the console
	 */
	console_offset = ctx->dev->sector_size;  /* reserve for the header */
	console_len = apanic_write_console_mmc(console_offset);
	if (console_len < 0) {
		printk(KERN_EMERG DRVNAME "failed while writing console "
		       "to panic log (%d)\n", console_len);
		console_len = 0;
	}

	log_buf_clear();
	for (con = console_drivers; con; con = con->next)
		con->flags &= ~CON_ENABLED;

	/*
	 * Write out all threads
	 */
	threads_offset = ALIGN(console_offset + console_len,
					ctx->dev->sector_size);
	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		touch_nmi_watchdog();
		sched_show_task(p);
		threads++;
		if (threads % THREADS_PER_PASS == 0) {
			rc = apanic_write_console_mmc(threads_offset +
								threads_len);
			if (rc < 0) {
				printk(KERN_EMERG DRVNAME "failed while "
					"writing threads to panic log (%d)\n",
					threads_len);
				read_unlock(&tasklist_lock);
				goto header;	/* cannot use break */
			}
			/*
			 * Force alignment to work around mmc_simple
			 * limitations.  Padding will be white space.
			 */
			threads_len = ALIGN(threads_len + rc,
						ctx->dev->sector_size);
			log_buf_clear();
		}
	} while_each_thread(g, p);
	read_unlock(&tasklist_lock);

	/* Trick to call sysrq_sched_debug_show() */
	show_state_filter(0x80000000);

	rc = apanic_write_console_mmc(threads_offset + threads_len);
	if (rc < 0) {
		printk(KERN_EMERG DRVNAME "failed while writing threads "
		       "to panic log (%d)\n", threads_len);
		rc = 0;
	}
	threads_len += rc;

	/*
	 * Finally write the panic header
	 */
header:
	log_buf_clear();
	memset(ctx->bounce, 0, PAGE_SIZE);

	hdr->magic = PANIC_MAGIC;
	hdr->version = PHDR_VERSION;

	hdr->console_offset = console_offset;
	hdr->console_length = console_len;

	hdr->threads_offset = threads_offset;
	hdr->threads_length = threads_len;

	rc = mmc_simple_write(ctx->bounce, ctx->dev->sector_size, 0);
	if (rc <= 0) {
		printk(KERN_EMERG DRVNAME "failed to write header "
                       "to panic log (%d)\n", rc);
		goto out;
	}

	/* Re-enable the consoles in case something goes badly at reboot. */
	for (con = console_drivers; con; con = con->next)
		con->flags |= CON_ENABLED;

	printk(KERN_EMERG DRVNAME "wrote %d bytes to MMC\n",
	       ctx->dev->sector_size + console_len + threads_len);

out:
#ifdef CONFIG_PREEMPT
	sub_preempt_count(PREEMPT_ACTIVE);
#endif
	did_panic = 1;
	in_panic = 0;

	return NOTIFY_DONE;
}

int apanic_annotate(const char *annotation)
{
	struct apanic_data *ctx = &drv_ctx;
	char *buffer;
	size_t oldlen = 0;
	size_t newlen;

	newlen = strlen(annotation);
	if (newlen == 0)
		return -EINVAL;

	if (ctx->annotation)
		oldlen = strlen(ctx->annotation);

	buffer = kmalloc(newlen + oldlen + 1, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if (ctx->annotation) {
		strcpy(buffer, ctx->annotation);
		kfree(ctx->annotation);
	}
	else
		buffer[0] = '\0';

	strcat(buffer, annotation);

	ctx->annotation = buffer;

	return 0;
}
EXPORT_SYMBOL(apanic_annotate);

static int apanic_proc_annotate(struct file *file,
				const char __user *annotation,
				unsigned long count, void *data)
{
	return apanic_annotate(annotation);
}

static struct notifier_block panic_blk = {
	.notifier_call	= apanic_mmc,
};

static int panic_dbg_get(void *data, u64 *val)
{
	printk(KERN_EMERG "*** kernel console dumped via debugfs ***\n");
	apanic_mmc(NULL, 0, NULL);
	return 0;
}

static int panic_dbg_set(void *data, u64 val)
{
	printk(KERN_EMERG "*** PANIC FORCED via debugfs ***\n");
	BUG();
	return -1;
}

DEFINE_SIMPLE_ATTRIBUTE(panic_dbg_fops, panic_dbg_get, panic_dbg_set, "%llu\n");

static int apanic_handle_mmc_probe(struct platform_device *pdev)
{
	struct apanic_data *ctx = &drv_ctx;
	struct apanic_mmc_platform_data *pdata =
		(struct apanic_mmc_platform_data*)pdev->dev.platform_data;

	if (pdata == NULL || pdata->sectors == 0 || pdata->sector_size == 0) {
		printk(KERN_ERR DRVNAME "invalid config: %d:%llu:%llu:%u\n",
		       pdata->id, pdata->start_sector,
		       pdata->sectors, pdata->sector_size);
		return -ENXIO;
	}

	ctx->dev = pdata;
	printk(KERN_INFO DRVNAME "on host %d at 0x%llX of size %llu with %u "
	       "byte sectors\n", pdata->id, pdata->start_sector,
	                         pdata->sectors, pdata->sector_size);

	/* FIXME: this should be sysfs */
	drv_ctx.proc_annotate = create_proc_entry("apanic_annotate",
	                                          S_IFREG | S_IRUGO, NULL);
	if (!drv_ctx.proc_annotate)
		printk(KERN_ERR "%s: failed creating procfile\n",
			   __func__);
	else {
		drv_ctx.proc_annotate->read_proc = NULL;
		drv_ctx.proc_annotate->write_proc = apanic_proc_annotate;
		drv_ctx.proc_annotate->size = 1;
		drv_ctx.proc_annotate->data = NULL;
	}

	return 0;
}

static struct platform_driver apanic_handle_mmc_driver = {
	.probe = apanic_handle_mmc_probe,
	.driver = {
		.name	= "apanic_handle_mmc",
	},
};

int __init apanic_handle_mmc_init(void)
{
	int result = 0;

	memset(&drv_ctx, 0, sizeof(drv_ctx));
	result = platform_driver_register(&apanic_handle_mmc_driver);
	if (result == 0) {
		atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
		debugfs_create_file("apanic", 0644, NULL, NULL, &panic_dbg_fops);
		drv_ctx.bounce = (void *) __get_free_page(GFP_KERNEL);
		printk(KERN_INFO DRVNAME "kernel panic handler initialized\n");
	}

	return result;
}

module_init(apanic_handle_mmc_init);

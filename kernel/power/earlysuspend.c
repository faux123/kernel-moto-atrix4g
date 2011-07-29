/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/signal.h>
#include <linux/delay.h>
#include "power.h"

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_WD = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static int early_suspend_timeout_value = 10;
module_param_named(early_suspend_timeout_value, early_suspend_timeout_value,
			int, S_IRUGO | S_IWUSR | S_IWGRP);
static int late_resume_timeout_value = 10;
module_param_named(late_resume_timeout_value, late_resume_timeout_value,
			int, S_IRUGO | S_IWUSR | S_IWGRP);
static int early_suspend_queue_timeout = 10;
module_param_named(early_suspend_queue_timeout, early_suspend_queue_timeout,
			int, S_IRUGO | S_IWUSR | S_IWGRP);
static int late_resume_queue_timeout = 10;
module_param_named(late_resume_queue_timeout, late_resume_queue_timeout,
			int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static void early_suspend_wd_enable(int suspend_type, void (*data), int timeout);
static void early_suspend_wd_disable(int suspend_type);
static void early_suspend_timeout(unsigned long data);
static DEFINE_TIMER(early_suspend_wd, early_suspend_timeout, 0, 0);
static void late_resume_timeout(unsigned long data);
static DEFINE_TIMER(late_resume_wd, late_resume_timeout, 0, 0);
static void tombstone_timeout(unsigned long data);
static DEFINE_TIMER(tombstone_timer, tombstone_timeout, 0, 0);
#define EARLY_SUSPEND 0
#define LATE_RESUME   1
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

extern pid_t s_nvrm_daemon_pid;
static pid_t suspend_pid;

static void abort_suspicious_process(int bad_pid)
{
	struct pid *bad_p = find_vpid(bad_pid);

	if (bad_p)
	{
		printk(KERN_ERR "Killing process(%d) with signal %d\n",
			bad_pid, SIGABRT);
		kill_pid(bad_p, SIGABRT, 1);
	}
	else
	{
		printk(KERN_ERR "%s: Invalid pid %d.\n", __func__, bad_pid);
	}
}

static void dump_process_state(int bad_pid)
{
	struct pid *bad_p;
	struct task_struct *p;

	bad_p = find_vpid(bad_pid);
	printk(KERN_ERR "###################################################\n");
	printk(KERN_ERR "%s: Running on PID %d (%p)\n", __func__, bad_pid, bad_p);
	printk(KERN_ERR "###################################################\n");
	if(!bad_p) return;

	do_each_pid_thread(bad_p, PIDTYPE_PID, p) {
		sched_show_task(p);
	} while_each_pid_thread(bad_p, PIDTYPE_PID, p);
}


void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	suspend_pid = task_pid_nr(current);

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	early_suspend_wd_disable(EARLY_SUSPEND);
	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL) {
			early_suspend_wd_enable(EARLY_SUSPEND, pos->suspend,
				early_suspend_timeout_value);
			pos->suspend(pos);
			early_suspend_wd_disable(EARLY_SUSPEND);
			}
	}
	mutex_unlock(&early_suspend_lock);

abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	early_suspend_wd_disable(LATE_RESUME);
	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link)
		if (pos->resume != NULL) {
			early_suspend_wd_enable(LATE_RESUME, pos->resume,
				late_resume_timeout_value);
			pos->resume(pos);
			early_suspend_wd_disable(LATE_RESUME);
			}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
		early_suspend_wd_enable(EARLY_SUSPEND, early_suspend,
			early_suspend_queue_timeout);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
		early_suspend_wd_enable(LATE_RESUME, late_resume,
			late_resume_queue_timeout);
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}

static void early_suspend_timeout(unsigned long data)
{
	if (data == (unsigned long)early_suspend)
		printk(KERN_EMERG "**** Early Suspend Timeout while "
			"waiting for early suspend work to start; "
			"state: %d, requested state: %d.\n", state,
			requested_suspend_state);
	else
		printk(KERN_EMERG "**** Early Suspend Timeout; function:"
			" %pF, state: %d, requested state: %d.\n",
			(void *)data, state, requested_suspend_state);

	dump_process_state(s_nvrm_daemon_pid);
	dump_process_state(suspend_pid);
	tombstone_timeout(0);
	/* Expect a BUG() shortly. */
}

static void late_resume_timeout(unsigned long data)
{
	if (data == (unsigned long)late_resume)
		printk(KERN_EMERG "**** Late Resume Timeout while "
			"waiting for late resume work to start; "
			"state: %d, requested state: %d.\n", state,
			requested_suspend_state);
	else
		printk(KERN_EMERG "**** Late Resume Timeout; function:"
			" %pF, state: %d, requested state: %d.\n",
			(void *)data, state, requested_suspend_state);

	dump_process_state(s_nvrm_daemon_pid);
	dump_process_state(suspend_pid);
	tombstone_timeout(0);
	/* Expect a BUG() shortly. */
}

static void tombstone_timeout(unsigned long data)
{
	/* need to kill -6 twice, with a pause inbetween,
	 * to get a tombstone.  Also need to sync filesystem.
	 */
	tombstone_timer.data = data + 1;

	switch(data) {
	case 0:
		abort_suspicious_process(s_nvrm_daemon_pid);
		mod_timer(&tombstone_timer, jiffies + HZ/10);
		break;
	case 1:
		abort_suspicious_process(s_nvrm_daemon_pid);
		mod_timer(&tombstone_timer, jiffies + HZ*3);
		break;
	case 2:
		emergency_sync();
		mod_timer(&tombstone_timer, jiffies + HZ/2);
		break;
	default:
		BUG();
		break;
	}
}

static void early_suspend_wd_enable(int suspend_type, void (*data), int timeout)
{
	if (timeout <= 0)
		return;
	if (suspend_type == EARLY_SUSPEND) {
		if (debug_mask & DEBUG_WD)
			pr_info("Early Suspend watchdog started; function:"
				" %pF, timeout: %d.\n",
				(void *)data, timeout);
		early_suspend_wd.data = (unsigned long) data;
		mod_timer(&early_suspend_wd, jiffies + (HZ * timeout));
	}
	else {
		if (debug_mask & DEBUG_WD)
			pr_info("Late Resume watchdog started; function:"
				" %pF, timeout: %d.\n",
				(void *)data, timeout);
		late_resume_wd.data = (unsigned long) data;
		mod_timer(&late_resume_wd, jiffies + (HZ * timeout));
	}
}

static void early_suspend_wd_disable(int suspend_type)
{
	if (suspend_type == EARLY_SUSPEND) {
		del_timer_sync(&early_suspend_wd);
		if (debug_mask & DEBUG_WD)
			pr_info("Early Suspend watchdog stopped.\n");
	}
	else {
                del_timer_sync(&late_resume_wd);
                if (debug_mask & DEBUG_WD)
                        pr_info("Late Resume watchdog stopped.\n");
	}
}

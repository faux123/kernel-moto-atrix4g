/*
 * drivers/watchdog/fiq_wdt.c
 *
 * Using AP20 timer0 FIQ to implement Watchdog
 *
 * Copyright (c) 2010, Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/smp.h>
#include <linux/platform_device.h>

#include <asm/cputype.h>
#include <asm/uaccess.h>

#include <mach/iomap.h>

#define TIMER_LOCKUP_WATCHDOG_TIMEOUT	24

#define TIMER1_BASE 0x0
#define TIMER2_BASE 0x8
#define TIMER3_BASE 0x50
#define TIMER4_BASE 0x58

#define TIMER_PTV 0x0
#define TIMER_PCR 0x4

static void __iomem *timer_reg_base = IO_ADDRESS(TEGRA_TMR1_BASE);

#define timer_writel(value, reg) \
	__raw_writel(value, (u32)timer_reg_base + (reg))
#define timer_readl(reg) \
	__raw_readl((u32)timer_reg_base + (reg))

extern void timer_stop(int timer_n);

static int lockup_monitor_exit = 0;
static void disable_timer1_watchdog(void)
{
	u32 reg;

	timer_writel(1 << 30, TIMER1_BASE + TIMER_PCR);
	reg = 0;
	timer_writel(reg, TIMER1_BASE + TIMER_PTV);
}

static void enable_timer1_watchdog(void)
{
	u32 reg;

	reg = 0x80000000 | (TIMER_LOCKUP_WATCHDOG_TIMEOUT * 1000000);
	timer_writel(reg, TIMER1_BASE + TIMER_PTV);
}

int thread_kick_timer1_watchdog(void * arg)
{
	daemonize("lockupmonitor");
	while(!lockup_monitor_exit)
	{
		u32 reg;
		reg = 0x80000000 | (TIMER_LOCKUP_WATCHDOG_TIMEOUT * 1000000);
		timer_writel(reg, TIMER1_BASE + TIMER_PTV);
		msleep_interruptible(TIMER_LOCKUP_WATCHDOG_TIMEOUT * 1000 / 2);
	}
	printk("lockup monitor thread exited!\n");
	return 0;
}

static fiqreturn_t wdt_fiq_handler(struct pt_regs *regs)
{
	if(is_irq_valid_status(INT_TMR1)) {
		timer_stop(1);
		tegra_clear_irq(INT_TMR1);
		if (smp_processor_id())
			return IRQ_HANDLED;

		printk(KERN_ERR "\nBUG: fiq lockup monitor - CPU#%d\n", smp_processor_id());
		show_regs(regs);
		panic("wdt fiq!\n");
	}
	return IRQ_HANDLED;
}

static irqreturn_t tegra_lockuptimer_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static struct irqaction tegra_lockuptimer_irq = {
	.name           = "timerlockup",
	.flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
	.handler        = tegra_lockuptimer_interrupt,
	.irq            = INT_TMR1,
};

static int __devinit fiq_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;

	timer_writel(1 << 30, TIMER1_BASE + TIMER_PCR);

	ret = setup_irq(tegra_lockuptimer_irq.irq, &tegra_lockuptimer_irq);
	if (ret) {
		printk(KERN_ERR "Failed to request lockuptimer: %d\n", ret);
		return -1;
	}

	/* Kicking thread will enable watchdog timer */
	ret = kernel_thread(thread_kick_timer1_watchdog, NULL, CLONE_VM);
	if (ret < 0) {
		printk(KERN_ERR "Failed to launch lockup watchdog thread: %d\n", ret);
		return -1;
	}

	set_fiq_handler(wdt_fiq_handler);
	tegra_start_irq2fiq(INT_TMR1);

	return 0;
}

static int __devexit fiq_wdt_remove(struct platform_device *pdev)
{
	lockup_monitor_exit = 1;
	disable_timer1_watchdog();
	return 0;
}

static int fiq_wdt_suspend(struct platform_device *pdev, pm_message_t message)
{
	disable_timer1_watchdog();
	return 0;
}

static int fiq_wdt_resume(struct platform_device *pdev)
{
	enable_timer1_watchdog();
	return 0;
}


static struct platform_driver fiq_wdt_driver = {
	.driver = {
		.name = "fiq_wdt",
	},
        .probe = fiq_wdt_probe,
	.remove = __devexit_p(fiq_wdt_remove),
	.suspend = fiq_wdt_suspend,
	.resume = fiq_wdt_resume,
};

static struct platform_device fiq_wdt_platform_device = {
	.name          = "fiq_wdt",
	.id            = 0,
};

static int __init fiq_wdt_init(void)
{
	platform_driver_register(&fiq_wdt_driver);
	platform_device_register(&fiq_wdt_platform_device);
	return 0;
}

late_initcall(fiq_wdt_init);

static void __exit fiq_wdt_exit(void)
{
	platform_device_unregister(&fiq_wdt_platform_device);
	platform_driver_unregister(&fiq_wdt_driver);
}

/*
 * Following functions are only for proc debug use
 */
static int lockup_monitor_proc_show(struct seq_file *m, void *v)
{
        seq_printf(m, "Timer 1 PTV:%x\n", timer_readl(TIMER1_BASE + TIMER_PTV));
        seq_printf(m, "Timer 1 PCR:%x\n", timer_readl(TIMER1_BASE + TIMER_PCR));
        seq_printf(m, "echo 1 > /proc/lockup_monitor - disable local int forever\n");
        seq_printf(m, "echo 2 > /proc/lockup_monitor - stuck in int handler\n");
        seq_printf(m, "echo 3 > /proc/lockup_monitor - stop kicking watchdog\n");
        return 0;
}

static irqreturn_t tegra_timerdead_interrupt(int irq, void *dev_id)
{
        printk("dead timer interrupt!%d in cpu %d.\n", irq, smp_processor_id());
        while(1);
        return IRQ_HANDLED;
}

static struct irqaction tegra_timerdebug_irq = {
        .name           = "timerdead",
        .flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_TRIGGER_HIGH,
        .handler        = tegra_timerdead_interrupt,
        .irq            = INT_TMR2,
};

static ssize_t lockup_monitor_write(struct file *file, const char __user *buf,
                            size_t count, loff_t *offs)
{
        int cmd;
        if(get_user(cmd, buf))
                return -EINVAL;

        if(cmd == '1')
        {
                printk("!!! Debug use. Local interrupt disabled. %d.\n", smp_processor_id());

		/* Wait for kernel to migrate current task to cpu 0 */
		while(smp_processor_id())
			msleep(500);

                local_irq_disable();
                while(1);
        }
        else if(cmd == '2')
        {
                printk("!!! Debug use. Timer 2 interrupt forever!. %d.\n", smp_processor_id());
                timer_writel(1 << 30, TIMER2_BASE + TIMER_PCR);
                setup_irq(tegra_timerdebug_irq.irq, &tegra_timerdebug_irq);
                timer_writel(0x80000000 | (1 * 1000000), TIMER2_BASE + TIMER_PTV);
        }
        else if(cmd == '3')
        {
                printk("!!! Debug use. Kicking thread exits soon! %d.\n", smp_processor_id());
                lockup_monitor_exit = 1;
        }
        else if(cmd == '4')
        {
                printk("!!! Debug use. Preempt disabled! %d.\n", smp_processor_id());
                preempt_disable();
		while(1);
        }
        else
        {
                printk("Unknown cmd: %d.\n", cmd);
        }
        return count;
}

static int lockup_monitor_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, lockup_monitor_proc_show, NULL);
}

static const struct file_operations lockup_monitor_proc_fops = {
        .open           = lockup_monitor_proc_open,
        .read           = seq_read,
        .write          = lockup_monitor_write,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static int __init proc_lockup_monitor_init(void)
{
        proc_create("lockup_monitor", 0, NULL, &lockup_monitor_proc_fops);
	printk("----------------lockup_monitor created....\n");
        return 0;
}
late_initcall(proc_lockup_monitor_init);


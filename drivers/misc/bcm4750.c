/*
* drivers/misc/bcm4750.c
*
* Supports for BCM AGPS chip - bcm4750
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 as
*  published by the Free Software Foundation.
*
* Copyright (C)  2010 MOTOROLA, Inc.
* Revision History:
* Author      Date            Comment
* Motorola    2010-10-11      <Power Management for Broadcom 4750 Chip>
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <../arch/arm/mach-tegra/gpio-names.h>
#include "bcm4750.h"
#include <mach/gpio.h>

static struct bcm4750 *bcm4750_data;
static int chip_status = CHIP_MOD_POWERON;

#define AGPS_RESET_N  TEGRA_GPIO_PM6
#define AGPS_POWER_EN TEGRA_GPIO_PL5

/*  power on/off */
static void set_standby(int sb)
{
	if (OP_POWER_EN == sb) {
		gpio_set_value(AGPS_POWER_EN, 1);
		chip_status = CHIP_MOD_STANDBY;
		printk(KERN_INFO "<#GPS#>: GPS Chip BCM4750 Power Enabled!"\
				"And gpio_get_value(AGPS_POWER_EN)=%d\n", \
				gpio_get_value(AGPS_POWER_EN));
	} else if (OP_STANDBY == sb) {
		gpio_set_value(AGPS_POWER_EN, 0);
		chip_status = CHIP_MOD_POWERON;
		printk(KERN_INFO "<#GPS#>: GPS Chip BCM4750 Enters standby!"\
				"And gpio_get_value(AGPS_POWER_EN)=%d\n", \
				gpio_get_value(AGPS_POWER_EN));
	} else {
	}
}

/* chip reset */
static void set_reset(void)
{
	gpio_set_value(AGPS_RESET_N, 1);
	mdelay(1);
	gpio_set_value(AGPS_RESET_N, 0);
	mdelay(1);
	gpio_set_value(AGPS_RESET_N, 1);
	printk(KERN_INFO "<#GPS#>: GPS Chip BCM4750 Chip Reset,"\
			"And gpio_get_value(AGPS_RESET_N)=%d\n",\
			gpio_get_value(AGPS_RESET_N));
}

static ssize_t show_runmode(struct kobject *kobj, struct bin_attribute *attr,
	char *buf, loff_t off, size_t count)
{
	if (chip_status == CHIP_MOD_POWERON)
		printk(KERN_DEBUG "<#GPS#>: BCM4750 Chip Mode is Power on\n");
	else
		printk(KERN_DEBUG "<#GPS#>: BCM4750 Chip mode is power off\n");
	count = sprintf(buf, "%d\n", chip_status);
	return count;
}

static ssize_t change_runmode(struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf,
		loff_t off,
		size_t count)
{
	int op;
	int ret;
	char *tail;
	int i;
	int value;
	tail = buf;
	op = 0;
	for (i = 0; i < 4; i++)	{
		value = *tail - '0';
		*tail++;
		op = op * 10 + value;
	}
	printk(KERN_DEBUG "===GPS===:buf=%s, op=%d\n", buf, op);
	/*
	ret = strict_strtol(buf, 10, &op);
	if (ret != 0) {
		printk(KERN_INFO "===GPS===:Critical error when str transformation\n");
	}
	*/
	switch (op)	{
	case OP_RESET:
		set_reset();
		break;
	case OP_POWER_EN:
	case OP_STANDBY:
		set_standby(op);
		break;
	default:
		break;
	}
	printk(KERN_DEBUG "===DEBUG===: count=%d\n", count);
	/*return count;*/
}

static struct bin_attribute bcm4750_mode_attr = {
	.attr = {
		.name = "bcm4750",
		.owner = THIS_MODULE,
		.mode = S_IRUGO | S_IWGRP | S_IWUSR,},
	.size = BCM4750_MODE_SIZE,
	.read = show_runmode,
	.write = change_runmode,
};

static int __init bcm_4750_probe(struct platform_device *pdev)
{
	int ret = 0;
	bcm4750_data = kzalloc(sizeof(struct bcm4750), GFP_KERNEL);
	if (!bcm4750_data) {
		printk(KERN_ERR "<#GPS#>: Failed to Alloc BCM4750 Data Memory!\n");
		return -ENOMEM;
	}
	bcm4750_data->pdata = pdev->dev.platform_data;
	ret = sysfs_create_bin_file(&(module_kset->kobj), &bcm4750_mode_attr);
	if (ret) {
		printk(KERN_ERR "<#GPS#>: Failed to Create Sys File\n");
		return -ENOMEM;
	}

	return 0;
}

static int __devexit bcm_4750_remove(struct platform_device *pdev)
{
	sysfs_remove_bin_file(&(module_kset->kobj), &bcm4750_mode_attr);
	return 0;
}

static struct platform_driver bcm_4750_driver = {
	.driver = {
		.name   = "bcm4750",
	},
	.probe = bcm_4750_probe,
	.remove = bcm_4750_remove,
};

static int __init bcm4750_init(void)
{
	return platform_driver_register(&bcm_4750_driver);
}

static void __exit bcm4750_exit(void)
{
	platform_driver_unregister(&bcm_4750_driver);
}

module_init(bcm4750_init);
module_exit(bcm4750_exit);
MODULE_AUTHOR("Motorola China Technology Ltd.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM4750 GPS Chip Power Management Driver");


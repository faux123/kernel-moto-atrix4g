/*
 * drivers/hwmon/tegra-tmon.c
 *
 * hwmon class driver for the NVIDIA Tegra SoC internal tmon temp sensor
 *
 * Copyright (c) 2010, Motorola Corporation.
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

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>

#include <nvodm_tmon.h>

static ssize_t show_ambient_temp(struct device *dev,
                                struct device_attribute *attr, char *buf);
static ssize_t show_core_temp(struct device *dev,
                                struct device_attribute *attr, char *buf);

struct tegra_tmon_driver_data {
	struct device           *hwmon_dev;
	NvOdmTmonDeviceHandle	core_dev;
	NvOdmTmonDeviceHandle	ambient_dev;
};

static ssize_t show_core_temp(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev=NULL;
        struct tegra_tmon_driver_data *tmon=NULL; 
	s32 degrees_c;

	if (dev) {
		pdev = to_platform_device(dev);
		if (pdev)
			tmon = platform_get_drvdata(pdev);
	}
	
	if (tmon)
	{
		if (NvOdmTmonTemperatureGet( tmon->core_dev, &degrees_c))
			return sprintf(buf, "%d\n", degrees_c * 100 );
	}

	return 0;
}
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_core_temp, NULL, 0);

static ssize_t show_ambient_temp(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev=NULL;
        struct tegra_tmon_driver_data *tmon=NULL; 
	s32 degrees_c;

	if (dev) {
		pdev = to_platform_device(dev);
		if (pdev) 
			tmon = platform_get_drvdata(pdev);
	}
	
	if (tmon)
	{
		if (NvOdmTmonTemperatureGet( tmon->ambient_dev, &degrees_c))
			return sprintf(buf, "%d\n", degrees_c * 100 );
	}

	return 0;
}
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_ambient_temp, NULL, 0);

static struct attribute *tmon_attributes[] = {
        &sensor_dev_attr_temp1_input.dev_attr.attr,
        &sensor_dev_attr_temp2_input.dev_attr.attr,
        NULL
};

static const struct attribute_group tmon_group = {
        .attrs = tmon_attributes,
};

static void tegra_tmon_cleanup(struct tegra_tmon_driver_data *tmon)
{
	if (!tmon)
		return;

	if (tmon->ambient_dev) {
		NvOdmTmonDeviceClose(tmon->ambient_dev);
		tmon->ambient_dev = NULL;
	}

	if (tmon->core_dev) {
		NvOdmTmonDeviceClose(tmon->core_dev);
		tmon->core_dev = NULL;
	}

	if (tmon->hwmon_dev) {
		hwmon_device_unregister(tmon->hwmon_dev);
		tmon->hwmon_dev = NULL;
	}

	kfree(tmon);
 	tmon=NULL;
}

static int __init tegra_tmon_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
	struct tegra_tmon_driver_data *tmon = NULL;
	int err;

	tmon = kzalloc(sizeof(struct tegra_tmon_driver_data), GFP_KERNEL);
	if (tmon == NULL) {
		err = -ENOMEM;
		pr_err("tegra_tmon_probe: Failed to allocate driver device\n");
		return err;
	}

	tmon->ambient_dev = NvOdmTmonDeviceOpen(NvOdmTmonZoneID_Ambient);
	if (tmon->ambient_dev == NULL) {
		err = -ENODEV;
		pr_err("tegra_tmon_probe: No ambient tmon device found\n");
		goto fail;
	}

	tmon->core_dev = NvOdmTmonDeviceOpen(NvOdmTmonZoneID_Core);
	if (tmon->core_dev == NULL) {
		err = -ENODEV;
		pr_err("tegra_tmon_probe: No core tmon device found\n");
		goto fail;
	}

	platform_set_drvdata(pdev, tmon);

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->kobj, &tmon_group);
	if (err)
		goto fail;

	tmon->hwmon_dev = hwmon_device_register(dev);

	if (IS_ERR(tmon->hwmon_dev)) {
		err = PTR_ERR(tmon->hwmon_dev);
		pr_err("tegra_tmon_probe: Unable to register tmon device\n");
		goto fail;
	}

	return 0;

fail:
	tegra_tmon_cleanup(tmon);	  
	return err;
}

static int tegra_tmon_remove(struct platform_device *pdev)
{
        struct tegra_tmon_driver_data *tmon = platform_get_drvdata(pdev);
	tegra_tmon_cleanup(tmon);
	return 0;
}

static int tegra_tmon_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_tmon_driver_data *tmon=NULL;
	int err = -EINVAL;

	if (pdev)
		tmon = platform_get_drvdata(pdev);

	if (tmon)
	{
		if (NvOdmTmonSuspend(tmon->ambient_dev) &&
		    NvOdmTmonSuspend(tmon->core_dev) )
			return 0;
		else
			err = -ENODEV;
	}

	return err;
}

static int tegra_tmon_resume(struct platform_device *pdev)
{
	struct tegra_tmon_driver_data *tmon=NULL;
	int err = -EINVAL;

	if (pdev)
		tmon = platform_get_drvdata(pdev);

	if (tmon)
	{
		if (NvOdmTmonResume(tmon->ambient_dev) &&
		    NvOdmTmonResume(tmon->core_dev) )
			return 0;
		else
			err = -ENODEV;
	}

	return err;
}

static struct platform_driver tegra_tmon_driver = {
	.probe		= tegra_tmon_probe,
	.remove		= tegra_tmon_remove,
	.suspend	= tegra_tmon_suspend,
	.resume		= tegra_tmon_resume,
	.driver		= {
		.name	= "tegra_tmon",
	},
};

static int __devinit tegra_tmon_init(void)
{
	return platform_driver_register(&tegra_tmon_driver);
}

static void __exit tegra_tmon_exit(void)
{
	platform_driver_unregister(&tegra_tmon_driver);
}

module_init(tegra_tmon_init);
module_exit(tegra_tmon_exit);

MODULE_DESCRIPTION("Tegra temp monitor driver");

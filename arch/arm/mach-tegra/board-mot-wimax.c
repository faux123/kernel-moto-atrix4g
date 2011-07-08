/*
 * arch/arm/mach-tegra/board-mot-wimax.c
 *
 * Board file with wimax specific functions and data structrures
 *
 * Copyright (c) 2010, Motorola Corporation.
 *
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include "board-mot.h"
#include "gpio-names.h"

#define WIMAX_ENABLE_GPIO         TEGRA_GPIO_PM5
#define WIMAX_SW_RESET_GPIO       TEGRA_GPIO_PL7
#define WIMAX_HOST_WAKEUP_GPIO    TEGRA_GPIO_PO5

static bool wimax_ctrl_ready;
static void (*wimax_status_cb)(void *dev_id);
static void *wimax_status_cb_devid;

int bcm_wimax_status_register(void (*callback)(void *dev_id), void *dev_id)
{
	if (wimax_status_cb)
		return -EAGAIN;
	wimax_status_cb = callback;
	wimax_status_cb_devid = dev_id;
	return 0;
}
EXPORT_SYMBOL(bcm_wimax_status_register);

int bcm_wimax_power(bool on)
{
	bool gpio_val = false;
	printk(KERN_INFO "%s(%d)\n", __func__, on);

	if (wimax_ctrl_ready) {
		gpio_val = on ? true : false;
		printk(KERN_INFO "gpio_set_value"
			"(WIMAX_ENABLE_GPIO, %d)\n", gpio_val);
		gpio_set_value(WIMAX_ENABLE_GPIO, gpio_val);
		msleep_interruptible(11);

		if (wimax_status_cb)
			wimax_status_cb(wimax_status_cb_devid);

		/* Let card detect function to be
		 * scheduled and probe the slot */
		msleep_interruptible(100);
	} else {
		printk(KERN_ERR "%s: WiMAX not ready\n", __func__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(bcm_wimax_power);

int bcm_wimax_get_irq(void)
{
	return gpio_to_irq(WIMAX_HOST_WAKEUP_GPIO);
}
EXPORT_SYMBOL(bcm_wimax_get_irq);

int __init mot_wimax_gpio_init(void)
{
	int ret = 0;
	printk(KERN_INFO "%s Enter\n", __func__);

	/* Configure WIMAX_ENABLE_GPIO */
	ret = gpio_request(WIMAX_ENABLE_GPIO, "wimax_enable_pin");
	if (ret) {
		printk(KERN_ERR "%s: Error (%d) - gpio_reqest"
			"(WIMAX_ENABLE_GPIO) failed\n", __func__, ret);
		goto exit;
	}

	ret = gpio_direction_output(WIMAX_ENABLE_GPIO, 0);
	if (ret) {
		printk(KERN_ERR "%s: Error (%d)- "
			"gpio_direction_output(WIMAX_ENABLE_GPIO, 0) failed\n",
			__func__, ret);
		goto exit;
	}

	/* Configure WIMAX_SW_RESET_GPIO */
	ret = gpio_request(WIMAX_SW_RESET_GPIO, "wimax_sw_reset_pin");
	if (ret) {
		printk(KERN_ERR "%s: Error (%d) - "
			"gpio_reqest(WIMAX_SW_RESET_GPIO) failed\n",
			__func__, ret);
		goto exit;
	}

	ret = gpio_direction_output(WIMAX_SW_RESET_GPIO, 0);
	if (ret) {
		printk(KERN_ERR "%s: Error (%d)- gpio_direction_output"
			"(WIMAX_SW_RESET_GPIO, 0) failed\n",
			__func__, ret);
		goto exit;
	}

	/* Configure WIMAX_HOST_WAKEUP_GPIO */
	ret = gpio_request(WIMAX_HOST_WAKEUP_GPIO, "wimax_host_wakeup_pin");
	if (ret) {
		printk(KERN_ERR "%s: Error (%d) - "
			"gpio_reqest(WIMAX_HOST_WAKEUP_GPIO) failed\n",
			__func__, ret);
		goto exit;
	}

	ret = gpio_direction_input(WIMAX_HOST_WAKEUP_GPIO);
	if (ret) {
		printk(KERN_ERR "%s: Error (%d)- "
			"gpio_direction_input(WIMAX_HOST_WAKEUP_GPIO) failed\n",
			__func__, ret);
		goto exit;
	}

	wimax_ctrl_ready = 1;

exit:
	return ret;
}



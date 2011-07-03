/*
 * arch/arm/mach-tegra/board-mot-wlan.c
 *
 * Board file with wlan specific functions and data structrures
 *
 * Copyright (c) 2010, Motorola Corporation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>


#include "board.h"
#include "board-mot.h"
#include "gpio-names.h"

#define WLAN_RESET_GPIO 	TEGRA_GPIO_PU2
#define WLAN_REG_ON_GPIO 	TEGRA_GPIO_PU3
#define WLAN_IRQ_GPIO 		TEGRA_GPIO_PU5

extern void sdhci_tegra_wlan_detect(void);

typedef struct chip_ctrls_nvodm {
	int ready;
} chip_ctrl;

static chip_ctrl wlan_ctrl = {0};

char bcm_wlan_mac[6] = {0x00, 0x90, 0xC3, 0x00, 0x00, 0x00};

int mot_wlan_gpio_init(void)
{
	int ret = 0;
	printk("%s Enter\n", __FUNCTION__);

        ret = gpio_request(WLAN_RESET_GPIO, "wlan_reset_pin");
        if (ret) {
		printk(KERN_ERR "%s: Error (%d) - gpio_reqest failed for WLAN reset\n", __func__,ret);
        } else {
		ret = gpio_direction_output(WLAN_RESET_GPIO, 0);
	}
        if (ret) {
		printk(KERN_ERR "%s: Error (%d)- gpio_direction failed config WLAN reset\n", __func__,ret);
		return -1;
	}

        ret = gpio_request(WLAN_REG_ON_GPIO, "wlan_reg_on_pin");
        if (ret) {
		printk(KERN_ERR "%s: Error (%d) - gpio_reqest failed for WLAN regulator control\n", __func__,ret);
        } else {
		ret = gpio_direction_output(WLAN_REG_ON_GPIO, 0);
	}
        if (ret) {
		printk(KERN_ERR "%s: Error (%d)- gpio_direction failed config WLAN regulator control\n", __func__,ret);
		return -1;
	}

        ret = gpio_request(WLAN_IRQ_GPIO, "wlan_irq_pin");
        if (ret) {
		printk(KERN_ERR "%s: Error (%d) - gpio_reqest failed for WLAN irq\n", __func__,ret);
        } else {
		ret = gpio_direction_input(WLAN_IRQ_GPIO);
	}
        if (ret) {
		printk(KERN_ERR "%s: Error (%d)- gpio_direction failed config WLAN irq\n", __func__,ret);
		return -1;
	}

	wlan_ctrl.ready = 1;
	return 0;
}

void bcm_wlan_power_on(unsigned mode)
{
	if (0 == wlan_ctrl.ready) {
    		printk("%s WLAN control not ready\n", __FUNCTION__);
		return;
	}
        gpio_set_value(WLAN_REG_ON_GPIO, 0x1);
        msleep(100);
        gpio_set_value(WLAN_RESET_GPIO, 0x1);
        msleep(100);
	if (1 == mode){
		sdhci_tegra_wlan_detect();
		msleep(100);
	}
}
EXPORT_SYMBOL(bcm_wlan_power_on);

void bcm_wlan_power_off(unsigned mode)
{
	if (0 == wlan_ctrl.ready) {
    		printk("%s WLAN control not ready\n", __FUNCTION__);
		return;
	}
        gpio_set_value(WLAN_RESET_GPIO, 0x0);
        msleep(100);
        gpio_set_value(WLAN_REG_ON_GPIO, 0x0);
        msleep(100);
	if (1 == mode){
		sdhci_tegra_wlan_detect();
		msleep(100);
	}
}
EXPORT_SYMBOL(bcm_wlan_power_off);

int bcm_wlan_get_irq(void)
{
	return gpio_to_irq(WLAN_IRQ_GPIO);
}
EXPORT_SYMBOL(bcm_wlan_get_irq);

/*
 * Parse the WLAN MAC ATAG
 */
static int __init parse_tag_wlan_mac(const struct tag *tag)
{
	const struct tag_wlan_mac *wlan_mac_tag = &tag->u.wlan_mac;

	pr_info("%s: WLAN MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", __func__,
	        wlan_mac_tag->addr[0], wlan_mac_tag->addr[1],
		wlan_mac_tag->addr[2], wlan_mac_tag->addr[3],
		wlan_mac_tag->addr[4], wlan_mac_tag->addr[5]);

	memcpy(bcm_wlan_mac, wlan_mac_tag->addr, sizeof(bcm_wlan_mac));

	return 0;
}
__tagtable(ATAG_WLAN_MAC, parse_tag_wlan_mac);
EXPORT_SYMBOL(bcm_wlan_mac);

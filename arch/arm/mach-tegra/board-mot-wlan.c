/*
 * arch/arm/mach-tegra/board-mot-wlan.c
 *
 * Board file with wlan specific functions and data structrures
 *
 * Copyright (c) 2010, Motorola Corporation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <linux/io.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <mach/sdhci.h>

#include "board.h"
#include "board-mot.h"
#include "gpio-names.h"

#define WLAN_RESET_GPIO 	TEGRA_GPIO_PU2
#define WLAN_REG_ON_GPIO 	TEGRA_GPIO_PU3
#define WLAN_IRQ_GPIO 		TEGRA_GPIO_PU5

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS        4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS         160
#define PREALLOC_WLAN_SECTION_HEADER            24

#define WLAN_SECTION_SIZE_0     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3     (PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM        16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
char mot_wlan_mac[6] = {0x00, 0x90, 0xC3, 0x00, 0x00, 0x00};

struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
};

static struct wifi_mem_prealloc_struct
	wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *mot_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

 int __init mot_init_wifi_mem(void)
 {
	int i;
	for (i = 0; (i < WLAN_SKB_BUF_NUM); i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
		}
	for (i = 0; (i < PREALLOC_WLAN_NUMBER_OF_SECTIONS); i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
			GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
return 0;
 }

static struct resource mot_wifi_resources[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= GPIO_TO_IRQ(WLAN_IRQ_GPIO),
		.end	= GPIO_TO_IRQ(WLAN_IRQ_GPIO),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL
			| IORESOURCE_IRQ_SHAREABLE,
	},
 };

 static int mot_wifi_cd; /* WIFI virtual 'card detect' status */
 static void (*wifi_status_cb)(int card_present, void *dev_id);
 static void *wifi_status_cb_devid;

 int mot_wifi_set_carddetect(int val)
 {
	pr_debug("%s: %d\n", __func__, val);
	mot_wifi_cd = val;
	sdhci_tegra_wlan_detect();
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
 }

 int mot_wifi_power(int on)
 {
	pr_debug("%s: %d\n", __func__, on);
	gpio_set_value(WLAN_REG_ON_GPIO, on);
	mdelay(100);
	gpio_set_value(WLAN_RESET_GPIO, on);
	mdelay(100);
	return 0;
 }

 int mot_wifi_reset(int on)
 {
	pr_debug("%s:\n", __func__);
	gpio_set_value(WLAN_RESET_GPIO, on);
	mdelay(100);
	return 0;
 }

 int mot_wifi_get_mac_addr(unsigned char *buf)
 {
	if (!buf)
		return -EINVAL;
	pr_debug("%s\n", __func__);
	memcpy(buf, mot_wlan_mac, sizeof(mot_wlan_mac));
	return 0;
 }

 static struct wifi_platform_data mot_wifi_control = {
	.set_power     = mot_wifi_power,
	.set_reset      = mot_wifi_reset,
	.set_carddetect = mot_wifi_set_carddetect,
	.mem_prealloc   = mot_wifi_mem_prealloc,
	.get_mac_addr   = mot_wifi_get_mac_addr,
 };

 static struct platform_device mot_wifi_device = {
	.name           = "bcm4329_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(mot_wifi_resources),
	.resource       = mot_wifi_resources,
	.dev            = {
	.platform_data = &mot_wifi_control,
	},
 };


 static int __init mot_wlan_gpio_init(void)
 {
	int ret = 0;
	pr_debug("%s Enter\n", __func__);

	tegra_gpio_enable(WLAN_RESET_GPIO);
	ret = gpio_request(WLAN_RESET_GPIO, "wlan_reset_pin");
	if (ret)
		pr_err("%s: %d gpio_reqest reset\n", __func__, ret);
	else
		ret = gpio_direction_output(WLAN_RESET_GPIO, 0);
	if (ret) {
		pr_err("%s: Err %d gpio_direction reset\n", __func__, ret);
		return -1;
	}

	tegra_gpio_enable(WLAN_REG_ON_GPIO);
	ret = gpio_request(WLAN_REG_ON_GPIO, "wlan_reg_on_pin");
	if (ret)
		pr_err("%s: Err %d gpio_reqest reg\n", __func__, ret);
	 else
		ret = gpio_direction_output(WLAN_REG_ON_GPIO, 0);
	if (ret) {
		pr_err("%s: Err %d gpio_direction reg\n", __func__, ret);
		return -1;
	}

	tegra_gpio_enable(WLAN_IRQ_GPIO);
	ret = gpio_request(WLAN_IRQ_GPIO, "wlan_irq_pin");
	if (ret)
		pr_err("%s: Error (%d) - gpio_reqest irq\n", __func__, ret);
	else
		ret = gpio_direction_input(WLAN_IRQ_GPIO);
	if (ret) {
		pr_err("%s: Err %d gpio_direction irq\n", __func__, ret);
		return -1;
	}
	return 0;
 }

 int __init mot_wlan_init(void)
 {
	pr_debug("%s: start\n", __func__);
	mot_wlan_gpio_init();
	mot_init_wifi_mem();
	return platform_device_register(&mot_wifi_device);
 }

#ifdef CONFIG_WIFI_CONTROL_EXPORT

 void bcm_wlan_power_on(unsigned mode)
 {
	if (0 == wlan_ctrl.ready) {
		pr_err("%s WLAN control not ready\n", __func__);
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
		pr_err("%s WLAN control not ready\n", __func__);
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

 char *bcm_wlan_mac = mot_wlan_mac;
 EXPORT_SYMBOL(bcm_wlan_mac);

#endif /*  CONFIG_WIFI_CONTROL_EXPORT */

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

	memcpy(mot_wlan_mac, wlan_mac_tag->addr, sizeof(mot_wlan_mac));

	return 0;
 }
 __tagtable(ATAG_WLAN_MAC, parse_tag_wlan_mac);

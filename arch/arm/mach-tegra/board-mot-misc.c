#include <linux/init.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/tcmd_driver.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/bootinfo.h>

#include <mach/iomap.h>
#include <mach/mdm_ctrl.h>
#include <mach/sec_linux.h>
#include <mach/nvrm_linux.h>
#include <mach/sdhci-simple.h>
#include <mach/nand.h>

#include "gpio-names.h"
#include "board-mot.h"
#include "nvodm_query.h"
#include "nvodm_query_pinmux.h"
#include "nvodm_query_discovery.h"
#include "nvrm_pinmux.h"
#include "nvrm_module.h"

#if defined(CONFIG_APANIC_MMC) || defined(CONFIG_APANIC_RAM)
#include <mach/apanic.h>
#endif

#ifdef CONFIG_APANIC_RAM
/* The kernel modifies these, so we need two. */
static struct resource apanic_handle_ram_resource[] = {
    {
        .flags = IORESOURCE_MEM,
    },
};
static struct resource apanic_report_ram_resource[] = {
    {
        .flags = IORESOURCE_MEM,
    },
};
static struct platform_device apanic_handle_ram_platform_device = {
    .name          = "apanic_handle_ram",
    .id            = 0,
    .num_resources = ARRAY_SIZE(apanic_handle_ram_resource),
    .resource      = apanic_handle_ram_resource,

};
static struct platform_device apanic_report_ram_platform_device = {
    .name          = "apanic_report_ram",
    .id            = 0,
    .num_resources = ARRAY_SIZE(apanic_report_ram_resource),
    .resource      = apanic_report_ram_resource,

};

static int __init apanic_ram_init(void)
{
    int result = -ENOMEM;

	printk(KERN_INFO "apanic_ram_init\n");
    if (mot_get_apanic_resource(&apanic_handle_ram_resource[0]) == 0)
    {
    	result = platform_device_register(&apanic_handle_ram_platform_device);
    }
    if (result == 0 &&
        mot_get_apanic_resource(&apanic_report_ram_resource[0]) == 0)
    {
        result = platform_device_register(&apanic_report_ram_platform_device);
    }

    return result;
}
#endif

#ifdef CONFIG_APANIC_MMC

static struct tegra_sdhci_simple_platform_data tegra_sdhci_simple_platform_data;
static struct platform_device tegra_sdhci_simple_device;

static struct apanic_mmc_platform_data apanic_mmc_platform_data;

static struct platform_device apanic_handle_mmc_platform_device = {
	.name          = "apanic_handle_mmc",
	.id            = 0,
	.dev =
	{
		.platform_data = &apanic_mmc_platform_data,
	}
};

extern struct tegra_nand_platform tegra_nand_plat;
extern int tegra_sdhci_boot_device;
extern struct platform_device tegra_sdhci_devices[];

int apanic_mmc_init(void)
{
	int i;
	int result = -ENOMEM;

	/*
	 * This is a little convoluted, but the simple driver needs to map the
	 * I/O port and access other resources in order to use the hardware.
	 * It can't do it through the normal means or else the kernel will try
	 * to claim the same resources as the real sdhci-tegra driver at boot.
	 */
	if (tegra_sdhci_boot_device >= 0) {
		tegra_sdhci_simple_platform_data.sdhci_pdata =
			tegra_sdhci_devices[tegra_sdhci_boot_device].dev.platform_data;
		tegra_sdhci_simple_platform_data.resource =
			tegra_sdhci_devices[tegra_sdhci_boot_device].resource;
		tegra_sdhci_simple_platform_data.num_resources =
			tegra_sdhci_devices[tegra_sdhci_boot_device].num_resources;
		tegra_sdhci_simple_platform_data.clk_dev_name =
			kasprintf(GFP_KERNEL, "tegra-sdhci.%d", tegra_sdhci_boot_device);

		tegra_sdhci_simple_device.id = tegra_sdhci_boot_device;
		tegra_sdhci_simple_device.name = "tegra-sdhci-simple";
		tegra_sdhci_simple_device.dev.platform_data =
				&tegra_sdhci_simple_platform_data;

		result = platform_device_register(&tegra_sdhci_simple_device);
	}

	/*
	 * FIXME: There is no way to "discover" the kpanic partition because
	 * it has no file system and the legacy MBR/EBR tables do not support
	 * labels.  GPT promises to address this in K35 or later.
	 */
	if (result == 0) {
		apanic_mmc_platform_data.id = 0;  /* mmc0 - not used by sdhci-tegra-simple */
		for (i = 0; i < tegra_nand_plat.nr_parts; i++) {
			if (strcmp(CONFIG_APANIC_PLABEL, tegra_nand_plat.parts[i].name))
				continue;
			apanic_mmc_platform_data.sector_size = 512;  /* fixme */
			apanic_mmc_platform_data.start_sector =
					tegra_nand_plat.parts[i].offset / 512;
			apanic_mmc_platform_data.sectors =
					tegra_nand_plat.parts[i].size / 512;
			break;
		}

		result = platform_device_register(&apanic_handle_mmc_platform_device);
	}

	return result;
}
#endif

void mot_set_hsj_mux(short hsj_mux_gpio)
{
	/* set pin M 2 to 1 to route audio onto headset or 0 to route console uart */

	if( gpio_request(TEGRA_GPIO_PM2, "hs_detect_enable") < 0 )
  		printk("\n%s: gpio_request error gpio %d  \n", 
				__FUNCTION__,TEGRA_GPIO_PM2);
	else {
		gpio_direction_output(TEGRA_GPIO_PM2, hsj_mux_gpio);
	}
}

/*
 * Security
 */
static struct sec_platform_data mot_sec_platform_data;
static struct platform_device mot_sec_platform_device = {
    .name          = "sec",
    .id            = 0,
    .dev =
    {
        .platform_data = &mot_sec_platform_data,
    }
};

void mot_sec_init(void)
{
    	platform_device_register(&mot_sec_platform_device);
}


/*
 * TCMD
 */
static struct tcmd_driver_platform_data mot_tcmd_platform_data;
static struct platform_device mot_tcmd_platform_device = {
    .name          = "tcmd_driver",
    .id            = 0,
    .dev = {
		.platform_data = &mot_tcmd_platform_data,
    }
};

void mot_tcmd_init(void)
{
	mot_tcmd_platform_data.gpio_list[TCMD_GPIO_ISL29030_INT]
      = TEGRA_GPIO_PE1;
	mot_tcmd_platform_data.gpio_list[TCMD_GPIO_KXTF9_INT]
      = TEGRA_GPIO_PV3;
	mot_tcmd_platform_data.gpio_list[TCMD_GPIO_MMC_DETECT]
      = TEGRA_GPIO_PI5;
	mot_tcmd_platform_data.gpio_list[TCMD_GPIO_INT_MAX_NUM]
      = -1;
	mot_tcmd_platform_data.size = TCMD_GPIO_INT_MAX_NUM;

	if (machine_is_etna()) {
		if (system_rev == 0x1100) {
			mot_tcmd_platform_data.gpio_list[TCMD_GPIO_KXTF9_INT]
				= TEGRA_GPIO_PN4;
      }
    }

	platform_device_register(&mot_tcmd_platform_device);
}

/*
 * Some global queries for the framebuffer, display, and backlight drivers.
 */
static unsigned int s_MotorolaDispInfo = 0;
static unsigned int s_MotorolaFBInfo = 1;

unsigned short bootloader_ver_major = 0;
unsigned short bootloader_ver_minor = 0;
unsigned short uboot_ver_major = 0;
unsigned short uboot_ver_minor = 0;

unsigned char lpddr2_mr[12];

int MotorolaBootFBArgGet(unsigned int *arg)
{
    *arg = s_MotorolaFBInfo;
    return 0;
}

int MotorolaBootDispArgGet(unsigned int *arg)
{
    if(s_MotorolaDispInfo)
    {
        *arg = s_MotorolaDispInfo;
        return 0;
    }

    return -1;
}


/*
 * Parse the Motorola-specific ATAG
 */
static int __init parse_tag_motorola(const struct tag *tag)
{
    const struct tag_motorola *moto_tag = &tag->u.motorola;
    int i = 0;

    s_MotorolaDispInfo = moto_tag->panel_size;
    s_MotorolaFBInfo = moto_tag->allow_fb_open;
    
    mot_sec_platform_data.fl_factory = moto_tag->in_factory;

    bootloader_ver_major = moto_tag->bl_ver_major;
    bootloader_ver_minor = moto_tag->bl_ver_minor;
    uboot_ver_major = moto_tag->uboot_ver_major;
    uboot_ver_minor = moto_tag->uboot_ver_minor;
	bi_set_cid_recover_boot(moto_tag->cid_suspend_boot);

    pr_info("%s: panel_size: %x\n", __func__, s_MotorolaDispInfo);
    pr_info("%s: allow_fb_open: %x\n", __func__, s_MotorolaFBInfo);
    pr_info("%s: factory: %d\n", __func__, mot_sec_platform_data.fl_factory);
    pr_info("%s: cid_suspend_boot: %u\n", __func__,
				(unsigned)moto_tag->cid_suspend_boot);

    /*
     *	Dump memory information
     */
     /* FIXME:  Add eMMC support */
	for (i = 0; i < 12; i++) {
		lpddr2_mr[i] = moto_tag->at_lpddr2_mr[i];
		pr_info("%s: LPDDR2 MR%d:     0x%04X (0x%04X)\n", __func__, i,
			lpddr2_mr[i],
			moto_tag->at_lpddr2_mr[i]);
	}

    return 0;
}
__tagtable(ATAG_MOTOROLA, parse_tag_motorola);

/*
 *   UTS tool needs to emulate numbers keys and send/end keys. Add them to tegra kbc driver keymap 
 */

static int keymap_update_connect(struct input_handler *handler, struct input_dev *dev,
					  const struct input_device_id *id)
 {
	int i;

	if (strcmp(dev->name , "tegra-kbc"))  return 0;

	set_bit(0x38, dev->keybit); // ALT
	set_bit(0x3E, dev->keybit); // CALL
	set_bit(0x3D, dev->keybit); // ENDCALL
	set_bit(0xE7, dev->keybit); // SEND
	set_bit(0x6A, dev->keybit); // DPAD_UP
	set_bit(0x69, dev->keybit); // DPAD_DOWN
	set_bit(0x67, dev->keybit); // DPAD_LEFT
	set_bit(0x6C, dev->keybit); // DPAD_RIGHT
	set_bit(0xE8, dev->keybit); // DPAD_CENTER
	set_bit(0xA2, dev->keybit); // Power


	pr_info("keymap_update add key codes to device %s\n", dev->name);
	return 0;
 }

static void keymap_update_disconnect(struct input_handle *handle)  {}
static void keymap_update_event(struct input_handle *handle, unsigned int type, unsigned int code, int value) {}

static const struct input_device_id keymap_update_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

MODULE_DEVICE_TABLE(input, keymap_update_ids);

static struct input_handler keymap_update_handler = {
	.event      = keymap_update_event,
	.connect    = keymap_update_connect,
	.disconnect = keymap_update_disconnect,
	.name       = "keymap_update",
	.id_table   = keymap_update_ids,
};


void mot_keymap_update_init(void)
{
	input_register_handler(&keymap_update_handler);
}

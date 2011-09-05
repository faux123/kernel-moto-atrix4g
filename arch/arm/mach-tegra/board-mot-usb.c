#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>

#include <mach/nvrm_linux.h>

#include <nvrm_module.h>
#include <nvrm_boot.h>
#include <nvodm_services.h>

#include "board-mot.h"
#include "nvrm_power.h"

#define BOOT_MODE_MAX_LEN 30

static char boot_mode[BOOT_MODE_MAX_LEN+1];
int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	printk(KERN_INFO "boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);

#ifdef CONFIG_USB_MOT_ANDROID
static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"acm",
	"usbnet",
	"mtp",
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_phone_portal[] = {
	"acm",
	"usbnet",
	"mtp",
};

static char *usb_functions_phone_portal_adb[] = {
	"acm",
	"usbnet",
	"mtp",
	"adb",
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *bp_usb_functions_bp[] = { "acm", "usbnet" };
static char *bp_usb_functions_bp_adb[] = { "acm", "usbnet", "adb" };
static char *bp_usb_functions_rndis_bp[] = { "rndis", "acm", "usbnet" };
static char *bp_usb_functions_all[] = {
	"rndis",
	"acm",
	"usbnet",
	"adb"
};
#endif

static struct android_usb_product usb_products[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	{
		.product_id     = 0x708a,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal),
		.functions      = usb_functions_phone_portal,
	},
	{
		.product_id     = 0x708b,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal_adb),
		.functions      = usb_functions_phone_portal_adb,
	},
	{
		.product_id     = 0x7088,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp),
		.functions      = usb_functions_mtp,
	},
	{
		.product_id     = 0x7089,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb),
		.functions      = usb_functions_mtp_adb,
	},
	{
		.product_id	= 0x7086,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x7087,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id	= 0x7091,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x7092,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#endif
#endif
};

static struct android_usb_product bp_usb_products[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	{
		.product_id     = 0x7093,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_bp),
		.functions      = bp_usb_functions_bp,
	},
	{
		.product_id     = 0x7094,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_bp_adb),
		.functions      = bp_usb_functions_bp_adb,
	},
	{
		.product_id     = 0x7095,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_rndis_bp),
		.functions      = bp_usb_functions_rndis_bp,
	},
	{
		.product_id     = 0x7096,
		.num_functions  = ARRAY_SIZE(bp_usb_functions_all),
		.functions      = bp_usb_functions_all,
	},
#endif
};

static struct android_usb_platform_data andusb_plat = {
	.vendor_id = 0x22b8,
	.product_id = 0x7081,
	.manufacturer_name = "Motorola",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct android_usb_platform_data andusb_plat_bp = {
	.vendor_id = 0x22b8,
	.product_id = 0x7094,
	.manufacturer_name = "Motorola",
	.serial_number = "0000",
	.num_products = ARRAY_SIZE(bp_usb_products),
	.products = bp_usb_products,
	.num_functions = ARRAY_SIZE(bp_usb_functions_all),
	.functions = bp_usb_functions_all,
};

static struct platform_device tegra_android_device = {
	.name = "android_usb",
	.id = -1,
	.dev = {
		.platform_data = &andusb_plat,
	},
};

static struct usb_mass_storage_platform_data tegra_usb_fsg_platform = {
	.vendor = "Motorola",
	.product = "Mass Storage",
#if defined(CONFIG_USB_MOT_MSC_CDROM)
	.nluns = 3, /* one for cdrom, one for external sd and one for eMMC */
#else
	.nluns = 2,   /* one for external sd and one for eMMC */
#endif
	.bulk_size = 16384,
};
static struct platform_device tegra_usb_fsg_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &tegra_usb_fsg_platform,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x22b8,
	.vendorDescr	= "Motorola",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct acm_platform_data acm_pdata = {
	/* Modify num_inst at runtime depending on boot_mode */
	.num_inst       = 1,
};

static struct platform_device acm_device = {
	.name	= "acm",
	.id	= -1,
	.dev	= {
		.platform_data = &acm_pdata,
	},
};

static char *usb_serial_num;

static char *olympus_dev = "MB860";
static char *etna_dev = "XT865";
static char *daytona_dev = "MB870";
static char *sunfire_dev = "MB855";
static char *default_dev = "Motorola AP20";

static int __init mot_usb_serial_num_setup(char *options)
{
    usb_serial_num = options;
    pr_info("%s: usb_serial_num: %s\n", __func__, usb_serial_num);
    return 0;
}
__setup("androidboot.serialno=", mot_usb_serial_num_setup);

void tegra_get_serial_number(void)
{
#ifdef CONFIG_USB_ANDROID_RNDIS
	unsigned int chip_id[2];
	char serial[17];
	int i;
	char *src;

	NvRmQueryChipUniqueId(s_hRmGlobal, sizeof(chip_id), (void *)chip_id);
	snprintf(serial, sizeof(serial), "%08x%08x", chip_id[1], chip_id[0]);

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	src = serial;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
#endif
}

void mot_setup_gadget(void)
{
	unsigned int chip_id[2];
	char serial[17];

	NvRmQueryChipUniqueId(s_hRmGlobal, sizeof(chip_id), (void *)chip_id);
	snprintf(serial, sizeof(serial), "%08x%08x", chip_id[1], chip_id[0]);

	tegra_get_serial_number();

	if (usb_serial_num)
		andusb_plat.serial_number = kstrdup(usb_serial_num, GFP_KERNEL);
	else
		andusb_plat.serial_number = kstrdup(serial, GFP_KERNEL);

	if (machine_is_olympus()) {
		andusb_plat.product_name = olympus_dev;
		andusb_plat_bp.product_name = olympus_dev;
		tegra_usb_fsg_platform.product = olympus_dev;
	} else if (machine_is_etna()) {
		andusb_plat.product_name = etna_dev;
		andusb_plat_bp.product_name = etna_dev;
		tegra_usb_fsg_platform.product = etna_dev;
	} else if (machine_is_tegra_daytona()) {
		andusb_plat.product_name = daytona_dev;
		andusb_plat_bp.product_name = daytona_dev;
		tegra_usb_fsg_platform.product = daytona_dev;
	} else if (machine_is_sunfire()) {
		andusb_plat.product_name = sunfire_dev;
		andusb_plat_bp.product_name = sunfire_dev;
		tegra_usb_fsg_platform.product = sunfire_dev;
	} else {
		andusb_plat.product_name = default_dev;
		andusb_plat_bp.product_name = default_dev;
		tegra_usb_fsg_platform.product = default_dev;
	}

	if (!strncmp(boot_mode, "bp-tools", BOOT_MODE_MAX_LEN)) {
		tegra_android_device.dev.platform_data = &andusb_plat_bp;
		acm_pdata.num_inst = 4;
		acm_pdata.use_iads = 1;
	}

	platform_device_register(&acm_device);
	platform_device_register(&tegra_usb_fsg_device);
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif
	platform_device_register(&tegra_android_device);
}

/*
 * arch/arm/mach-tegra/board-mot.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 NVIDIA Corporation
 * Copyright (C) 2010 Motorola, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/pda_power.h>
#include <linux/io.h>
#include <linux/usb/android_composite.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

#include <asm/bootinfo.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/nvrm_linux.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/cpcap-accy.h>
#include <linux/reboot.h>
#if defined(CONFIG_KEYBOARD_GPIO)
#include <linux/gpio_keys.h>
#include <linux/input.h>
#endif
#include <nvrm_module.h>
#include <nvrm_boot.h>
#include <nvodm_services.h>

#include "gpio-names.h"

#include <linux/qtouch_obp_ts.h>
#include <linux/isl29030.h>

#include <linux/leds-lm3530.h>
#include <linux/leds-lm3532.h>
#include <linux/l3g4200d.h>

#include "board.h"
#include "hwrev.h"

#include "board-mot.h"
#include "nvrm_power.h"


#ifdef CONFIG_USB_MOT_ANDROID
static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"acm0",
	"acm1",
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

static char oly_unused_pins_p3[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,
        TEGRA_GPIO_PC1,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PE5,
        TEGRA_GPIO_PE6,
        TEGRA_GPIO_PE7,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PT4,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS0,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PS2,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        TEGRA_GPIO_PK5,
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
};

static char oly_unused_pins_p2[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,
        TEGRA_GPIO_PC1,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PE7,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PT4,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS0,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PS2,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        TEGRA_GPIO_PK5,
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
        TEGRA_GPIO_PD1,
};

static char oly_unused_pins_p1[] = {
        TEGRA_GPIO_PO1,
        TEGRA_GPIO_PO2,
        TEGRA_GPIO_PO3,
        TEGRA_GPIO_PO4,
        TEGRA_GPIO_PO5,
        TEGRA_GPIO_PO6,
        TEGRA_GPIO_PO7,
        TEGRA_GPIO_PO0,
        TEGRA_GPIO_PY0,
        TEGRA_GPIO_PY1,
        TEGRA_GPIO_PY2,
        TEGRA_GPIO_PY3,        
        TEGRA_GPIO_PN5,
        TEGRA_GPIO_PN6,
        TEGRA_GPIO_PW1,
        TEGRA_GPIO_PB3,
        TEGRA_GPIO_PJ3,
        TEGRA_GPIO_PF0,
        TEGRA_GPIO_PM6,
        TEGRA_GPIO_PM7,
        TEGRA_GPIO_PL5,
        TEGRA_GPIO_PL6,
        TEGRA_GPIO_PL7,
        TEGRA_GPIO_PT2,
        TEGRA_GPIO_PD6,
        TEGRA_GPIO_PD7,
        TEGRA_GPIO_PR3,
        TEGRA_GPIO_PR4,
        TEGRA_GPIO_PR5,
        TEGRA_GPIO_PR6,
        TEGRA_GPIO_PR7,
        TEGRA_GPIO_PS1,
        TEGRA_GPIO_PQ3,
        TEGRA_GPIO_PQ4,
        TEGRA_GPIO_PQ5,
        TEGRA_GPIO_PBB0,
        TEGRA_GPIO_PZ5,
        TEGRA_GPIO_PK5,
        TEGRA_GPIO_PK6,
        TEGRA_GPIO_PW5,
        TEGRA_GPIO_PD3,
        TEGRA_GPIO_PI7,
        TEGRA_GPIO_PJ0,
        TEGRA_GPIO_PJ2,
        TEGRA_GPIO_PK3,
        TEGRA_GPIO_PK4,
        TEGRA_GPIO_PK2,
        TEGRA_GPIO_PG0,
        TEGRA_GPIO_PG1,
        TEGRA_GPIO_PG2,
        TEGRA_GPIO_PG3,
        TEGRA_GPIO_PG4,
        TEGRA_GPIO_PG5,
        TEGRA_GPIO_PG6,
        TEGRA_GPIO_PG7,
        TEGRA_GPIO_PH0,
        TEGRA_GPIO_PH1,
        TEGRA_GPIO_PH2,
        TEGRA_GPIO_PH3,
        TEGRA_GPIO_PI0,
        TEGRA_GPIO_PI4,
        TEGRA_GPIO_PT5,
        TEGRA_GPIO_PT6,
        TEGRA_GPIO_PC7,
        TEGRA_GPIO_PV7,
        TEGRA_GPIO_PD1,
};
static struct android_usb_product usb_products[] = {
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

static char *usb_serial_num;

static char *olympus_dev = "ME860";
static char *etna_dev = "Etna";
static char *sunfire_dev = "Sunfire";
static char *default_dev = "Motorola AP20";
#endif  /* CONFIG_USB_MOT_ANDROID */

static struct platform_device *platform_devices[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	&tegra_usb_fsg_device,
	&tegra_android_device,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
        &rndis_device,
#endif
};

extern void __init tegra_setup_nvodm(bool standard_i2c, bool standard_spi);
extern void __init tegra_register_socdev(void);


/* 
 * List of i2c devices
 */
/* IMPORTANT: backlight has to be first, don't move it!!! */
static struct i2c_board_info tegra_i2c_bus0_board_info[] = {
	[BACKLIGHT_DEV] = { /* Display backlight */
		I2C_BOARD_INFO(LM3532_NAME, LM3532_I2C_ADDR),
		.platform_data = &lm3532_pdata,
		/*.irq = ..., */
	},
	[TOUCHSCREEN_DEV] = {
		/* XMegaT touchscreen driver */
		I2C_BOARD_INFO(QTOUCH_TS_NAME, XMEGAT_I2C_ADDR),
		.irq = TOUCH_GPIO_INTR,
		.platform_data = &ts_platform_olympus_m_1,
	},
#if defined(CONFIG_TEGRA_ODM_OLYMPUS)
	{
		/*  ISL 29030 (prox/ALS) driver */
		I2C_BOARD_INFO(LD_ISL29030_NAME, 0x44),
		.platform_data = &isl29030_als_ir_data_Olympus,
		.irq = 180,
	},
#endif
};
static struct i2c_board_info tegra_i2c_bus2_board_info[] = {
	{
	/* only exists on Etna P2+; probe will gracefully fail if HW doesn't exist */
		I2C_BOARD_INFO(L3G4200D_NAME, 0x68),
		.platform_data = &tegra_gyro_pdata,
		.irq = GPIO_TO_IRQ(TEGRA_GPIO_PH2),
	},
};
static struct i2c_board_info tegra_i2c_bus3_board_info[] = {
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.platform_data = &akm8975_data,
		.irq = GPIO_TO_IRQ(TEGRA_GPIO_PE2),
	},
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.platform_data = &kxtf9_data,
	},
#if defined(CONFIG_TEGRA_ODM_ETNA)
	{
		/*  ISL 29030 (prox/ALS) driver */
		I2C_BOARD_INFO(LD_ISL29030_NAME, 0x44),
		.platform_data = &isl29030_als_ir_data_Etna,
		.irq = 180,
	},
#endif
#if defined(CONFIG_TEGRA_ODM_SUNFIRE)
	{
		/*  ISL 29030 (prox/ALS) driver */
		I2C_BOARD_INFO(LD_ISL29030_NAME, 0x44),
		.platform_data = &isl29030_als_ir_data_Sunfire,
		.irq = 180,
	},
#endif
};

#if 1
static noinline void __init tegra_setup_bluesleep(void)
{
       struct platform_device *pDev = NULL;
       struct resource *res;

       pDev = platform_device_alloc("bluesleep", 0);
       if (!pDev) {
               pr_err("unable to allocate platform device for bluesleep");
               goto fail;
       }

       res = kzalloc(sizeof(struct resource)*3, GFP_KERNEL);
       if (!res) {
               pr_err("unable to allocate resource for bluesleep\n");
               goto fail;
       }

       res[0].name   = "gpio_host_wake";
       res[0].start  = TEGRA_GPIO_PU6;
       res[0].end    = TEGRA_GPIO_PU6;
       res[0].flags  = IORESOURCE_IO;

       res[1].name   = "gpio_ext_wake";
       res[1].start  = TEGRA_GPIO_PU1;
       res[1].end    = TEGRA_GPIO_PU1;
       res[1].flags  = IORESOURCE_IO;

       res[2].name   = "host_wake";
       res[2].start  = gpio_to_irq(TEGRA_GPIO_PU6);
       res[2].end    = gpio_to_irq(TEGRA_GPIO_PU6);
       res[2].flags  = IORESOURCE_IRQ;

       if (platform_device_add_resources(pDev, res, 3)) {
               pr_err("unable to add resources to bluesleep device\n");
               goto fail;
       }

       if (platform_device_add(pDev)) {
               pr_err("unable to add bluesleep device\n");
               goto fail;
       }

fail:
       if (pDev)
               return;
}
#else
static inline void tegra_setup_bluesleep(void) { }
#endif



static int __init mot_usb_serial_num_setup(char *options)
{
    usb_serial_num = options;
    pr_info("%s: usb_serial_num: %s\n", __func__, usb_serial_num);
    return 0;
}
__setup("androidboot.serialno=", mot_usb_serial_num_setup);

#ifdef NEED_FACT_BUSY_HINT
static void FactoryBusyHint(void)
{
	/* Fix battery calibration by bumping min freq to 250 MHz in the factory. */
	NvRmDfsBusyHint pFactoryHintOn[] =
	{
		{ NvRmDfsClockId_Cpu, NV_WAIT_INFINITE, 250000, NV_TRUE }
	};

	NvU32 PowerClientId = NVRM_POWER_CLIENT_TAG('F','a','c','t');

	if(NvRmPowerRegister(s_hRmGlobal, 0, &PowerClientId) != NvSuccess) {
		printk(KERN_ERR "%s: failed to register\n", __func__);
		return;
	}

	if(NvRmPowerBusyHintMulti(s_hRmGlobal,
	                          PowerClientId,
	                          pFactoryHintOn,
	                          1, /*number of hints*/
	                          NvRmDfsBusyHintSyncMode_Async) != NvSuccess) {
		printk(KERN_ERR "%s: Busy Hint failed\n", __func__);
	}

	return;
}
#endif

static int config_unused_pins(char *pins, int num)
{
        int i, ret = 0;
        
        pr_info("%s: ENTRY\n", __func__);

        for (i = 0; i < num; i++) {
                ret = gpio_request(pins[i], "unused");
                if (ret) {
                        printk(KERN_ERR "%s: Error (%d) - gpio_reqest failed for unused GPIO %d\n", __func__,ret, pins[i]);
                } else {                
                        ret = gpio_direction_output(pins[i], 1);
                        if (ret) {
                                printk(KERN_ERR "%s: Error (%d)- gpio_direction failed for unused GPIO %d\n", __func__,ret, pins[i]);
                        }
                }
        }
        
        pr_info("%s: EXIT\n", __func__);

        return ret;
}

void tegra_get_serial_number(void)
{
#ifdef CONFIG_USB_ANDROID_RNDIS
	unsigned int chip_id[2];
	char serial[17];
	int i;
	char *src;

	NvRmQueryChipUniqueId(s_hRmGlobal, sizeof(chip_id), (void*)chip_id);
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

static void __init tegra_mot_init(void)
{
	unsigned int chip_id[2];
	char serial[17];

	tegra_common_init();
	tegra_setup_nvodm(true, true);
	tegra_register_socdev();

#ifdef CONFIG_APANIC_RAM
	apanic_ram_init();
#endif
#ifdef CONFIG_APANIC_MMC
	apanic_mmc_init();
#endif

	mot_setup_power();
	mot_setup_lights(&tegra_i2c_bus0_board_info[BACKLIGHT_DEV]);
	mot_setup_touch(&tegra_i2c_bus0_board_info[TOUCHSCREEN_DEV]);

	mot_sec_init();

	NvRmQueryChipUniqueId(s_hRmGlobal, sizeof(chip_id), (void*)chip_id);
	snprintf(serial, sizeof(serial), "%08x%08x", chip_id[1], chip_id[0]);

        tegra_get_serial_number();

#ifdef CONFIG_USB_MOT_ANDROID
	if (usb_serial_num)
		andusb_plat.serial_number = kstrdup(usb_serial_num, GFP_KERNEL);
	else
		andusb_plat.serial_number = kstrdup(serial, GFP_KERNEL);

	if(machine_is_olympus()) {
		andusb_plat.product_name = olympus_dev;
		tegra_usb_fsg_platform.product = olympus_dev;

		tegra_uart_platform[UART_IPC_OLYMPUS].uart_ipc = 1;
		tegra_uart_platform[UART_IPC_OLYMPUS].uart_wake_host = TEGRA_GPIO_PA0;
		tegra_uart_platform[UART_IPC_OLYMPUS].uart_wake_request = TEGRA_GPIO_PF1;
		tegra_uart_platform[UART_IPC_OLYMPUS].peer_register = mot_mdm_ctrl_peer_register;
	}
	else if(machine_is_etna() || machine_is_tegra_daytona()) {
		andusb_plat.product_name = etna_dev;
		tegra_usb_fsg_platform.product = etna_dev;
		
		if (HWREV_TYPE_IS_BRASSBOARD(system_rev)) {
			/* The modem is dead on S2, which makes the UART angry. */
			tegra_uart_platform[UART_IPC_ETNA].uart_ipc = 0;
			tegra_uart_platform[UART_IPC_ETNA].p.irq = ~0;
		} else {
			tegra_uart_platform[UART_IPC_ETNA].uart_ipc = 1;
			tegra_uart_platform[UART_IPC_ETNA].uart_wake_host = TEGRA_GPIO_PA0;
			tegra_uart_platform[UART_IPC_ETNA].uart_wake_request = TEGRA_GPIO_PF1;
			tegra_uart_platform[UART_IPC_ETNA].peer_register = mot_mdm_ctrl_peer_register;
		}
	}
	else if(machine_is_sunfire()) {
		andusb_plat.product_name = sunfire_dev;
		tegra_usb_fsg_platform.product = sunfire_dev;
		
		tegra_uart_platform[UART_IPC_SUNFIRE].uart_ipc = 1;
		tegra_uart_platform[UART_IPC_SUNFIRE].uart_wake_host = TEGRA_GPIO_PA0;
		tegra_uart_platform[UART_IPC_SUNFIRE].uart_wake_request = TEGRA_GPIO_PF1;
		tegra_uart_platform[UART_IPC_SUNFIRE].peer_register = mot_mdm_ctrl_peer_register;
	}
	else {
		andusb_plat.product_name = default_dev;
		tegra_usb_fsg_platform.product = default_dev;
	}

	if( (bi_powerup_reason() & PWRUP_FACTORY_CABLE) &&
	    (bi_powerup_reason() != PWRUP_INVALID) ){
#ifdef NEED_FACT_BUSY_HINT
		FactoryBusyHint(); //factory workaround no longer needed
#endif
	}
#endif

	mot_mdm_ctrl_init();

	(void) platform_driver_register(&cpcap_usb_connected_driver);

	mot_wlan_gpio_init();
	mot_sensors_init();

	printk("%s: registering i2c devices...\n", __func__);

	if(!(bi_powerup_reason() & PWRUP_BAREBOARD)) {
		printk("bus 0: %d devices\n", ARRAY_SIZE(tegra_i2c_bus0_board_info));
		i2c_register_board_info(0, tegra_i2c_bus0_board_info, 
								ARRAY_SIZE(tegra_i2c_bus0_board_info));
	}
	if (machine_is_etna() || machine_is_tegra_daytona() || machine_is_sunfire()) {
		printk("bus 2: %d devices\n", ARRAY_SIZE(tegra_i2c_bus2_board_info));
		i2c_register_board_info(2, tegra_i2c_bus2_board_info,
								ARRAY_SIZE(tegra_i2c_bus2_board_info));
	}
	printk("bus 3: %d devices\n", ARRAY_SIZE(tegra_i2c_bus3_board_info));
	i2c_register_board_info(3, tegra_i2c_bus3_board_info, 
							ARRAY_SIZE(tegra_i2c_bus3_board_info));

	if (machine_is_olympus()){
		/* console UART can be routed to headset jack by setting HSJ mux to 0*/
		short hsj_mux_gpio=1;

		if ( HWREV_TYPE_IS_DEBUG(system_rev) ){
			printk("%s: Enabling console on headset jack\n", __FUNCTION__);
			hsj_mux_gpio=0;
		}
		mot_set_hsj_mux( hsj_mux_gpio );
	}

	if (machine_is_olympus())
		mot_setup_spi_ipc();

	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
	pm_power_off = mot_system_power_off;
	tegra_setup_bluesleep();

	if (machine_is_olympus() && (HWREV_TYPE_IS_PORTABLE(system_rev) || HWREV_TYPE_IS_FINAL(system_rev)))
	{
		if (HWREV_REV(system_rev) >= HWREV_REV_1 && HWREV_REV(system_rev) < HWREV_REV_2)
		{
			// Olympus P1
			config_unused_pins(oly_unused_pins_p1, ARRAY_SIZE(oly_unused_pins_p1));
		}
		else if (HWREV_REV(system_rev) >= HWREV_REV_2 && HWREV_REV(system_rev) < HWREV_REV_3)
		{
			// Olympus P2
			config_unused_pins(oly_unused_pins_p2, ARRAY_SIZE(oly_unused_pins_p2));
		}
		else if (HWREV_REV(system_rev) >= HWREV_REV_3 || HWREV_TYPE_IS_FINAL(system_rev))
		{
			// Olympus P3 and newer
			config_unused_pins(oly_unused_pins_p3, ARRAY_SIZE(oly_unused_pins_p3));
		}
	}
	if (machine_is_etna() || machine_is_tegra_daytona() || machine_is_sunfire())
		// UTS tool support
		mot_keymap_update_init();
}

static void __init mot_fixup(struct machine_desc *desc, struct tag *tags,
                 char **cmdline, struct meminfo *mi)
{
    struct tag *t;
	int i;

    /* 
	 *	Dump some key ATAGs
     */
    for (t=tags; t->hdr.size; t = tag_next(t)) {
        if (t->hdr.tag == ATAG_CMDLINE) {
			printk("%s: atag_cmdline=\"%s\"\n", __func__, t->u.cmdline.cmdline);
        }
        else if (t->hdr.tag == ATAG_REVISION) {
			printk("%s: atag_revision=%x\n", __func__, t->u.revision.rev);
        }
        else if (t->hdr.tag == ATAG_SERIAL) {
			printk("%s: atag_serial=%x%x\n", __func__, t->u.serialnr.low, t->u.serialnr.high );
		}
        else if (t->hdr.tag == ATAG_MEM) {
            printk("%s: atag_mem.start=%d, atag_mem.size=%d\n", __func__, t->u.mem.start, t->u.mem.size);
        }
        else if (t->hdr.tag == ATAG_BLDEBUG) {
            printk("%s: powerup reason regs: INTS1=0x%4.4x  INT2=0x%4.4x  INTS2=0x%4.4x INT3=0x%4.4x  "
                   "PC2=0x%4.4x MEMA=0x%4.4x  ACCY=%d  UBOOT=%d\n", __func__, t->u.bldebug.ints1,
                   t->u.bldebug.int2, t->u.bldebug.ints2, t->u.bldebug.int3, t->u.bldebug.pc2,
                   t->u.bldebug.mema, t->u.bldebug.accy, t->u.bldebug.uboot);
        }
		else {
			printk("%s: ATAG %X\n", __func__, t->hdr.tag);
		}
    }

    /* 
	 *	Dump memory nodes
     */
	for (i=0; i<mi->nr_banks; i++) {
	    printk("%s: bank[%d]=%lx@%lx\n", __func__, i, mi->bank[i].size, mi->bank[i].start);
	}
}

MACHINE_START(OLYMPUS, "Olympus")

    .boot_params  = 0x00000100,
    .fixup        = mot_fixup,
    .map_io       = tegra_map_common_io,
    .init_irq     = tegra_init_irq,
    .init_machine = tegra_mot_init,
    .timer        = &tegra_timer,

MACHINE_END

MACHINE_START(ETNA, "Etna")

    .boot_params  = 0x00000100,
    .fixup        = mot_fixup,
    .map_io       = tegra_map_common_io,
    .init_irq     = tegra_init_irq,
    .init_machine = tegra_mot_init,
    .timer        = &tegra_timer,

MACHINE_END

MACHINE_START(SUNFIRE, "sunfire")

    .boot_params  = 0x00000100,
    .fixup        = mot_fixup,
    .map_io       = tegra_map_common_io,
    .init_irq     = tegra_init_irq,
    .init_machine = tegra_mot_init,
    .timer        = &tegra_timer,

MACHINE_END
MACHINE_START(TEGRA_DAYTONA, "Daytona")

    .boot_params  = 0x00000100,
    .fixup        = mot_fixup,
    .map_io       = tegra_map_common_io,
    .init_irq     = tegra_init_irq,
    .init_machine = tegra_mot_init,
    .timer        = &tegra_timer,

MACHINE_END


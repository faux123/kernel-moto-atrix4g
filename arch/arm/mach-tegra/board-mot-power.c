#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/pda_power.h>
#include <linux/io.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/rtc.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <asm/bootinfo.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/nvrm_linux.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/usb/android_composite.h>
#include <linux/gpio.h>
#include <linux/cpcap-accy.h>
#include <nvrm_module.h>
#include <nvrm_boot.h>
#include <nvodm_services.h>

#include "gpio-names.h"
#include "board.h"
#include "hwrev.h"

#include "board-mot.h"

extern void tegra_machine_restart(char mode, const char *cmd);
static int disable_rtc_alarms(struct device *dev, void *cnt);

void mot_system_power_off(void)
{
	// If there's external power, let's restart instead ...
	//  except for the case when phone was powered on with factory cable 
	//  and thus has to stay powered off after Turn-Off TCMD INKVSSW-994
	if (cpcap_misc_is_ext_power() &&
	   !((bi_powerup_reason() & PWRUP_FACTORY_CABLE) &&
	     (bi_powerup_reason() != PWRUP_INVALID)) )
	{
		printk("External power detected- rebooting\r\n");
		cpcap_misc_clear_power_handoff_info();
		tegra_machine_restart(0,"");
		while(1);
	}

	printk(KERN_ERR "%s(): Powering down system\n", __func__);

	// Disable RTC alarms to prevent unwanted powerups
	class_for_each_device(rtc_class, NULL, NULL, disable_rtc_alarms);

	// Disable powercut detection before power off
	cpcap_disable_powercut();

	// We need to set the WDI bit low to power down normally
	if (machine_is_olympus())
	{
		if (HWREV_TYPE_IS_PORTABLE(system_rev) &&
		    HWREV_REV(system_rev) >= HWREV_REV_1 &&
		    HWREV_REV(system_rev) <= HWREV_REV_1C )
		{
			// Olympus P1
			gpio_request(TEGRA_GPIO_PT4, "P1 WDI");
			gpio_direction_output(TEGRA_GPIO_PT4, 1);
			gpio_set_value(TEGRA_GPIO_PT4, 0);
		}
		else
		{
			// Olympus Mortable, P0, P2 and later
			gpio_request(TEGRA_GPIO_PV7, "P2 WDI");
			gpio_direction_output(TEGRA_GPIO_PV7, 1);
			gpio_set_value(TEGRA_GPIO_PV7, 0);
		}
	}
	else if (machine_is_etna())
	{
		if ( HWREV_TYPE_IS_BRASSBOARD(system_rev) &&
		     HWREV_REV(system_rev) == HWREV_REV_1 )
		{
			// Etna S1
			gpio_request(TEGRA_GPIO_PK4, "S1 WDI");
			gpio_direction_output(TEGRA_GPIO_PK4, 1);
			gpio_set_value(TEGRA_GPIO_PK4, 0);
		}
		else
		{
			// Etna S2, P1 and later
			gpio_request(TEGRA_GPIO_PT4, "S2 WDI");
			gpio_direction_output(TEGRA_GPIO_PT4, 1);
			gpio_set_value(TEGRA_GPIO_PT4, 0);

			/* Etna P1B and later has a gate on WDI */
			if ( machine_is_etna() &&
                             ( HWREV_TYPE_IS_FINAL(system_rev) ||
			       ( HWREV_TYPE_IS_PORTABLE(system_rev) &&
			         HWREV_REV(system_rev)  >= HWREV_REV_1B)))
				cpcap_set_wdigate(0);
		}
	}
	else if (machine_is_tegra_daytona())
	{
                        gpio_request(TEGRA_GPIO_PV7, "P2 WDI");
                        gpio_direction_output(TEGRA_GPIO_PV7, 1);
                        gpio_set_value(TEGRA_GPIO_PV7, 0);
	}
	else if (machine_is_sunfire())
	{
			gpio_request(TEGRA_GPIO_PT4, "S2 WDI");
			gpio_direction_output(TEGRA_GPIO_PT4, 1);
			gpio_set_value(TEGRA_GPIO_PT4, 0);
			cpcap_set_wdigate(0);
	}
	else
	{
		printk(KERN_ERR "Could not poweroff.  Unkown hardware revision: 0x%x\n", system_rev);
	}

	mdelay(500);
	printk("Power-off failed (Factory cable inserted?), rebooting\r\n");
	tegra_machine_restart(0,"");
}

#define CPCAP_REG_WLAN1_INDEX 18  /* The index of the VWLAN1 entry, which differs per hardware revision. */
#define CPCAP_REG_WLAN2_INDEX 19  /* The index of the VWLAN2 entry, which differs per hardware revision. */

struct cpcap_spi_init_data tegra_cpcap_spi_init[] = {
	{CPCAP_REG_S1C1,      0x4820},   /* Set SW1 to AMS/AMS 1.0v. */
	{CPCAP_REG_S2C1,      0x4830},   /* Set SW2 to AMS/AMS 1.2v. */
	{CPCAP_REG_S3C,       0x043d},   /* Set SW3 to Pulse Skip/PFM. */
	{CPCAP_REG_S4C1,      0x4930},   /* Set SW4 to PFM/PFM 1.2v. */
	{CPCAP_REG_S4C2,      0x301c},   /* Set SW4 down to 0.95v when secondary standby is asserted. */
	{CPCAP_REG_S5C,       0x0022},   /* Set SW5 to On/PFM. TODO: Once message LED macro is implemented
									    this can be On/Off. */
	{CPCAP_REG_S6C,       0x0000},   /* Set SW6 to Off/Off. */
	{CPCAP_REG_VCAMC,     0x0030},   /* Set VCAM to Off/Off. */
	{CPCAP_REG_VCSIC,     0x0017},   /* Set VCSI to AMS/Off 1.8v. */
	{CPCAP_REG_VDACC,     0x0000},
	{CPCAP_REG_VDIGC,     0x0000},
	{CPCAP_REG_VFUSEC,    0x0000},
	{CPCAP_REG_VHVIOC,    0x0002},   /* Set VHVIO to On/LP. */
	{CPCAP_REG_VSDIOC,    0x003A},   /* Set VSDIO to On/LP. */
	{CPCAP_REG_VPLLC,     0x0019},   /* Set VPLL to On/Off. */
	{CPCAP_REG_VRF1C,     0x0002},
	{CPCAP_REG_VRF2C,     0x0000},
	{CPCAP_REG_VRFREFC,   0x0000},
	{CPCAP_REG_VWLAN1C,   0x0005},   /* Set VWLAN1 to AMS/AMS 1.8v for P1 and P2.  For P3 and greater
									    The #define CPCAP_REG_WLAN1_INDEX above is used to modify this
									    based on hardware revision. */
	{CPCAP_REG_VWLAN2C,   0x008d},   /* Set VWLAN2 to On/On 3.3v for P1 and P2.  For P3 and greater
									    The #define CPCAP_REG_WLAN2_INDEX above is used to modify this
									    based on hardware revision. */
	{CPCAP_REG_VSIMC,     0x1e08},   /* Set VSIMCARD to AMS/Off 2.9v. */
	{CPCAP_REG_VVIBC,     0x000C},   /* Set to off 3.0v. */
	{CPCAP_REG_VUSBC,     0x004C},   /* Set VUSB to On/On */
	{CPCAP_REG_VUSBINT1C, 0x0000},
	{CPCAP_REG_USBC1,     0x1201},
	{CPCAP_REG_USBC2,     0xC058},
	{CPCAP_REG_USBC3,     0x7DFF},
	{CPCAP_REG_OWDC,      0x0003},	 /* one wire level shifter */
	{CPCAP_REG_PC1,       0x010A},   /* power cut is enabled, the timer is set to 312.5 ms */
	{CPCAP_REG_PC2,       0x0150},   /* power cut counter is enabled to prevent ambulance mode */
	{CPCAP_REG_CCCC2,     0x002B},   /* Enable coin cell charger and set charger voltage to 3.0 V 
					    Enable coulomb counter, enable dithering and set integration period to 250 mS*/
	{CPCAP_REG_ADCC1,     0x9000},   /* Set ADC_CLK to 3 MHZ
					    Disable leakage currents into channels between ADC conversions */
	{CPCAP_REG_ADCC2,     0x4136},   /* Disable TS_REF
					    Enable coin cell charger input to A/D 
					    Ignore ADTRIG signal
				            THERMBIAS pin is open circuit
					    Use B+ for ADC channel 4, Bank 0
					    Enable BATDETB comparator
					    Do not apply calibration offsets to ADC current readings */
	{CPCAP_REG_UCC1,      0x0000},   /* Clear UC Control 1 */
};

struct cpcap_leds tegra_cpcap_leds = {
        .button_led = {
                .button_reg = CPCAP_REG_KLC,
                .button_mask = 0x03FF,
                .button_on = 0xFFFF,
                .button_off = 0x0000,
        },
        .rgb_led = {
            .rgb_on = 0x0053, 
        },
};

extern struct platform_device cpcap_disp_button_led;
extern struct platform_device cpcap_rgb_led;

unsigned short cpcap_regulator_mode_values[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW1]      = 0x6800,  /* AMS/AMS Primary control via Macro */
	[CPCAP_SW2]      = 0x4804,  /* AMS/AMS Secondary control via Macro */
	[CPCAP_SW3]      = 0x043c,  /* Pulse Skip/PFM Secondary Standby */
	[CPCAP_SW4]      = 0x4909,  /* PFM/PFM Secondary Standby */
	[CPCAP_SW5]      = 0x0022,  /* On/PFM Secondary Standby.  TODO: On/Off Secondary Standby
								   Macro control when message LED Enabled */
	[CPCAP_VCAM]     = 0x0007,  /* AMS/Off Secondary Standby */
	[CPCAP_VCSI]     = 0x0007,  /* AMS/Off Secondary Standby */
	[CPCAP_VDAC]     = 0x0000,  /* Off/Off */
	[CPCAP_VDIG]     = 0x0000,  /* Off/Off */
	[CPCAP_VFUSE]    = 0x0000,  /* Off/Off */
	[CPCAP_VHVIO]    = 0x0002,  /* On/LP  Secondary Standby */
	[CPCAP_VSDIO]    = 0x0002,  /* On/LP  Secondary Standby */
	[CPCAP_VPLL]     = 0x0001,  /* On/Off Secondary Standby */
	[CPCAP_VRF1]     = 0x0000,  /* Off/Off */
	[CPCAP_VRF2]     = 0x0000,  /* Off/Off */
	[CPCAP_VRFREF]   = 0x0000,  /* Off/Off */
	[CPCAP_VWLAN1]   = 0x0005,  /* AMS/AMS P1 and P2 only, P3 and greater are reinitialized in
								   mot_setup_power(). */
	[CPCAP_VWLAN2]   = 0x000D,  /* On/On 3.3v (external pass) P1 and P2 only, P3 and greater
								   are reinitialized in mot_setup_power(). */
	[CPCAP_VSIM]     = 0x0000,  /* Off/Off */
	[CPCAP_VSIMCARD] = 0x1E00,  /* AMS/Off Secondary Standby */
	[CPCAP_VVIB]     = 0x0001,  /* On */
	[CPCAP_VUSB]     = 0x000C,  /* On/On */
	[CPCAP_VAUDIO]   = 0x0005,  /* On/LP Secondary Standby */
};

unsigned short cpcap_regulator_off_mode_values[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW1]      = 0x0000,
	[CPCAP_SW2]      = 0x0000,
	[CPCAP_SW3]      = 0x0000,
	[CPCAP_SW4]      = 0x0000,
	[CPCAP_SW5]      = 0x0000,
	[CPCAP_VCAM]     = 0x0000,
	[CPCAP_VCSI]     = 0x0000,
	[CPCAP_VDAC]     = 0x0000,
	[CPCAP_VDIG]     = 0x0000,
	[CPCAP_VFUSE]    = 0x0000,
	[CPCAP_VHVIO]    = 0x0000,
	[CPCAP_VSDIO]    = 0x0000,
	[CPCAP_VPLL]     = 0x0000,
	[CPCAP_VRF1]     = 0x0000,
	[CPCAP_VRF2]     = 0x0000,
	[CPCAP_VRFREF]   = 0x0000,
	[CPCAP_VWLAN1]   = 0x0000,
	[CPCAP_VWLAN2]   = 0x0000,
	[CPCAP_VSIM]     = 0x0000,
	[CPCAP_VSIMCARD] = 0x0000,
	[CPCAP_VVIB]     = 0x0000,
	[CPCAP_VUSB]     = 0x0000,
	[CPCAP_VAUDIO]   = 0x0000,
};

#define REGULATOR_CONSUMER(name, device) { .supply = name, .dev = device, }

struct regulator_consumer_supply cpcap_sw1_consumers[] = {
	REGULATOR_CONSUMER("sw1", NULL /* core */),
};

struct regulator_consumer_supply cpcap_sw2_consumers[] = {
	REGULATOR_CONSUMER("sw2", NULL /* core */),
};

struct regulator_consumer_supply cpcap_sw3_consumers[] = {
	REGULATOR_CONSUMER("sw3", NULL /* VIO */),
	REGULATOR_CONSUMER("odm-kit-vio", NULL),
};

struct regulator_consumer_supply cpcap_sw4_consumers[] = {
	REGULATOR_CONSUMER("sw4", NULL /* core */),
};

struct regulator_consumer_supply cpcap_sw5_consumers[] = {
	REGULATOR_SUPPLY("sw5", "button-backlight"),
	REGULATOR_SUPPLY("sw5", "notification-led"),
	REGULATOR_CONSUMER("odm-kit-sw5", NULL),
	REGULATOR_SUPPLY("sw5", NULL),
};

struct regulator_consumer_supply cpcap_vcam_consumers[] = {
	REGULATOR_CONSUMER("vcam", NULL /* cpcap_cam_device */),
	REGULATOR_CONSUMER("odm-kit-vcam", NULL),
};

struct regulator_consumer_supply cpcap_vhvio_consumers[] = {
	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
	REGULATOR_CONSUMER("odm-kit-vhvio", NULL),
#if 0
	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
	REGULATOR_CONSUMER("vhvio", NULL /* magnetometer */),
	REGULATOR_CONSUMER("vhvio", NULL /* light sensor */),
	REGULATOR_CONSUMER("vhvio", NULL /* accelerometer */),
	REGULATOR_CONSUMER("vhvio", NULL /* display */),
#endif
};

struct regulator_consumer_supply cpcap_vsdio_consumers[] = {
	REGULATOR_CONSUMER("vsdio", NULL),
	REGULATOR_CONSUMER("odm-kit-vsdio", NULL)
};

struct regulator_consumer_supply cpcap_vpll_consumers[] = {
	REGULATOR_CONSUMER("vpll", NULL),
	REGULATOR_CONSUMER("odm-kit-vpll", NULL),
};

#if 0
struct regulator_consumer_supply cpcap_vcsi_consumers[] = {
	REGULATOR_CONSUMER("vdds_dsi", &sholes_dss_device.dev),
};
#endif

struct regulator_consumer_supply cpcap_vcsi_consumers[] = {
	REGULATOR_CONSUMER("vcsi", NULL),
};

struct regulator_consumer_supply cpcap_vwlan1_consumers[] = {
	REGULATOR_CONSUMER("vwlan1", NULL),
	REGULATOR_CONSUMER("odm-kit-vwlan1", NULL),
};

struct regulator_consumer_supply cpcap_vwlan2_consumers[] = {
	REGULATOR_CONSUMER("vwlan2", NULL),
	/* Powers the tegra usb block, cannot be named vusb, since
	   this name already exists in regulator-cpcap.c. */
	REGULATOR_CONSUMER("vusb_modem_flash", NULL),
	REGULATOR_CONSUMER("vusb_modem_ipc", NULL),
	REGULATOR_CONSUMER("vhdmi", NULL),
	REGULATOR_CONSUMER("odm-kit-vwlan2", NULL),
};

struct regulator_consumer_supply cpcap_vsimcard_consumers[] = {
	REGULATOR_CONSUMER("vsimcard", NULL /* sd slot */),
	REGULATOR_CONSUMER("odm-kit-vsimcard", NULL),
};

struct regulator_consumer_supply cpcap_vvib_consumers[] = {
	REGULATOR_CONSUMER("vvib", NULL /* vibrator */),
};

struct regulator_consumer_supply cpcap_vaudio_consumers[] = {
	REGULATOR_CONSUMER("vaudio", NULL /* mic opamp */),
	REGULATOR_CONSUMER("odm-kit-vaudio", NULL /* mic opamp */),
};

static struct regulator_init_data cpcap_regulator[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW1] = {
		.constraints = {
			.min_uV			= 750000,
			.max_uV			= 1100000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                  REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw1_consumers),
		.consumer_supplies	= cpcap_sw1_consumers,
	},
	[CPCAP_SW2] = {
		.constraints = {
			.min_uV			= 900000,
			.max_uV			= 1200000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                  REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw2_consumers),
		.consumer_supplies	= cpcap_sw2_consumers,
	},
	[CPCAP_SW3] = {
		.constraints = {
			.min_uV			= 1350000,
			.max_uV			= 1875000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                 REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw3_consumers),
		.consumer_supplies	= cpcap_sw3_consumers,
	},
	[CPCAP_SW4] = {
		.constraints = {
			.min_uV			= 900000,
			.max_uV			= 1200000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
                                                  REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw4_consumers),
		.consumer_supplies	= cpcap_sw4_consumers,
	},
	[CPCAP_SW5] = {
		.constraints = {
			.min_uV			= 5050000,
			.max_uV			= 5050000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw5_consumers),
		.consumer_supplies	= cpcap_sw5_consumers,
	},
	[CPCAP_VCAM] = {
		.constraints = {
			.min_uV			= 2600000,
			.max_uV			= 2900000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcam_consumers),
		.consumer_supplies	= cpcap_vcam_consumers,
	},
	[CPCAP_VCSI] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 1200000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.boot_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcsi_consumers),
		.consumer_supplies	= cpcap_vcsi_consumers,
	},
	[CPCAP_VDAC] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 2500000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VDIG] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 1875000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VFUSE] = {
		.constraints = {
			.min_uV			= 1500000,
			.max_uV			= 3150000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VHVIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vhvio_consumers),
		.consumer_supplies	= cpcap_vhvio_consumers,
	},
	[CPCAP_VSDIO] = {
		.constraints = {
			.min_uV			= 1500000,
			.max_uV			= 3000000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsdio_consumers),
		.consumer_supplies	= cpcap_vsdio_consumers,
	},
	[CPCAP_VPLL] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_ops_mask		= 0,
			.apply_uV		= 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(cpcap_vpll_consumers),
		.consumer_supplies = cpcap_vpll_consumers,
	},
	[CPCAP_VRF1] = {
		.constraints = {
			.min_uV			= 2500000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VRF2] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VRFREF] = {
		.constraints = {
			.min_uV			= 2500000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VWLAN1] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1900000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan1_consumers),
		.consumer_supplies	= cpcap_vwlan1_consumers,
	},
	[CPCAP_VWLAN2] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 3300000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),
			.always_on		= 1,  /* This must remain on at all times. */
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan2_consumers),
		.consumer_supplies	= cpcap_vwlan2_consumers,
	},
	[CPCAP_VSIM] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VSIMCARD] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsimcard_consumers),
		.consumer_supplies	= cpcap_vsimcard_consumers,
	},
	[CPCAP_VVIB] = {
		.constraints = {
			.min_uV			= 1300000,
			.max_uV			= 3000000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vvib_consumers),
		.consumer_supplies	= cpcap_vvib_consumers,
	},
	[CPCAP_VUSB] = {
		.constraints = {
			.min_uV			= 3300000,
			.max_uV			= 3300000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
	},
	[CPCAP_VAUDIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_modes_mask	= (REGULATOR_MODE_NORMAL |
								  REGULATOR_MODE_STANDBY |
								  REGULATOR_MODE_IDLE),
			.valid_ops_mask		= REGULATOR_CHANGE_MODE,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vaudio_consumers),
		.consumer_supplies	= cpcap_vaudio_consumers,
	},
};

/* ADC conversion delays for battery V and I measurments taken in and out of TX burst  */
static struct cpcap_adc_ato cpcap_adc_ato = {
	.ato_in 		= 0x0300,
	.atox_in 		= 0x0000,
	.adc_ps_factor_in 	= 0x0200,
	.atox_ps_factor_in 	= 0x0000,
	.ato_out 		= 0x0780,
	.atox_out 		= 0x0000,
	.adc_ps_factor_out 	= 0x0600,
	.atox_ps_factor_out 	= 0x0000,
};

struct cpcap_platform_data tegra_cpcap_data = 
{
	.init = tegra_cpcap_spi_init,
	.init_len = ARRAY_SIZE(tegra_cpcap_spi_init),
	.leds = &tegra_cpcap_leds,
	.regulator_mode_values = cpcap_regulator_mode_values,
	.regulator_off_mode_values = cpcap_regulator_off_mode_values,
	.regulator_init = cpcap_regulator,
	.adc_ato = &cpcap_adc_ato,
	.wdt_disable = 0,
	.hwcfg = {
		(CPCAP_HWCFG0_SEC_STBY_SW3 |
		 CPCAP_HWCFG0_SEC_STBY_SW4 |
		 CPCAP_HWCFG0_SEC_STBY_SW5 |
		 CPCAP_HWCFG0_SEC_STBY_VAUDIO |
		 CPCAP_HWCFG0_SEC_STBY_VCAM |
		 CPCAP_HWCFG0_SEC_STBY_VCSI |
		 CPCAP_HWCFG0_SEC_STBY_VHVIO |
		 CPCAP_HWCFG0_SEC_STBY_VPLL |
		 CPCAP_HWCFG0_SEC_STBY_VSDIO),
		(CPCAP_HWCFG1_SEC_STBY_VWLAN1 |    /* WLAN1 may be reset in mot_setup_power(). */
		 CPCAP_HWCFG1_SEC_STBY_VSIMCARD)}
};

struct regulator_consumer_supply fixed_sdio_en_consumers[] = {
	REGULATOR_SUPPLY("vsdio_ext", NULL),
};

static struct regulator_init_data fixed_sdio_regulator = {
	.constraints = {
		.min_uV = 2800000,
		.max_uV = 2800000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(fixed_sdio_en_consumers),
	.consumer_supplies = fixed_sdio_en_consumers
};

static struct fixed_voltage_config fixed_sdio_config = {
	.supply_name = "sdio_en",
	.microvolts = 2800000,
	.gpio = TEGRA_GPIO_PF3,
	.enable_high = 1,
	.enabled_at_boot = 0,		/* Needs to be enabled on all hardware prior to P3
								   This is handled below. */
	.init_data = &fixed_sdio_regulator,
};

static struct platform_device fixed_regulator_devices[] = {
	{
		.name = "reg-fixed-voltage",
		.id = 1,
		.dev = {
			.platform_data = &fixed_sdio_config,
		},
	},
};

struct spi_board_info tegra_spi_devices[] __initdata = {
#ifdef CONFIG_MFD_CPCAP
    {
        .modalias = "cpcap",
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        .max_speed_hz = 8000000,
        .controller_data = &tegra_cpcap_data,
        .irq = INT_EXTERNAL_PMU,
    },
#elif defined CONFIG_SPI_SPIDEV
    {
        .modalias = "spidev",
        .bus_num = 1,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        .max_speed_hz = 18000000,
        .platform_data = NULL,
        .irq = 0,
    },
#endif     
};


struct cpcap_usb_connected_data {
	NvOdmServicesGpioHandle h_gpio;
	NvOdmGpioPinHandle h_pin;
	NvU32 port;
	NvU32 pin;
	enum cpcap_accy accy;
};

static int cpcap_usb_connected_probe(struct platform_device *pdev)
{
	struct cpcap_usb_connected_data *data;
	struct cpcap_accy_platform_data *pdata = pdev->dev.platform_data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if(!data)
		return -ENOMEM;

	data->accy = pdata->accy;

	/* Configure CPCAP-AP20 USB Mux to AP20 */
	data->port = NVODM_PORT('v');
	data->pin = 6;
	data->h_gpio = NvOdmGpioOpen();
	data->h_pin = NvOdmGpioAcquirePinHandle(data->h_gpio, data->port, data->pin);
	NvOdmGpioConfig(data->h_gpio, data->h_pin, NvOdmGpioPinMode_Output);
	NvOdmGpioSetState(data->h_gpio, data->h_pin, 0x1);

	platform_set_drvdata(pdev, data);

	/* when the phone is the host do not start the gadget driver */
	if((data->accy == CPCAP_ACCY_USB) || (data->accy == CPCAP_ACCY_FACTORY)) {
#ifdef CONFIG_USB_TEGRA_OTG
		tegra_otg_set_mode(0);
#endif
		android_usb_set_connected(1);
	}
	if(data->accy == CPCAP_ACCY_USB_DEVICE) {
#ifdef CONFIG_USB_TEGRA_OTG
		tegra_otg_set_mode(1);
#endif
	}

    return 0;
}

static int cpcap_usb_connected_remove(struct platform_device *pdev)
{
	struct cpcap_usb_connected_data *data = platform_get_drvdata(pdev);

	/* Configure CPCAP-AP20 USB Mux to CPCAP */
	NvOdmGpioSetState(data->h_gpio, data->h_pin, 0x0);
	NvOdmGpioReleasePinHandle(data->h_gpio, data->h_pin);
	NvOdmGpioClose(data->h_gpio);

	if((data->accy == CPCAP_ACCY_USB) || (data->accy == CPCAP_ACCY_FACTORY))
		android_usb_set_connected(0);

#ifdef CONFIG_USB_TEGRA_OTG
	tegra_otg_set_mode(2);
#endif

	kfree(data);

        return 0;
}



struct platform_driver cpcap_usb_connected_driver = {
        .probe          = cpcap_usb_connected_probe,
        .remove         = cpcap_usb_connected_remove,
        .driver         = {
                .name   = "cpcap_usb_connected",
                .owner  = THIS_MODULE,
    },
};

#ifdef CONFIG_REGULATOR_VIRTUAL_CONSUMER
static struct platform_device cpcap_reg_virt_vcam =
{
    .name = "reg-virt-vcam",
    .id   = -1,
    .dev =
    {
        .platform_data = "vcam",
    },
};
static struct platform_device cpcap_reg_virt_vcsi =
{
    .name = "reg-virt-vcsi",
    .id   = -1,
    .dev =
    {
        .platform_data = "vcsi",
    },
};
static struct platform_device cpcap_reg_virt_vcsi_2 =
{
    .name = "reg-virt-vcsi_2",
    .id   = -1,
    .dev =
    {
        .platform_data = "vcsi",
    },
};
static struct platform_device cpcap_reg_virt_sw5 =
{
    .name = "reg-virt-sw5",
    .id   = -1,
    .dev =
    {
        .platform_data = "sw5",
    },
};
#endif

void mot_setup_power(void)
{
	unsigned int i;
	int error;

	/* CPCAP standby lines connected to CPCAP GPIOs on Etna P1B & Olympus P2 */
	if ( HWREV_TYPE_IS_FINAL(system_rev) ||
	     (machine_is_etna() &&
	       HWREV_TYPE_IS_PORTABLE(system_rev) &&
	       (HWREV_REV(system_rev)  >= HWREV_REV_1B))  ||
	     (machine_is_olympus() &&
	       HWREV_TYPE_IS_PORTABLE(system_rev) &&
	       (HWREV_REV(system_rev)  >= HWREV_REV_2)) ||
		  machine_is_tegra_daytona() || machine_is_sunfire()) {
		tegra_cpcap_data.hwcfg[1] |= CPCAP_HWCFG1_STBY_GPIO;
	}
	/* Safe to turn off SW4 & VLWAN2 in standby on Etna P2 */
	if ((machine_is_etna() &&
		(HWREV_TYPE_IS_FINAL(system_rev) ||
		 (HWREV_TYPE_IS_PORTABLE(system_rev) &&
		  (HWREV_REV(system_rev)  >= HWREV_REV_2)))) || 
		  machine_is_tegra_daytona() || machine_is_sunfire()) {
		tegra_cpcap_data.hwcfg[1] |= (CPCAP_HWCFG1_SEC_STBY_VWLAN2);
	}

	/* For Olympus P3 the following is done:
	 * 1. VWLAN2 is  shutdown in standby by the CPCAP uC.
	 * 2. VWLAN1 is shutdown all of the time.
	 */
	if (machine_is_olympus() &&
		(HWREV_TYPE_IS_FINAL(system_rev) ||
		 (HWREV_TYPE_IS_PORTABLE(system_rev) &&
		  (HWREV_REV(system_rev) >= HWREV_REV_3)))) {
		pr_info("Detected P3 Olympus hardware.\n");
		/* This is not the best, but it works until P1 and P2 no longer
		   need to be supported. */
		if (tegra_cpcap_spi_init[CPCAP_REG_WLAN1_INDEX].reg == CPCAP_REG_VWLAN1C) {
			tegra_cpcap_spi_init[CPCAP_REG_WLAN1_INDEX].data = 0x0000; /* Set VWLAN1 to Off/Off */
		}
		else {
			printk(KERN_WARNING "Unable to set WLAN1 to the correct settings for P3 and greater hardware.\n");
		}
		if (tegra_cpcap_spi_init[CPCAP_REG_WLAN2_INDEX].reg == CPCAP_REG_VWLAN2C) {
			tegra_cpcap_spi_init[CPCAP_REG_WLAN2_INDEX].data =  0x008d; /* Set VWLAN2 to On/LP 3.3v. */
		}
		else {
			printk(KERN_WARNING "Unable to set WLAN2 to the correct settings for P3 and greater hardware.\n");
		}
		tegra_cpcap_data.hwcfg[1] |= CPCAP_HWCFG1_SEC_STBY_VWLAN2;
		tegra_cpcap_data.hwcfg[1] &= ~CPCAP_HWCFG1_SEC_STBY_VWLAN1;
		cpcap_regulator_mode_values[CPCAP_VWLAN1] = 0x0000; /* Off/Off */
		cpcap_regulator_mode_values[CPCAP_VWLAN2] = 0x000d; /* On/LP 3.3v Secondary Standby */
	}
	else {
		/* Currently only Olympus P3 or greater can handle turning off the
		   external SD card. */
		fixed_sdio_config.enabled_at_boot = 1;
	}

	if ((machine_is_etna() &&
	     (HWREV_TYPE_IS_FINAL(system_rev) ||
	        (HWREV_TYPE_IS_PORTABLE(system_rev) &&
	          (HWREV_REV(system_rev)  >= HWREV_REV_1)))) || machine_is_sunfire()) {
		printk(KERN_INFO "%s: updating Etna button backlight for portable\n",
		       __func__);
		tegra_cpcap_leds.button_led.button_reg = CPCAP_REG_ADLC;
	}

	/* For all machine types, disable watchdog when HWREV is debug, brassboard or mortable */
	if (HWREV_TYPE_IS_DEBUG(system_rev) || HWREV_TYPE_IS_BRASSBOARD(system_rev) ||
	    HWREV_TYPE_IS_MORTABLE(system_rev) ){
		tegra_cpcap_data.wdt_disable = 1;
	}

	spi_register_board_info(tegra_spi_devices, ARRAY_SIZE(tegra_spi_devices));

	for (i = 0; i < sizeof(fixed_regulator_devices)/sizeof(fixed_regulator_devices[0]); i++) {
		error = platform_device_register(&fixed_regulator_devices[i]);
		pr_info("Registered reg-fixed-voltage: %d result: %d\n", i, error);
	}

#ifdef CONFIG_REGULATOR_VIRTUAL_CONSUMER
	(void) platform_device_register(&cpcap_reg_virt_vcam);
	(void) platform_device_register(&cpcap_reg_virt_vcsi);
	(void) platform_device_register(&cpcap_reg_virt_vcsi_2);
	(void) platform_device_register(&cpcap_reg_virt_sw5);
#endif
}

static int disable_rtc_alarms(struct device *dev, void *data)
{
	return (rtc_alarm_irq_enable((struct rtc_device *)dev, 0));
}

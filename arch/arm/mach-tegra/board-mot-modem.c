#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/qtouch_obp_ts.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/mdm_ctrl.h>

#ifdef CONFIG_SPI_SLAVE
#include <linux/spi/spi_slave.h>
#include <linux/spi/mdm6600_spi.h>
#endif

#include "gpio-names.h"
#include "board-mot.h"

struct mdm_ctrl_peer_entry
{
	void (*startup)(void*);
	void (*shutdown)(void*);
	void* context;
};

#define MDM_CTRL_MAX_PEERS 8
static struct mdm_ctrl_peer_entry mdm_ctrl_peer[MDM_CTRL_MAX_PEERS];
static unsigned int mdm_ctrl_peers = 0;
static bool mdm_ctrl_state = true;

int mot_mdm_ctrl_peer_register(void (*peer_startup)(void*),
                               void (*peer_shutdown)(void*),
                               void* peer_context)
{
	if (mdm_ctrl_peers >= MDM_CTRL_MAX_PEERS)
		return -ENOMEM;

	mdm_ctrl_peer[mdm_ctrl_peers].startup = peer_startup;
	mdm_ctrl_peer[mdm_ctrl_peers].shutdown = peer_shutdown;
	mdm_ctrl_peer[mdm_ctrl_peers].context = peer_context;
	mdm_ctrl_peers++;

	return 0;
}

static void mot_on_bp_startup(void)
{
	int i;

	if (mdm_ctrl_state)
		return;

	for (i = 0; i < mdm_ctrl_peers; i++) {
		if (mdm_ctrl_peer[i].startup)
			mdm_ctrl_peer[i].startup(mdm_ctrl_peer[i].context);
	}

	mdm_ctrl_state = true;
}

static void mot_on_bp_shutdown(void)
{
	int i;

	if (!mdm_ctrl_state)
		return;

	for (i = 0; i < mdm_ctrl_peers; i++) {
		if (mdm_ctrl_peer[i].shutdown)
			mdm_ctrl_peer[i].shutdown(mdm_ctrl_peer[i].context);
	}

	mdm_ctrl_state = false;
}


#define AP_STATUS0_GPIO TEGRA_GPIO_PL0
#define AP_STATUS1_GPIO TEGRA_GPIO_PL3
#define AP_STATUS2_GPIO TEGRA_GPIO_PD5
#define BP_STATUS0_GPIO TEGRA_GPIO_PM0
#define BP_STATUS1_GPIO TEGRA_GPIO_PM1
#define BP_STATUS2_GPIO TEGRA_GPIO_PT0
#define BP_RESIN_GPIO   TEGRA_GPIO_PV1
#define BP_PSHOLD_GPIO  TEGRA_GPIO_PV1
#define BP_RESOUT_GPIO  TEGRA_GPIO_PV2
#define BP_BYPASSS_GPIO TEGRA_GPIO_PE4
#define BP_PWRON_GPIO   TEGRA_GPIO_PV0
#define BP_FLASH1_GPIO  TEGRA_GPIO_PF1
#define BP_FLASH2_GPIO  TEGRA_GPIO_PA0

static struct mdm_ctrl_platform_data mdm_ctrl_platform_data;
static struct platform_device mdm_ctrl_platform_device = {
	.name = MDM_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &mdm_ctrl_platform_data,
	},
};

static const char mdm_ctrl_usb_regulator[] = "vusb_modem_flash";

int __init mot_mdm_ctrl_init(void)
{
	int value;

	if ((machine_is_olympus() &&
	     !(HWREV_TYPE_IS_MORTABLE(system_rev) &&
	       HWREV_REV(system_rev) <= HWREV_REV_1)) ||
	    (machine_is_etna() &&
	     HWREV_TYPE_IS_PORTABLE(system_rev) &&
	     HWREV_REV(system_rev) >= HWREV_REV_2C) || 
	    machine_is_tegra_daytona() || machine_is_sunfire()) {
		mdm_ctrl_platform_data.on_bp_startup = mot_on_bp_startup;
		mdm_ctrl_platform_data.on_bp_shutdown = mot_on_bp_shutdown;

		if (machine_is_olympus() &&
			(HWREV_TYPE_IS_FINAL(system_rev) ||
			(HWREV_TYPE_IS_PORTABLE(system_rev) &&
			(HWREV_REV(system_rev) >= HWREV_REV_3)))) {
			mdm_ctrl_platform_data.usb_regulator = mdm_ctrl_usb_regulator;
		}

		mdm_ctrl_platform_data.ap_status0_gpio = AP_STATUS0_GPIO;
		mdm_ctrl_platform_data.ap_status1_gpio = AP_STATUS1_GPIO;
		mdm_ctrl_platform_data.ap_status2_gpio = AP_STATUS2_GPIO;
		mdm_ctrl_platform_data.bp_status0_gpio = BP_STATUS0_GPIO;
		mdm_ctrl_platform_data.bp_status1_gpio = BP_STATUS1_GPIO;
		mdm_ctrl_platform_data.bp_status2_gpio = BP_STATUS2_GPIO;
		mdm_ctrl_platform_data.bp_pshold_gpio = MDM_GPIO_INVALID;
		mdm_ctrl_platform_data.bp_resin_gpio = BP_RESIN_GPIO;
		mdm_ctrl_platform_data.bp_resout_gpio = BP_RESOUT_GPIO;
		mdm_ctrl_platform_data.bp_bypass_gpio = BP_BYPASSS_GPIO;
		mdm_ctrl_platform_data.bp_pwron_gpio = BP_PWRON_GPIO;
		mdm_ctrl_platform_data.bp_flash_en1_gpio = BP_FLASH1_GPIO;
		mdm_ctrl_platform_data.bp_flash_en2_gpio = BP_FLASH2_GPIO;
		
		mdm_ctrl_platform_data.bp_status0_gpio_irq_type =
						IRQ_TYPE_EDGE_BOTH;
		mdm_ctrl_platform_data.bp_status1_gpio_irq_type =
						IRQ_TYPE_EDGE_BOTH;
		mdm_ctrl_platform_data.bp_status2_gpio_irq_type =
						IRQ_TYPE_EDGE_BOTH;

		/*
		 * Tegra doesn't support edge triggering on GPIOs that can wake
		 * the system from deep sleep.  If the BP goes down while AP is
		 * sleeping, the AP won't notice.  So we must level trigger and
		 * toggle it in the driver.  Setting it to "high" will cause
		 * the interrupt to fire immediately so that the driver's state
		 * is accurate.
		 */
		mdm_ctrl_platform_data.bp_resout_gpio_irq_type =
						IRQ_TYPE_LEVEL_HIGH;

		gpio_request(AP_STATUS0_GPIO, "AP Status 0");
		value = gpio_get_value(AP_STATUS0_GPIO);
		gpio_direction_output(AP_STATUS0_GPIO, value);

		gpio_request(AP_STATUS1_GPIO, "AP Status 1");
		value = gpio_get_value(AP_STATUS1_GPIO);
		gpio_direction_output(AP_STATUS1_GPIO, value);

		gpio_request(AP_STATUS2_GPIO, "AP Status 2");
		value = gpio_get_value(AP_STATUS2_GPIO);
		gpio_direction_output(AP_STATUS2_GPIO, value);

		gpio_request(BP_STATUS0_GPIO, "BP Status 0");
		gpio_direction_input(BP_STATUS0_GPIO);

		gpio_request(BP_STATUS1_GPIO, "BP Status 1");
		gpio_direction_input(BP_STATUS1_GPIO);

		gpio_request(BP_STATUS2_GPIO, "BP Status 2");
		gpio_direction_input(BP_STATUS2_GPIO);

		gpio_request(BP_RESIN_GPIO, "BP Reset");
		value = gpio_get_value(BP_RESIN_GPIO);
		gpio_direction_output(BP_RESIN_GPIO, value);

		gpio_request(BP_RESOUT_GPIO, "BP Reset Output");
		gpio_direction_input(BP_RESOUT_GPIO);

		gpio_request(BP_PWRON_GPIO, "BP Power On");
		value = gpio_get_value(BP_PWRON_GPIO);
		gpio_direction_output(BP_PWRON_GPIO, value);

		return platform_device_register(&mdm_ctrl_platform_device);
	}

	printk (KERN_INFO "%s: disabling mdm_ctrl for unsupported "
		"hardware\n", __FUNCTION__);

	return -ENODEV;
}

#ifdef CONFIG_SPI_SLAVE
struct mdm6600_spi_platform_data mdm6600_spi_platform_data = {
	.gpio_mrdy = TEGRA_GPIO_PL1,
	.gpio_srdy = TEGRA_GPIO_PF2,
	.peer_register = mot_mdm_ctrl_peer_register,
};

struct spi_slave_board_info tegra_spi_slave_devices[] __initdata = {
{
	.modalias = "mdm6600_spi",
	.bus_num = 0,
	.chip_select = 0,
	.mode = SPI_MODE_0,
	.max_speed_hz = 26000000,
	.platform_data = &mdm6600_spi_platform_data,
	.irq = 0,
    },
};

void mot_setup_spi_ipc()
{
	spi_slave_register_board_info(tegra_spi_slave_devices,
				ARRAY_SIZE(tegra_spi_slave_devices));
}
#else
struct mdm6600_spi_platform_data mdm6600_spi_platform_data;
void mot_setup_spi_ipc()
{}
#endif



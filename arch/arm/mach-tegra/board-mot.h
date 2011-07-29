#ifndef __MACH_TEGRA_BOARD_MOT_H
#define __MACH_TEGRA_BOARD_MOT_H

#include <mach/serial.h>
#include <linux/i2c.h>
#include <linux/i2c/akm8975.h>
#include "hwrev.h"

extern struct kxtf9_platform_data kxtf9_data;
extern struct akm8975_platform_data akm8975_data;
extern struct isl29030_platform_data isl29030_als_ir_data_Olympus;
extern struct isl29030_platform_data isl29030_als_ir_data_Etna;
extern struct isl29030_platform_data isl29030_als_ir_data_Daytona;
extern struct isl29030_platform_data isl29030_als_ir_data_Sunfire;
extern struct lm3532_platform_data lm3532_pdata;
extern struct qtouch_ts_platform_data ts_platform_olympus_m_1;
extern struct cpcap_platform_data tegra_cpcap_data;
extern struct cpcap_leds tegra_cpcap_leds;
extern struct platform_driver cpcap_usb_connected_driver;
extern struct l3g4200d_platform_data tegra_gyro_pdata;

extern void __init mot_setup_power(void);
extern void __init mot_setup_gadget(void);
extern void __init mot_setup_lights(struct i2c_board_info *info);
extern void __init mot_setup_touch(struct i2c_board_info *info);

extern int mot_mdm_ctrl_shutdown(void);
extern int mot_mdm_ctrl_peer_register(void (*)(void*),
                                      void (*)(void*),
                                      void*);
extern int __init mot_wlan_init(void);
extern int __init mot_modem_init(void);
#ifdef CONFIG_MOT_WIMAX
extern int __init mot_wimax_gpio_init(void);
extern int bcm_wimax_status_register(
	void (*callback)(void *dev_id), void *dev_id);
#endif
extern void __init mot_hdmi_init(void);

extern void __init mot_sensors_init(void);

extern int __init mot_nvodmcam_init(void);

//extern void sdio_rail_init(void);

extern void mot_system_power_off(void);
extern void mot_set_hsj_mux(short hsj_mux_gpio);
extern void mot_sec_init(void);
extern void mot_tcmd_init(void);
extern int apanic_mmc_init(void);

extern void tegra_otg_set_mode(int);
extern void sdhci_tegra_wlan_detect(void);
extern void mot_setup_spi_ipc(void);

extern void mot_keymap_update_init(void);

extern struct tegra_serial_platform_data tegra_uart_platform[];

extern void cpcap_set_dock_switch(int state);

#define	BACKLIGHT_DEV		0
#define	TOUCHSCREEN_DEV		1

#define SERIAL_NUMBER_STRING_LEN 17

#define NVODM_PORT(x) ((x) - 'a')

#define TOUCH_GPIO_RESET	TEGRA_GPIO_PF4
#define TOUCH_GPIO_INTR		TEGRA_GPIO_PF5

#ifndef PROX_INT_GPIO
#define	PROX_INT_GPIO	TEGRA_GPIO_PE1
#endif

#define UART_IPC_OLYMPUS	3
#define UART_IPC_ETNA		3
#define UART_IPC_SUNFIRE		3
#define UART_IPC_DAYTONA		3
#endif

/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/list.h>

#ifndef _LINUX_MACH_TD_BP_CTRL_H__
#define _LINUX_MACH_TD_BP_CTRL_H__

/*
 * define GPIO numbers...
 */
#define GPIO_TD_BP_WDI		TEGRA_GPIO_PV2
#define GPIO_TD_BP_ON_KEY 	TEGRA_GPIO_PV0
#define GPIO_TD_BP_RESET  	TEGRA_GPIO_PV1
#define GPIO_TD_BP_FLASH_EN TEGRA_GPIO_PE7

struct mot_td_bpp_pdata {
/*
 * output GPIOs defined here...
 */
  unsigned int gpio_bp_on_key;
  unsigned int gpio_bp_wdi;
  unsigned int gpio_bp_pmu_shutdown;
  unsigned int gpio_bp_reset;
  unsigned int gpio_bp_flash_en;
};

/*
 * Interface set to other modules like IPC in kernel space
 */
struct td_bp_ext_interface{
  const char * name;				/*must be provided*/
  /*called before bp ctrl driver power on BP, call may sleep*/
  void (*prior_on)(void);
  /*called after bp ctrl driver power on BP successfully (wdi=1),call may sleep*/
  void (*post_on)(void);
  /*called before bp ctrl driver shuts down BP, call may sleep*/
  void (*prior_shutdown)(void);
  /*called after bp ctrl driver shuts down BP (wdi=0), call may sleep*/
  void (*post_shutdown)(void);
  /*called when a high-to-low interrupt triggered on BP wdi,
   * call in interrupt handler
   * NEED TO BE IMPLEMENTED AS SWIFT AS POSSIBLY
   * */
  void (*on_wdi_interrupt)(void);
  /*called when user trigger TD modem in TAT mode,
   * all prior_tat_mode hooks will be called just
   * before We reset BP for TAT mode.
   * */
  void (*prior_tat_mode)(void);
  /*
   * Register does not need to init list'next, bp driver will do that
   * when register an external interface.
   */
  struct list_head	list;

};
/*
 * Register an external interface to td bp ctrl driver
 * call may sleep.
 * @intf: pointer to an alloc-ed (by caller) external interface
 * @return -
 * 0  success,
 * -EINVAL fail. Null name.
 */
extern int td_bp_register_ext_interface(struct td_bp_ext_interface* intf);

/*
 * Unregister an external interface to td bp ctrl driver
 * call may sleep.
 * td bp ctrl driver won't free and memory of interface.
 * @name: name of register
 * @return -
 * 0 success,
 * -EINVAL: name not found
 */
extern int td_bp_unregister_ext_interface_by_name(const char * name);



/*
 *IOCTL CMD Definitions
 */

#define TDBP_IO 0xFF

#define TDBP_IOCTL_BP_RESET        _IOWR(TDBP_IO, 0x01, char)
#define TDBP_IOCTL_BP_FAIL_RESET   _IOWR(TDBP_IO, 0x02, char)

#if 0
#define TDBP_IOCTL_KEY             _IOWR(TDBP_IO, 0x00, char)
#define TDBP_IOCTL_BP_RESET        _IOWR(TDBP_IO, 0x01, char)
#define TDBP_IOCTL_SWTICH          _IOWR(TDBP_IO, 0x02, char)
#define TD_TCMD_GPIO_MODEM_LOG_EN  TEGRA_GPIO_PE4
#define TD_TCMD_AP_TO_BP_FLASH_EN  TEGRA_GPIO_PE7
#define BP_USB_PWR_EN              TEGRA_GPIO_PD5
#define BP_RESIN_GPIO              TEGRA_GPIO_PV1
#define USB_DATA_EN                TEGRA_GPIO_PV6
#define BP_PWRON                   TEGRA_GPIO_PV0

#endif

#endif  /* _LINUX_MACH_TD_BP_CTRL_H__ */


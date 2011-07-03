/* include/linux/vib-omap-pwm.h
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
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

#ifndef __VIB_PWM_H
#define __VIB_PWM_H
#define VIB_PWM_NAME "vib-pwm"

struct vib_pwm_platform_data {
        unsigned gpio;
        int max_timeout;
        u8 active_low;
	int initial_vibrate;
	unsigned int freq;
	unsigned int duty_cycle;
	int (*init) (void);
	void (*exit) (void);
	int (*power_on) (int freq, int duty_cycle);
	int (*power_off) (void);
	char *device_name;
};

void pwm_vibrator_haptic_fire(int value);

#endif

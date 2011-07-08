/*
 * Copyright (C) 2011 Motorola, Inc.
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

#ifndef __MACH_NVODMCAM_H__
#define __MACH_NVODMCAM_H__

#define NVODMCAM_DRIVER_NAME "nvodmcam"

enum nvodmcam_sensor_t {
	OV5650 = 1,
	AP8140,
	OV7692,
	OV7739,
	SOC380
};

struct nvodmcam_gpio_cfg {
	int num;    /* gpio number */
	int state;  /* gpio state for camera on */
};

struct nvodmcam_campwr_data {
	enum   nvodmcam_sensor_t sensor;
	struct nvodmcam_gpio_cfg cam_pd;
	struct nvodmcam_gpio_cfg cam_rs;
	struct nvodmcam_gpio_cfg flash_rs;
};

#define NVODMCAM_MAX_CAMERAS 2

struct nvodmcam_platform_data {
	unsigned int num_cameras;
	struct nvodmcam_campwr_data camera[NVODMCAM_MAX_CAMERAS];
};

#endif

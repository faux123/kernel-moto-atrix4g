/*
 * mach/apanic.h
 *
 * Common header for various apanic implementations.
 *
 * Copyright (c) 2010, Motorola.
 *
 * Authors:
 *	Russ W. Knize	<russ.knize@motorola.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */
#ifndef __LINUX_MACH_APANIC_H__
#define __LINUX_MACH_APANIC_H__

struct apanic_mmc_platform_data {
	int id;
	sector_t start_sector;
	sector_t sectors;
	size_t sector_size;
};

#endif

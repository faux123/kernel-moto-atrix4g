/*
 * arch/arm/mach-tegra/include/mach/sdhci-simple.h
 *
 * Simplified SDHCI driver platform data definitions
 *
 * Copyright (C) 2010 Motorola
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __MACH_TEGRA_SDHCI_SIMPLE_H
#define __MACH_TEGRA_SDHCI_SIMPLE_H

#include <mach/sdhci.h>

struct tegra_sdhci_simple_platform_data {
	struct tegra_sdhci_platform_data *sdhci_pdata;
        struct resource                  *resource;
        int                              num_resources;
        char                             *clk_dev_name;
};


#endif

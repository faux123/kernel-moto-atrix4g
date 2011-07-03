/*
 * include/linux/tegra_devices.h
 *
 * Definitions for platform devices and related flags NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 NVIDIA Corporation
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

#ifndef _LINUX_DEVICE_TEGRA_H
#define _LINUX_DEVICE_TEGRA_H

#ifndef NV_DEBUG
#if defined(CONFIG_MACH_TEGRA_GENERIC_DEBUG)
#define NV_DEBUG 1
#else
#define NV_DEBUG 0
#endif
#endif

#include "nvcommon.h"
#include "nvrm_gpio.h"
#include "nvddk_usbphy.h"
#include "nvodm_query.h"
#include "nvodm_query_pinmux.h"

/* Platform data for EHCI HCD driver */
struct tegra_hcd_platform_data {
	NvU32			instance;
	NvRmGpioPinHandle	hGpioIDpin;
	const NvOdmUsbProperty	*pUsbProperty;
	NvU32			powerClientId;
	NvU32			vBusPowerRail;
	/* USB PHY power rail. Tegra has integrated UTMI (USB transciver
	 * macrocell interface) PHY on USB controllers 0 and 2. These 2 PHYs
	 * have its own rails.
	 */
	NvU32			phyPowerRail;
	NvDdkUsbPhyHandle	hUsbPhy;
        struct work_struct      work;
};

/* Platform data for USB OTG driver */
struct tegra_otg_platform_data {
	NvU32			instance;
	const NvOdmUsbProperty	*usb_property;
};

/* Platfrom data for I2C bus driver */
struct tegra_i2c_platform_data {
	NvU32		IoModuleID;
	NvU32		Instance;
	NvU32		ClockInKHz;
	NvOdmI2cPinMap	PinMuxConfig;
};

/* Platfrom data for W1 bus driver */
struct tegra_w1_platform_data {
	NvU32		Instance;
	NvOdmOwrPinMap	PinMuxConfig;
};


/* Platfrom data for SPI bus driver */
struct tegra_spi_platform_data {
	NvU32 IoModuleID;
	NvU32 Instance;
	NvU32 PinMuxConfig;
};

struct tegra_sdio_platform_data {
	NvU32 StartOffset; /* start sector offset to MBR for the card */
	NvRmGpioPinHandle	hResetPin;
	NvRmGpioPinHandle	hPowerPin;
	NvRmGpioHandle		hGpio;
	int			index;
	int			isWifi;
};

#ifdef CONFIG_DEVNVMAP
int nvmap_add_carveout_heap(unsigned long base, size_t size,
	const char *name, unsigned int bitmask);
#endif

#endif

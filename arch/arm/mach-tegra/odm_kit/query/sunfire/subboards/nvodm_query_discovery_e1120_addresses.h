/*
 * arch/arm/mach-tegra/odm_kit/query/whistler/subboards/nvodm_query_discovery_e1120_addresses.h
 *
 * Specifies the peripheral connectivity database peripheral entries for the E1120
 * development system motherboard
 *
 * Copyright (c) 2010 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include "pmu/max8907b/max8907b_supply_info_table.h"

static const NvOdmIoAddress s_enc28j60EthernetAddresses[] =
{
    { NvOdmIoModule_Spi, 1, 1 ,0 },
    { NvOdmIoModule_Gpio, (NvU32)'c'-'a', 1 ,0 }
};

static const NvOdmIoAddress s_SdioAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x0, 0x0 ,0 },
    { NvOdmIoModule_Sdio, 0x2, 0x0 ,0 },
    { NvOdmIoModule_Sdio, 0x3, 0x0 ,0 },
//    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO12 ,0 }, /* VDDIO_SDIO -> VOUT12 */
//    { NvOdmIoModule_Vdd, 0x00, Max8907bPmuSupply_LDO5 ,0 } /* VCORE_MMC -> VOUT05 */
};

static const NvOdmIoAddress s_VibAddresses[] =
{
//    { NvOdmIoModule_Vdd, 0x0, Max8907bPmuSupply_LDO16,0 },
};


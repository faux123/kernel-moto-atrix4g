/*
 * Copyright (c) 2007-2010 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include "nvodm_query_discovery.h"
#include "nvodm_query.h"
#include "nvodm_services.h"
#include "odm_regulator.h"
#include "regulator_supply_info_table.h"
#include <asm/mach-types.h>
#include "hwrev.h"

#define NVODM_PORT(x) ((x) - 'a')

// Private PMU context info
RegulatorPrivData *hRegulatorPmu;

// This board-specific table is indexed by RegulatorPmuSupply
static RegulatorPmuSupplyInfo RegulatorSupplyInfoTable[] =
{
    {
        RegulatorPmuSupply_Invalid,
        NULL,
        NULL,
        {NV_TRUE, 0, 0, 0, 0},
    },

    // cpu
    {
        RegulatorCpcapSupply_SW1,
        "sw1",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            10000,
            REGULATOR_REQUESTVOLTAGE_SW1
        },
    },

    // core
    {
        RegulatorCpcapSupply_SW2,
        "sw2",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            10000,
            REGULATOR_REQUESTVOLTAGE_SW2
        },
    },

    // rtc
    {
        RegulatorCpcapSupply_SW4,
        "sw4",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            10000,
            REGULATOR_REQUESTVOLTAGE_SW4
        },
    },

    // Boost
    {
        RegulatorCpcapSupply_SW5,
        "odm-kit-sw5",
        NULL,
        {
            NV_TRUE,
            0,
            0,
            0,
            0
        },
    },

    // VIO
    {
        RegulatorCpcapSupply_SW3,
        "odm-kit-vio",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // WLAN2
    {
        RegulatorCpcapSupply_WLAN2,
        "odm-kit-vwlan2",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            10000,
            REGULATOR_REQUESTVOLTAGE_WLAN2
        },
    },

    // WLAN1
    {
        RegulatorCpcapSupply_WLAN1,
        "odm-kit-vwlan1",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // VAUDIO
    {
        RegulatorCpcapSupply_VAUDIO,
        "odm-kit-vaudio",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // SIM Cad
    {
        RegulatorCpcapSupply_VSIMCARD,
        "odm-kit-vsimcard",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // VPLL
    {
        RegulatorCpcapSupply_VPLL,
        "odm-kit-vpll",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // HVIO
    {
        RegulatorCpcapSupply_VHVIO,
        "odm-kit-vhvio",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // CAM
    {
        RegulatorCpcapSupply_VCAM,
        "odm-kit-vcam",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // SDIO
    {
        RegulatorCpcapSupply_VSDIO,
        "odm-kit-vsdio",
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // S1
    {
        RegulatorPm8028Supply_S1,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO11 (VOUT11)
    {
        RegulatorPm8028Supply_S2,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO12 (VOUT12)
    {
        RegulatorPm8028Supply_S3,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO13 (VOUT13)
    {
        RegulatorPm8028Supply_S4,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO14 (VOUT14)
    {
        RegulatorPm8028Supply_L1,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO15 (VOUT15)
    {
        RegulatorPm8028Supply_L3,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO16 (VOUT16)
    {
        RegulatorPm8028Supply_L4,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO17 (VOUT17)
    {
        RegulatorPm8028Supply_L5,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO18 (VOUT18)
    {
        RegulatorPm8028Supply_L6,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO19 (VOUT19)
    {
        RegulatorPm8028Supply_L7,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    // LDO20 (VOUT20)
    {
        RegulatorPm8028Supply_L8,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },
    {
        RegulatorPm8028Supply_L9,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    {
        RegulatorPm8028Supply_L10,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    {
        RegulatorPm8028Supply_L11,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    {
        RegulatorPm8028Supply_L13,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    {
        RegulatorPm8028Supply_L18,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            0,
            0
        },
    },

    {
        RegulatorMiscSupply_VHDMI,
        NULL,
        NULL,
        {
            NV_FALSE,
            0,
            0,
            REGULATOR_REQUESTVOLTAGE_VHDMI,
            REGULATOR_REQUESTVOLTAGE_VHDMI
        },
    },
};


static NvBool RegulatorMiscSetup(NvOdmPmuDeviceHandle hDevice);
static void   RegulatorMiscRelease(NvOdmPmuDeviceHandle hDevice);
static NvU32  RegulatorMiscGetVoltage(NvOdmPmuDeviceHandle hDevice, NvU32 vddRail);
static NvBool RegulatorMiscSetVoltage(NvOdmPmuDeviceHandle hDevice,
                                      NvU32 vddRail,
                                      NvU32 MilliVolts,
                                      NvU32* pSettleMicroSeconds);


static NvBool
RegulatorCanBeShutdown(
    NvU32 vddRail)
{
    switch (vddRail) {
        case RegulatorCpcapSupply_WLAN2:
            /* VWLAN2 can only be shutdown for P3 and greater olympus. */
            if (machine_is_olympus() &&
                (HWREV_TYPE_IS_FINAL(system_rev) ||
                 (HWREV_TYPE_IS_PORTABLE(system_rev) &&
                  (HWREV_REV(system_rev) >= HWREV_REV_3))))
                return NV_TRUE;
            /* VWLAN2 can only be shutdown for P3B+ or S3+ on etna. */
            if (machine_is_etna() &&
                (HWREV_TYPE_IS_FINAL(system_rev) ||
                 (HWREV_TYPE_IS_BRASSBOARD(system_rev) &&
                  (HWREV_REV(system_rev) >= HWREV_REV_3)) ||
                 (HWREV_TYPE_IS_PORTABLE(system_rev) &&
                  (HWREV_REV(system_rev) >= HWREV_REV_3B))))
                return NV_TRUE;
            /* VWLAN2 can be shutdown on daytona and sunfire */
            if (machine_is_tegra_daytona() || machine_is_sunfire())
                return NV_TRUE;

            return NV_FALSE;
    }
    return NV_TRUE;
}

/*
 * Translate regulators to different rails based on the hardware revision.
 */
static NvU32
RemapRail(
    NvU32 vddRail
)
{
    switch (vddRail)
    {
        case RegulatorCpcapSupply_WLAN1:
            if (machine_is_olympus() &&
                (HWREV_TYPE_IS_FINAL(system_rev) ||
                 (HWREV_TYPE_IS_PORTABLE(system_rev) &&
                  (HWREV_REV(system_rev) >= HWREV_REV_3))))
                return RegulatorCpcapSupply_SW3;
            if (machine_is_etna() &&
                (HWREV_TYPE_IS_FINAL(system_rev) ||
                 (HWREV_TYPE_IS_BRASSBOARD(system_rev) &&
                  (HWREV_REV(system_rev) >= HWREV_REV_3)) ||
                 (HWREV_TYPE_IS_PORTABLE(system_rev) &&
                  (HWREV_REV(system_rev) >= HWREV_REV_3B))))
                return RegulatorCpcapSupply_SW3;
    }
    return vddRail;
}

NvBool
RegulatorSetup(NvOdmPmuDeviceHandle hDevice)
{
    NvU32  i           = 0;

    NV_ASSERT(hDevice);

    hRegulatorPmu = (RegulatorPrivData*) NvOdmOsAlloc(sizeof(RegulatorPrivData));
    if (hRegulatorPmu == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating RegulatorPrivData.\n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(hRegulatorPmu, 0, sizeof(RegulatorPrivData));
    hDevice->pPrivate = hRegulatorPmu;

    ((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable =
        NvOdmOsAlloc(sizeof(NvU32) * RegulatorPmuSupply_Num);
    if (((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating RefCntTable. \n"));
        goto fail;
    }
    ((RegulatorPrivData*)hDevice->pPrivate)->pVoltages =
        NvOdmOsAlloc(sizeof(NvU32) * RegulatorPmuSupply_Num);
    if (((RegulatorPrivData*)hDevice->pPrivate)->pVoltages == NULL)
    {
        NVODMPMU_PRINTF(("Error Allocating shadow voltages table. \n"));
        goto fail;
    }

    for (i = 0; i < RegulatorPmuSupply_Num; i++)
    {
        ((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable[i] = 0;
        // Setting shadow to 0 would cause spare delay on the 1st scaling of
        // always On rail; however the alternative reading of initial settings
        // over I2C is even worse.
        ((RegulatorPrivData*)hDevice->pPrivate)->pVoltages[i] = 0;
        if( RegulatorSupplyInfoTable[i].name)
        {
            RegulatorSupplyInfoTable[i].sRegulator = regulator_get(NULL, RegulatorSupplyInfoTable[i].name);
        }

    }

    ((RegulatorPrivData*)hDevice->pPrivate)->hOdmPmuSevice = NvOdmServicesPmuOpen();

    //Check battery presence
    ((RegulatorPrivData*)hDevice->pPrivate)->battPresence = NV_FALSE;

    if ( RegulatorMiscSetup(hDevice) != NV_TRUE )
    {
        NVODMPMU_PRINTF(("Error with Misc Setup\n"));
        goto fail;
    }

    return NV_TRUE;

fail:
    RegulatorRelease(hDevice);
    return NV_FALSE;
}

void
RegulatorRelease(NvOdmPmuDeviceHandle hDevice)
{
    if (hDevice->pPrivate != NULL)
    {
        RegulatorMiscRelease(hDevice);

        if (((RegulatorPrivData*)hDevice->pPrivate)->hOdmPmuSevice != NULL)
        {
            NvOdmServicesPmuClose(((RegulatorPrivData*)hDevice->pPrivate)->hOdmPmuSevice);
            ((RegulatorPrivData*)hDevice->pPrivate)->hOdmPmuSevice = NULL;
        }

        if (((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable != NULL)
        {
            NvOdmOsFree(((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable);
            ((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable = NULL;
        }

        if (((RegulatorPrivData*)hDevice->pPrivate)->pVoltages != NULL)
        {
            NvOdmOsFree(((RegulatorPrivData*)hDevice->pPrivate)->pVoltages);
            ((RegulatorPrivData*)hDevice->pPrivate)->pVoltages = NULL;
        }


        NvOdmOsFree(hDevice->pPrivate);
        hDevice->pPrivate = NULL;
    }
}

NvBool
RegulatorGetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(pMilliVolts);
    NV_ASSERT(vddRail < RegulatorPmuSupply_Num);

    vddRail = RemapRail(vddRail);
    if ( vddRail >= RegulatorMiscSupply_Min && vddRail <= RegulatorMiscSupply_Max )
    {
        *pMilliVolts = RegulatorMiscGetVoltage(hDevice, vddRail);
        return NV_TRUE;
    }
    if( !IS_ERR_OR_NULL(RegulatorSupplyInfoTable[vddRail].sRegulator) )
    {
        *pMilliVolts = regulator_get_voltage(RegulatorSupplyInfoTable[vddRail].sRegulator)/1000;
        ((RegulatorPrivData*)hDevice->pPrivate)->pVoltages[vddRail] = *pMilliVolts;
        return NV_TRUE;
    }
    return NV_FALSE;
}

NvBool
RegulatorSetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(vddRail < RegulatorPmuSupply_Num);

    vddRail = RemapRail(vddRail);
    if ( RegulatorSupplyInfoTable[vddRail].cap.OdmProtected == NV_TRUE)
    {
        NVODMPMU_PRINTF(("The voltage is protected and cannot be set.\n"));
        return NV_TRUE;
    }

    if ((MilliVolts == ODM_VOLTAGE_ENABLE_EXT_ONOFF) ||
        (MilliVolts == ODM_VOLTAGE_DISABLE_EXT_ONOFF))
    {
        return NV_TRUE;
    }

    if ( vddRail >= RegulatorMiscSupply_Min && vddRail <= RegulatorMiscSupply_Max )
    {
        return RegulatorMiscSetVoltage(hDevice, vddRail, MilliVolts, pSettleMicroSeconds);
    }

    if( IS_ERR_OR_NULL(RegulatorSupplyInfoTable[vddRail].sRegulator) )
    {
        NVODMPMU_PRINTF(("Regulater is invalid.\n"));
        return NV_FALSE;
    }

    if (MilliVolts == ODM_VOLTAGE_OFF)
    {
        if(((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply]==1)
        {
            /* Older etna and olympus hardware can't shutdown VWLAN2 */
            if (RegulatorCanBeShutdown(vddRail))
            {
                regulator_disable(RegulatorSupplyInfoTable[vddRail].sRegulator);
                ((RegulatorPrivData*)hDevice->pPrivate)->pVoltages[vddRail] = 0;
                NvOdmServicesPmuSetSocRailPowerState(((RegulatorPrivData*)hDevice->pPrivate)->hOdmPmuSevice,
                                                     RegulatorSupplyInfoTable[vddRail].supply, NV_FALSE);
            }
        }
        if( ((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply] )
        ((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply]--;
    }else if (((MilliVolts <=  RegulatorSupplyInfoTable[vddRail].cap.MaxMilliVolts) &&
               (MilliVolts >=  RegulatorSupplyInfoTable[vddRail].cap.MinMilliVolts)))
    {
        if(((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply]==0)
            NvOdmServicesPmuSetSocRailPowerState(
            ((RegulatorPrivData*)hDevice->pPrivate)->hOdmPmuSevice, RegulatorSupplyInfoTable[vddRail].supply, NV_TRUE);
        regulator_set_voltage(RegulatorSupplyInfoTable[vddRail].sRegulator,
                                  MilliVolts*1000, MilliVolts*1000);
        ((RegulatorPrivData*)hDevice->pPrivate)->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply]++;
    }
    else
    {
        NVODMPMU_PRINTF(("The requested voltage is not supported.\n"));
        return NV_FALSE;
    }

    return NV_TRUE;
}

void
RegulatorGetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities)
{
    NV_ASSERT(pCapabilities);
    NV_ASSERT(vddRail < RegulatorPmuSupply_Num);

    vddRail = RemapRail(vddRail);
    *pCapabilities = RegulatorSupplyInfoTable[vddRail].cap;
}

NvBool
RegulatorGetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuAcLineStatus *pStatus)
{

    NV_ASSERT(hDevice);
    NV_ASSERT(pStatus);

    // check if battery is present
    if (((RegulatorPrivData*)hDevice->pPrivate)->battPresence == NV_FALSE)
    {
        *pStatus = NvOdmPmuAcLine_Online;
    }
    return NV_TRUE;
}

NvBool
RegulatorGetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(pStatus);
    NV_ASSERT(batteryInst <= NvOdmPmuBatteryInst_Num);

    /* Battery is actually not present */
    *pStatus = NVODM_BATTERY_STATUS_NO_BATTERY;
    return NV_TRUE;
}

NvBool
RegulatorGetBatteryData(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData)
{
    NvOdmPmuBatteryData batteryData;

    batteryData.batteryAverageCurrent  = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryAverageInterval = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryCurrent         = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryLifePercent     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryLifeTime        = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryMahConsumed     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryTemperature     = NVODM_BATTERY_DATA_UNKNOWN;
    batteryData.batteryVoltage         = NVODM_BATTERY_DATA_UNKNOWN;

    NV_ASSERT(hDevice);
    NV_ASSERT(pData);
    NV_ASSERT(batteryInst <= NvOdmPmuBatteryInst_Num);

    *pData = batteryData;

    return NV_TRUE;
}

void
RegulatorGetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime)
{
    *pLifeTime = NVODM_BATTERY_DATA_UNKNOWN;
}

void
RegulatorGetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry)
{
    *pChemistry = NvOdmPmuBatteryChemistry_LION;
}

NvBool
RegulatorSetChargingCurrent(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuChargingPath chargingPath,
    NvU32 chargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType)
{
    NV_ASSERT(hDevice);

    // If no battery is connected, then do nothing.
    return NV_TRUE;
}

void RegulatorInterruptHandler( NvOdmPmuDeviceHandle  hDevice)
{
    // If the interrupt handle is called, the interrupt is supported.
}

NvBool
RegulatorRtcCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count)
{
    return NV_FALSE;
}

NvBool
RegulatorRtcCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count)
{
    return NV_FALSE;
}

NvBool
RegulatorIsRtcInitialized(NvOdmPmuDeviceHandle hDevice)
{
    return NV_FALSE;
}

NvBool
RegulatorMiscSetup(NvOdmPmuDeviceHandle hDevice)
{
    int i = RegulatorMiscSupply_Min;
    RegulatorPrivData *p = NULL;
    RegulatorMiscData *d = NULL;

    p = (RegulatorPrivData*) hDevice->pPrivate;
    d = &p->miscData;

    /* Early setup */
    d->gpio_hndl = NvOdmGpioOpen();
    if (d->gpio_hndl == NULL)
        goto fail;

    while (i <= RegulatorMiscSupply_Max) {
        switch (i) {
        case RegulatorMiscSupply_VHDMI:
            {
                int port = NVODM_PORT('f');
                int pin  = 6;

                if (machine_is_etna() && HWREV_TYPE_IS_BRASSBOARD(system_rev) &&
                    HWREV_REV(system_rev) == HWREV_REV_1) {
                    port = NVODM_PORT('d');
                    pin  = 0;
                }

                d->vhdmi_gpio = NvOdmGpioAcquirePinHandle(d->gpio_hndl, port, pin);
                if (d->vhdmi_gpio == NULL) {
                    goto fail;
                }

                NvOdmGpioConfig(d->gpio_hndl, d->vhdmi_gpio, NvOdmGpioPinMode_Output);
                p->supplyRefCntTable[RegulatorSupplyInfoTable[i].supply] = 0;

                NvOdmGpioSetState(d->gpio_hndl, d->vhdmi_gpio, 0);
                /* Once we are on kernel 36 or later the correct way to do this is to
                   use a fixed regulator with "vhdmi" set in supply_regulator of the regulator
                   init data. */
                RegulatorSupplyInfoTable[i].sRegulator = regulator_get(NULL, "vhdmi");
                if (IS_ERR_OR_NULL(RegulatorSupplyInfoTable[i].sRegulator)) {
                    NVODMPMU_PRINTF(("Error geting regulator vhdmi %ld.\n",
                                    PTR_ERR(RegulatorSupplyInfoTable[i].sRegulator)));
                }
            }
            break;
        }

        i++;
    }

    return NV_TRUE;

fail:
    if (d->vhdmi_gpio)
        NvOdmGpioReleasePinHandle(d->gpio_hndl, d->vhdmi_gpio);
    if (d->gpio_hndl)
        NvOdmGpioClose(d->gpio_hndl);

    return NV_FALSE;
}

void
RegulatorMiscRelease(NvOdmPmuDeviceHandle hDevice)
{
    int i = RegulatorMiscSupply_Min;
    RegulatorPrivData *p = NULL;
    RegulatorMiscData *d = NULL;

    p = (RegulatorPrivData*) hDevice->pPrivate;
    d = &p->miscData;

    while (i <= RegulatorMiscSupply_Max) {
        switch (i) {
        case RegulatorMiscSupply_VHDMI:
            if (d->vhdmi_gpio) {
                NvOdmGpioReleasePinHandle(d->gpio_hndl, d->vhdmi_gpio);
                d->vhdmi_gpio = NULL;
                if (!IS_ERR_OR_NULL(RegulatorSupplyInfoTable[i].sRegulator)) {
                    regulator_put(RegulatorSupplyInfoTable[i].sRegulator);
                }
            }
            break;
        }

        i++;
    }

    if (d->gpio_hndl) {
        NvOdmGpioClose(d->gpio_hndl);
        d->gpio_hndl = NULL;
    }
}

NvU32
RegulatorMiscGetVoltage(NvOdmPmuDeviceHandle hDevice, NvU32 vddRail)
{
    NvU32 voltage = 0;
    RegulatorPrivData *p = (RegulatorPrivData*) hDevice->pPrivate;

    switch (vddRail) {
    case RegulatorMiscSupply_VHDMI:
        if (p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply])
            voltage = RegulatorSupplyInfoTable[vddRail].cap.requestMilliVolts;
        break;
    }

    return voltage;
}

NvBool
RegulatorMiscSetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds)
{
    NvBool rc = NV_FALSE;
    RegulatorPrivData *p = NULL;
    RegulatorMiscData *d = NULL;

    p = (RegulatorPrivData*) hDevice->pPrivate;
    d = &p->miscData;

    switch (vddRail) {
    case RegulatorMiscSupply_VHDMI:
        if (MilliVolts == RegulatorSupplyInfoTable[vddRail].cap.requestMilliVolts) {
            if (p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply]) {
                p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply] ++;
                rc = NV_TRUE;
            } else {
                if (!IS_ERR_OR_NULL(RegulatorSupplyInfoTable[vddRail].sRegulator)) {
                    (void)regulator_enable(RegulatorSupplyInfoTable[vddRail].sRegulator);
                }
                NvOdmGpioSetState(d->gpio_hndl, d->vhdmi_gpio, 1);
                p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply] ++;
                rc = NV_TRUE;
            }
        } else if (MilliVolts == 0) {
            if (p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply] > 1) {
                p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply] --;
                rc = NV_TRUE;
            } else if (p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply] == 1) {
                NvOdmGpioSetState(d->gpio_hndl, d->vhdmi_gpio, 0);
                p->supplyRefCntTable[RegulatorSupplyInfoTable[vddRail].supply] --;
                if (!IS_ERR_OR_NULL(RegulatorSupplyInfoTable[vddRail].sRegulator)) {
                    (void)regulator_disable(RegulatorSupplyInfoTable[vddRail].sRegulator);
                }
                rc = NV_TRUE;
            }
        }
        break;
    }

    return rc;
}


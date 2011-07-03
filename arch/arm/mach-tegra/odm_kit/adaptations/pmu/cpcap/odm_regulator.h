/*
 * Copyright (c) 2009 NVIDIA Corporation.
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

#ifndef INCLUDED_PMU_ODM_REGULATOR_H
#define INCLUDED_PMU_ODM_REGULATOR_H

#include "nvodm_pmu.h"
#include "pmu_hal.h"

#if defined(__cplusplus)
extern "C"
{
#endif

#if (NV_DEBUG)
#define NVODMPMU_PRINTF(x)   NvOdmOsDebugPrintf x
#else
#define NVODMPMU_PRINTF(x)
#endif

typedef struct
{
	NvOdmServicesGpioHandle gpio_hndl;
	NvOdmGpioPinHandle      vhdmi_gpio;
} RegulatorMiscData;

typedef struct RegulatorStatusRec
{
    /* Low Battery voltage detected by BVM */
    NvBool lowBatt;

    /* PMU high temperature */
    NvBool highTemp;

    /* Main charger Present */
    NvBool mChgPresent;

    /* battery Full */
    NvBool batFull;

} RegulatorStatus;

typedef struct
{
    /* The odm pmu service handle */
    NvOdmServicesPmuHandle hOdmPmuSevice;
    
    /* the PMU status */
    RegulatorStatus pmuStatus;

    /* battery presence */
    NvBool battPresence;

    /* The ref cnt table of the power supplies */
    NvU32 *supplyRefCntTable;

    /* The pointer to supply voltages shadow */
    NvU32 *pVoltages;

    /* Misc supply data */
    RegulatorMiscData miscData;

} RegulatorPrivData;

NvBool 
RegulatorSetup(NvOdmPmuDeviceHandle hDevice);

void 
RegulatorRelease(NvOdmPmuDeviceHandle hDevice);

NvBool
RegulatorGetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32* pMilliVolts);

NvBool
RegulatorSetVoltage(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 vddRail,
    NvU32 MilliVolts,
    NvU32* pSettleMicroSeconds);

void
RegulatorGetCapabilities(
    NvU32 vddRail,
    NvOdmPmuVddRailCapabilities* pCapabilities);

NvBool
RegulatorGetAcLineStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuAcLineStatus *pStatus);

NvBool
RegulatorGetBatteryStatus(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU8 *pStatus);

NvBool
RegulatorGetBatteryData(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryData *pData);

void
RegulatorGetBatteryFullLifeTime(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvU32 *pLifeTime);

void
RegulatorGetBatteryChemistry(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuBatteryInstance batteryInst,
    NvOdmPmuBatteryChemistry *pChemistry);

NvBool
RegulatorSetChargingCurrent(
    NvOdmPmuDeviceHandle hDevice,
    NvOdmPmuChargingPath chargingPath,
    NvU32 chargingCurrentLimitMa,
    NvOdmUsbChargerType ChargerType); 

void RegulatorInterruptHandler( NvOdmPmuDeviceHandle  hDevice);

NvBool
RegulatorRtcCountWrite(
    NvOdmPmuDeviceHandle hDevice,
    NvU32 Count);

NvBool
RegulatorRtcCountRead(
    NvOdmPmuDeviceHandle hDevice,
    NvU32* Count);

NvBool
RegulatorIsRtcInitialized(NvOdmPmuDeviceHandle hDevice);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_PMU_REGULATOR_H

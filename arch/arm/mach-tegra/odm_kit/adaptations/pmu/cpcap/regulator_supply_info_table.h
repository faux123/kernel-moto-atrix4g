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
 
#ifndef INCLUDED_REGULATOR_SUPPLY_INFO_HEADER
#define INCLUDED_REGULATOR_SUPPLY_INFO_HEADER

#include "nvodm_pmu.h"

#if defined(__cplusplus)
extern "C"
{
#endif

// Defines for the requested voltage for each supply (mV).
// This is board specific and ODM should change this based on device.
#define REGULATOR_REQUESTVOLTAGE_SW1       900
#define REGULATOR_REQUESTVOLTAGE_SW2       1200
#define REGULATOR_REQUESTVOLTAGE_SW4       1200
#define REGULATOR_REQUESTVOLTAGE_SW5       5000
#define REGULATOR_REQUESTVOLTAGE_SW3       1800
#define REGULATOR_REQUESTVOLTAGE_WLAN2     3300
#define REGULATOR_REQUESTVOLTAGE_WLAN1     1800
#define REGULATOR_REQUESTVOLTAGE_VAUDIO    2775
#define REGULATOR_REQUESTVOLTAGE_VSIMCARD  2900
#define REGULATOR_REQUESTVOLTAGE_VPLL      1800
#define REGULATOR_REQUESTVOLTAGE_VHVIO     2775
#define REGULATOR_REQUESTVOLTAGE_VCAM      2900
#define REGULATOR_REQUESTVOLTAGE_VSDIO     2900
#define REGULATOR_REQUESTVOLTAGE_S1        1100
#define REGULATOR_REQUESTVOLTAGE_S2        1300
#define REGULATOR_REQUESTVOLTAGE_S3        1800
#define REGULATOR_REQUESTVOLTAGE_S4        2200
#define REGULATOR_REQUESTVOLTAGE_L1        2050
#define REGULATOR_REQUESTVOLTAGE_L3        1200
#define REGULATOR_REQUESTVOLTAGE_L4        2600
#define REGULATOR_REQUESTVOLTAGE_L5        1500
#define REGULATOR_REQUESTVOLTAGE_L6        2850
#define REGULATOR_REQUESTVOLTAGE_L7        1100
#define REGULATOR_REQUESTVOLTAGE_L8        1800
#define REGULATOR_REQUESTVOLTAGE_L9        2850
#define REGULATOR_REQUESTVOLTAGE_L10       3075
#define REGULATOR_REQUESTVOLTAGE_L11       1800
#define REGULATOR_REQUESTVOLTAGE_L13       2200
#define REGULATOR_REQUESTVOLTAGE_L18       1800
#define REGULATOR_REQUESTVOLTAGE_VHDMI     5000

// Output voltages supplied by PMU
typedef enum
{
    RegulatorPmuSupply_Invalid = 0x0,

    /*-- CPCAP Regulators --*/
    RegulatorCpcapSupply_SW1,         // cpu            (default = 0.9V)
    RegulatorCpcapSupply_SW2,         // core           (default = 1.2V)
    RegulatorCpcapSupply_SW4,         // rtc            (default = 1.2V)
    RegulatorCpcapSupply_SW5,         // Boost          (default = 5.0V)
    RegulatorCpcapSupply_SW3,         // VIO            (default = 1.8V)
    RegulatorCpcapSupply_WLAN2,       // WLAN2          (default = 3.3V)
    RegulatorCpcapSupply_WLAN1,       // WLAN1          (default = 1.8V)
    RegulatorCpcapSupply_VAUDIO,      // Audio          (default = 2.775V)
    RegulatorCpcapSupply_VSIMCARD,    // Sim-Card       (default 2.9V)
    RegulatorCpcapSupply_VPLL,        // Enable PLL LDO (default = 1.8V)
    RegulatorCpcapSupply_VHVIO,       // HVIO           (default = 2.775V)
    RegulatorCpcapSupply_VCAM,        // CAM            (default = 2.9V)
    RegulatorCpcapSupply_VSDIO,       // SDIO           (default = 2.9V)
    /*-- LPM8028 Regulator --*/
    RegulatorPm8028Supply_S1,         // MSMC           (default = 1.1V)
    RegulatorPm8028Supply_S2,         // RF1            (default = 1.3V)
    RegulatorPm8028Supply_S3,         // MSME           (default = 1.8V)
    RegulatorPm8028Supply_S4,         // RF2            (default = 2.2V)
    RegulatorPm8028Supply_L1,         // GP             (default = 2.05V)
    RegulatorPm8028Supply_L3,         // GP             (default = 1.2V)
    RegulatorPm8028Supply_L4,         // QFUSE          (default = 2.6V)
    RegulatorPm8028Supply_L5,         //                (default = 1.5V)
    RegulatorPm8028Supply_L6,         // SDCC1          (default = 2.85V)
    RegulatorPm8028Supply_L7,         // MPLL           (default = 1.1V)
    RegulatorPm8028Supply_L8,         // USIM           (default = 1.8V)
    RegulatorPm8028Supply_L9,         // RF_SW          (default = 2.85V)
    RegulatorPm8028Supply_L10,        // USB            (default = 3.075V)
    RegulatorPm8028Supply_L11,        // GP             (default = 1.8V)
    RegulatorPm8028Supply_L13,        // RFA            (default = 2.2V)
    RegulatorPm8028Supply_L18,        // USB            (default = 1.8V)

    /*-- Misc. Regulators --*/
    RegulatorMiscSupply_Min,
    RegulatorMiscSupply_VHDMI = RegulatorMiscSupply_Min, // HDMI  (default = 5.0V)
    RegulatorMiscSupply_Max   = RegulatorMiscSupply_VHDMI,

    RegulatorPmuSupply_Num,
    RegulatorPmuSupply_Force32 = 0x7FFFFFFF
} RegulatorPmuSupply;

typedef NvU32  (*RegulatorPmuVoltageFunc)(const NvU32 data);

typedef struct RegulatorPmuSupplyInfoRec
{
    RegulatorPmuSupply supply;
    char *name;
    struct regulator *sRegulator;
    NvOdmPmuVddRailCapabilities cap;
} RegulatorPmuSupplyInfo;

#if defined(__cplusplus)
}
#endif

#endif //INCLUDED_REGULATOR_SUPPLY_INFO_HEADER


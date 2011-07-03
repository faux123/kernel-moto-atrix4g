/*
 * Copyright (c) 2009 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

/**
 * @file
 * <b>NVIDIA APX ODM Kit::
 *         Implementation of the ODM Peripheral Discovery API</b>
 *
 * @b Description: Specifies the peripheral connectivity 
 *                 database NvOdmIoAddress entries for the E1116
 *                 Power module.
 */

#include "pmu/cpcap/regulator_supply_info_table.h"

// Persistent voltage rail (ie, for RTC, Standby, etc...)
static const NvOdmIoAddress s_ffaRtcAddresses[] = 
{
    // On Maxim 8907B, the standby rail automatically follows V2
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW4, 0 }  /* VDD_RTC -> RTC */
};

// Core voltage rail
static const NvOdmIoAddress s_ffaCoreAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW2 , 0}  /* VDD_CORE */
};

// PMU CPU voltage rail
static const NvOdmIoAddress s_ffaCpuAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW1 , 0}  /* VDD_CPU_PMU  */
};

// PLLA voltage rail
static const NvOdmIoAddress s_ffaPllAAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VPLL , 0 } /* AVDD_PLLA_P_C_S -> VOUT2 */
};

// PLLM voltage rail
static const NvOdmIoAddress s_ffaPllMAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VPLL , 0 } /* AVDD_PLLM -> VOUT2 */
};

// PLLP voltage rail
static const NvOdmIoAddress s_ffaPllPAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorPmuSupply_Invalid , 0} /* AVDD_PLLA_P_C_S  */
};

// PLLC voltage rail
static const NvOdmIoAddress s_ffaPllCAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorPmuSupply_Invalid , 0} /* AVDD_PLLA_P_C_S */
};

// PLLE voltage rail
static const NvOdmIoAddress s_ffaPllEAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorPmuSupply_Invalid , 0} /* AVDD_PLL_E (not used)*/
};

// PLLU voltage rail
static const NvOdmIoAddress s_ffaPllUAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VPLL, 0 } /* AVDD_PLLU */
};

// PLLS voltage rail
static const NvOdmIoAddress s_ffaPllSAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorPmuSupply_Invalid , 0} /* AVDD_PLLA_P_C_S */
};

// PLLHD voltage rail
static const NvOdmIoAddress s_ffaPllHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* AVDD_VHVIO */
};

// OSC voltage rail
static const NvOdmIoAddress s_ffaVddOscAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* AVDD_OSC */
};

// PLLX voltage rail
static const NvOdmIoAddress s_ffaPllXAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VPLL , 0 } /* AVDD_PLLX */
};

// PLL_USB voltage rail
static const NvOdmIoAddress s_ffaPllUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_WLAN2 , 0} /* AVDD_USB_PLL */
};

// SYS IO voltage rail
static const NvOdmIoAddress s_ffaVddSysAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* VDDIO_SYS ????? */
};

// USB voltage rail
static const NvOdmIoAddress s_ffaVddUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_WLAN2 , 0} /* AVDD_USB -> VOUT4 */
};

// HDMI voltage rail
static const NvOdmIoAddress s_ffaVddHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_WLAN2 , 0 } /* AVDD_HDMI -> VOUT11 */
};

// MIPI voltage rail
static const NvOdmIoAddress s_ffaVddMipiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorPmuSupply_Invalid , 0} /* VDDIO_MIPI -> VOUT17 */
};

// LCD voltage rail
static const NvOdmIoAddress s_ffaVddLcdAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* VDDIO_LCD_PMU -> V3 */
};

// Audio voltage rail
static const NvOdmIoAddress s_ffaVddAudAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* VDDIO_AUDIO -> V3 */
};

// LPDDR2 voltage rail (default)
static const NvOdmIoAddress s_ffaVddDdrAddresses[] = 
{
    /* VHVIO is used to gate an LDO off of B+. */
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VHVIO , 0 }  /* VDDIO_DDR_1V2 -> VOUT20 */
};

// DDR2 voltage rail 
static const NvOdmIoAddress s_ffaVddDdr2Addresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorPmuSupply_Invalid , 0}  /* VDDIO_DDR_1V8 -> VOUT05 */
};

// DDR_RX voltage rail
static const NvOdmIoAddress s_ffaVddDdrRxAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VHVIO , 0}  /* VDDIO_RX_DDR(2.7-3.3) -> VOUT1 */
};

// NAND voltage rail
static const NvOdmIoAddress s_ffaVddNandAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* VDDIO_NAND_PMU -> V3 */
};

// UART voltage rail
static const NvOdmIoAddress s_ffaVddUartAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* VDDIO_UART -> V3 */
};

// SDIO voltage rail
static const NvOdmIoAddress s_ffaVddSdioAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VSDIO , 0 } /* VDDIO_SDIO -> VOUT12 */
};

// VDAC voltage rail
static const NvOdmIoAddress s_ffaVddVdacAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_VSIMCARD , 0} /* AVDD_VDAC -> VOUT14 */
};

// VI voltage rail
static const NvOdmIoAddress s_ffaVddViAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* VDDIO_VI -> VOUT18 */
};

// BB voltage rail
static const NvOdmIoAddress s_ffaVddBbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW3 , 0} /* VDDIO_BB -> V3 */
};

// PEX_CLK voltage rail
static const NvOdmIoAddress s_ffaVddPexClkAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorPmuSupply_Invalid , 0}, /* VDDIO_PEX_CLK -> VOUT11 */
};

// PMU0
static const NvOdmIoAddress s_Pmu0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x78 , 0},
};

// USB1 VBus voltage rail
static const NvOdmIoAddress s_ffaVddUsb1VBusAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, RegulatorCpcapSupply_SW5 , 0},
};


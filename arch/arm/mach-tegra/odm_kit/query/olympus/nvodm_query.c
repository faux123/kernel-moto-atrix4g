/*
 * arch/arm/mach-tegra/odm_kit/query/olympus/nvodm_query.c
 *
 * Copyright (c) 2009 NVIDIA Corporation.
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
#include <linux/kernel.h>
#include "nvodm_query.h"
#include "nvodm_query_gpio.h"
#include "nvodm_query_memc.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_pins.h"
#include "nvodm_query_pins_ap20.h"
#include "nvodm_query_dvfs.h"
#include "tegra_devkit_custopt.h"
#include "nvodm_keylist_reserved.h"
#include "nvrm_drf.h"
#include "nvrm_init.h"
#include "nvrm_module.h"
#include <ap20/aremc.h>
#include <asm/mach-types.h>
#include "hwrev.h"

#define BOARD_ID_WHISTLER_E1108 0x0B08
#define BOARD_ID_WHISTLER_E1109 0x0B09
#define BOARD_ID_WHISTLER_PMU_E1116 0x0B10
#define BOARD_ID_WHISTLER_MOTHERBOARD_E1120 0xB14
#define BOARD_ID_VOYAGER_MAINBOARD_E1215    0xC0F
#define BOARD_REV_ALL ((NvU8)0xFF)

#define NVODM_ENABLE_EMC_DVFS (1)
#define NUM_LPDDR2_PARTS 9

// Function to auto-detect boards with external CPU power supply
NvBool NvOdmIsCpuExtSupply(void);

static const NvU8
s_NvOdmQueryDeviceNamePrefixValue[] = { 'T','e','g','r','a',0};

static const NvU8
s_NvOdmQueryManufacturerSetting[] = {'N','V','I','D','I','A',0};

static const NvU8
s_NvOdmQueryModelSetting[] = {'A','P','2','0',0};

static const NvU8
s_NvOdmQueryPlatformSetting[] = {'W','h','i','s','t','l','e','r',0};

static const NvU8
s_NvOdmQueryProjectNameSetting[] = {'O','D','M',' ','K','i','t',0};

static const NvOdmDownloadTransport
s_NvOdmQueryDownloadTransportSetting = NvOdmDownloadTransport_None;

static NvOdmQuerySdioInterfaceProperty s_NvOdmQuerySdioInterfaceProperty_Whistler[4] =
{
// WLAN Dima swapped lines one and two because we use different controller
    { NV_FALSE, 10, NV_TRUE, 0x6, NvOdmQuerySdioSlotUsage_wlan },
    {  NV_FALSE, 10,  NV_FALSE, 0x6, NvOdmQuerySdioSlotUsage_unused },
    { NV_TRUE, 10, NV_FALSE, 0x4, NvOdmQuerySdioSlotUsage_Media  },
    {  NV_FALSE, 10,  NV_TRUE, 0x6, NvOdmQuerySdioSlotUsage_Boot   },
};

static const NvOdmQueryOwrDeviceInfo s_NvOdmQueryOwrInfo = {
    NV_FALSE,
    0x1, /* Tsu */
    0xF, /* TRelease */
    0xF,  /* TRdv */
    0X3C, /* TLow0 */
    0x1, /* TLow1 */
    0x77, /* TSlot */

    0x3C, /* TPdl */
    0x1E, /* TPdh */
    0x1DF, /* TRstl */
    0x1DF, /* TRsth */

    0x1E0, /* Tpp */
    0x5, /* Tfp */
    0x5, /* Trp */
    0x5, /* Tdv */
    0x5, /* Tpd */

    0x7, /* Read data sample clk */
    0x50, /* Presence sample clk */
    2,  /* Memory address size */
    0x80 /* Memory size*/
};

static struct {
  const unsigned char vendor;
  const unsigned char revid1;
  const unsigned char revid2;
  const unsigned char product;
  const NvOdmSdramControllerConfigAdv *table;
  const unsigned int entries;
} lpddr2_tbl[NUM_LPDDR2_PARTS] = {
  {0x06, 0, 0, 0x14, s_NvOdmHynix512M44nmEmcConfigTable, NV_ARRAY_SIZE(s_NvOdmHynix512M44nmEmcConfigTable)},
  {0x06, 0, 1, 0x14, s_NvOdmHynix512M44nmEmcConfigTable,
	NV_ARRAY_SIZE(s_NvOdmHynix512M44nmEmcConfigTable)},
  {0x03, 0, 0, 0x14, s_NvOdmElpida512M50nmEmcConfigTable, NV_ARRAY_SIZE(s_NvOdmElpida512M50nmEmcConfigTable)},
  {0x03, 1, 0, 0x14, s_NvOdmElpida512M40nmEmcConfigTable, NV_ARRAY_SIZE(s_NvOdmElpida512M40nmEmcConfigTable)}, 
  {0x06, 0, 0, 0x54, s_NvOdmHynix1G54nmEmcConfigTable, NV_ARRAY_SIZE(s_NvOdmHynix1G54nmEmcConfigTable)},
  {0x06, 0, 1, 0x54, s_NvOdmHynix1G54nmEmcConfigTable,
	NV_ARRAY_SIZE(s_NvOdmHynix1G54nmEmcConfigTable)},
  {0x03, 0, 0, 0x54, s_NvOdmElpida1G50nmEmcConfigTable, NV_ARRAY_SIZE(s_NvOdmElpida1G50nmEmcConfigTable)},
  {0x03, 1, 0, 0x54, s_NvOdmElpida1G40nmEmcConfigTable, NV_ARRAY_SIZE(s_NvOdmElpida1G40nmEmcConfigTable)},
  {0xFF, 0, 0, 0x54, s_NvOdmMicron1GEmcConfigTable, NV_ARRAY_SIZE(s_NvOdmMicron1GEmcConfigTable)},
};

// Wake Events
static NvOdmWakeupPadInfo s_NvOdmWakeupPadInfo[] =
{
    {NV_FALSE,  0, NvOdmWakeupPadPolarity_Low},     // Wake Event  0 - ulpi_data4 (UART_RI)
    {NV_FALSE,  1, NvOdmWakeupPadPolarity_High},    // Wake Event  1 - gp3_pv[3] (BB_MOD, MODEM_RESET_OUT)
    {NV_TRUE,   2, NvOdmWakeupPadPolarity_Low},     // Wake Event  2 - dvi_d3 (BP_SPI_MRDY)
    {NV_FALSE,  3, NvOdmWakeupPadPolarity_Low},     // Wake Event  3 - sdio3_dat1
    {NV_FALSE,  4, NvOdmWakeupPadPolarity_High},    // Wake Event  4 - hdmi_int (HDMI_HPD)
    {NV_TRUE,   5, NvOdmWakeupPadPolarity_High},    // Wake Event  5 - vgp[6] (VI_GP6, Flash_EN2)
    {NV_TRUE,  	6, NvOdmWakeupPadPolarity_AnyEdge},    // Wake Event  6 - gp3_pu[5] WLAN WAKE 
    {NV_TRUE,	7, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event  7 - gp3_pu[6] (GPS_INT, BT_IRQ)
    {NV_FALSE,  8, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event  8 - gmi_wp_n (MICRO SD_CD)
    {NV_FALSE,  9, NvOdmWakeupPadPolarity_High},    // Wake Event  9 - gp3_ps[2] (KB_COL10)
    {NV_FALSE, 10, NvOdmWakeupPadPolarity_High},    // Wake Event 10 - gmi_ad21 (Accelerometer_TH/TAP)
    {NV_FALSE, 11, NvOdmWakeupPadPolarity_Low},     // Wake Event 11 - spi2_cs2 (PEN_INT, AUDIO-IRQ)
    {NV_FALSE, 12, NvOdmWakeupPadPolarity_Low},     // Wake Event 12 - spi2_cs1 (HEADSET_DET, not used)
    {NV_FALSE, 13, NvOdmWakeupPadPolarity_Low},     // Wake Event 13 - sdio1_dat1
    {NV_FALSE, 14, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 14 - gp3_pv[6] (WLAN_INT)
    {NV_FALSE, 15, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 15 - gmi_ad16  (SPI3_DOUT, DTV_SPI4_CS1)
    {NV_FALSE, 16, NvOdmWakeupPadPolarity_High},    // Wake Event 16 - rtc_irq
    {NV_TRUE,  17, NvOdmWakeupPadPolarity_High},    // Wake Event 17 - kbc_interrupt
    {NV_TRUE,  18, NvOdmWakeupPadPolarity_High},     // Wake Event 18 - pwr_int (PMIC_INT)
    {NV_FALSE, 19, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 19 - usb_vbus_wakeup[0]
    {NV_FALSE, 20, NvOdmWakeupPadPolarity_High},    // Wake Event 20 - usb_vbus_wakeup[1]
    {NV_FALSE, 21, NvOdmWakeupPadPolarity_Low},     // Wake Event 21 - usb_iddig[0]
    {NV_FALSE, 22, NvOdmWakeupPadPolarity_Low},     // Wake Event 22 - usb_iddig[1]
    {NV_FALSE, 23, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 23 - gmi_iordy (HSMMC_CLK)
    {NV_TRUE,  24, NvOdmWakeupPadPolarity_AnyEdge}, // Wake Event 24 - gp3_pv[2] (BB_MOD, MODEM WAKEUP_AP15, SPI-SS)
    {NV_FALSE, 25, NvOdmWakeupPadPolarity_High},    // Wake Event 25 - gp3_ps[4] (KB_COL12)
    {NV_FALSE, 26, NvOdmWakeupPadPolarity_High},    // Wake Event 26 - gp3_ps[5] (KB_COL10)
    {NV_FALSE, 27, NvOdmWakeupPadPolarity_High},    // Wake Event 27 - gp3_ps[0] (KB_COL8)
    {NV_FALSE, 28, NvOdmWakeupPadPolarity_Low},     // Wake Event 28 - gp3_pq[6] (KB_ROW6)
    {NV_FALSE, 29, NvOdmWakeupPadPolarity_Low},     // Wake Event 29 - gp3_pq[7] (KB_ROW6)
    {NV_FALSE, 30, NvOdmWakeupPadPolarity_High}     // Wake Event 30 - dap1_dout (DAP1_DOUT)
};


/* --- Function Implementations ---*/

static NvBool
NvOdmIsBoardPresent(
    const NvOdmBoardInfo* pBoardList,
    NvU32 ListSize)
{
    NvU32 i;
    NvOdmBoardInfo BoardInfo;

    // Scan for presence of any board in the list
    // ID/SKU/FAB fields must match, revision may be masked
    if (pBoardList)
    {
        for (i=0; i < ListSize; i++)
        {
            if (NvOdmPeripheralGetBoardInfo(
                pBoardList[i].BoardID, &BoardInfo))
            {
                if ((pBoardList[i].Fab == BoardInfo.Fab) &&
                    (pBoardList[i].SKU == BoardInfo.SKU) &&
                    ((pBoardList[i].Revision == BOARD_REV_ALL) ||
                     (pBoardList[i].Revision == BoardInfo.Revision)) &&
                    ((pBoardList[i].MinorRevision == BOARD_REV_ALL) ||
                     (pBoardList[i].MinorRevision == BoardInfo.MinorRevision)))
                {
                    return NV_TRUE; // Board found
                }
            }
        }
    }
    return NV_FALSE;
}

NvBool NvOdmIsCpuExtSupply(void)
{
    // A list of Whistler processor boards that use external DCDC as CPU
    // power supply (fill in ID/SKU/FAB fields, revision fields are ignored)
    static const NvOdmBoardInfo s_WhistlerCpuExtSupplyBoards[] =
    {
        // ID                      SKU     FAB   Rev            Minor Rev  
        { BOARD_ID_WHISTLER_E1108, 0x0A14, 0x01, BOARD_REV_ALL, BOARD_REV_ALL},
        { BOARD_ID_WHISTLER_E1108, 0x0A00, 0x02, BOARD_REV_ALL, BOARD_REV_ALL}
    };
    return NvOdmIsBoardPresent(s_WhistlerCpuExtSupplyBoards,
                               NV_ARRAY_SIZE(s_WhistlerCpuExtSupplyBoards));
}

static NvU32
GetBctKeyValue(void)
{
    NvOdmServicesKeyListHandle hKeyList = NULL;
    NvU32 BctCustOpt = 0;

    hKeyList = NvOdmServicesKeyListOpen();
    if (hKeyList)
    {
        BctCustOpt =
            NvOdmServicesGetKeyValue(hKeyList,
                                     NvOdmKeyListId_ReservedBctCustomerOption);
        NvOdmServicesKeyListClose(hKeyList);
    }

    return BctCustOpt;
}

NvOdmDebugConsole
NvOdmQueryDebugConsole(void)
{
    NvU32 CustOpt = GetBctKeyValue();
    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, CONSOLE, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DEFAULT:
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_DCC:
        return NvOdmDebugConsole_Dcc;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_NONE:
        return NvOdmDebugConsole_None;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_CONSOLE_UART:
        return NvOdmDebugConsole_UartA +
            NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, CONSOLE_OPTION, CustOpt);
    default:
        return NvOdmDebugConsole_None;
    }
}

NvOdmDownloadTransport
NvOdmQueryDownloadTransport(void)
{
    NvU32 CustOpt = GetBctKeyValue();

    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, TRANSPORT, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_NONE:
        return NvOdmDownloadTransport_None;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_USB:
        return NvOdmDownloadTransport_Usb;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_ETHERNET:
        switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, ETHERNET_OPTION, CustOpt))
        {
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_SPI:
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_ETHERNET_OPTION_DEFAULT:
        default:
            return NvOdmDownloadTransport_SpiEthernet;
        }
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_UART:
        switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, UART_OPTION, CustOpt))
        {
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_B:
            return NvOdmDownloadTransport_UartB;
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_C:
            return NvOdmDownloadTransport_UartC;
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_DEFAULT:
        case TEGRA_DEVKIT_BCT_CUSTOPT_0_UART_OPTION_A:
        default:
            return NvOdmDownloadTransport_UartA;
        }
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_TRANSPORT_DEFAULT:
    default:
        return s_NvOdmQueryDownloadTransportSetting;
    }
}

const NvU8*
NvOdmQueryDeviceNamePrefix(void)
{
    return s_NvOdmQueryDeviceNamePrefixValue;
}

static const NvOdmQuerySpdifInterfaceProperty s_NvOdmQuerySpdifInterfacePropertySetting =
{
    NvOdmQuerySpdifDataCaptureControl_FromLeft
};

const NvOdmQuerySpiDeviceInfo *

NvOdmQuerySpiGetDeviceInfo(
    NvOdmIoModule OdmIoModule,
    NvU32 ControllerId,
    NvU32 ChipSelect)
{
    /* SPI-1 CS0 is for BP IPC */
    static const NvOdmQuerySpiDeviceInfo s_Spi1Cs0Info =
        {NvOdmQuerySpiSignalMode_0, NV_TRUE, NV_TRUE, NV_FALSE, 0, 0, NV_TRUE};

    /* SPI-2 CS0 & CS1 used for CPCAP */
    static const NvOdmQuerySpiDeviceInfo s_Spi2Cs0Info =
        {NvOdmQuerySpiSignalMode_0, NV_FALSE, NV_FALSE, NV_TRUE, 6, 0};

    /* SPI-2 CS2 is used for fingerprint sensor */
    static const NvOdmQuerySpiDeviceInfo s_Spi2Cs2Info =
        {NvOdmQuerySpiSignalMode_1, NV_TRUE, NV_FALSE, NV_FALSE, 0, 0};

    if ((OdmIoModule == NvOdmIoModule_Spi) &&
        (ControllerId == 0 ) && (ChipSelect == 0))
        return &s_Spi1Cs0Info;

    if ((OdmIoModule == NvOdmIoModule_Spi) &&
        (ControllerId == 1 ) && (ChipSelect == 0 || ChipSelect == 1))
        return &s_Spi2Cs0Info;

    if ((OdmIoModule == NvOdmIoModule_Spi) &&
        (ControllerId == 1) && (ChipSelect == 2)) {
        return &s_Spi2Cs2Info;
    }

    return NULL;
}

const NvOdmQuerySpiIdleSignalState *
NvOdmQuerySpiGetIdleSignalState(
    NvOdmIoModule OdmIoModule,
    NvU32 ControllerId)
{
    return NULL;
}

const NvOdmQueryI2sInterfaceProperty *
NvOdmQueryI2sGetInterfaceProperty(
    NvU32 I2sInstanceId)
{
    static const NvOdmQueryI2sInterfaceProperty s_Property =
    {
        NvOdmQueryI2sMode_Slave,
        NvOdmQueryI2sLRLineControl_LeftOnLow,
        NvOdmQueryI2sDataCommFormat_I2S,
        NV_FALSE,
        0
    };

    if ((!I2sInstanceId) || (I2sInstanceId == 1))
        return &s_Property;

    return NULL;
}

const NvOdmQueryDapPortProperty *
NvOdmQueryDapPortGetProperty(
    NvU32 DapPortId)
{
    static const NvOdmQueryDapPortProperty s_Property[] =
    {
        { NvOdmDapPort_None, NvOdmDapPort_None, { 0, 0, 0, 0 } },
        // I2S1 (DAC1) <-> DAP1 <-> HIFICODEC
        { NvOdmDapPort_I2s1, NvOdmDapPort_HifiCodecType,
          { 2, 16, 44100, NvOdmQueryI2sDataCommFormat_I2S } }, // Dap1
          // I2S2 (DAC2) <-> DAP2 <-> VOICECODEC
        {NvOdmDapPort_I2s2, NvOdmDapPort_VoiceCodecType,
          {2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S } },   // Dap2
    };
    static const NvOdmQueryDapPortProperty s_Property_Ril_Emp_Rainbow[] =
    {
        { NvOdmDapPort_None, NvOdmDapPort_None, { 0, 0, 0, 0 } },
        // I2S1 (DAC1) <-> DAP1 <-> HIFICODEC
        { NvOdmDapPort_I2s1, NvOdmDapPort_HifiCodecType,
          { 2, 16, 44100, NvOdmQueryI2sDataCommFormat_I2S } }, // Dap1
        // I2S2 (DAC2) <-> DAP2 <-> VOICECODEC
        {NvOdmDapPort_I2s2, NvOdmDapPort_VoiceCodecType,
          {2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S } },   // Dap2
        // I2S2 (DAC2) <-> DAP3 <-> BASEBAND
        {NvOdmDapPort_I2s2, NvOdmDapPort_BaseBand,
          {2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S } },   // Dap3
        // I2S2 (DAC2) <-> DAP4 <-> BLUETOOTH
        {NvOdmDapPort_I2s2, NvOdmDapPort_BlueTooth,
          {2, 16, 8000, NvOdmQueryI2sDataCommFormat_I2S } },   // Dap4
    };
    NvU32 CustOpt = GetBctKeyValue();
    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, RIL, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW:
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW_ULPI:
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_IFX:
        if (DapPortId && DapPortId<NV_ARRAY_SIZE(s_Property_Ril_Emp_Rainbow))
            return &s_Property_Ril_Emp_Rainbow[DapPortId];
        break;
    default:
        if (DapPortId && DapPortId<NV_ARRAY_SIZE(s_Property))
            return &s_Property[DapPortId];
        break;
    }
    return NULL;
}

const NvOdmQueryDapPortConnection*
NvOdmQueryDapPortGetConnectionTable(
    NvU32 ConnectionIndex)
{
    static const NvOdmQueryDapPortConnection s_Property[] =
    {
        { NvOdmDapConnectionIndex_Music_Path,
          2, { {NvOdmDapPort_I2s1, NvOdmDapPort_Dap1, NV_FALSE},
               {NvOdmDapPort_Dap1, NvOdmDapPort_I2s1, NV_TRUE} } },
    };
    static const NvOdmQueryDapPortConnection s_Property_Ril_Emp_Rainbow[] =
    {
        { NvOdmDapConnectionIndex_Music_Path,
          2, { {NvOdmDapPort_I2s1, NvOdmDapPort_Dap1, NV_FALSE},
               {NvOdmDapPort_Dap1, NvOdmDapPort_I2s1, NV_TRUE} } },

        // Voicecall without Bluetooth
        { NvOdmDapConnectionIndex_VoiceCall_NoBlueTooth,
          2, { {NvOdmDapPort_Dap3, NvOdmDapPort_Dap2, NV_FALSE},
               {NvOdmDapPort_Dap2, NvOdmDapPort_Dap3, NV_TRUE} } },

        // Voicecall with Bluetooth
        { NvOdmDapConnectionIndex_VoiceCall_WithBlueTooth,
          2, { {NvOdmDapPort_Dap4, NvOdmDapPort_Dap3, NV_TRUE},
              {NvOdmDapPort_Dap3, NvOdmDapPort_Dap4, NV_FALSE}
            }},

    };
    NvU32 TableIndex = 0;
    NvU32 CustOpt = GetBctKeyValue();
    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, RIL, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW:
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW_ULPI:
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_IFX:
        {
            for( TableIndex = 0;
                 TableIndex < NV_ARRAY_SIZE(s_Property_Ril_Emp_Rainbow); TableIndex++)
            {
                if (s_Property_Ril_Emp_Rainbow[TableIndex].UseIndex == ConnectionIndex)
                    return &s_Property_Ril_Emp_Rainbow[TableIndex];
            }
        }
        break;
    default:
        {
            for( TableIndex = 0; TableIndex < NV_ARRAY_SIZE(s_Property); TableIndex++)
            {
                if (s_Property[TableIndex].UseIndex == ConnectionIndex)
                    return &s_Property[TableIndex];
            }
        }
        break;
    }
    return NULL;
}

const NvOdmQuerySpdifInterfaceProperty *
NvOdmQuerySpdifGetInterfaceProperty(
    NvU32 SpdifInstanceId)
{
    if (SpdifInstanceId == 0)
        return &s_NvOdmQuerySpdifInterfacePropertySetting;

    return NULL;
}

const NvOdmQueryAc97InterfaceProperty *
NvOdmQueryAc97GetInterfaceProperty(
    NvU32 Ac97InstanceId)
{
    return NULL;
}

const NvOdmQueryI2sACodecInterfaceProp *
NvOdmQueryGetI2sACodecInterfaceProperty(
    NvU32 AudioCodecId)
{
    static const NvOdmQueryI2sACodecInterfaceProp s_Property =
    {
        NV_TRUE,
        0,
        0x34,
        NV_FALSE,
        NvOdmQueryI2sLRLineControl_LeftOnLow,
        NvOdmQueryI2sDataCommFormat_I2S
    };
    if (!AudioCodecId)
        return &s_Property;
    return NULL;
}

NvBool NvOdmQueryAsynchMemConfig(
    NvU32 ChipSelect,
    NvOdmAsynchMemConfig *pMemConfig)
{
    return NV_FALSE;
}

const void*
NvOdmQuerySdramControllerConfigGet(NvU32 *pEntries, NvU32 *pRevision)
{
#if NVODM_ENABLE_EMC_DVFS
    extern unsigned char lpddr2_mr[];
    int i;

    for(i=0; i<NUM_LPDDR2_PARTS; i++) {
        if (lpddr2_mr[5] == lpddr2_tbl[i].vendor &&
            lpddr2_mr[6] == lpddr2_tbl[i].revid1 &&
            lpddr2_mr[7] == lpddr2_tbl[i].revid2 &&
            lpddr2_mr[8] == lpddr2_tbl[i].product) {

            if (pRevision)
                *pRevision = lpddr2_tbl[i].table->Revision;

            if (pEntries)
                *pEntries = lpddr2_tbl[i].entries;

            return (const void*)lpddr2_tbl[i].table;
        }
    }
#endif
    if (pEntries)
        *pEntries = 0;
    return NULL;
}

NvOdmQueryOscillator NvOdmQueryGetOscillatorSource(void)
{
    return NvOdmQueryOscillator_Xtal;
}

NvU32 NvOdmQueryGetOscillatorDriveStrength(void)
{
    return 0x0A;
}

const NvOdmWakeupPadInfo *NvOdmQueryGetWakeupPadTable(NvU32 *pSize)
{
    NvU32 CustOpt;
    if (pSize)
        *pSize = NV_ARRAY_SIZE(s_NvOdmWakeupPadInfo);

    CustOpt = GetBctKeyValue();
    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, RIL, CustOpt))
    {
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW:
        s_NvOdmWakeupPadInfo[24].enable = NV_TRUE;
        break;
    case TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_IFX:
        s_NvOdmWakeupPadInfo[13].enable = NV_TRUE;
        break;
    default:
        break;
    }

    return (const NvOdmWakeupPadInfo *) s_NvOdmWakeupPadInfo;
}


const NvU8* NvOdmQueryManufacturer(void)
{
    return s_NvOdmQueryManufacturerSetting;
}

const NvU8* NvOdmQueryModel(void)
{
    return s_NvOdmQueryModelSetting;
}

const NvU8* NvOdmQueryPlatform(void)
{
    return s_NvOdmQueryPlatformSetting;
}

const NvU8* NvOdmQueryProjectName(void)
{
    return s_NvOdmQueryProjectNameSetting;
}


#define EXT 0     // external pull-up/down resistor
#define INT_PU 1  // internal pull-up
#define INT_PD 2  // internal pull-down

#define HIGHSPEED 1
#define SCHMITT 1
#define VREF    1
#define OHM_50 3
#define OHM_100 2
#define OHM_200 1
#define OHM_400 0

#define PULL_NORM 0
#define PULL_DOWN 1
#define PULL_UP 2
#define PULL_RSVD 3

 // Pin attributes
 static const NvOdmPinAttrib s_pin_config_attributes[] = {

    /* Pull ups for the various pins
    ATA, ATB, ATC, ATD, ATE, DAP1, DAP2, DAP3, DAP4, DTA, DTB, DTC, DTD, DTE, DTF, GPV
    0x2, 0x2, 0x2, 0x2, 0x2, 0x1,  0x1,  0x1,  0x1,  0x1, 0x1, 0x1, 0x1, 0x0, 0x2, 0x0
    */
    { NvOdmPinRegister_Ap20_PullUpDown_A,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_A(0x2, 0x2, 0x2, 0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0, 0x1, 0x0) },

    /* Pull ups for the kbc pins
    RM,  I2CP, PTA, GPU7, KBCA, KBCB, KBCC, KBCD, SPDI, SPDO, GPU, SLXA, CRTP, SLXC, SLXD, SLXK
    0x2, 0x2,  0x2, 0x2,  0x1,  0x1,  0x2,  0x1,  0x0,  0x2,  0x0, 0x2,  0x2,  0x2,  0x2,  0x1
    */
    { NvOdmPinRegister_Ap20_PullUpDown_B,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_B(0x0, 0x0, 0x0, 0x0, 0x2, 0x2, 0x2, 0x2, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0) },

    /* Pull ups for the kbc pins
    CDEV1, CDEV2, SPIA, SPIB, SPIC, SPID, SPIE, SPIF, SPIG, SPIH, IRTX, IRRX, GME, NOT-USED, XM2D, XM2C
    0x1,   0x1,   0x1,  0x1,  0x2,  0x1,  0x2,  0x1,  0x2,  0x2,  0x2,  0x2,  0x0, x,        0x0,  0x0
    */
    { NvOdmPinRegister_Ap20_PullUpDown_C,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_C(0x0, 0x0, 0x1, 0x1, 0x1, 0x0, 0x0, 0x0, 0x1, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0) },

    /* Pull ups for the various pins
    UAA, UAB, UAC, UAD, UCA, UCB, LD17_0, LD19_18, LD21_20, LD23_22, LS,  LC,  CSUS, DDRC, SDC, SDD
    0x2, 0x2, 0x0, 0x2, 0x2, 0x2, 0x1,    0x1,     0x1,     0x1,     0x2, 0x2, 0x0,  0x2,  0x2, 0x2    
    */
    { NvOdmPinRegister_Ap20_PullUpDown_D,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_D(0x2, 0x2, 0x0, 0x2, 0x2, 0x2, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0, 0x2, 0x2, 0x2) },

    /* Pull ups for the kbc pins
    KBCF, KBCE, PMCA, PMCB, PMCC, PMCD, PMCE, CK32, UDA, SDIO1, GMA, GMB, GMC, GMD, DDC, OWC
    0x1,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x2,   0x0, 0x0, 0x0, 0x0, 0x2, 0x2
    */
    { NvOdmPinRegister_Ap20_PullUpDown_E,
     NVODM_QUERY_PIN_AP20_PULLUPDOWN_E(0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x2, 0x0, 0x0, 0x2, 0x0, 0x2, 0x2) },

    // Set pad control for the sdio2 - - AOCFG1 and AOCFG2 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_AOCFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_AOCFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio1 - SDIO1 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_SDIO1CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio3 - SDIO2 and SDIO3 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_SDIO2CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_SDIO3CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for the sdio4- ATCCFG1 and ATCCFG2 pad control register
    { NvOdmPinRegister_Ap20_PadCtrl_ATCFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_ATCFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for I2C1 pins
    { NvOdmPinRegister_Ap20_PadCtrl_DBGCFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_VICFG1PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    { NvOdmPinRegister_Ap20_PadCtrl_VICFG2PADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },

    // Set pad control for I2C2 (DDC) pins
    { NvOdmPinRegister_Ap20_PadCtrl_DDCCFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_50, 31, 31, 3, 3) },

    // Set pad control for Dap pins 1 - 4
    { NvOdmPinRegister_Ap20_PadCtrl_DAP1CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },

    { NvOdmPinRegister_Ap20_PadCtrl_DAP2CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },

    { NvOdmPinRegister_Ap20_PadCtrl_DAP3CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },

    { NvOdmPinRegister_Ap20_PadCtrl_DAP3CFGPADCTRL,
      NVODM_QUERY_PIN_AP20_PADCTRL_AOCFG1PADCTRL(!HIGHSPEED, SCHMITT, OHM_400, 0, 0, 0, 0) },
 };


NvU32
NvOdmQueryPinAttributes(const NvOdmPinAttrib** pPinAttributes)
{
    *pPinAttributes = &s_pin_config_attributes[0];
    return NV_ARRAY_SIZE(s_pin_config_attributes);
}

NvBool NvOdmQueryGetPmuProperty(NvOdmPmuProperty* pPmuProperty)
{
    pPmuProperty->IrqConnected = NV_FALSE;

    pPmuProperty->PowerGoodCount = 0x0732;
    pPmuProperty->IrqPolarity = NvOdmInterruptPolarity_Low;
    
    // Not there yet, add it later ...
    //pPmuProperty->CpuPowerReqPolarity = ;

    pPmuProperty->CorePowerReqPolarity = NvOdmCorePowerReqPolarity_High;
    pPmuProperty->SysClockReqPolarity = NvOdmSysClockReqPolarity_High;
    pPmuProperty->CombinedPowerReq = NV_FALSE;

    pPmuProperty->CpuPowerGoodUs = 800;
    pPmuProperty->AccuracyPercent = 3;

    pPmuProperty->VCpuOTPOnWakeup = NV_FALSE;

    pPmuProperty->PowerOffCount = 0x1F;
    pPmuProperty->CpuPowerOffUs = 600;
    return NV_TRUE;
}

/**
 * Gets the lowest soc power state supported by the hardware
 *
 * @returns information about the SocPowerState
 */
const NvOdmSocPowerStateInfo* NvOdmQueryLowestSocPowerState(void)
{

    static                      NvOdmSocPowerStateInfo  PowerStateInfo;
    const static                NvOdmSocPowerStateInfo* pPowerStateInfo = NULL;
    NvOdmServicesKeyListHandle  hKeyList;
    NvU32                       LPStateSelection = 0;
    if (pPowerStateInfo == NULL)
    {
        hKeyList = NvOdmServicesKeyListOpen();
        if (hKeyList)
        {
            LPStateSelection = NvOdmServicesGetKeyValue(hKeyList,
                                                NvOdmKeyListId_ReservedBctCustomerOption);
            NvOdmServicesKeyListClose(hKeyList);
            LPStateSelection = NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, LPSTATE, LPStateSelection);
        }
        // Lowest power state controlled by the flashed custom option.
        PowerStateInfo.LowestPowerState =  ((LPStateSelection != TEGRA_DEVKIT_BCT_CUSTOPT_0_LPSTATE_LP1)?
                                            NvOdmSocPowerState_Suspend : NvOdmSocPowerState_DeepSleep);
        //remove the next hardcode line when want to read from odmdata
//        PowerStateInfo.LowestPowerState = NvOdmSocPowerState_DeepSleep;
        pPowerStateInfo = (const NvOdmSocPowerStateInfo*) &PowerStateInfo;
    }
    return (pPowerStateInfo);
}

const NvOdmUsbProperty*
NvOdmQueryGetUsbProperty(NvOdmIoModule OdmIoModule,
                         NvU32 Instance)
{
#ifdef CONFIG_TEGRA_USB_VBUS_DETECT_BY_PMU
#define NVODM_USE_INTERNAL_PHY_VBUS_DETECTION  NV_FALSE
#else
#define NVODM_USE_INTERNAL_PHY_VBUS_DETECTION  NV_TRUE
#endif
    static const NvOdmUsbProperty Usb1Property =
    {
        NvOdmUsbInterfaceType_Utmi,
        (NvOdmUsbChargerType_SE0 | NvOdmUsbChargerType_SE1 | NvOdmUsbChargerType_SK),
        20,
        NV_FALSE,
#ifdef CONFIG_USB_TEGRA_OTG
        NvOdmUsbModeType_OTG,
#else
        NvOdmUsbModeType_Device,
#endif
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE
    };

    static const NvOdmUsbProperty Usb2Property =
    {
        NvOdmUsbInterfaceType_UlpiExternalPhy,
        NvOdmUsbChargerType_UsbHost,
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
        NvOdmUsbModeType_None,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE
    };

    static const NvOdmUsbProperty Usb2NullPhyProperty =
    {
        NvOdmUsbInterfaceType_UlpiNullPhy,
        NvOdmUsbChargerType_UsbHost,
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
        NvOdmUsbModeType_Host,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE,
        {10, 1, 1, 1}
    };

    static const NvOdmUsbProperty Usb3Property =
    {
        NvOdmUsbInterfaceType_Utmi,
        NvOdmUsbChargerType_UsbHost,
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
        NvOdmUsbModeType_Host,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE
    };

    /* E1108 has no ID pin for USB3, so disable USB3 Host */
    static const NvOdmUsbProperty Usb3Property_E1108 =
    {
        NvOdmUsbInterfaceType_Utmi,
        NvOdmUsbChargerType_UsbHost,
        20,
        NVODM_USE_INTERNAL_PHY_VBUS_DETECTION,
        NvOdmUsbModeType_None,
        NvOdmUsbIdPinType_None,
        NvOdmUsbConnectorsMuxType_None,
        NV_FALSE
    };

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 0)
        return &(Usb1Property);

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 1)
    {
        NvU32 CustOpt = GetBctKeyValue();

        if (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CUSTOPT, RIL, CustOpt) ==
            TEGRA_DEVKIT_BCT_CUSTOPT_0_RIL_EMP_RAINBOW_ULPI)
            return &(Usb2NullPhyProperty);
        else
            return &(Usb2Property);
    }

    if (OdmIoModule == NvOdmIoModule_Usb && Instance == 2)
    {
        NvOdmBoardInfo BoardInfo;
        if (NvOdmPeripheralGetBoardInfo(BOARD_ID_WHISTLER_E1108, &BoardInfo))
        {
            return &(Usb3Property_E1108);
        }
        else
        {
            return &(Usb3Property);
        }
    }

    return (const NvOdmUsbProperty *)NULL;
}

const NvOdmQuerySdioInterfaceProperty* NvOdmQueryGetSdioInterfaceProperty(NvU32 Instance)
{
    return &s_NvOdmQuerySdioInterfaceProperty_Whistler[Instance];
}

const NvOdmQueryHsmmcInterfaceProperty* NvOdmQueryGetHsmmcInterfaceProperty(NvU32 Instance)
{
    return NULL;
}

NvU32
NvOdmQueryGetBlockDeviceSectorSize(NvOdmIoModule OdmIoModule)
{
    return 0;
}

const NvOdmQueryOwrDeviceInfo* NvOdmQueryGetOwrDeviceInfo(NvU32 Instance)
{
    return &s_NvOdmQueryOwrInfo;
}

const NvOdmGpioWakeupSource *NvOdmQueryGetWakeupSources(NvU32 *pCount)
{
    *pCount = 0;
    return NULL;
}

/**
 * This function is called from early boot process.
 * Therefore, it cannot use global variables.
 */
NvU32 NvOdmQueryMemSize(NvOdmMemoryType MemType)
{
    NvOdmOsOsInfo Info;
    NvU32 SdramSize;
    NvU32 SdramBctCustOpt;
    
    switch (MemType)
    {
        // NOTE:
        // For Windows CE/WM operating systems the total size of SDRAM may
        // need to be reduced due to limitations in the virtual address map.
        // Under the legacy physical memory manager, Windows OSs have a
        // maximum 512MB statically mapped virtual address space. Under the
        // new physical memory manager, Windows OSs have a maximum 1GB
        // statically mapped virtual address space. Out of that virtual
        // address space, the upper 32 or 36 MB (depending upon the SOC)
        // of the virtual address space is reserved for SOC register
        // apertures.
        //
        // Refer to virtual_tables_apxx.arm for the reserved aperture list.
        // If the cumulative size of the reserved apertures changes, the
        // maximum size of SDRAM will also change.
        case NvOdmMemoryType_Sdram:
        {
            SdramBctCustOpt = GetBctKeyValue();
            switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_SYSTEM, MEMORY, SdramBctCustOpt))
            {
                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_1:
                    SdramSize = 0x10000000; //256 MB
                    break;
                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_3:
                    SdramSize = 0x40000000; //1GB
                    break;
                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_2:
                case TEGRA_DEVKIT_BCT_SYSTEM_0_MEMORY_DEFAULT:
                default:
                    SdramSize = 0x20000000; //512 MB
                    break;
            }
            
            if ( NvOdmOsGetOsInformation(&Info) &&
                 ((Info.OsType!=NvOdmOsOs_Windows) ||
                  (Info.OsType==NvOdmOsOs_Windows && Info.MajorVersion>=7)) )
                return SdramSize;

            // Legacy Physical Memory Manager: SdramSize MB - Carveout MB
            return (SdramSize - NvOdmQueryCarveoutSize());
        }
        
        case NvOdmMemoryType_Nor:
            return 0x00400000;  // 4 MB

        case NvOdmMemoryType_Nand:
        case NvOdmMemoryType_I2CEeprom:
        case NvOdmMemoryType_Hsmmc:
        case NvOdmMemoryType_Mio:
        default:
            return 0;
    }
}

NvU32 NvOdmQueryCarveoutSize(void)
{
    NvU32 CarveBctCustOpt = GetBctKeyValue();

    switch (NV_DRF_VAL(TEGRA_DEVKIT, BCT_CARVEOUT, MEMORY, CarveBctCustOpt))
    {
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_1:
            return 0x00400000;// 4MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_2:
            return 0x00800000;// 8MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_3:
            return 0x00C00000;// 12MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_4:
            return 0x01000000;// 16MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_5:
            return 0x01400000;// 20MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_6:
            return 0x01800000;// 24MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_7:
            return 0x01C00000;// 28MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_8:
            return 0x02000000; // 32 MB
        case TEGRA_DEVKIT_BCT_CARVEOUT_0_MEMORY_DEFAULT:
        default:
            return 0x04000000; // 64 MB
    }
}

NvU32 NvOdmQuerySecureRegionSize(void)
{
    return 0x00800000;// 8 MB
}


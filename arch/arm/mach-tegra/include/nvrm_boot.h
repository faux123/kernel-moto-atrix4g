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

#ifndef INCLUDED_NVRM_BOOT_H
#define INCLUDED_NVRM_BOOT_H


#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvrm_init.h"

/* Bootflag powerup reason definitions */
#define PWRUP_TIME_OF_DAY_ALARM     0x00000008 /* Bit 3  */
#define PWRUP_USB_CABLE             0x00000010 /* Bit 4  */
#define PWRUP_FACTORY_CABLE         0x00000020 /* Bit 5  */
#define PWRUP_AIRPLANE_MODE         0x00000040 /* Bit 6  */
#define PWRUP_PWR_KEY_PRESS         0x00000080 /* Bit 7  */
#define PWRUP_CHARGER               0x00000100 /* Bit 8  */
#define PWRUP_POWER_CUT             0x00000200 /* Bit 9  */
#define PWRUP_REGRESSION_CABLE      0x00000400 /* Bit 10 */
#define PWRUP_SYSTEM_RESTART        0x00000800 /* Bit 11 */
#define PWRUP_MODEL_ASSEMBLY        0x00001000 /* Bit 12 */
#define PWRUP_MODEL_ASSEMBLY_VOL    0x00002000 /* Bit 13 */
#define PWRUP_SW_AP_RESET           0x00004000 /* Bit 14 */
#define PWRUP_WDOG_AP_RESET         0x00008000 /* Bit 15 */
#define PWRUP_CLKMON_CKIH_RESET     0x00010000 /* Bit 16 */
#define PWRUP_AP_KERNEL_PANIC       0x00020000 /* Bit 17 */
#define PWRUP_CPCAP_WDOG            0x00040000 /* Bit 18 */
#define PWRUP_CIDTCMD               0x00080000 /* Bit 19 */
#define PWRUP_BAREBOARD             0x00100000 /* Bit 20 */
#define PWRUP_INVALID               0xFFFFFFFF

/**
 * Sets the RM chip shmoo data as a boot argument from the system's
 * boot loader.
 * 
 * @param hRmDevice The RM device handle.
 * 
 * @retval NvSuccess If successful, or the appropriate error code.
 */
NvError NvRmBootArgChipShmooSet(NvRmDeviceHandle hRmDevice);

#if defined(__cplusplus)
}
#endif

#endif // INCLUDED_NVRM_BOOT_H

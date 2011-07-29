/*
 * Copyright (c) 2011 NVIDIA Corporation.
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

/** @file
 * @brief <b>NVIDIA Driver Development Kit: Fuse API</b>
 *
 * @b Description: Contains the NvRM chip unique id implementation.
 */
#include "nvassert.h"
#include "nvrm_module.h"
#include "ap20/arfuse.h"
#include "ap20/ap20rm_misc_private.h"
#include "nvddk_fuse.h"
NvError NvRmPrivAp20ChipUniqueId(NvRmDeviceHandle hDevHandle,void* pId)
{
    NvU64*  pOut = (NvU64*)pId; // Pointer to output buffer
    NvError e = NvSuccess;
    NvU32 j1 = 0, j2 = 0;

    NV_ASSERT(hDevHandle);
    NV_ASSERT(pId);

    e = NvDdkFuseReadOffset(FUSE_JTAG_SECUREID_0_0, &j1);
    if (e != NvSuccess)
        goto failed;
    e = NvDdkFuseReadOffset(FUSE_JTAG_SECUREID_1_0, &j2);

      // Read the secure id from the fuse registers and copy to the output buffer.
    *pOut = (((NvU64)j1) | (((NvU64)j2) << 32));

failed:
    NV_ASSERT(e == NvSuccess);
    return e;
}

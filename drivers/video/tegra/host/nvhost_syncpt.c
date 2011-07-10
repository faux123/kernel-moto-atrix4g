/*
 * drivers/video/tegra/host/nvhost_syncpt.c
 *
 * Tegra Graphics Host Syncpoints
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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

#include "nvhost_syncpt.h"
#include "nvhost_dev.h"
#define DEBUG_SYSTEM_SERVER_CRASH 1
#if DEBUG_SYSTEM_SERVER_CRASH
#include <linux/mm.h>
#include <asm/page.h>
#include <linux/kernel.h>
#include <linux/swap.h>
#include "nvrm_memmgr.h"
#include "nvrm_init.h"
#include "nvrm_module.h"
#include "nvrm_power.h"
#include "nvrm_hardware_access.h"
#include <asm/io.h>
extern int nvhost_channel_fifo_debug(struct nvhost_dev *m);
extern void nvhost_sync_reg_dump(struct nvhost_dev *m);
#endif

#define client_managed(id) (BIT(id) & NVSYNCPTS_CLIENT_MANAGED)
#define syncpt_to_dev(sp) container_of(sp, struct nvhost_dev, syncpt)
#define SYNCPT_CHECK_PERIOD 2*HZ

static bool check_max(struct nvhost_syncpt *sp, u32 id, u32 real)
{
	u32 max;
	if (client_managed(id))
		return true;
	smp_rmb();
	max = (u32)atomic_read(&sp->max_val[id]);
	return ((s32)(max - real) >= 0);
}

/**
 * Write the current syncpoint value back to hw.
 */
static void reset_syncpt(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_dev *dev = syncpt_to_dev(sp);
	int min;
	smp_rmb();
	min = atomic_read(&sp->min_val[id]);
	writel(min, dev->sync_aperture + (HOST1X_SYNC_SYNCPT_0 + id * 4));
}

/**
 * Write the current waitbase value back to hw.
 */
static void reset_syncpt_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_dev *dev = syncpt_to_dev(sp);
	writel(sp->base_val[id],
		dev->sync_aperture + (HOST1X_SYNC_SYNCPT_BASE_0 + id * 4));
}

/**
 * Read waitbase value from hw.
 */
static void read_syncpt_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_dev *dev = syncpt_to_dev(sp);
	sp->base_val[id] = readl(dev->sync_aperture +
				(HOST1X_SYNC_SYNCPT_BASE_0 + id * 4));
}

/**
 * Resets syncpoint and waitbase values to sw shadows
 */
void nvhost_syncpt_reset(struct nvhost_syncpt *sp)
{
	u32 i;
	for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++)
		reset_syncpt(sp, i);
	for (i = 0; i < NV_HOST1X_SYNCPT_NB_BASES; i++)
		reset_syncpt_wait_base(sp, i);
	wmb();
}

/**
 * Updates sw shadow state for client managed registers
 */
void nvhost_syncpt_save(struct nvhost_syncpt *sp)
{
	u32 i;

	for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
		if (client_managed(i))
			nvhost_syncpt_update_min(sp, i);
		else
			BUG_ON(!nvhost_syncpt_min_eq_max(sp, i));
	}

	for (i = 0; i < NV_HOST1X_SYNCPT_NB_BASES; i++)
		read_syncpt_wait_base(sp, i);
}

/**
 * Updates the last value read from hardware.
 */
u32 nvhost_syncpt_update_min(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_dev *dev = syncpt_to_dev(sp);
	void __iomem *sync_regs = dev->sync_aperture;
	u32 old, live;

	do {
		smp_rmb();
		old = (u32)atomic_read(&sp->min_val[id]);
		live = readl(sync_regs + (HOST1X_SYNC_SYNCPT_0 + id * 4));
	} while ((u32)atomic_cmpxchg(&sp->min_val[id], old, live) != old);

	BUG_ON(!check_max(sp, id, live));

	return live;
}

/**
 * Get the current syncpoint value
 */
u32 nvhost_syncpt_read(struct nvhost_syncpt *sp, u32 id)
{
	u32 val;

	nvhost_module_busy(&syncpt_to_dev(sp)->mod);
	val = nvhost_syncpt_update_min(sp, id);
	nvhost_module_idle(&syncpt_to_dev(sp)->mod);
	return val;
}

/**
 * Write a cpu syncpoint increment to the hardware, without touching
 * the cache. Caller is responsible for host being powered.
 */
void nvhost_syncpt_cpu_incr(struct nvhost_syncpt *sp, u32 id)
{
	struct nvhost_dev *dev = syncpt_to_dev(sp);
	BUG_ON(!client_managed(id) && nvhost_syncpt_min_eq_max(sp, id));
	writel(BIT(id), dev->sync_aperture + HOST1X_SYNC_SYNCPT_CPU_INCR);
	wmb();
}

/**
 * Increment syncpoint value from cpu, updating cache
 */
void nvhost_syncpt_incr(struct nvhost_syncpt *sp, u32 id)
{
	nvhost_syncpt_incr_max(sp, id, 1);
	nvhost_module_busy(&syncpt_to_dev(sp)->mod);
	nvhost_syncpt_cpu_incr(sp, id);
	nvhost_module_idle(&syncpt_to_dev(sp)->mod);
}


#if DEBUG_SYSTEM_SERVER_CRASH
typedef struct{
	NvU32           startAddr;
	NvU32           length;
}NvRegMappingInfo;

typedef struct{
	NvU32           startAddr;
	NvU32           endAddr;
}NvRegAddrInfo;

typedef struct{
	char*   winName;
	NvU32   regVal;
}NvWinModuleInfo;

typedef struct{
	char*               moduleName;
	NvRmModuleID        moduleId;
	NvBool              clkEn;
	NvRegMappingInfo    mapAddr;
	NvBool              addrDiv4;
	NvBool              winRegValid;
	NvU32               numAddr;
	NvU32               idx;
}NvModuleNameRangeInfo;

NvWinModuleInfo dispWinRegVal [] =
{
	{"disp WinA", 0x10000,},
	{"disp WinB", 0x100000,},
	{"disp WinC", 0x1000000,},
};

NvRegAddrInfo addrArray[] =
{
	{0x0,     0x270  },  //MC
	{0x0,     0x1e0  },  //HOST1X_Sync
	{0x340,   0x774  },  //HOST1X_Sync
	{0x0,     0x43 *4},  //DISP
	{0x400*4, 0x4dc*4},  //DISP
	{0x600*4, 0x628*4},  //DISP_WinA
	{0x700*4, 0x714*4},  //DISP_WinB
	{0x800*4, 0x80a*4},  //DISP_WinC
	{0x0,     0x4d *4},  //Dsi
	{0x0,     0x344  },  //CLK-RST
};

NvModuleNameRangeInfo modules[] =
{
	{
		"CLK-RST",
		NvRmPrivModuleID_ClockAndReset,
		NV_FALSE,
		{
			0x60006000,
			0x1000,
		},
		NV_FALSE,
		NV_FALSE,
		1,
		9,
	},
	{
		"MC",
		NvRmPrivModuleID_MemoryController,
		NV_FALSE,
		{
			0x7000f000,
			0x400,
		},
		NV_FALSE,
		NV_FALSE,
		1,
		0,
	},
	{
		"HOST1X_Sync",
		NvRmModuleID_GraphicsHost,
		NV_TRUE,
		{
			0x50003000,
			0x1000,
		},
		NV_FALSE,
		NV_FALSE,
		2,
		1,
	},
	{
		"DISP-A",
		NVRM_MODULE_ID(NvRmModuleID_Display, 0),
		NV_TRUE,
		{
			0x54200000,
			0x40000,
		},
		NV_TRUE,
		NV_TRUE,
		2,
		3,
	},
	{
		"DISP-B",
		NVRM_MODULE_ID(NvRmModuleID_Display, 1),
		NV_TRUE,
		{
			0x54240000,
			0x40000,
		},
		NV_TRUE,
		NV_TRUE,
		2,
		3,
	},
	{
		"DSI",
		NvRmModuleID_Dsi,
		NV_TRUE,
		{
			0x54300000,
			0x1000,
		},
		NV_FALSE,
		NV_FALSE,
		1,
		8,
	},
};

//===========================================================================
// NvReadOneModule() - Read out all the register value of one module.
//===========================================================================
void NvPrintRegVal(void *adr,  NvU32 start, NvU32 end, NvBool div4)
{
	void __iomem *mappedAddr = adr;
	NvU32 i;

	for(i=start; i<=end; i+=4)
	{
		if( !(i&0xf))
			printk("\n0x%08x : ", (div4 ? (i>>2) : i));
		printk("%08x  ", readl(mappedAddr + i));
	}
	printk("\n");
}

NvError NvReadOneModule(NvRmDeviceHandle rm, NvModuleNameRangeInfo* pModule)
{
	NvError err;
	NvU32 i;
	NvU32 powerId;
	void __iomem *mappedAddr;

	if( NV_TRUE == pModule->clkEn )
	{
		err = NvRmPowerRegister( rm, 0, &powerId );
		if ( err != NvSuccess )
			return err;
		err = NvRmPowerModuleClockControl(rm,
		            NvRmModuleID_GraphicsHost, powerId, NV_TRUE);
		if ( err != NvSuccess )
			return err;
		if( pModule->moduleId != NvRmModuleID_GraphicsHost )
		{
			err = NvRmPowerModuleClockControl(rm,
		            pModule->moduleId, powerId, NV_TRUE);
		}
	}

	printk( "Reading %s registers", pModule->moduleName);

	mappedAddr = ioremap(pModule->mapAddr.startAddr, pModule->mapAddr.length);
	for(i=0; i<pModule->numAddr; i++)
	{
		NvPrintRegVal( mappedAddr,
		               addrArray[pModule->idx+i].startAddr,
		               addrArray[pModule->idx+i].endAddr,
		               pModule->addrDiv4);
	}
	if ( NV_TRUE == pModule->winRegValid )
	{
		for(i=0; i<3; i++)
		{
			printk("\n --- Reading %s", dispWinRegVal[i].winName);
			writel(dispWinRegVal[i].regVal, mappedAddr+(0x42<<2));
			NvPrintRegVal( mappedAddr,
			    addrArray[pModule->idx+pModule->numAddr+i].startAddr,
			    addrArray[pModule->idx+pModule->numAddr+i].endAddr,
			    pModule->addrDiv4);
		}
	}
	printk("\n");
	iounmap(mappedAddr);

	if ( NV_TRUE == pModule->clkEn )
	{
        	err=NvRmPowerModuleClockControl(rm, NvRmModuleID_GraphicsHost,
		                            powerId, NV_FALSE);
		NvRmPowerUnRegister( rm, powerId );
	}
	return err;
}

void dumpinfo()
{
	/* mem info */
	printk("********************************\n");
	{
		struct sysinfo ii;
		#define K(x) ((x) << (PAGE_SHIFT - 10))
		si_meminfo(&ii);
		si_swapinfo(&ii);
		printk("MemTotal:       %8lu kB\n",
				K(ii.totalram));
		printk("MemFree:        %8lu kB\n",
				K(ii.freeram));
		printk("Buffers:        %8lu kB\n",
				K(ii.bufferram));
	}
	/* Carveout info */
	printk("********************************\n");
	{
		NvU32 Result;
		NvRmMemGetStat(NvRmMemStat_TotalCarveout, &Result);
		printk("TotalCarveout:   %d\n", Result);
		NvRmMemGetStat(NvRmMemStat_UsedCarveout, &Result);
		printk("UsedCarveout:   %d\n", Result);
		NvRmMemGetStat(NvRmMemStat_LargestFreeCarveoutBlock,
				&Result);
		printk("LargestFreeCarveoutBlock:   %d\n",
				Result);
	}
	/* Registers */
	{
		NvU32 i;
		NvRmDeviceHandle  rm;
		printk("********************************\n");
		NvRmOpen(&rm, 0);
		for (i = 0;
		     i<sizeof(modules)/sizeof(NvModuleNameRangeInfo);
		     i++)
		{
			NvReadOneModule( rm, &(modules[i]) );
		}
		NvRmClose(rm);
	}
}
#endif



/**
 * Main entrypoint for syncpoint value waits.
 */
int nvhost_syncpt_wait_timeout(struct nvhost_syncpt *sp, u32 id,
			u32 thresh, u32 timeout)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	void *ref;
	int err = 0;
#if DEBUG_SYSTEM_SERVER_CRASH
        static int debug_done=0;
	int firsttime;
	int i;
	struct nvhost_dev *dev = syncpt_to_dev(sp);
#endif
	BUG_ON(!check_max(sp, id, thresh));

	/* first check cache */
	if (nvhost_syncpt_min_cmp(sp, id, thresh))
		return 0;

	/* keep host alive */
	nvhost_module_busy(&syncpt_to_dev(sp)->mod);

	if (client_managed(id) || !nvhost_syncpt_min_eq_max(sp, id)) {
		/* try to read from register */
		u32 val = nvhost_syncpt_update_min(sp, id);
		if ((s32)(val - thresh) >= 0)
			goto done;
	}

	if (!timeout) {
		err = -EAGAIN;
		goto done;
	}

	/* schedule a wakeup when the syncpoint value is reached */
	err = nvhost_intr_add_action(&(syncpt_to_dev(sp)->intr), id, thresh,
				NVHOST_INTR_ACTION_WAKEUP_INTERRUPTIBLE, &wq, &ref);
	if (err)
		goto done;

	/* wait for the syncpoint, or timeout, or signal */
        firsttime = 1;
	while (timeout) {
		u32 check = min_t(u32, SYNCPT_CHECK_PERIOD, timeout);
		err = wait_event_interruptible_timeout(wq,
						nvhost_syncpt_min_cmp(sp, id, thresh),
						check);
		if (err != 0)
			break;
		if (timeout != NVHOST_NO_TIMEOUT)
			timeout -= SYNCPT_CHECK_PERIOD;
		if (timeout) {
			if( firsttime <= 40 )
			{
			dev_warn(&syncpt_to_dev(sp)->pdev->dev,
				"syncpoint id %d (%s) stuck waiting %d  timeout=%d\n",
				id, nvhost_syncpt_name(id), thresh, timeout);
			nvhost_syncpt_debug(sp);
			}
#if DEBUG_SYSTEM_SERVER_CRASH
			if(firsttime==40 && !debug_done)
			{
			nvhost_channel_fifo_debug(dev);
			nvhost_sync_reg_dump(dev);
				dumpinfo();
				debug_done = 1;
				break;
			}
			if( firsttime <= 40 )
				firsttime++;
#endif
		}
	};
	if (err > 0)
		err = 0;
	else if (err == 0)
		err = -EAGAIN;
	nvhost_intr_put_ref(&(syncpt_to_dev(sp)->intr), ref);

done:
	nvhost_module_idle(&syncpt_to_dev(sp)->mod);
	return err;
}

static const char *s_syncpt_names[32] = {
	"gfx_host", "", "", "", "", "", "", "", "", "", "", "",
	"vi_isp_0", "vi_isp_1", "vi_isp_2", "vi_isp_3", "vi_isp_4", "vi_isp_5",
	"2d_0", "2d_1",
	"", "",
	"3d", "mpe", "disp0", "disp1", "vblank0", "vblank1", "mpe_ebm_eof", "mpe_wr_safe",
	"2d_tinyblt", "dsi"
};

const char *nvhost_syncpt_name(u32 id)
{
	BUG_ON(id > ARRAY_SIZE(s_syncpt_names));
	return s_syncpt_names[id];
}

void nvhost_syncpt_debug(struct nvhost_syncpt *sp)
{
	u32 i;
	for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
		u32 max = nvhost_syncpt_read_max(sp, i);
		if (!max)
			continue;
		dev_info(&syncpt_to_dev(sp)->pdev->dev,
			"id %d (%s) min %d max %d\n",
			i, nvhost_syncpt_name(i),
			nvhost_syncpt_update_min(sp, i), max);

	}
}

/* returns true, if a <= b < c using wrapping comparison */
static inline bool nvhost_syncpt_is_between(u32 a, u32 b, u32 c)
{
	return b-a < c-a;
}

/* returns true, if x >= y (mod 1 << 32) */
static bool nvhost_syncpt_wrapping_comparison(u32 x, u32 y)
{
	return nvhost_syncpt_is_between(y, x, (1UL<<31UL)+y);
}

/* check for old WAITs to be removed (avoiding a wrap) */
int nvhost_syncpt_wait_check(struct nvhost_syncpt *sp, u32 waitchk_mask,
		struct nvhost_waitchk *waitp, u32 waitchks)
{
	u32 idx;
	int err = 0;

	/* get current syncpt values */
	for (idx = 0; idx < NV_HOST1X_SYNCPT_NB_PTS; idx++) {
		if (BIT(idx) & waitchk_mask) {
			nvhost_syncpt_update_min(sp, idx);
		}
	}

	BUG_ON(!waitp);

	/* compare syncpt vs wait threshold */
	while (waitchks) {
		u32 syncpt, override;

		BUG_ON(waitp->syncpt_id > NV_HOST1X_SYNCPT_NB_PTS);

		syncpt = atomic_read(&sp->min_val[waitp->syncpt_id]);
		if (nvhost_syncpt_wrapping_comparison(syncpt, waitp->thresh)) {

			/* wait has completed already, so can be removed */
			dev_dbg(&syncpt_to_dev(sp)->pdev->dev,
					"drop WAIT id %d (%s) thresh 0x%x, syncpt 0x%x\n",
					waitp->syncpt_id,  nvhost_syncpt_name(waitp->syncpt_id),
					waitp->thresh, syncpt);

			/* move wait to a kernel reserved syncpt (that's always 0) */
			override = nvhost_class_host_wait_syncpt(NVSYNCPT_GRAPHICS_HOST, 0);

			/* patch the wait */
			err = nvmap_patch_wait((struct nvmap_handle *)waitp->mem,
						waitp->offset, override);
			if (err)
				break;
		}
		waitchks--;
		waitp++;
	}
	return err;
}

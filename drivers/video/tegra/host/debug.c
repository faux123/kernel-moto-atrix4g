/*
 * drivers/video/tegra/host/debug.c
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

#include <linux/kernel.h>
#include <asm/io.h>
#include "nvhost_dev.h"

enum {
	NVHOST_DBG_STATE_CMD = 0,
	NVHOST_DBG_STATE_DATA = 1,
};

#define HOST1X_CHANNEL_DMAGET		0x1c

#define HOST1X_SYNC_CF_SETUP(x)		(0x3080 + (4 * (x)))

#define HOST1X_SYNC_SYNCPT_BASE(x)	(0x3600 + (4 * (x)))

#define HOST1X_SYNC_CBREAD(x)		(0x3720 + (4 * (x)))
#define HOST1X_SYNC_CFPEEK_CTRL		0x374c
#define HOST1X_SYNC_CFPEEK_READ		0x3750
#define HOST1X_SYNC_CFPEEK_PTRS		0x3754
#define HOST1X_SYNC_CBSTAT(x)		(0x3758 + (4 * (x)))

/**
 * Find the currently executing gather in the push buffer and return
 * its physical address and size.
 */
void nvhost_cdma_find_gather(struct nvhost_cdma *cdma, u32 dmaget, u32 *addr, u32 *size)
{
	u32 offset = dmaget - cdma->push_buffer.phys;

	*addr = *size = 0;

	if (offset >= 8 && offset < cdma->push_buffer.cur) {
		u32 *p = cdma->push_buffer.mapped + (offset - 8) / 4;

		/* Make sure we have a gather */
		if ((p[0] >> 28) == 6) {
			*addr = p[1];
			*size = p[0] & 0x3fff;
		}
	}
}

static int nvhost_debug_handle_cmd(u32 val, int *count)
{
	unsigned mask;
	unsigned subop;

	switch (val >> 28) {
	case 0x0:
		mask = val & 0x3f;
		if (mask) {
			printk("SETCL(class=%03x, offset=%03x, mask=%02x, [",
				   val >> 6 & 0x3ff, val >> 16 & 0xfff, mask);
			*count = hweight8(mask);
			return NVHOST_DBG_STATE_DATA;
		} else {
			printk("SETCL(class=%03x)\n", val >> 6 & 0x3ff);
			return NVHOST_DBG_STATE_CMD;
		}

	case 0x1:
		printk("INCR(offset=%03x, [", val >> 16 & 0xfff);
		*count = val & 0xffff;
		return NVHOST_DBG_STATE_DATA;

	case 0x2:
		printk("NONINCR(offset=%03x, [", val >> 16 & 0xfff);
		*count = val & 0xffff;
		return NVHOST_DBG_STATE_DATA;

	case 0x3:
		mask = val & 0xffff;
		printk("MASK(offset=%03x, mask=%03x, [",
			   val >> 16 & 0xfff, mask);
		*count = hweight16(mask);
		return NVHOST_DBG_STATE_DATA;

	case 0x4:
		printk("IMM(offset=%03x, data=%03x)\n",
			   val >> 16 & 0xfff, val & 0xffff);
		return NVHOST_DBG_STATE_CMD;

	case 0x5:
		printk("RESTART(offset=%08x)\n", val << 4);
		return NVHOST_DBG_STATE_CMD;

	case 0x6:
		printk("GATHER(offset=%03x, insert=%d, type=%d, count=%04x, addr=[",
			   val >> 16 & 0x3ff, val >> 15 & 0x1, val >> 15 & 0x1,
			   val & 0x3fff);
		*count = 1;
		return NVHOST_DBG_STATE_DATA;

	case 0xe:
		subop = val >> 24 & 0xf;
		if (subop == 0)
			printk("ACQUIRE_MLOCK(index=%d)\n", val & 0xff);
		else if (subop == 1)
			printk("RELEASE_MLOCK(index=%d)\n", val & 0xff);
		else
			printk("EXTEND_UNKNOWN(%08x)\n", val);

		return NVHOST_DBG_STATE_CMD;

	case 0xf:
		printk("DONE()\n");
		return NVHOST_DBG_STATE_CMD;

	default:
		return NVHOST_DBG_STATE_CMD;
	}
}

static void nvhost_debug_handle_word(int *state, int *count,
				     u32 addr, int channel, u32 val)
{
	switch (*state) {
	case NVHOST_DBG_STATE_CMD:
		if (addr)
			printk("%d: %08x: %08x:", channel, addr, val);
		else
			printk("%d: %08x:", channel, val);

		*state = nvhost_debug_handle_cmd(val, count);
		if (*state == NVHOST_DBG_STATE_DATA && *count == 0) {
			*state = NVHOST_DBG_STATE_CMD;
			printk("])\n");
		}
		break;

	case NVHOST_DBG_STATE_DATA:
		(*count)--;
		printk("%08x%s", val, *count > 0 ? ", " : "])\n");
		if (*count == 0)
			*state = NVHOST_DBG_STATE_CMD;
		break;
	}
}


int nvhost_channel_fifo_debug(struct nvhost_dev *m)
{
	int i;

	nvhost_module_busy(&m->mod);

	for (i = 0; i < NVHOST_NUMCHANNELS; i++) {
		void __iomem *regs = m->channels[i].aperture;
		u32 dmaput, dmaget, dmactrl;
		u32 cbstat, cbread;
		u32 fifostat;
		u32 val, base, baseval;
		unsigned start, end;
		unsigned wr_ptr, rd_ptr;
		int state;
		int count;
		u32 phys_addr, size;

		dmaput = readl(regs + HOST1X_CHANNEL_DMAPUT);
		dmaget = readl(regs + HOST1X_CHANNEL_DMAGET);
		dmactrl = readl(regs + HOST1X_CHANNEL_DMACTRL);
		cbread = readl(m->aperture + HOST1X_SYNC_CBREAD(i));
		cbstat = readl(m->aperture + HOST1X_SYNC_CBSTAT(i));

		if (dmactrl != 0x0 || !m->channels[i].cdma.push_buffer.mapped) {
			printk("%d: inactive\n", i);
			continue;
		}

		switch (cbstat) {
		case 0x00010008:
			printk("%d: waiting on syncpt %d val %d\n",
				   i, cbread >> 24, cbread & 0xffffff);
			break;

		case 0x00010009:
			base = cbread >> 16 & 0xff;
			baseval = readl(m->aperture + HOST1X_SYNC_SYNCPT_BASE(base)) & 0xffff;

			val = cbread & 0xffff;

			printk("%d: waiting on syncpt %d val %d (base %d = %d; offset = %d)\n",
				i, cbread >> 24, baseval + val, base, baseval, val);
			break;

		default:
			printk("%d: active class %02x, offset %04x, val %08x\n",
				   i, cbstat >> 16, cbstat & 0xffff, cbread);
			break;
		}

		nvhost_cdma_find_gather(&m->channels[i].cdma, dmaget, &phys_addr, &size);

		/* If dmaget is in the pushbuffer (should always be?),
		 * check if we're executing a fetch, and if so dump
		 * it. */
		if (size) {
			u32 map_base = phys_addr & PAGE_MASK;
			u32 map_size = ((phys_addr + size * 4 + PAGE_SIZE - 1) & PAGE_MASK) - map_base;
			u32 map_offset = phys_addr - map_base;
			void *map_addr = ioremap_nocache(map_base, map_size);

			if (map_addr) {
				u32 ii;

				printk("%d: gather (%d words)\n", i, size);
				state = NVHOST_DBG_STATE_CMD;
				for (ii = 0; ii < size; ii++) {
					val = readl(map_addr + map_offset + ii*sizeof(u32));
					nvhost_debug_handle_word(&state, &count, phys_addr + ii * 4, i, val);
				}
				iounmap(map_addr);
			}
		}

		fifostat = readl(regs + HOST1X_CHANNEL_FIFOSTAT);
		if ((fifostat & 1 << 10) == 0 ) {

			printk("%d: fifo:\n", i);
			writel(0x0, m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
			writel(1 << 31 | i << 16, m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
			rd_ptr = readl(m->aperture + HOST1X_SYNC_CFPEEK_PTRS) & 0x1ff;
			wr_ptr = readl(m->aperture + HOST1X_SYNC_CFPEEK_PTRS) >> 16 & 0x1ff;

			start = readl(m->aperture + HOST1X_SYNC_CF_SETUP(i)) & 0x1ff;
			end = (readl(m->aperture + HOST1X_SYNC_CF_SETUP(i)) >> 16) & 0x1ff;

			state = NVHOST_DBG_STATE_CMD;

			do {
				writel(0x0, m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
				writel(1 << 31 | i << 16 | rd_ptr, m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
				val = readl(m->aperture + HOST1X_SYNC_CFPEEK_READ);

				nvhost_debug_handle_word(&state, &count, 0, i, val);

				if (rd_ptr == end)
					rd_ptr = start;
				else
					rd_ptr++;


			} while (rd_ptr != wr_ptr);

			if (state == NVHOST_DBG_STATE_DATA)
				printk(", ...])\n");
		}
	}

	nvhost_module_idle(&m->mod);
	return 0;
}

void nvhost_sync_reg_dump(struct nvhost_dev *m)
{
	int i;

	for(i=0; i<0x1e0; i+=4)
	{
		if( !(i&0xf) )
			printk("\n0x%08x : ", i);
		printk("%08x  ", readl(m->sync_aperture + i));
	}
	printk("\n\n");
	for(i=0x340; i<0x774; i+=4)
	{
		if( !(i&0xf) )
			printk("\n0x%08x : ", i);
		printk("%08x  ", readl(m->sync_aperture + i));
	}

}




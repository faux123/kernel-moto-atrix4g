/*
 * drivers/mmc/host/sdhci-simple.c
 *
 * Simple SDHCI driver implementation that does not depend on the kernel MMC
 * framework.   The purpose is to be able to access an MMC/SD/SDIO device after
 * the kernel has panicked (no scheduling).
 *
 * Copyright (c) 2010, Motorola.
 *
 * Based on:
 *   drivers/mmc/host/sdhci.c
 *
 * Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/pagemap.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc_simple.h>

#include "sdhci.h"

#define DRIVER_NAME "sdhci_simple"
#define SDHCI_COMMAND_TIMEOUT	1000	/* milliseconds */

/* FIXME: steal this quirk until upstream adds another bitfield */
#define SDHCI_QUIRK_BROKEN_PIO SDHCI_QUIRK_RUNTIME_DISABLE

#define DBG(f, x...) \
	pr_debug(DRIVER_NAME " [%s()]: " f, __func__,## x)


extern int sdhci_simple_host_init(int);


static struct sdhci_host sdhci_simple_host;
static struct mmc_host sdhci_simple_mmc_host;
static unsigned int debug_quirks = 0;

static void sdhci_simple_send_command(struct sdhci_host *, struct mmc_command *);
static void sdhci_simple_finish_command(struct sdhci_host *);

static void sdhci_simple_dumpregs(struct sdhci_host *host)
{
	pr_debug(DRIVER_NAME ": ============ REGISTER DUMP =============\n");

	pr_debug(DRIVER_NAME ": Sys addr: 0x%08x | Version:  0x%08x\n",
		sdhci_readl(host, SDHCI_DMA_ADDRESS),
		sdhci_readw(host, SDHCI_HOST_VERSION));
	pr_debug(DRIVER_NAME ": Blk size: 0x%08x | Blk cnt:  0x%08x\n",
		sdhci_readw(host, SDHCI_BLOCK_SIZE),
		sdhci_readw(host, SDHCI_BLOCK_COUNT));
	pr_debug(DRIVER_NAME ": Argument: 0x%08x | Trn mode: 0x%08x\n",
		sdhci_readl(host, SDHCI_ARGUMENT),
		sdhci_readw(host, SDHCI_TRANSFER_MODE));
	pr_debug(DRIVER_NAME ": Present:  0x%08x | Host ctl: 0x%08x\n",
		sdhci_readl(host, SDHCI_PRESENT_STATE),
		sdhci_readb(host, SDHCI_HOST_CONTROL));
	pr_debug(DRIVER_NAME ": Power:    0x%08x | Blk gap:  0x%08x\n",
		sdhci_readb(host, SDHCI_POWER_CONTROL),
		sdhci_readb(host, SDHCI_BLOCK_GAP_CONTROL));
	pr_debug(DRIVER_NAME ": Wake-up:  0x%08x | Clock:    0x%08x\n",
		sdhci_readb(host, SDHCI_WAKE_UP_CONTROL),
		sdhci_readw(host, SDHCI_CLOCK_CONTROL));
	pr_debug(DRIVER_NAME ": Timeout:  0x%08x | Int stat: 0x%08x\n",
		sdhci_readb(host, SDHCI_TIMEOUT_CONTROL),
		sdhci_readl(host, SDHCI_INT_STATUS));
	pr_debug(DRIVER_NAME ": Int enab: 0x%08x | Sig enab: 0x%08x\n",
		sdhci_readl(host, SDHCI_INT_ENABLE),
		sdhci_readl(host, SDHCI_SIGNAL_ENABLE));
	pr_debug(DRIVER_NAME ": AC12 err: 0x%08x | Slot int: 0x%08x\n",
		sdhci_readw(host, SDHCI_ACMD12_ERR),
		sdhci_readw(host, SDHCI_SLOT_INT_STATUS));
	pr_debug(DRIVER_NAME ": Caps:     0x%08x | Max curr: 0x%08x\n",
		sdhci_readl(host, SDHCI_CAPABILITIES),
		sdhci_readl(host, SDHCI_MAX_CURRENT));

	if (host->flags & SDHCI_USE_ADMA)
		pr_debug(DRIVER_NAME ": ADMA Err: 0x%08x | ADMA Ptr: 0x%08x\n",
		       sdhci_readl(host, SDHCI_ADMA_ERROR),
		       sdhci_readl(host, SDHCI_ADMA_ADDRESS));

	pr_debug(DRIVER_NAME ": ========================================\n");
}

/*****************************************************************************\
 *                                                                           *
 * Low level functions                                                       *
 *                                                                           *
\*****************************************************************************/

static void sdhci_simple_reset(struct sdhci_host *host, u8 mask)
{
	unsigned long timeout;

	if (host->quirks & SDHCI_QUIRK_NO_CARD_NO_RESET) {
		if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
			SDHCI_CARD_PRESENT))
			return;
	}

	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);

	if (mask & SDHCI_RESET_ALL)
		host->clock = 0;

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			pr_err("%s: Reset 0x%x never completed.\n",
				__func__, (int)mask);
			sdhci_simple_dumpregs(host);
			return;
		}
		timeout--;
		mdelay(1);
	}
}

static void sdhci_simple_init(struct sdhci_host *host)
{
	u32 intmask;

	sdhci_simple_reset(host, SDHCI_RESET_ALL);

	/*
	 * The SDHCI controller has no way to poll for command timeouts.  The
	 * method used to identify card types depends on certain commands
	 * timing-out.  We enable this interrupt so that we can poll the
	 * interrupt status bit.  As long as we clear the interrupt quickly,
	 * the real SDHCI's interrupt handler doesn't seem to notice.
	 */
	intmask = SDHCI_INT_TIMEOUT;

	/*
	 * The Tegra SDHCI controller buffer bits are completely broken, which
	 * means that we need to abuse some more interrupt status bits to see
	 * what is going on in the controller for flow control.
	 */
	if (host->quirks & SDHCI_QUIRK_BROKEN_PIO)
		intmask |= SDHCI_INT_DATA_AVAIL |
		           SDHCI_INT_SPACE_AVAIL |
			   SDHCI_INT_DATA_END;

	sdhci_writel(host, intmask, SDHCI_INT_ENABLE);
	sdhci_writel(host, intmask, SDHCI_SIGNAL_ENABLE);
}

/*****************************************************************************\
 *                                                                           *
 * Core functions                                                            *
 *                                                                           *
\*****************************************************************************/

#define SDHCI_POLL_INTERVAL 100	/* microseconds */
static int sdhci_simple_poll_for_state(struct sdhci_host *host, u32 mask,
                                       u32 state, unsigned long timeout)
{
	u32 present;

	state &= mask;
	timeout *= (1000 / SDHCI_POLL_INTERVAL);

        pr_debug("%s: waiting for controller state 0x%08X\n", __func__, mask);
	present = sdhci_readl(host, SDHCI_PRESENT_STATE);
	pr_debug("%s: state=0x%08X\n", __func__, present);
	while ((present & mask) != state) {
		pr_debug("%s: state=0x%08X\n", __func__, present);
		if (timeout == 0) {
			pr_err("%s: controller never reached state "
				"0x%08X:0x%08X\n", __func__, mask, state);
			sdhci_simple_dumpregs(host);
			return -1;
		}
		timeout--;
		udelay(SDHCI_POLL_INTERVAL);

		present = sdhci_readl(host, SDHCI_PRESENT_STATE);
	}

	return 0;
}

static int sdhci_simple_poll_for_irq(struct sdhci_host *host, u32 mask,
                                     u32 irq, unsigned long timeout)
{
	u32 status;

	irq &= mask;
	timeout *= (1000 / SDHCI_POLL_INTERVAL);

        pr_debug("%s: waiting for controller IRQ 0x%08X\n", __func__, mask);
	status = sdhci_readl(host, SDHCI_INT_STATUS);
	pr_debug("%s: status=0x%08X\n", __func__, status);
	while ((status & mask) != irq) {
		pr_debug("%s: status=0x%08X\n", __func__, status);
		if (timeout == 0) {
			pr_err("%s: interrupt never arrived "
				"0x%08X:0x%08X\n", __func__, mask, irq);
			sdhci_simple_dumpregs(host);
			return -1;
		}
		timeout--;
		udelay(SDHCI_POLL_INTERVAL);

		status = sdhci_readl(host, SDHCI_INT_STATUS);
	}

	sdhci_writel(host, mask, SDHCI_INT_STATUS);

	return 0;
}

static void sdhci_simple_read_block_pio(struct sdhci_host *host)
{
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 uninitialized_var(scratch);
	u8 *buf;

	DBG("PIO reading\n");

	if (host->sg_miter.length == 0)
		return;

	blksize = host->data->blksz;
	chunk = 0;

	local_irq_save(flags);

	while (blksize) {
		len = min(host->sg_miter.length, blksize);

		blksize -= len;

		buf = host->sg_miter.addr + host->sg_miter.consumed;
		host->sg_miter.consumed += len;

		while (len) {
			if (chunk == 0) {
				scratch = sdhci_readl(host, SDHCI_BUFFER);
				chunk = 4;
			}

			*buf = scratch & 0xFF;

			buf++;
			scratch >>= 8;
			chunk--;
			len--;
		}
	}

	local_irq_restore(flags);
}

static void sdhci_simple_write_block_pio(struct sdhci_host *host)
{
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 scratch;
	u8 *buf;

	DBG("PIO writing\n");

	if (host->sg_miter.length == 0)
		return;

	blksize = host->data->blksz;
	chunk = 0;
	scratch = 0;

	local_irq_save(flags);

	while (blksize) {
		len = min(host->sg_miter.length, blksize);

		blksize -= len;

		buf = host->sg_miter.addr + host->sg_miter.consumed;
		host->sg_miter.consumed += len;

		while (len) {
			scratch |= (u32)*buf << (chunk * 8);

			buf++;
			chunk++;
			len--;

			if ((chunk == 4) || ((len == 0) && (blksize == 0))) {
				sdhci_writel(host, scratch, SDHCI_BUFFER);
				chunk = 0;
				scratch = 0;
			}
		}
	}

	local_irq_restore(flags);
}

static int sdhci_simple_transfer_data(struct sdhci_host *host)
{
	u32 mask, irq = 0;
	int err = 0;

	if (host->blocks == 0)
		return 0;

	/*
	 * Some controllers' buffer bits are completely worthless.
	 */
	if (host->quirks & SDHCI_QUIRK_BROKEN_PIO) {
		mask = ~0;  /* make the loop below a while(1) */
		if (host->data->flags & MMC_DATA_READ)
			irq = SDHCI_INT_DATA_AVAIL;
		else
			irq = SDHCI_INT_SPACE_AVAIL;
	} else {
		if (host->data->flags & MMC_DATA_READ)
			mask = SDHCI_DATA_AVAILABLE;
		else
			mask = SDHCI_SPACE_AVAILABLE;

		/* Wait for the transfer to start */
		if (sdhci_simple_poll_for_state(host, mask, mask,
						SDHCI_COMMAND_TIMEOUT) != 0) {
			err = -ETIMEDOUT;
			goto fail;
		}
	}

	/*
	 * Some controllers (JMicron JMB38x) mess up the buffer bits
	 * for transfers < 4 bytes. As long as it is just one block,
	 * we can ignore the bits.
	 */
	if ((host->quirks & SDHCI_QUIRK_BROKEN_SMALL_PIO) &&
		(host->data->blocks == 1))
		mask = ~0;

	pr_debug("%s: state=0x%08X, mask=0x%08X.\n", __func__, sdhci_readl(host,
	SDHCI_PRESENT_STATE), mask);
	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		/* Use the interrupt status bits as the buffer bits */
		if (sdhci_simple_poll_for_irq(host, irq, irq,
					      SDHCI_COMMAND_TIMEOUT) != 0) {
			err = -ETIMEDOUT;
			goto fail;
		}

		if (host->data->flags & MMC_DATA_READ)
			sdhci_simple_read_block_pio(host);
		else
			sdhci_simple_write_block_pio(host);

		host->blocks--;
		if (host->blocks == 0)
			break;
	}

	if (host->quirks & SDHCI_QUIRK_BROKEN_PIO)
		if (sdhci_simple_poll_for_irq(host, SDHCI_INT_DATA_END,
					      SDHCI_INT_DATA_END,
					      SDHCI_COMMAND_TIMEOUT) != 0) {
			err = -ETIMEDOUT;
			goto fail;
		}

	if (sdhci_simple_poll_for_state(host, SDHCI_DATA_INHIBIT, 0,
					SDHCI_COMMAND_TIMEOUT) != 0)
		err = -ETIMEDOUT;

fail:
	pr_debug("%s: transfer complete (%d).\n", __func__, err);
	return err;
}

static u8 sdhci_simple_calc_timeout(struct sdhci_host *host, struct mmc_data *data)
{
	u8 count;
	unsigned target_timeout, current_timeout;

	/*
	 * If the host controller provides us with an incorrect timeout
	 * value, just skip the check and use 0xE.  The hardware may take
	 * longer to time out, but that's much better than having a too-short
	 * timeout value.
	 */
	if ((host->quirks & SDHCI_QUIRK_BROKEN_TIMEOUT_VAL))
		return 0xE;

	/* timeout in us */
	target_timeout = data->timeout_ns / 1000 +
		data->timeout_clks / host->clock;

	/*
	 * Figure out needed cycles.
	 * We do this in steps in order to fit inside a 32 bit int.
	 * The first step is the minimum timeout, which will have a
	 * minimum resolution of 6 bits:
	 * (1) 2^13*1000 > 2^22,
	 * (2) host->timeout_clk < 2^16
	 *     =>
	 *     (1) / (2) > 2^6
	 */
	count = 0;
	current_timeout = (1 << 13) * 1000 / host->timeout_clk;
	while (current_timeout < target_timeout) {
		count++;
		current_timeout <<= 1;
		if (count >= 0xF)
			break;
	}

	if (count >= 0xF) {
		pr_warning("%s: Too large timeout requested!\n",
			__func__);
		count = 0xE;
	}

	return count;
}

static void sdhci_simple_prepare_data(struct sdhci_host *host, struct mmc_data *data)
{
	u8 count;
	u8 ctrl;

	if (data == NULL)
		return;

	host->data = data;

	count = sdhci_simple_calc_timeout(host, data);
	sdhci_writeb(host, count, SDHCI_TIMEOUT_CONTROL);

	/*
	 * Always adjust the DMA selection as some controllers
	 * (e.g. JMicron) can't do PIO properly when the selection
	 * is ADMA.
	 */
	if (host->version >= SDHCI_SPEC_200) {
		ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		ctrl &= ~SDHCI_CTRL_DMA_MASK;
		ctrl |= SDHCI_CTRL_SDMA;
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	}

	/* We make a mockery of the scatterlist interator here and use it
	   instead to keep track of our flat buffer during PIO transfers. */
	host->sg_miter.addr = (void*)data->sg->dma_address;
	host->sg_miter.length = data->sg->length;
	host->sg_miter.consumed = 0;

	host->blocks = data->blocks;

	/* We do not handle DMA boundaries, so set it to max (512 KiB) */
	sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, data->blksz), SDHCI_BLOCK_SIZE);
	sdhci_writew(host, data->blocks, SDHCI_BLOCK_COUNT);
}

static void sdhci_simple_set_transfer_mode(struct sdhci_host *host,
	struct mmc_data *data)
{
	u16 mode;

	if (data == NULL)
		return;

	WARN_ON(!host->data);

	mode = SDHCI_TRNS_BLK_CNT_EN;
	if (data->blocks > 1)
		mode |= SDHCI_TRNS_MULTI;
	if (data->flags & MMC_DATA_READ)
		mode |= SDHCI_TRNS_READ;

	sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
}

static void sdhci_simple_set_clock(struct sdhci_host *host, unsigned int clock)
{
	int div;
	u16 clk;
	unsigned long timeout;

	if (clock == host->clock)
		return;

	if (host->ops->set_clock) {
		host->ops->set_clock(host, clock);
		if (host->quirks & SDHCI_QUIRK_NONSTANDARD_CLOCK)
			return;
	}

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;

	host->last_clk = clock;

	for (div = 1;div < 256;div *= 2) {
		if ((host->max_clk / div) <= clock)
			break;
	}
	div >>= 1;

	clk = div << SDHCI_DIVIDER_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never "
				"stabilised.\n", __func__);
			sdhci_simple_dumpregs(host);
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

out:
	host->clock = clock;
}

static void sdhci_simple_set_power(struct sdhci_host *host, unsigned short power)
{
	u8 pwr;

	if (power == (unsigned short)-1)
		pwr = 0;
	else {
		switch (1 << power) {
		case MMC_VDD_165_195:
			pwr = SDHCI_POWER_180;
			break;
		case MMC_VDD_29_30:
		case MMC_VDD_30_31:
			pwr = SDHCI_POWER_300;
			break;
		case MMC_VDD_32_33:
		case MMC_VDD_33_34:
			pwr = SDHCI_POWER_330;
			break;
		default:
			BUG();
		}
	}

	if (host->pwr == pwr)
		return;

	host->pwr = pwr;

	if (pwr == 0) {
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
		return;
	}

	/*
	 * Spec says that we should clear the power reg before setting
	 * a new value. Some controllers don't seem to like this though.
	 */
	if (!(host->quirks & SDHCI_QUIRK_SINGLE_POWER_WRITE))
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);

	/*
	 * At least the Marvell CaFe chip gets confused if we set the voltage
	 * and set turn on power at the same time, so set the voltage first.
	 */
	if (host->quirks & SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER)
		sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

	pwr |= SDHCI_POWER_ON;

	sdhci_writeb(host, pwr, SDHCI_POWER_CONTROL);

	/*
	 * Some controllers need an extra 10ms delay of 10ms before they
	 * can apply clock after applying power
	 */
	if (host->quirks & SDHCI_QUIRK_DELAY_AFTER_POWER)
		mdelay(10);
}

static void sdhci_simple_finish(struct sdhci_host *host)
{
	struct mmc_request *mrq;

	mrq = host->mrq;

	/* FIXME: this isn't a tasklet anymore...need to fix double-calls. */
	if (!mrq)
		return;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if (!(host->flags & SDHCI_DEVICE_DEAD) &&
		(mrq->cmd->error ||
		 (mrq->data && (mrq->data->error ||
		  (mrq->data->stop && mrq->data->stop->error))) ||
		   (host->quirks & SDHCI_QUIRK_RESET_AFTER_REQUEST))) {
		/* Some controllers need this kick or reset won't work here */
		if (host->quirks & SDHCI_QUIRK_CLOCK_BEFORE_RESET) {
			unsigned int clock;

			/* This is to force an update */
			clock = host->clock;
			host->clock = 0;
			sdhci_simple_set_clock(host, clock);
		}

		/* Spec says we should do both at the same time, but Ricoh
		   controllers do not like that. */
		sdhci_simple_reset(host, SDHCI_RESET_CMD);
		sdhci_simple_reset(host, SDHCI_RESET_DATA);
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	/* Clear any IRQ dingleberries */
	sdhci_writel(host, sdhci_readl(host, SDHCI_INT_ENABLE),
	             SDHCI_INT_STATUS);
}

static void sdhci_simple_finish_data(struct sdhci_host *host)
{
	struct mmc_data *data;

	data = host->data;
	host->data = NULL;

	/*
	 * The specification states that the block count register must
	 * be updated, but it does not specify at what point in the
	 * data flow. That makes the register entirely useless to read
	 * back so we have to assume that nothing made it to the card
	 * in the event of an error.
	 */
	if (data->error)
		data->bytes_xfered = 0;
	else
		data->bytes_xfered = data->blksz * data->blocks;

	if (data->stop) {
		/*
		 * The controller needs a reset of internal state machines
		 * upon error conditions.
		 */
		if (data->error) {
			sdhci_simple_reset(host, SDHCI_RESET_CMD);
			sdhci_simple_reset(host, SDHCI_RESET_DATA);
		}

		host->cmd = NULL;
		sdhci_simple_send_command(host, data->stop);
	} else
		sdhci_simple_finish(host);
}

static void sdhci_simple_send_command(struct sdhci_host *host, struct mmc_command *cmd)
{
	int flags;
	u32 mask;
	WARN_ON(host->cmd);

	mask = SDHCI_CMD_INHIBIT;
	if ((cmd->data != NULL) || (cmd->flags & MMC_RSP_BUSY))
		mask |= SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (host->mrq->data && (cmd == host->mrq->data->stop))
		mask &= ~SDHCI_DATA_INHIBIT;

	/* Wait max 10 ms */
	if (sdhci_simple_poll_for_state(host, mask, 0, 10) != 0) {
		cmd->error = -EIO;
		sdhci_simple_finish(host);
		return;
	}

	/* FIXME: 10 second timeout started here */

	/* Clear any dangling interrupts */
	sdhci_writel(host, sdhci_readl(host, SDHCI_INT_STATUS),
		     SDHCI_INT_STATUS);

	host->cmd = cmd;

	sdhci_simple_prepare_data(host, cmd->data);

	sdhci_writel(host, cmd->arg, SDHCI_ARGUMENT);

	sdhci_simple_set_transfer_mode(host, cmd->data);

	if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY)) {
		pr_err("%s: Unsupported response type!\n", __func__);
		cmd->error = -EINVAL;
		sdhci_simple_finish(host);
		return;
	}

	if (!(cmd->flags & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->flags & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->flags & MMC_RSP_BUSY)
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
	else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->flags & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->flags & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (cmd->data)
		flags |= SDHCI_CMD_DATA;

	sdhci_writew(host, SDHCI_MAKE_CMD(cmd->opcode, flags), SDHCI_COMMAND);

	if (sdhci_simple_poll_for_state(host, SDHCI_CMD_INHIBIT, 0,
	                                SDHCI_COMMAND_TIMEOUT) != 0) {
		cmd->error = -ETIMEDOUT;
		return;
	}

	if (cmd->data)
		cmd->error = sdhci_simple_transfer_data(host);
}

static void sdhci_simple_finish_command(struct sdhci_host *host)
{
	int i;
	int intmask = sdhci_readl(host, SDHCI_INT_STATUS);

	if (!host->cmd)
		return;

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			/* CRC is stripped so we need to do some shifting. */
			for (i = 0;i < 4;i++) {
				host->cmd->resp[i] = sdhci_readl(host,
					SDHCI_RESPONSE + (3-i)*4) << 8;
				if (i != 3)
					host->cmd->resp[i] |=
						sdhci_readb(host,
						SDHCI_RESPONSE + (3-i)*4-1);
			}
		} else {
			host->cmd->resp[0] = sdhci_readl(host, SDHCI_RESPONSE);
		}
	}

	host->cmd->error = 0;

	if (intmask) {
		if (intmask & SDHCI_INT_TIMEOUT) {
			host->cmd->error = -ETIMEDOUT;
			pr_debug("%s: CMD%d timed out\n", __func__, host->cmd->opcode);
		}
		sdhci_writel(host, intmask, SDHCI_INT_STATUS);
	}

	if (host->data)
		sdhci_simple_finish_data(host);

	if (host->cmd && !host->cmd->data)
		sdhci_simple_finish(host);

	host->cmd = NULL;
}

static void sdhci_simple_do_request(struct sdhci_host *host, struct mmc_request *mrq)
{
	unsigned long flags;
	int present;

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);

	host->mrq = mrq;

	/* If polling, assume that the card is always present. */
	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION) {
		if (host->ops->card_detect)
			present = host->ops->card_detect(host);
		else
			present = true;
	} else
		present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
				SDHCI_CARD_PRESENT;

	if (!present || host->flags & SDHCI_DEVICE_DEAD) {
		host->mrq->cmd->error = -ENOMEDIUM;
		sdhci_simple_finish(host);
	} else {
		sdhci_simple_send_command(host, mrq->cmd);
		sdhci_simple_finish_command(host);
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_simple_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	return sdhci_simple_do_request(&sdhci_simple_host, mrq);
}

static void sdhci_simple_do_set_ios(struct sdhci_host *host, struct mmc_ios *ios)
{
	unsigned long flags;
	u8 ctrl;

	spin_lock_irqsave(&host->lock, flags);

	if (host->flags & SDHCI_DEVICE_DEAD)
		goto out;

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
	if (ios->power_mode == MMC_POWER_OFF) {
		sdhci_writel(host, 0, SDHCI_SIGNAL_ENABLE);
		sdhci_simple_init(host);
	}

	sdhci_simple_set_clock(host, ios->clock);

	if (ios->power_mode == MMC_POWER_OFF)
		sdhci_simple_set_power(host, -1);
	else
		sdhci_simple_set_power(host, ios->vdd);

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	ctrl &= ~(SDHCI_CTRL_8BITBUS|SDHCI_CTRL_4BITBUS);
	if (ios->bus_width == MMC_BUS_WIDTH_8)
		ctrl |= SDHCI_CTRL_8BITBUS;
	else if (ios->bus_width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;

	/* Tegra controllers often fail to detect high-speed cards when
	 * CTRL_HISPD is programmed
	 */
	if (!(host->quirks & SDHCI_QUIRK_BROKEN_CTRL_HISPD)) {
		if (ios->timing == MMC_TIMING_SD_HS)
			ctrl |= SDHCI_CTRL_HISPD;
		else
			ctrl &= ~SDHCI_CTRL_HISPD;
	}

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

	/*
	 * Some (ENE) controllers go apeshit on some ios operation,
	 * signalling timeout and CRC errors even on CMD0. Resetting
	 * it on each ios seems to solve the problem.
	 */
	if(host->quirks & SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS)
		sdhci_simple_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

out:
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_simple_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	return sdhci_simple_do_set_ios(&sdhci_simple_host, ios);
}

#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
static unsigned int sdhci_simple_get_host_offset(struct mmc_host *mmc) {
	return sdhci_simple_host.start_offset;
}
#endif

static const struct mmc_host_ops sdhci_ops = {
	.request	= sdhci_simple_request,
	.set_ios	= sdhci_simple_set_ios,
	.get_ro		= NULL,
	.enable 	= NULL,
	.disable	= NULL,
	.enable_sdio_irq = NULL,
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	.get_host_offset = sdhci_simple_get_host_offset,
#endif
};

/*****************************************************************************\
 *                                                                           *
 * Device allocation/registration                                            *
 *                                                                           *
\*****************************************************************************/

struct sdhci_host *sdhci_simple_alloc_host(int id)
{
	struct sdhci_host *host = &sdhci_simple_host;

	host->mmc = &sdhci_simple_mmc_host;

	host->mmc->index = id;
	host->mmc->max_hw_segs = 1;
	host->mmc->max_phys_segs = 1;
	host->mmc->max_seg_size = PAGE_CACHE_SIZE;
	host->mmc->max_req_size = PAGE_CACHE_SIZE;
	host->mmc->max_blk_size = 512;
	host->mmc->max_blk_count = PAGE_CACHE_SIZE / 512;

	return host;
}

EXPORT_SYMBOL_GPL(sdhci_simple_alloc_host);

int sdhci_simple_add_host(struct sdhci_host *host)
{
	struct mmc_host *mmc;
	unsigned int caps;
	int ret;

	WARN_ON(host == NULL);
	if (host == NULL)
		return -EINVAL;

	mmc = host->mmc;

	if (debug_quirks)
		host->quirks = debug_quirks;

	sdhci_simple_reset(host, SDHCI_RESET_ALL);

	if (!(host->quirks & SDHCI_QUIRK_BROKEN_SPEC_VERSION)) {
		host->version = sdhci_readw(host, SDHCI_HOST_VERSION);
		host->version = (host->version & SDHCI_SPEC_VER_MASK)
			>> SDHCI_SPEC_VER_SHIFT;
	}
	if (host->version > SDHCI_SPEC_200) {
		pr_err("%s: Unknown controller version (%d). "
			"You may experience problems.\n", __func__,
			host->version);
	}

	caps = sdhci_readl(host, SDHCI_CAPABILITIES);

	host->flags &= ~SDHCI_USE_ADMA;

	host->start_offset = 0;

	host->max_clk = 0;
	if (host->ops->get_max_clock)
		host->max_clk = host->ops->get_max_clock(host);

	if (host->max_clk == 0)
		host->max_clk =
			(caps & SDHCI_CLOCK_BASE_MASK) >> SDHCI_CLOCK_BASE_SHIFT;
	if (host->max_clk == 0) {
		pr_err("%s: Hardware doesn't specify base clock "
			"frequency.\n", __func__);
		return -ENODEV;
	}
	host->max_clk *= 1000000;

	host->timeout_clk =
		(caps & SDHCI_TIMEOUT_CLK_MASK) >> SDHCI_TIMEOUT_CLK_SHIFT;
	if (host->timeout_clk == 0) {
		pr_err("%s: Hardware doesn't specify timeout clock "
			"frequency.\n", __func__);
		return -ENODEV;
	}
	if (caps & SDHCI_TIMEOUT_CLK_UNIT)
		host->timeout_clk *= 1000;

	/*
	 * Set host parameters.
	 */
	mmc->ops = &sdhci_ops;
	if (host->quirks & SDHCI_QUIRK_NONSTANDARD_CLOCK &&
			host->ops->set_clock && host->ops->get_min_clock)
		mmc->f_min = host->ops->get_min_clock(host);
	else
		mmc->f_min = host->max_clk / 256;
	mmc->f_max = host->max_clk;
	mmc->caps = 0;

	if (!(host->quirks & SDHCI_QUIRK_FORCE_1_BIT_DATA))
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (!(host->quirks & SDHCI_QUIRK_NO_SDIO_IRQ))
		mmc->caps |= MMC_CAP_SDIO_IRQ;

	if (caps & SDHCI_CAN_DO_HISPD)
		mmc->caps |= (MMC_CAP_SD_HIGHSPEED |
			MMC_CAP_MMC_HIGHSPEED);

	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION) {
		if (!host->ops->card_detect)
			mmc->caps |= MMC_CAP_NEEDS_POLL;
	}

	if (host->data_width >= 8)
		mmc->caps |= MMC_CAP_8_BIT_DATA;

	mmc->ocr_avail = 0;
	if (caps & SDHCI_CAN_VDD_330)
		mmc->ocr_avail |= MMC_VDD_32_33|MMC_VDD_33_34;
	if (caps & SDHCI_CAN_VDD_300)
		mmc->ocr_avail |= MMC_VDD_29_30|MMC_VDD_30_31;
	if (caps & SDHCI_CAN_VDD_180)
		mmc->ocr_avail |= MMC_VDD_165_195;

	if (mmc->ocr_avail == 0) {
		pr_err("%s: Hardware doesn't report any "
			"support voltages.\n", __func__);
		return -ENODEV;
	}

	spin_lock_init(&host->lock);

	mmc->max_hw_segs = 128;
	mmc->max_phys_segs = 128;
	mmc->max_req_size = 524288;
	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Maximum block size. This varies from controller to controller and
	 * is specified in the capabilities register.
	 */
	if (host->quirks & SDHCI_QUIRK_FORCE_BLK_SZ_2048) {
		mmc->max_blk_size = 2;
	} else {
		mmc->max_blk_size = (caps & SDHCI_MAX_BLOCK_MASK) >>
				SDHCI_MAX_BLOCK_SHIFT;
		if (mmc->max_blk_size >= 3) {
			pr_debug("%s: Invalid maximum block size (%d), "
				 "assuming 512 bytes\n", __func__, 
				 512 << mmc->max_blk_size);
			mmc->max_blk_size = 0;
		}
	}

	mmc->max_blk_count = (host->quirks & SDHCI_QUIRK_NO_MULTIBLOCK) ? 1 : 65535;

	/*
	 * Maximum block count.
	 */
	mmc->max_blk_count = 65535;

	sdhci_simple_init(host);

#ifdef CONFIG_MMC_DEBUG
	sdhci_simple_dumpregs(host);
#endif

	mmiowb();

	ret = mmc_simple_add_host(mmc);

	pr_debug("%s: SDHCI-simple controller (%d)\n",
		 __func__, ret);

	return ret;
}

EXPORT_SYMBOL_GPL(sdhci_simple_add_host);


/*****************************************************************************\
 *                                                                           *
 * Simple MMC framework platform driver hooks                                *
 *                                                                           *
\*****************************************************************************/

/*
 * mmc_simple_platform_init - hookup to the simple MMC/SD/SDIO framework
 *
 * @id:          the controller interface index (i.e. the port)
 *
 * This "simple" platform driver for the simple MMC framework is in itself a
 * framework of sorts for SDHCI-compliant hardware drivers.  In a similar vein,
 * an actual controller driver is required to make a complete SDHCI driver.
 * That controller driver must implement these ops:
 *
 *   set_clock()
 */
int mmc_simple_platform_init(int id)
{
	return sdhci_simple_host_init(id);
}

EXPORT_SYMBOL_GPL(mmc_simple_platform_init);

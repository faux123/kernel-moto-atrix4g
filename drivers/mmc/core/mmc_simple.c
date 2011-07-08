/*
 *  linux/drivers/mmc/core/mmc_simple.c
 *
 * Simple MMC framework implementation that does not depend on the kernel MMC
 * framework.  The purpose is to be able to access an MMC/SD/SDIO device after
 * the kernel has panicked (no scheduling).  It depends on a simple driver
 * implementation for the current platform.
 *
 * Copyright (c) 2010, Motorola.
 *
 * Authors:
 *	Russ W. Knize	<russ.knize@motorola.com>
 *
 * Based on:
 *   drivers/mmc/core/core.c
 *   drivers/mmc/core/host.c
 *   drivers/mmc/core/mmc.c
 *   drivers/mmc/core/mmc_ops.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2006-2008 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/types.h>
#include <asm/scatterlist.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>

#include <linux/mmc/mmc_simple.h>

#include "core.h"
#include "mmc_ops.h"


#define FORCE_512_BLOCKS

#define CARD_STATE_1	(1 << 0)
#define CARD_STATE_2	(1 << 1)
#define CARD_STATE_TRAN	(1 << 2)
#define CARD_STATE_3	(1 << 3)

#define MMC_SIMPLE_TYPE_SDIO	1
#define MMC_SIMPLE_TYPE_SD	2
#define MMC_SIMPLE_TYPE_MMC	3


static struct mmc_host *mmc_simple_host;
static struct mmc_card mmc_simple_card;
static sector_t mmc_simple_first_block;
static sector_t mmc_simple_total_blocks;
static size_t mmc_simple_block_size;


/* Keep this beast off of the stack */
#define MMC_EXT_CSD_LEN 512
static u8 mmc_simple_ext_csd[MMC_EXT_CSD_LEN];


static void mmc_simple_do_request(struct mmc_host *host, struct mmc_request *mrq)
{
	int err;

	pr_debug("%s: starting CMD%u arg %08x flags %08x\n",
		 __func__, mrq->cmd->opcode,
		 mrq->cmd->arg, mrq->cmd->flags);

	if (mrq->data) {
		pr_debug("%s:     data 0x%p len %d blksz %d blocks %d "
			 "flags %08x tsac %d ms nsac %d\n",
			 __func__, mrq->data->sg,
			 mrq->data->sg_len, mrq->data->blksz,
			 mrq->data->blocks, mrq->data->flags,
			 mrq->data->timeout_ns / 1000000,
			 mrq->data->timeout_clks);
	}

	if (mrq->stop) {
		pr_debug("%s:     CMD%u arg %08x flags %08x\n",
			 __func__, mrq->stop->opcode,
			 mrq->stop->arg, mrq->stop->flags);
	}

	do {
		mrq->cmd->error = 0;
		mrq->cmd->mrq = mrq;
		if (mrq->data) {
			mrq->cmd->data = mrq->data;
			mrq->data->error = 0;
			mrq->data->mrq = mrq;
			if (mrq->stop) {
				mrq->data->stop = mrq->stop;
				mrq->stop->error = 0;
				mrq->stop->mrq = mrq;
			}
		}

		/* Synchronous function.  Returns when command is complete. */
		host->ops->request(host, mrq);

		err = mrq->cmd->error;
		if (err) {
			if (mrq->cmd->retries == 0)
				break;

			pr_info("%s: req failed (CMD%u): %d, retrying...\n",
				__func__, mrq->cmd->opcode, err);
			mrq->cmd->retries--;
		}
	} while (err);

	pr_debug("%s: req done (CMD%u): %d: %08x %08x %08x %08x\n",
		 __func__, mrq->cmd->opcode, err,
		 mrq->cmd->resp[0], mrq->cmd->resp[1],
		 mrq->cmd->resp[2], mrq->cmd->resp[3]);

	if (mrq->data) {
		pr_debug("%s:     %d bytes transferred: %d\n",
			 __func__,
			 mrq->data->bytes_xfered, mrq->data->error);
	}

	if (mrq->stop) {
		pr_debug("%s:     (CMD%u): %d: %08x %08x %08x %08x\n",
			 __func__, mrq->stop->opcode,
			 mrq->stop->error,
			 mrq->stop->resp[0], mrq->stop->resp[1],
			 mrq->stop->resp[2], mrq->stop->resp[3]);
	}
}

static int mmc_simple_wait_for_cmd(struct mmc_host *host, struct mmc_command *cmd, int retries)
{
	struct mmc_request mrq;

	memset(&mrq, 0, sizeof(struct mmc_request));

	memset(cmd->resp, 0, sizeof(cmd->resp));
	cmd->retries = retries;

	mrq.cmd = cmd;
	cmd->data = NULL;

	mmc_simple_do_request(host, &mrq);

	return cmd->error;
}

/* For SD cards */
static int mmc_simple_app_cmd(struct mmc_host *host, struct mmc_card *card)
{
	int err;
	struct mmc_command cmd;

	cmd.opcode = MMC_APP_CMD;

	if (card) {
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.arg = 0;
		cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_BCR;
	}

	err = mmc_simple_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	/* Check that card supported application commands */
	if (!(cmd.resp[0] & R1_APP_CMD))
		return -EOPNOTSUPP;

	return 0;
}

int mmc_simple_wait_for_app_cmd(struct mmc_host *host, struct mmc_card *card,
                                struct mmc_command *cmd, int retries)
{
	struct mmc_request mrq;

	int i, err;

	err = -EIO;

	/*
	 * We have to resend MMC_APP_CMD for each attempt so
	 * we cannot use the retries field in mmc_command.
	 */
	for (i = 0;i <= retries;i++) {
		memset(&mrq, 0, sizeof(struct mmc_request));

		err = mmc_simple_app_cmd(host, card);
		if (err)
			continue;

		memset(&mrq, 0, sizeof(struct mmc_request));

		memset(cmd->resp, 0, sizeof(cmd->resp));
		cmd->retries = 0;

		mrq.cmd = cmd;
		cmd->data = NULL;

		mmc_simple_do_request(host, &mrq);

		err = cmd->error;
		if (!cmd->error)
			break;
	}

	return err;
}

static int mmc_simple_select_card(struct mmc_host *host, struct mmc_card *card)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_simple_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);

	if (err)
		pr_err("mmc_simple: host(%d) select card failed (%d)\n",
		       host->index, err);
	return 0;
}

static int mmc_simple_go_idle(struct mmc_host *host)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_GO_IDLE_STATE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_NONE | MMC_CMD_BC;

	err = mmc_simple_wait_for_cmd(host, &cmd, 0);

	mdelay(1);

	if (err)
		pr_err("mmc_simple: host(%d) go idle failed (%d)\n",
		       host->index, err);
	return err;
}

static int mmc_simple_send_if_cond(struct mmc_host *host, u32 ocr)
{
	struct mmc_command cmd;
	int err;
	static const u8 test_pattern = 0xAA;
	u8 result_pattern;

	/*
	 * To support SD 2.0 cards, we must always invoke SD_SEND_IF_COND
	 * before SD_APP_OP_COND. This command will harmlessly fail for
	 * SD 1.0 cards.
	 */
	cmd.opcode = SD_SEND_IF_COND;
	cmd.arg = ((ocr & 0xFF8000) != 0) << 8 | test_pattern;
	cmd.flags = MMC_RSP_SPI_R7 | MMC_RSP_R7 | MMC_CMD_BCR;

	err = mmc_simple_wait_for_cmd(host, &cmd, 0);
	if (err)
		return err;

	result_pattern = cmd.resp[0] & 0xFF;

	if (result_pattern != test_pattern)
		return -EIO;

	return 0;
}

static int mmc_simple_send_op_cond(struct mmc_host *host, int type, u32 ocr, u32 *rocr)
{
	struct mmc_command cmd;
	int i, err = 0;

	memset(&cmd, 0, sizeof(struct mmc_command));

	switch (type) {
		case MMC_SIMPLE_TYPE_SDIO:
			cmd.opcode = SD_IO_SEND_OP_COND;
			cmd.arg = ocr;
			cmd.flags = MMC_RSP_SPI_R4 | MMC_RSP_R4 | MMC_CMD_BCR;
			break;
		case MMC_SIMPLE_TYPE_SD:
			cmd.opcode = SD_APP_OP_COND;
			cmd.arg = mmc_host_is_spi(host) ? ocr & (1 << 30) : ocr;
			cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;
			break;
		case MMC_SIMPLE_TYPE_MMC:
			cmd.opcode = MMC_SEND_OP_COND;
			cmd.arg = mmc_host_is_spi(host) ? 0 : ocr;
			cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;
			break;
		default:
			return -EINVAL;
	}

	for (i = 100; i; i--) {
		if (type == MMC_SIMPLE_TYPE_SD)
			err = mmc_simple_wait_for_app_cmd(host, NULL, &cmd, MMC_CMD_RETRIES);
		else
			err = mmc_simple_wait_for_cmd(host, &cmd, 0);

		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (cmd.resp[0] & MMC_CARD_BUSY)
			break;

		err = -ETIMEDOUT;

		mdelay(10);
	}

	if (rocr)
		*rocr = cmd.resp[0];

	if (err)
		pr_debug("mmc_simple: host(%d) check voltage failed (%d)\n",
		         host->index, err);
	return err;
}

static int mmc_simple_all_send_cid(struct mmc_host *host, u32 *cid)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_ALL_SEND_CID;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_BCR;

	err = mmc_simple_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err) {
		pr_err("mmc_simple: host(%d) send all CID failed (%d)\n",
		       host->index, err);
		return err;
	}

	memcpy(cid, cmd.resp, sizeof(u32) * 4);

	return 0;
}

static int mmc_simple_set_relative_addr(struct mmc_card *card)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SET_RELATIVE_ADDR;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_simple_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);

	if (err)
		pr_err("mmc_simple: host(%d) set relative address failed (%d)\n",
		       card->host->index, err);
	return err;
}

static int
mmc_simple_send_cxd_native(struct mmc_host *host, u32 arg, u32 *cxd, int opcode)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = mmc_simple_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err) {
		pr_err("mmc_simple: host(%d) send CxD %d failed (%d)\n",
		       opcode, host->index, err);
		return err;
	}

	memcpy(cxd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

static int mmc_simple_send_csd(struct mmc_card *card, u32 *csd)
{
	return mmc_simple_send_cxd_native(card->host, card->rca << 16,
			csd, MMC_SEND_CSD);
}

static int mmc_simple_send_ext_csd(struct mmc_card *card)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));
	memset(&sg, 0, sizeof(struct scatterlist));
	memset(mmc_simple_ext_csd, 0, sizeof(mmc_simple_ext_csd));

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = MMC_SEND_EXT_CSD;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = MMC_EXT_CSD_LEN;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;

	/* Cludge a scatterlist structure to get our buffer through. */
	sg.dma_address = (dma_addr_t)mmc_simple_ext_csd;
	sg.length = MMC_EXT_CSD_LEN;
	data.sg = &sg;

	mmc_simple_do_request(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

static int mmc_simple_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	/*
	 * The selection of the format here is based upon published
	 * specs from sandisk and from what people have reported.
	 */
	switch (card->csd.mmca_vsn) {
	case 0: /* MMC v1.0 - v1.2 */
	case 1: /* MMC v1.4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 104, 24);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.prod_name[6]	= UNSTUFF_BITS(resp, 48, 8);
		card->cid.hwrev		= UNSTUFF_BITS(resp, 44, 4);
		card->cid.fwrev		= UNSTUFF_BITS(resp, 40, 4);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 24);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	case 2: /* MMC v2.0 - v2.2 */
	case 3: /* MMC v3.1 - v3.3 */
	case 4: /* MMC v4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 120, 8);
		card->cid.cbx           = UNSTUFF_BITS(resp, 112, 2);
		card->cid.oemid         = UNSTUFF_BITS(resp, 104, 8);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 32);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	default:
		pr_err("%s: card has unknown MMCA version %d\n",
			__func__, card->csd.mmca_vsn);
		return -EINVAL;
	}

	return 0;
}

static int mmc_simple_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, csd_struct;
	u32 *resp = card->raw_csd;

	/*
	 * We only understand CSD structure v1.1 and v1.2.
	 * v1.2 has extra information in bits 15, 11 and 10.
	 */
	csd_struct = UNSTUFF_BITS(resp, 126, 2);
	if (csd_struct != 1 && csd_struct != 2 && csd_struct != 3) {
		pr_err("%s: unrecognised CSD structure version %d\n",
			__func__, csd_struct);
		return -EINVAL;
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
	m = UNSTUFF_BITS(resp, 115, 4);
	e = UNSTUFF_BITS(resp, 112, 3);
	csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
	csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

	m = UNSTUFF_BITS(resp, 99, 4);
	e = UNSTUFF_BITS(resp, 96, 3);
	csd->max_dtr	  = tran_exp[e] * tran_mant[m];
	csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

	e = UNSTUFF_BITS(resp, 47, 3);
	m = UNSTUFF_BITS(resp, 62, 12);
	csd->capacity	  = (1 + m) << (e + 2);
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	/* for sector-addressed cards, this will cause csd->capacity to wrap */
	csd->capacity -= card->host->ops->get_host_offset(card->host);
#endif

	csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
	csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
	csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
	csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
	csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
	csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
	csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

	return 0;
}

static int mmc_simple_read_ext_csd(struct mmc_card *card)
{
	int err;

	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;

	err = mmc_simple_send_ext_csd(card);  /* writes to mmc_simple_ext_csd */
	if (err) {
		/* If the host or the card can't do the switch,
		 * fail more gracefully. */
		if ((err != -EINVAL)
		 && (err != -ENOSYS)
		 && (err != -EFAULT))
			goto out;

		/*
		 * High capacity cards should have this "magic" size
		 * stored in their CSD.
		 */
		if (card->csd.capacity == (4096 * 512)) {
			pr_warning("%s: unable to read EXT_CSD "
				   "on a possible high capacity card. "
				   "Card will be ignored.\n",
				   __func__);
		} else {
			pr_warning("%s: unable to read "
				   "EXT_CSD, performance might "
				   "suffer.\n",
				   __func__);
			err = 0;
		}

		goto out;
	}

	card->ext_csd.rev = mmc_simple_ext_csd[EXT_CSD_REV];
	if (card->ext_csd.rev > 5) {
		pr_err("%s: unrecognised EXT_CSD structure "
		       "version %d\n", __func__, card->ext_csd.rev);
		err = -EINVAL;
		goto out;
	}

	if (card->ext_csd.rev >= 2) {
		card->ext_csd.sectors =
			mmc_simple_ext_csd[EXT_CSD_SEC_CNT + 0] << 0 |
			mmc_simple_ext_csd[EXT_CSD_SEC_CNT + 1] << 8 |
			mmc_simple_ext_csd[EXT_CSD_SEC_CNT + 2] << 16 |
			mmc_simple_ext_csd[EXT_CSD_SEC_CNT + 3] << 24;
		if (card->ext_csd.sectors) {
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
			unsigned offs;
			offs = card->host->ops->get_host_offset(card->host);
			offs >>= 9;
			BUG_ON(offs > card->ext_csd.sectors);
			card->ext_csd.sectors -= offs;
#endif
			mmc_card_set_blockaddr(card);
		}

	}

	switch (mmc_simple_ext_csd[EXT_CSD_CARD_TYPE]) {
	case EXT_CSD_CARD_TYPE_52 | EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		break;
	case EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 26000000;
		break;
	default:
		/* MMC v4 spec says this cannot happen */
		pr_debug("%s: card is mmc v4 but doesn't "
			 "support any high-speed modes.\n", __func__);
		goto out;
	}

	if (card->ext_csd.rev >= 3) {
		u8 sa_shift = mmc_simple_ext_csd[EXT_CSD_S_A_TIMEOUT];

		/* Sleep / awake timeout in 100ns units */
		if (sa_shift > 0 && sa_shift <= 0x17)
			card->ext_csd.sa_timeout =
				1 << mmc_simple_ext_csd[EXT_CSD_S_A_TIMEOUT];
	}

out:
	return err;
}

static int mmc_simple_switch(struct mmc_card *card, u8 set, u8 index, u8 value)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		  (index << 16) |
		  (value << 8) |
		  set;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	err = mmc_simple_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);

	if (err)
		pr_err("mmc_simple: host(%d) set bus width failed (%d)\n",
		       card->host->index, err);
	return err;
}

static int mmc_simple_send_status(struct mmc_card *card, u32 *status)
{
	int err;
	struct mmc_command cmd;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_simple_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err) {
		pr_err("mmc_simple: host(%d) send status failed (%d)\n",
		       card->host->index, err);
		return err;
	}

	/* NOTE: callers are required to understand the difference
	 * between "native" and SPI format status words!
	 */
	if (status)
		*status = cmd.resp[0];

	return 0;
}

static void mmc_simple_set_ios(struct mmc_host *host)
{
	struct mmc_ios *ios = &host->ios;

	pr_debug("%s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 __func__, ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);

	if (host->ops->set_ios)
		host->ops->set_ios(host, ios);
}

/*
 * Apply power to the MMC stack.  This is a two-stage process.
 * First, we enable power to the card without the clock running.
 * We then wait a bit for the power to stabilise.  Finally,
 * enable the bus drivers and clock to the card.
 *
 * We must _NOT_ enable the clock prior to power stablising.
 *
 * If a host does all the power sequencing itself, ignore the
 * initial MMC_POWER_UP stage.
 */
static void mmc_simple_power_up(struct mmc_host *host)
{
	int bit;

	/* If ocr is set, we use it */
	if (host->ocr)
		bit = ffs(host->ocr) - 1;
	else
		bit = fls(host->ocr_avail) - 1;

	host->ios.vdd = bit;
	host->ios.chip_select = MMC_CS_HIGH;
	host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	host->ios.power_mode = MMC_POWER_UP;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	mmc_simple_set_ios(host);

	/*
	 * This delay should be sufficient to allow the power supply
	 * to reach the minimum voltage.
	 */
	mdelay(10);

	host->ios.clock = host->f_min;
	host->ios.power_mode = MMC_POWER_ON;
	mmc_simple_set_ios(host);

	/*
	 * This delay must be at least 74 clock sizes, or 1 ms, or the
	 * time required to reach a stable voltage.
	 */
	mdelay(10);
}

static int mmc_simple_identify_card(struct mmc_host *host)
{
	u32 ocr;
	int type, bit, err;

	mmc_simple_power_up(host);
	mmc_simple_go_idle(host);
	mmc_simple_send_if_cond(host, host->ocr_avail);

	for (type = 1; type <= 3; type++)
	{
		mdelay(20);
		err = mmc_simple_send_op_cond(host, type, 0, &ocr);
		if (err == 0) {
			break;
		}
	}
	if (err) {
		pr_err("mmc_simple: host(%d) setting OCR failed "
		       "(%d)\n", host->index, type);
		goto fail;
	}

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		pr_debug("mmc_simple: card claims to support voltages "
		         "below the defined range. These will be ignored.\n");
		ocr &= ~0x7F;
	}
	if (ocr & MMC_VDD_165_195) {
		pr_debug("mmc_simple: card claims to support the "
		         "incompletely defined 'low voltage range'. This "
		         "will be ignored.\n");
		ocr &= ~MMC_VDD_165_195;
	}

	ocr &= host->ocr_avail;

	bit = ffs(ocr);
	if (bit) {
		bit -= 1;

		ocr &= 3 << bit;

		host->ios.vdd = bit;
		mmc_simple_set_ios(host);
	} else {
		pr_warning("mmc_simple: host doesn't support card's voltages\n");
		ocr = 0;
	}

	host->ocr = ocr;

	return type;

fail:
	return err;
}

static int mmc_simple_init_mmc(struct mmc_host *host)
{
	u32 cid[4];
	unsigned int max_dtr;
	u32 status;
	int err;

	mmc_simple_go_idle(host);

	/* The extra bit indicates that we support high capacity */
	err = mmc_simple_send_op_cond(host, MMC_SIMPLE_TYPE_MMC,
	                              host->ocr | (1 << 30), NULL);
	if (err)
		goto fail;

	err = mmc_simple_all_send_cid(host, cid);
	if (err)
		goto fail;

	host->card = &mmc_simple_card;
	host->card->host = host;
	host->card->type = MMC_TYPE_MMC;
	host->card->rca = 1;
	memcpy(host->card->raw_cid, cid, sizeof(host->card->raw_cid));

	err = mmc_simple_set_relative_addr(host->card);
	if (err)
		goto fail;

	host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	mmc_simple_set_ios(host);

	err = mmc_simple_send_csd(host->card, host->card->raw_csd);
	if (err)
		goto fail;

	err = mmc_simple_decode_csd(host->card);
	if (err)
		goto fail;

	err = mmc_simple_decode_cid(host->card);
	if (err)
		goto fail;

	err = mmc_simple_select_card(host, host->card);
	if (err)
		goto fail;

	err = mmc_simple_read_ext_csd(host->card);
	if (err)
		goto fail;

	mdelay(20);

	/* Try to activate high speed mode */
	if ((host->card->ext_csd.hs_max_dtr != 0) &&
	    (host->caps & MMC_CAP_MMC_HIGHSPEED)) {
		err = mmc_simple_switch(host->card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_HS_TIMING, 1);
		if (err) {
			pr_warning("mmc_simple: host(%d) switching to HIGH"
				   " speed failed (%d)\n", host->index, err);
		} else {
			mmc_card_set_highspeed(host->card);  /* macro */

			host->ios.timing = MMC_TIMING_MMC_HS;
			mmc_simple_set_ios(host);
		}
	}

	/* Compute bus speed */
	max_dtr = (unsigned int)-1;
	if (mmc_card_highspeed(host->card)) {
		if (max_dtr > host->card->ext_csd.hs_max_dtr)
			max_dtr = host->card->ext_csd.hs_max_dtr;
	} else if (max_dtr > host->card->csd.max_dtr) {
		max_dtr = host->card->csd.max_dtr;
	}

	if (max_dtr > host->f_max)
	max_dtr = host->f_max;

	host->ios.clock = max_dtr;
	mmc_simple_set_ios(host);

	/* Can we talk to the card? */
	err = mmc_simple_send_status(host->card, &status);
	if (err)
		goto fail;

	if (R1_CURRENT_STATE(status) != CARD_STATE_TRAN)
		pr_warning("mmc_simple: host(%d) state=%#x\n",
			host->index, R1_CURRENT_STATE(status));

	mdelay(20);

	/* Try to activate wide bus */
	if ((host->card->csd.mmca_vsn >= CSD_SPEC_VER_4) &&
	    (host->caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA))) {
		unsigned ext_csd_bit, bus_width;

		if (host->caps & MMC_CAP_8_BIT_DATA) {
			ext_csd_bit = EXT_CSD_BUS_WIDTH_8;
			bus_width = MMC_BUS_WIDTH_8;
		} else {
			ext_csd_bit = EXT_CSD_BUS_WIDTH_4;
			bus_width = MMC_BUS_WIDTH_4;
		}

		err = mmc_simple_switch(host->card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_BUS_WIDTH, ext_csd_bit);

		if (err) {
			pr_warning("mmc_simple: host(%d) switching to wide"
				   " bus failed (%d)\n", host->index, err);
		} else {
			host->ios.bus_width = bus_width;
			mmc_simple_set_ios(host);
		}
	}

	mdelay(20);

	/* Can we talk to the card? */
	err = mmc_simple_send_status(host->card, &status);
	if (err)
		goto fail;

	if (R1_CURRENT_STATE(status) != CARD_STATE_TRAN)
		pr_warning("mmc_simple: host(%d) state=%#x\n",
			host->index, R1_CURRENT_STATE(status));

fail:
	return err;
}

static int mmc_simple_init_sd(struct mmc_host *host)
{
	int err = -EOPNOTSUPP;

	pr_err("%s: SDIO cards are not supported\n", __func__);

	return err;
}

static int mmc_simple_init_sdio(struct mmc_host *host)
{
	int err = -EOPNOTSUPP;

	pr_err("%s: SDIO cards are not supported\n", __func__);

	return err;
}

static size_t mmc_simple_transfer(char *buf, size_t len, size_t offset,
                                  unsigned int mode)
{
	struct mmc_command cmd;
	struct mmc_data    data;
	struct mmc_command stop;
	struct mmc_request req;
	struct scatterlist sg;
	u32 sector;
	u32 opcode, opcode_multi;
	size_t result = -1;

	if (mode & MMC_DATA_READ) {
		opcode = MMC_READ_SINGLE_BLOCK;
		opcode_multi = MMC_READ_MULTIPLE_BLOCK;
		pr_debug("%s: read", __func__);
	} else if (mode & MMC_DATA_WRITE) {
		opcode = MMC_WRITE_BLOCK;
		opcode_multi = MMC_WRITE_MULTIPLE_BLOCK;
		pr_debug("%s: write", __func__);
	} else {
		pr_err("mmc_simple: unsupported transfer mode: %u\n", mode);
		return -1;
	}
	pr_debug("0x%p: %d@%d\n", buf, len, offset);

	/* Some sanity checks */
	if (len == 0 || !mmc_simple_host)
		return 0;

	if (offset + len > mmc_simple_total_blocks * mmc_simple_block_size) {
		pr_err("mmc_simple: request too long\n");
		return -1;
	}

	if (offset % mmc_simple_block_size) {
		pr_err("mmc_simple: unaligned transfers not supported\n");
		return -1;
	}

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&stop, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));
	memset(&sg, 0, sizeof(struct scatterlist));

	sector = (u32)mmc_simple_first_block +
			(offset / mmc_simple_block_size);

	data.blksz = mmc_simple_block_size;
	data.blocks = (len + mmc_simple_block_size - 1) /
						mmc_simple_block_size;
	data.timeout_ns = 300000000;
	data.timeout_clks = 0;
	data.flags |= mode;

	/* Cludge a scatterlist structure to get our buffer through. */
	sg.dma_address = (dma_addr_t)buf;
	sg.length = len;
	data.sg = &sg;

	cmd.arg = sector;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	if (data.blocks > 1) {
		cmd.opcode = opcode_multi;

		stop.opcode = MMC_STOP_TRANSMISSION;
		stop.arg = 0;
		stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
		req.stop = &stop;
	} else
		cmd.opcode = opcode;

	req.cmd = &cmd;
	req.data = &data;

	mmc_simple_do_request(mmc_simple_host, &req);

	result = (req.cmd->error ? req.cmd->error :
	          (req.data->error ? req.data->error :
		   ((req.stop && req.stop->error) ? req.stop->error : len)));

	if (result < 0)
		pr_err("mmc_simple: transfer failed: mode=%#x, cmd->error=%#x,"
			"data->error=%#x, stop->error=%#x, result=%d\n",
			mode, req.cmd->error, req.data->error,
			(req.stop && req.stop->error), result);

	return result;
}

/*
 * mmc_simple_init - initialize the simple MMC/SD/SDIO framework stack
 *
 * @id:          the controller interface index (i.e. the port)
 * @start_block: starting block for the partition to be accessed
 * @blocks:      number of blocks in the partition
 * @block_size:  the block size that the card is expecting
 *
 * This "simple" framework emulates the real framework in some ways, mainly to
 * ease the implementation of the "simple" platform driver.  Since it has to
 * provide hooks similar to a how a normal MMC controller driver does, one can
 * use the "real" hardware controller driver as a starting point.  In short,
 * platform driver implements the mmc_simple_platform_init() call and must
 * provide these ops, all of which must be fully synchronous:
 *
 *   request()
 *   set_ios()
 *   get_host_offset() - if configured
 */
int mmc_simple_init(int id, sector_t start_block,
                    sector_t blocks, size_t block_size)
{
	mmc_simple_host = NULL;

	mmc_simple_first_block  = start_block;
	mmc_simple_total_blocks = blocks;
	mmc_simple_block_size   = block_size;

	/* FIXME: we really need to reset the card more completely */
#ifdef FORCE_512_BLOCKS
	mmc_simple_first_block *= mmc_simple_block_size / 512;
	mmc_simple_block_size   = 512;
#endif

	pr_debug("%s: %llu blocks at block %llu of %u bytes each\n", __func__,
		 mmc_simple_total_blocks,
		 mmc_simple_first_block,
		 mmc_simple_block_size);

	/* Sanity check the partition configuration. */
	if (mmc_simple_total_blocks == 0 ||
	    mmc_simple_block_size % 512) {
		pr_err("%s: invalid partition configuration\n", __func__);
		return -1;
	}

	return mmc_simple_platform_init(id);
}

EXPORT_SYMBOL_GPL(mmc_simple_init);

/*
 * mmc_simple_add_host - add a host to the framework
 *
 * @host: pointer to the host to add
 *
 * This is provided to emulate the normal call flow for an MMC driver.  Only
 * one host can be supported at a time.
 */
int mmc_simple_add_host(struct mmc_host *host)
{
	int type, err = -ENODEV;

	if (!host->ops->request || !host->ops->set_ios)
	{
		pr_err("mmc_simple: required ops missing\n");
		goto fail;
	}

	type = mmc_simple_identify_card(host);
	if (type < 0)
		goto fail;

	switch (type)
	{
		case MMC_SIMPLE_TYPE_SDIO:
			err = mmc_simple_init_sdio(host);
			break;
		case MMC_SIMPLE_TYPE_SD:
			err = mmc_simple_init_sd(host);
			break;
		case MMC_SIMPLE_TYPE_MMC:
			err = mmc_simple_init_mmc(host);
			break;
		default:
			pr_err("mmc_simple: host(%d) invalid host type "
				"(%d)\n", host->index, type);
			goto fail;
	}

	mmc_simple_host = host;

fail:
	return err;
}

EXPORT_SYMBOL_GPL(mmc_simple_add_host);

/*
 * mmc_simple_read - read from the configured partition
 *
 * @buf:    pointer to the buffer to store the read data
 * @len:    length of the buffer in bytes
 * @offset: starting offset within the partition to begin the operation
 *
 * NOTE: read operations currently must be aligned to block boundaries.
 */
size_t mmc_simple_read(char *buf, size_t len, size_t offset)
{
	return mmc_simple_transfer(buf, len, offset, MMC_DATA_READ);
}

EXPORT_SYMBOL_GPL(mmc_simple_read);

/*
 * mmc_simple_write - write to the configured partition
 *
 * @buf:    pointer to the buffer to read from
 * @len:    length of the buffer in bytes
 * @offset: starting offset within the partition to begin the operation
 *
 * NOTE: write operations currently must be aligned to block boundaries.
 */
size_t mmc_simple_write(char *buf, size_t len, size_t offset)
{
	return mmc_simple_transfer(buf, len, offset, MMC_DATA_WRITE);
}

EXPORT_SYMBOL_GPL(mmc_simple_write);

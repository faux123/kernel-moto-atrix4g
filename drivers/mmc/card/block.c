/*
 * Block driver for media (i.e., flash cards)
 *
 * Copyright 2002 Hewlett-Packard Company
 * Copyright 2005-2008 Pierre Ossman
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * HEWLETT-PACKARD COMPANY MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Author:  Andrew Christian
 *          28 May 2002
 */
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/string_helpers.h>
#include <linux/apanic.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include "queue.h"

MODULE_ALIAS("mmc:block");

/*
 * max 32 partitions per card
 */
#define MMC_SHIFT	5
#define MMC_NUM_MINORS	(256 >> MMC_SHIFT)

static DECLARE_BITMAP(dev_use, MMC_NUM_MINORS);

/*
 * There is one mmc_blk_data per slot.
 */
struct mmc_blk_data {
	spinlock_t	lock;
	struct gendisk	*disk;
	struct mmc_queue queue;
	char            *bounce;

	unsigned int	usage;
	unsigned int	read_only;
};

static DEFINE_MUTEX(open_lock);

static struct mmc_blk_data *mmc_blk_get(struct gendisk *disk)
{
	struct mmc_blk_data *md;

	mutex_lock(&open_lock);
	md = disk->private_data;
	if (md && md->usage == 0)
		md = NULL;
	if (md)
		md->usage++;
	mutex_unlock(&open_lock);

	return md;
}

static void mmc_blk_put(struct mmc_blk_data *md)
{
	mutex_lock(&open_lock);
	md->usage--;
	if (md->usage == 0) {
		int devmaj = MAJOR(disk_devt(md->disk));
		int devidx = MINOR(disk_devt(md->disk)) >> MMC_SHIFT;

		if (!devmaj)
			devidx = md->disk->first_minor >> MMC_SHIFT;

		blk_cleanup_queue(md->queue.queue);

		__clear_bit(devidx, dev_use);

		if (md->bounce)
			kfree(md->bounce);

		put_disk(md->disk);
		kfree(md);
	}
	mutex_unlock(&open_lock);
}

static int mmc_blk_open(struct block_device *bdev, fmode_t mode)
{
	struct mmc_blk_data *md = mmc_blk_get(bdev->bd_disk);
	int ret = -ENXIO;

	if (md) {
		if (md->usage == 2)
			check_disk_change(bdev);
		ret = 0;

		if ((mode & FMODE_WRITE) && md->read_only) {
			mmc_blk_put(md);
			ret = -EROFS;
		}
	}

	return ret;
}

static int mmc_blk_release(struct gendisk *disk, fmode_t mode)
{
	struct mmc_blk_data *md = disk->private_data;

	mmc_blk_put(md);
	return 0;
}

static int
mmc_blk_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	geo->cylinders = get_capacity(bdev->bd_disk) / (4 * 16);
	geo->heads = 4;
	geo->sectors = 16;
	return 0;
}

static const struct block_device_operations mmc_bdops = {
	.open			= mmc_blk_open,
	.release		= mmc_blk_release,
	.getgeo			= mmc_blk_getgeo,
	.owner			= THIS_MODULE,
};

struct mmc_blk_request {
	struct mmc_request	mrq;
	struct mmc_command	cmd;
	struct mmc_command	stop;
	struct mmc_data		data;
};

static u32 mmc_sd_num_wr_blocks(struct mmc_card *card)
{
	int err;
	u32 result;
	__be32 *blocks;

	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	unsigned int timeout_us;

	struct scatterlist sg;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = MMC_APP_CMD;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return (u32)-1;
	if (!mmc_host_is_spi(card->host) && !(cmd.resp[0] & R1_APP_CMD))
		return (u32)-1;

	memset(&cmd, 0, sizeof(struct mmc_command));

	cmd.opcode = SD_APP_SEND_NUM_WR_BLKS;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	memset(&data, 0, sizeof(struct mmc_data));

	data.timeout_ns = card->csd.tacc_ns * 100;
	data.timeout_clks = card->csd.tacc_clks * 100;

	timeout_us = data.timeout_ns / 1000;
	timeout_us += data.timeout_clks * 1000 /
		(card->host->ios.clock / 1000);

	if (timeout_us > 100000) {
		data.timeout_ns = 100000000;
		data.timeout_clks = 0;
	}

	data.blksz = 4;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	memset(&mrq, 0, sizeof(struct mmc_request));

	mrq.cmd = &cmd;
	mrq.data = &data;

	blocks = kmalloc(4, GFP_KERNEL);
	if (!blocks)
		return (u32)-1;

	sg_init_one(&sg, blocks, 4);

	mmc_wait_for_req(card->host, &mrq);

	result = ntohl(*blocks);
	kfree(blocks);

	if (cmd.error || data.error)
		result = (u32)-1;

	return result;
}

static u32 get_card_status(struct mmc_card *card, struct request *req)
{
	struct mmc_command cmd;
	int err;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	if (!mmc_host_is_spi(card->host))
		cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		printk(KERN_ERR "%s: error %d sending status comand",
		       req->rq_disk->disk_name, err);
	return cmd.resp[0];
}

static int
mmc_blk_set_blksize(struct mmc_blk_data *md, struct mmc_card *card)
{
	struct mmc_command cmd;
	int err;

	/* Block-addressed cards ignore MMC_SET_BLOCKLEN. */
	if (mmc_card_blockaddr(card))
		return 0;

	mmc_claim_host(card->host);
	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = 512;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 5);
	mmc_release_host(card->host);

	if (err) {
		printk(KERN_ERR "%s: unable to set block size to %d: %d\n",
			md->disk->disk_name, cmd.arg, err);
		return -EINVAL;
	}

	return 0;
}

/*
 * Workaround for Toshiba eMMC performance.  If the request is less than two
 * flash pages in size, then we want to split the write into one or two
 * page-aligned writes to take advantage of faster buffering.  Here we can
 * adjust the size of the MMC request and let the block layer request handler
 * deal with generating another MMC request.
 */
#define TOSHIBA_MANFID 0x11
#define TOSHIBA_PAGE_SIZE 16		/* sectors */
#define TOSHIBA_ADJUST_THRESHOLD 24	/* sectors */
static bool mmc_adjust_toshiba_write(struct mmc_card *card,
                                     struct mmc_request *mrq)
{
	if (mmc_card_mmc(card) && card->cid.manfid == TOSHIBA_MANFID &&
	    mrq->data->blocks <= TOSHIBA_ADJUST_THRESHOLD) {
		int sectors_in_page = TOSHIBA_PAGE_SIZE -
		                      (mrq->cmd->arg % TOSHIBA_PAGE_SIZE);
		if (mrq->data->blocks > sectors_in_page) {
			mrq->data->blocks = sectors_in_page;
			return true;
		}
	}

	return false;
}

/*
 * This is another strange workaround to try to close the gap on Toshiba eMMC
 * performance when compared to other vendors.  In order to take advantage
 * of certain optimizations and assumptions in those cards, we will look for
 * multiblock write transfers below a certain size and we do the following:
 *
 * - Break them up into seperate page-aligned (8k flash pages) transfers.
 * - Execute the transfers in reverse order.
 * - Use "reliable write" transfer mode.
 *
 * Neither the block I/O layer nor the scatterlist design seem to lend them-
 * selves well to executing a block request out of order.  So instead we let
 * mmc_blk_issue_rq() setup the MMC request for the entire transfer and then
 * break it up and reorder it here.  This also requires that we put the data
 * into a bounce buffer and send it as individual sg's.
 */
#define TOSHIBA_LOW_THRESHOLD 48	/* sectors */
#define TOSHIBA_HIGH_THRESHOLD 64	/* sectors */
static bool mmc_handle_toshiba_write(struct mmc_queue *mq,
                                     struct mmc_card *card,
                                     struct mmc_request *mrq)
{
	struct mmc_blk_data *md = mq->data;
	unsigned int first_page, last_page, page;
	unsigned long flags;

	if (!md->bounce ||
	    mrq->data->blocks > TOSHIBA_HIGH_THRESHOLD ||
	    mrq->data->blocks < TOSHIBA_LOW_THRESHOLD)
		return false;

	first_page = mrq->cmd->arg / TOSHIBA_PAGE_SIZE;
	last_page = (mrq->cmd->arg + mrq->data->blocks - 1) / TOSHIBA_PAGE_SIZE;

	/* Single page write: just do it the normal way */
	if (first_page == last_page)
		return false;

	local_irq_save(flags);
	sg_copy_to_buffer(mrq->data->sg, mrq->data->sg_len,
	                  md->bounce, mrq->data->blocks * 512);
	local_irq_restore(flags);

	for (page = last_page; page >= first_page; page--) {
		unsigned long offset, length;
		struct mmc_blk_request brq;
		struct mmc_command cmd;
		struct scatterlist sg;

		memset(&brq, 0, sizeof(struct mmc_blk_request));
		brq.mrq.cmd = &brq.cmd;
		brq.mrq.data = &brq.data;

		brq.cmd.arg = page * TOSHIBA_PAGE_SIZE;
		brq.data.blksz = 512;
		if (page == first_page) {
			brq.cmd.arg = mrq->cmd->arg;
			brq.data.blocks = TOSHIBA_PAGE_SIZE -
			                  (mrq->cmd->arg % TOSHIBA_PAGE_SIZE);
		} else if (page == last_page)
			brq.data.blocks = (mrq->cmd->arg + mrq->data->blocks) %
			                  TOSHIBA_PAGE_SIZE;
		if (brq.data.blocks == 0)
			brq.data.blocks = TOSHIBA_PAGE_SIZE;

		if (!mmc_card_blockaddr(card))
			brq.cmd.arg <<= 9;
		brq.cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
		brq.stop.opcode = MMC_STOP_TRANSMISSION;
		brq.stop.arg = 0;
		brq.stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

		brq.data.flags |= MMC_DATA_WRITE;
		if (brq.data.blocks > 1) {
			if (!mmc_host_is_spi(card->host))
				brq.mrq.stop = &brq.stop;
			brq.cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
		} else {
			brq.mrq.stop = NULL;
			brq.cmd.opcode = MMC_WRITE_BLOCK;
		}

		if (brq.cmd.opcode == MMC_WRITE_MULTIPLE_BLOCK &&
		    brq.data.blocks <= card->ext_csd.rel_wr_sec_c) {
			int err;

			cmd.opcode = MMC_SET_BLOCK_COUNT;
			cmd.arg = brq.data.blocks | (1 << 31);
			cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
			err = mmc_wait_for_cmd(card->host, &cmd, 0);
			if (!err)
				brq.mrq.stop = NULL;
		}

		mmc_set_data_timeout(&brq.data, card);

		offset = (brq.cmd.arg - mrq->cmd->arg) * 512;
		length = brq.data.blocks * 512;
		sg_init_one(&sg, md->bounce + offset, length);
		brq.data.sg = &sg;
		brq.data.sg_len = 1;

		mmc_wait_for_req(card->host, &brq.mrq);

		mrq->data->bytes_xfered += brq.data.bytes_xfered;

		if (brq.cmd.error || brq.data.error || brq.stop.error) {
			mrq->cmd->error = brq.cmd.error;
			mrq->data->error = brq.data.error;
			mrq->stop->error = brq.stop.error;

			/*
			 * We're executing the request backwards, so don't let
			 * the block layer think some part of it has succeeded.
			 * It will get it wrong.  Since the failure will cause
			 * us to fall back on single block writes, we're better
			 * off reporting that none of the data was written.
			 */
			mrq->data->bytes_xfered = 0;
			break;
		}
	}

	return true;
}

static int mmc_blk_xfer_rq(struct mmc_blk_data *md,
	struct request *req, unsigned int *bytes_xfered)
{
	struct mmc_queue *mq = &md->queue;
	struct mmc_card *card = mq->card;
	struct mmc_blk_request brq;
	int ret = 1;
	int disable_multi = 0;
	int retry = 0;

#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	if (mmc_bus_needs_resume(card->host)) {
		mmc_resume_bus(card->host);
		mmc_blk_set_blksize(md, card);
	}
#endif

	BUG_ON(!bytes_xfered);

	do {
		struct mmc_command cmd;
		u32 readcmd, writecmd, status = 0;

		memset(&brq, 0, sizeof(struct mmc_blk_request));
		brq.mrq.cmd = &brq.cmd;
		brq.mrq.data = &brq.data;

		brq.cmd.arg = blk_rq_pos(req);
		if (!mmc_card_blockaddr(card))
			brq.cmd.arg <<= 9;
		brq.cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
		brq.data.blksz = 512;
		brq.stop.opcode = MMC_STOP_TRANSMISSION;
		brq.stop.arg = 0;
		brq.stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
		brq.data.blocks = blk_rq_sectors(req);

		/*
		 * The block layer doesn't support all sector count
		 * restrictions, so we need to be prepared for too big
		 * requests.
		 */
		if (brq.data.blocks > card->host->max_blk_count)
			brq.data.blocks = card->host->max_blk_count;

		/*
		 * After a read error, we redo the request one sector at a time
		 * in order to accurately determine which sectors can be read
		 * successfully.
		 */
		if (disable_multi && brq.data.blocks > 1)
			brq.data.blocks = 1;

		if (brq.data.blocks > 1) {
			/* SPI multiblock writes terminate using a special
			 * token, not a STOP_TRANSMISSION request.
			 */
			if (!mmc_host_is_spi(card->host)
					|| rq_data_dir(req) == READ)
				brq.mrq.stop = &brq.stop;
			readcmd = MMC_READ_MULTIPLE_BLOCK;
			writecmd = MMC_WRITE_MULTIPLE_BLOCK;
		} else {
			brq.mrq.stop = NULL;
			readcmd = MMC_READ_SINGLE_BLOCK;
			writecmd = MMC_WRITE_BLOCK;
		}

		if (rq_data_dir(req) == READ) {
			brq.cmd.opcode = readcmd;
			brq.data.flags |= MMC_DATA_READ;
		} else {
			brq.cmd.opcode = writecmd;
			brq.data.flags |= MMC_DATA_WRITE;
		}

		if (rq_data_dir(req) == WRITE)
			mmc_adjust_toshiba_write(card, &brq.mrq);

		mmc_set_data_timeout(&brq.data, card);

		brq.data.sg = mq->sg;
		brq.data.sg_len = mmc_queue_map_sg(mq);

		/*
		 * Adjust the sg list so it is the same size as the
		 * request.
		 */
		if (brq.data.blocks != blk_rq_sectors(req)) {
			int i, data_size = brq.data.blocks << 9;
			struct scatterlist *sg;

			for_each_sg(brq.data.sg, sg, brq.data.sg_len, i) {
				data_size -= sg->length;
				if (data_size <= 0) {
					sg->length += data_size;
					i++;
					break;
				}
			}
			brq.data.sg_len = i;
		}

		mmc_queue_bounce_pre(mq);

		/*
		 * Try the workaround first for writes, then fall back.
		 */
		if (rq_data_dir(req) != WRITE || disable_multi ||
		    !mmc_handle_toshiba_write(mq, card, &brq.mrq))
			mmc_wait_for_req(card->host, &brq.mrq);

		mmc_queue_bounce_post(mq);

		ret = 0;
		*bytes_xfered = brq.data.bytes_xfered;
		/*
		 * Check for errors here, but don't jump to cmd_err
		 * until later as we need to wait for the card to leave
		 * programming mode even when things go wrong.
		 */
		if (brq.cmd.error || brq.data.error || brq.stop.error) {
			if (brq.data.blocks > 1 && rq_data_dir(req) == READ) {
				/* Redo read one sector at a time */
				printk(KERN_WARNING "%s: retrying using single "
				       "block read\n", req->rq_disk->disk_name);
				disable_multi = 1;
				retry = 1;
				continue;
			}
			status = get_card_status(card, req);
		} else if (disable_multi == 1) {
			disable_multi = 0;
			printk(KERN_INFO "%s: multi block enabled\n",
				req->rq_disk->disk_name);
		}
		retry = 0;

		if (brq.cmd.error) {
			ret = brq.cmd.error;
			printk(KERN_ERR "%s: error %d sending read/write "
			       "command, response %#x, card status %#x\n",
			       req->rq_disk->disk_name, brq.cmd.error,
			       brq.cmd.resp[0], status);
		}

		if (brq.data.error) {
			ret = brq.data.error;
			if (brq.data.error == -ETIMEDOUT && brq.mrq.stop)
				/* 'Stop' response contains card status */
				status = brq.mrq.stop->resp[0];
			printk(KERN_ERR "%s: error %d transferring data,"
			       " sector %u, nr %u, card status %#x\n",
			       req->rq_disk->disk_name, brq.data.error,
			       (unsigned)blk_rq_pos(req),
			       (unsigned)blk_rq_sectors(req), status);
		}

		if (brq.stop.error) {
			ret = brq.stop.error;
			printk(KERN_ERR "%s: error %d sending stop command, "
			       "response %#x, card status %#x\n",
			       req->rq_disk->disk_name, brq.stop.error,
			       brq.stop.resp[0], status);
		}

		/*
		* We need to wait for the card to leave programming mode
		* even when things go wrong.
		*/
		if (!mmc_host_is_spi(card->host) && rq_data_dir(req) != READ) {
			do {
				int err;
				cmd.opcode = MMC_SEND_STATUS;
				cmd.arg = card->rca << 16;
				cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
				err = mmc_wait_for_cmd(card->host, &cmd, 5);
				if (err) {
					printk(KERN_ERR "%s: error %d requesting status\n",
					       req->rq_disk->disk_name, err);
					ret = err;
					break;
				}
				/*
				 * Some cards mishandle the status bits,
				 * so make sure to check both the busy
				 * indication and the card state.
				 */
			} while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
				(R1_CURRENT_STATE(cmd.resp[0]) == 7));
		}

		/*
		 * Adjust the number of bytes transferred if there has been
		 * an error...
		 */
		if (ret) {
			/*
			 * For reads we just fail the entire chunk as that
			 * should be safe in all cases.
			 *
			 * If this is an SD card and we're writing, we can ask
			 * the card for known good sectors.
			 *
			 * If the card is not SD, we can still ok written
			 * sectors as reported by the controller (which might
			 * be less than the real number of written sectors, but
			 * never more).
			 */
			if (rq_data_dir(req) == READ)
				*bytes_xfered = 0;
			else if (mmc_card_sd(card)) {
				u32 blocks = mmc_sd_num_wr_blocks(card);
				if (blocks == (u32)-1)
					*bytes_xfered = 0;
				else
					*bytes_xfered = blocks << 9;
			}
		}
	} while (retry);

	return ret;
}

static int mmc_blk_erase_rq(struct mmc_blk_data *md,
	struct request *req, unsigned int *bytes_xfered)
{
	struct mmc_card *card;
	struct mmc_command cmd;
	int ret;

	uint64_t start, end, blocks;

	BUG_ON(!bytes_xfered);

	*bytes_xfered = 0;

	card = md->queue.card;

	BUG_ON(mmc_card_blockaddr(card) && (card->csd.erase_size % 512));

	start = ((uint64_t)blk_rq_pos(req)) << 9;
	end = start + (((uint64_t)blk_rq_sectors(req)) << 9);

	/*
	 * The specs talk about the card removing the least
	 * significant bits, but the erase sizes are not guaranteed
	 * to be a power of two, so do a proper calculation.
	 */
	blocks = start;
	if (do_div(blocks, card->csd.erase_size)) /* start % erase_size */
		start = (blocks + 1) * card->csd.erase_size; /* roundup() */
	blocks = end;
	if (do_div(blocks, card->csd.erase_size))
		end = blocks * card->csd.erase_size;

	if (start == end)
		goto out;

	/*
	 * The MMC spec isn't entirely clear that this should be done,
	 * but it would be impossible to erase the entire card if the
	 * addresses aren't sector based.
	 */
	if (mmc_card_blockaddr(card)) {
		start >>= 9;
		end >>= 9;
	}

	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_START;
	else
		cmd.opcode = MMC_ERASE_GROUP_START;
	cmd.arg = start;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	ret = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (ret) {
		printk(KERN_ERR "%s: error %d setting block erase start address\n",
		       req->rq_disk->disk_name, ret);
		return ret;
	}

	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_END;
	else
		cmd.opcode = MMC_ERASE_GROUP_END;
	cmd.arg = end - 1; /* the span is inclusive */
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	ret = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (ret) {
		printk(KERN_ERR "%s: error %d setting block erase end address\n",
		       req->rq_disk->disk_name, ret);
		return ret;
	}

	cmd.opcode = MMC_ERASE;
	cmd.arg = 0;	/* was MMC_ERASE_TYPE_SECURE */
	cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;

	ret = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (ret) {
		printk(KERN_ERR "%s: error %d starting block erase\n",
		       req->rq_disk->disk_name, ret);
		return ret;
	}

	/*
	 * Wait for the card to finish the erase request...
	 */
	if (!mmc_host_is_spi(card->host)) {
		do {
			cmd.opcode = MMC_SEND_STATUS;
			cmd.arg = card->rca << 16;
			cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
			ret = mmc_wait_for_cmd(card->host, &cmd, 5);
			if (ret) {
				printk(KERN_ERR "%s: error %d requesting status\n",
				       req->rq_disk->disk_name, ret);
				return ret;
			}
			/*
			 * Some cards mishandle the status bits,
			 * so make sure to check both the busy
			 * indication and the card state.
			 */
		} while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
			(R1_CURRENT_STATE(cmd.resp[0]) == 7));
	}

out:
	*bytes_xfered = blk_rq_sectors(req) << 9;

	return 0;
}

static int mmc_blk_issue_rq(struct mmc_queue *mq, struct request *req)
{
	struct mmc_blk_data *md = mq->data;
	struct mmc_card *card = md->queue.card;
	int ret, err, bytes_xfered;

	mmc_claim_host(card->host);

	do {
		if (blk_discard_rq(req))
			err = mmc_blk_erase_rq(md, req, &bytes_xfered);
		else
			err = mmc_blk_xfer_rq(md, req, &bytes_xfered);

		/*
		 * First handle the sectors that got transferred
		 * successfully...
		 */
		spin_lock_irq(&md->lock);
		ret = __blk_end_request(req, 0, bytes_xfered);
		spin_unlock_irq(&md->lock);

		/*
		 * ...then check if things went south.
		 */
		if (err) {
			mmc_release_host(card->host);

			/*
			 * Kill of the rest of the request...
			 */
			spin_lock_irq(&md->lock);
			while (ret)
				ret = __blk_end_request(req, -EIO,
					blk_rq_cur_bytes(req));
			spin_unlock_irq(&md->lock);

			return 0;
		}
	} while (ret);

	mmc_release_host(card->host);

	return 1;
}


static inline int mmc_blk_readonly(struct mmc_card *card)
{
	return mmc_card_readonly(card) ||
	       !(card->csd.cmdclass & CCC_BLOCK_WRITE);
}

static struct mmc_blk_data *mmc_blk_alloc(struct mmc_card *card)
{
	struct mmc_blk_data *md;
	int devidx, ret;
#ifdef CONFIG_MMC_BLOCK_DEVICE_NUMBERING
	int idx;
	const char *mmc_blk_name;
#endif

	devidx = find_first_zero_bit(dev_use, MMC_NUM_MINORS);
	if (devidx >= MMC_NUM_MINORS)
		return ERR_PTR(-ENOSPC);
	__set_bit(devidx, dev_use);

#ifdef CONFIG_MMC_BLOCK_DEVICE_NUMBERING
	mmc_blk_name = kobject_name(&card->host->parent->kobj);
	mmc_blk_name = mmc_blk_name ? strrchr(mmc_blk_name, '.') : mmc_blk_name;
	if (mmc_blk_name) {
		mmc_blk_name++;
		idx = simple_strtol(mmc_blk_name, NULL, 10);
		if ( (idx >= 0) && (idx != devidx) &&
			(!test_bit(idx, dev_use)) && (idx < MMC_NUM_MINORS) ) {
			__set_bit(idx, dev_use);
			__clear_bit(devidx, dev_use);
			devidx = idx;
		}
	}
#endif

	md = kzalloc(sizeof(struct mmc_blk_data), GFP_KERNEL);
	if (!md) {
		ret = -ENOMEM;
		goto out;
	}

	if (card->cid.manfid == TOSHIBA_MANFID && mmc_card_mmc(card)) {
		pr_info("%s: enable Toshiba workaround\n",
			mmc_hostname(card->host));
		md->bounce = kmalloc(TOSHIBA_HIGH_THRESHOLD * 512, GFP_KERNEL);
		if (!md->bounce) {
			ret = -ENOMEM;
			goto err_kfree;
		}
	}

	/*
	 * Set the read-only status based on the supported commands
	 * and the write protect switch.
	 */
	md->read_only = mmc_blk_readonly(card);

	md->disk = alloc_disk(1 << MMC_SHIFT);
	if (md->disk == NULL) {
		ret = -ENOMEM;
		goto err_kfree;
	}

	spin_lock_init(&md->lock);
	md->usage = 1;

	ret = mmc_init_queue(&md->queue, card, &md->lock);
	if (ret)
		goto err_putdisk;

	md->queue.issue_fn = mmc_blk_issue_rq;
	md->queue.data = md;

	md->disk->major	= MMC_BLOCK_MAJOR;
	md->disk->first_minor = devidx << MMC_SHIFT;
	md->disk->fops = &mmc_bdops;
	md->disk->private_data = md;
	md->disk->queue = md->queue.queue;
	md->disk->driverfs_dev = &card->dev;

	/*
	 * As discussed on lkml, GENHD_FL_REMOVABLE should:
	 *
	 * - be set for removable media with permanent block devices
	 * - be unset for removable block devices with permanent media
	 *
	 * Since MMC block devices clearly fall under the second
	 * case, we do not set GENHD_FL_REMOVABLE.  Userspace
	 * should use the block device creation/destruction hotplug
	 * messages to tell when the card is present.
	 */

	sprintf(md->disk->disk_name, "mmcblk%d", devidx);

	blk_queue_logical_block_size(md->queue.queue, 512);

	if (!mmc_card_sd(card) && mmc_card_blockaddr(card)) {
		/*
		 * The EXT_CSD sector count is in number or 512 byte
		 * sectors.
		 */
		set_capacity(md->disk, card->ext_csd.sectors);
	} else {
		/*
		 * The CSD capacity field is in units of read_blkbits.
		 * set_capacity takes units of 512 bytes.
		 */
		set_capacity(md->disk,
			card->csd.capacity << (card->csd.read_blkbits - 9));
	}
	return md;

 err_putdisk:
	put_disk(md->disk);
 err_kfree:
	if (md->bounce)
		kfree(md->bounce);
	kfree(md);
 out:
	return ERR_PTR(ret);
}

#if defined(CONFIG_MACH_MOT) && defined(CONFIG_APANIC_MMC)
static int mmc_apanic_annotate(struct mmc_blk_data *md, struct mmc_card *card)
{
	char cap_str[10];
	char manf_str[8] = "Unknown";
	char ident[80];
	static bool once = true;

	string_get_size((u64)get_capacity(md->disk) << 9, STRING_UNITS_2,
			cap_str, sizeof(cap_str));

	switch (card->cid.manfid) {
	case 0x02:
	case 0x45:
		strcpy(manf_str, "Sandisk");
		break;
	case 0x11:
		strcpy(manf_str, "Toshiba");
		break;
	}

	snprintf(ident, 80, "%s: %s %s %s %s %s\n",
		md->disk->disk_name, mmc_card_id(card), manf_str,
		mmc_card_name(card), cap_str, md->read_only ? "(ro)" : "");

	printk(KERN_INFO "%s", ident);

	if (once) {
		once = false;
		return apanic_annotate(ident);
	} else
		return 0;
}
#endif

static int mmc_blk_probe(struct mmc_card *card)
{
	struct mmc_blk_data *md;
	int err;

	char cap_str[10];

	/*
	 * Check that the card supports the command class(es) we need.
	 */
	if (!(card->csd.cmdclass & CCC_BLOCK_READ))
		return -ENODEV;

	md = mmc_blk_alloc(card);
	if (IS_ERR(md))
		return PTR_ERR(md);

	err = mmc_blk_set_blksize(md, card);
	if (err)
		goto out;

	string_get_size((u64)get_capacity(md->disk) << 9, STRING_UNITS_2,
			cap_str, sizeof(cap_str));

#if defined(CONFIG_MACH_MOT) && defined(CONFIG_APANIC_MMC)
	mmc_apanic_annotate(md, card);
#else
	printk(KERN_INFO "%s: %s %s %s %s\n",
		md->disk->disk_name, mmc_card_id(card), mmc_card_name(card),
		cap_str, md->read_only ? "(ro)" : "");
#endif

	mmc_set_drvdata(card, md);
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	mmc_set_bus_resume_policy(card->host, 1);
#endif
	add_disk(md->disk);
	return 0;

 out:
	mmc_cleanup_queue(&md->queue);
	mmc_blk_put(md);

	return err;
}

static void mmc_blk_remove(struct mmc_card *card)
{
	struct mmc_blk_data *md = mmc_get_drvdata(card);

	if (md) {
		/* Stop new requests from getting into the queue */
		del_gendisk(md->disk);

		/* Then flush out any already in there */
		mmc_cleanup_queue(&md->queue);

		mmc_blk_put(md);
	}
	mmc_set_drvdata(card, NULL);
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	mmc_set_bus_resume_policy(card->host, 0);
#endif
}

#ifdef CONFIG_PM
static int mmc_blk_suspend(struct mmc_card *card, pm_message_t state)
{
	struct mmc_blk_data *md = mmc_get_drvdata(card);

	if (md) {
		mmc_queue_suspend(&md->queue);
	}
	return 0;
}

static int mmc_blk_resume(struct mmc_card *card)
{
	struct mmc_blk_data *md = mmc_get_drvdata(card);

	if (md) {
#ifndef CONFIG_MMC_BLOCK_DEFERRED_RESUME
		mmc_blk_set_blksize(md, card);
#endif
		mmc_queue_resume(&md->queue);
	}
	return 0;
}
#else
#define	mmc_blk_suspend	NULL
#define mmc_blk_resume	NULL
#endif

static struct mmc_driver mmc_driver = {
	.drv		= {
		.name	= "mmcblk",
	},
	.probe		= mmc_blk_probe,
	.remove		= mmc_blk_remove,
	.suspend	= mmc_blk_suspend,
	.resume		= mmc_blk_resume,
};

static int __init mmc_blk_init(void)
{
	int res;

	res = register_blkdev(MMC_BLOCK_MAJOR, "mmc");
	if (res)
		goto out;

	res = mmc_register_driver(&mmc_driver);
	if (res)
		goto out2;

	return 0;
 out2:
	unregister_blkdev(MMC_BLOCK_MAJOR, "mmc");
 out:
	return res;
}

static void __exit mmc_blk_exit(void)
{
	mmc_unregister_driver(&mmc_driver);
	unregister_blkdev(MMC_BLOCK_MAJOR, "mmc");
}

module_init(mmc_blk_init);
module_exit(mmc_blk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Multimedia Card (MMC) block device driver");


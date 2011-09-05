#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/tegra_devices.h>
#include <linux/spi/spi_slave.h>
#include <linux/spi/mdm6600_spi.h>

#include "mach/nvrm_linux.h"
#include "mach/io.h"
#include "nvrm_spi.h"
#include "nvrm_power.h"
#include "nvodm_query.h"
#include "nvodm_services.h"

#include "rm_spi_slink.h"

static unsigned int tx_count = 0;
module_param(tx_count, uint, S_IRUGO|S_IWUSR);
static unsigned long tx_length = 0;
module_param(tx_length, ulong, S_IRUGO|S_IWUSR);
unsigned int tx_state = 0;
module_param(tx_state, uint, S_IRUGO|S_IWUSR);

#define TEGRA_SPI_SLAVE_DEBUG 1
#ifdef TEGRA_SPI_SLAVE_DEBUG

static int debug_mask = 0;
module_param(debug_mask, int, S_IRUGO|S_IWUSR);

#define TEGRA_SPI_SLAVE_INFO(fmt, args...) do { \
		if (debug_mask) \
			printk(KERN_INFO fmt, ## args); \
        }while(0)
#else
#define TEGRA_SPI_SLAVE_INFO(fmt, args...) do{}while(0)
#endif

/* Cannot use spinlocks as the NvRm SPI apis uses mutextes and one cannot use
 * mutextes inside a spinlock.
 */
#define USE_SPINLOCK 0
#if USE_SPINLOCK
#define LOCK_T          spinlock_t
#define CREATELOCK(_l)  spin_lock_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        spin_lock(&(_l))
#define UNLOCK(_l)      spin_unlock(&(_l))
#define ATOMIC(_l,_f)   spin_lock_irqsave(&(_l),(_f))
#define UNATOMIC(_l,_f) spin_unlock_irqrestore(&(_l),(_f))
#else
#define LOCK_T          struct mutex
#define CREATELOCK(_l)  mutex_init(&(_l))
#define DELETELOCK(_l)
#define LOCK(_l)        mutex_lock(&(_l))
#define UNLOCK(_l)      mutex_unlock(&(_l))
#define ATOMIC(_l,_f)   local_irq_save((_f))
#define UNATOMIC(_l,_f) local_irq_restore((_f))
#endif

#define NVODM_PORT(x) ((x) - 'a')
#define SPI_TRANSACTION_SIZE (16256)
#define SPI_TRANSACTION_TIMEOUT_MS 300
//TODO: verify if both working
#define spi_pad_transaction_size(x)   (((x) > 4) ? ((((x) + 15) / 16) * 16) : (x))
#define spi_pad_transaction_size_32(x)   (((x) > 4) ? ((((x) + 31) / 32) * 32) : (x))

struct NvSpiSlave {
	NvOdmServicesSpiHandle  hOdmSpi;
	NvU32			IoModuleID;
	NvU32			Instance;
	NvU32			ChipSelect;
	NvU32			PinMuxConfig;

	NvU32			Mode;
	NvU32			BitsPerWord;
	NvU32			ClockInKHz;

	struct list_head	msg_queue;
	NvRmSpiTransactionInfo trans;
	LOCK_T			lock;
	struct work_struct	work;
	struct workqueue_struct	*WorkQueue;
	NvU32 RmPowerClientId;
};

extern NvRmFreqKHz NvRmPrivDfsGetCurrentKHz(NvRmDfsClockId ClockId);

/* Only these signaling mode are supported */
#define NV_SUPPORTED_MODE_BITS (SPI_CPOL | SPI_CPHA)

static int tegra_spi_slave_setup(struct spi_slave_device *pSpiDevice)
{
	struct NvSpiSlave  *pShimSpi;

	TEGRA_SPI_SLAVE_INFO("%s\n", __func__);
	pShimSpi = spi_slave_get_devdata(pSpiDevice->slave);

	if (pSpiDevice->mode & ~NV_SUPPORTED_MODE_BITS) {
		dev_dbg(&pSpiDevice->dev, "setup: unsupported mode bits 0x%x\n",
			pSpiDevice->mode & ~NV_SUPPORTED_MODE_BITS);
	}

	pShimSpi->Mode = pSpiDevice->mode & NV_SUPPORTED_MODE_BITS;
	switch (pShimSpi->Mode) {
	case SPI_MODE_0:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_0;
		break;
	case SPI_MODE_1:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_1;
		break;
	case SPI_MODE_2:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_2;
		break;
	case SPI_MODE_3:
		pShimSpi->Mode = NvOdmQuerySpiSignalMode_3;
		break;
	}

	if (pSpiDevice->bits_per_word == 0)
		pSpiDevice->bits_per_word = 8;

	pShimSpi->BitsPerWord = pSpiDevice->bits_per_word;
	pShimSpi->ClockInKHz = pSpiDevice->max_speed_hz / 1000;
	pShimSpi->ChipSelect = pSpiDevice->chip_select;

	return 0;
}

static int tegra_spi_slave_transfer(struct spi_slave_device *pSpiDevice,
	struct spi_slave_message *msg)
{
	struct NvSpiSlave  *pShimSpi;

	TEGRA_SPI_SLAVE_INFO("%s\n", __func__);
	if (unlikely(list_empty(&msg->transfers) ||
		!pSpiDevice->max_speed_hz))
		return -EINVAL;

	/* FIXME validate the msg */

	pShimSpi = spi_slave_get_devdata(pSpiDevice->slave);

	/* Add the message to the queue and signal the worker thread */
	LOCK(pShimSpi->lock);
	list_add_tail(&msg->queue, &pShimSpi->msg_queue);
	UNLOCK(pShimSpi->lock);
	queue_work(pShimSpi->WorkQueue, &pShimSpi->work);

	return 0;
}

static void tegra_spi_slave_cleanup(struct spi_slave_device *pSpiDevice)
{
	return;
}

static int tegra_spi_slave_transaction(struct NvSpiSlave *pShimSpi, struct spi_slave_message *msg)
{
	NvBool ret;
	NvU32 BitsPerWord;
	NvU32 ClockInKHz;
	NvU32 ChipSelect;
	NvU32 actual_length = 0;
	struct spi_slave_transfer *t = NULL;
	struct mdm6600_spi_device *mdm6600;

	TEGRA_SPI_SLAVE_INFO("%s\n", __func__);

	mdm6600 = (struct mdm6600_spi_device *)msg->spi->controller_data;
	msg->actual_length = 0;
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		if (t->tx_buf == NULL || t->rx_buf == NULL || !t->len) {
			TEGRA_SPI_SLAVE_INFO("%s: bad request!\n", __func__);
			ret = -EINVAL;
			break;
		}
		if (t->len) {
			/* Get slave config.  PinMuxConfig is a property of the slave. */
			BitsPerWord = msg->spi->bits_per_word;
			ClockInKHz = msg->spi->max_speed_hz / 1000;
			ChipSelect = msg->spi->chip_select;

			if (t->cs_change) {
				/* What should we do here? */
			}

			// we have seen a issue that Nvidia SPI slave API enter dead loop, this will help
			// to find out if this happens, tx_state will always be 1 in this case
			BUG_ON(tx_state != 0);
			tx_state = 1;

			// start SPI slave transaction
			ret = NvOdmSpiSlaveStartTransaction(pShimSpi->hOdmSpi,
								0, 26000,
								1,
								t->tx_buf,
								spi_pad_transaction_size_32
								(SPI_TRANSACTION_SIZE),
								32);
			BUG_ON(tx_state != 1);
			tx_state = 2;
			if (ret == NV_FALSE) {
				pr_err("%s failed to start transaction, retval = %d",
					__func__, ret);
				ret = -EIO;
				break;
			}else{
				mdm6600->active_slave_srdy(mdm6600);
			}

			ret = NvOdmSpiSlaveGetTransactionData(pShimSpi->hOdmSpi,
							 t->rx_buf,
							spi_pad_transaction_size(SPI_TRANSACTION_SIZE),
							(NvU32*)&actual_length, SPI_TRANSACTION_TIMEOUT_MS);
			BUG_ON(tx_state != 2);
			tx_state = 0;
			if (ret != NV_TRUE || actual_length != spi_pad_transaction_size(SPI_TRANSACTION_SIZE)) {
				pr_err("spi transaction interruptted, ret=%d, transmitted=%d\n", ret, actual_length);
				// TODO: how to recovery from transaction error? A CR is opened for Nvidia
			}

			mdm6600->deactive_slave_srdy(mdm6600);
			// Call callback
			// mdm6600->callback(msg->spi->cb_context, NULL, 0);
			msg->actual_length += actual_length;

			tx_count++;
			tx_length += actual_length;
		}
	}

	return ret;
}

static void tegra_spi_slave_workerthread(struct work_struct *w)
{
	struct NvSpiSlave *pShimSpi;
	struct spi_slave_message *m;
	int status = 0;
	NvRmDfsBusyHint BusyHints[4];

	TEGRA_SPI_SLAVE_INFO("%s\n", __func__);
	pShimSpi = container_of(w, struct NvSpiSlave, work);

	BusyHints[0].ClockId = NvRmDfsClockId_Emc;
	BusyHints[0].BoostDurationMs = NV_WAIT_INFINITE;
	BusyHints[0].BusyAttribute = NV_TRUE;
	BusyHints[0].BoostKHz = 150000; // Emc

	BusyHints[1].ClockId = NvRmDfsClockId_Ahb;
	BusyHints[1].BoostDurationMs = NV_WAIT_INFINITE;
	BusyHints[1].BusyAttribute = NV_TRUE;
	BusyHints[1].BoostKHz = 150000; // AHB

	BusyHints[2].ClockId = NvRmDfsClockId_Apb;
	BusyHints[2].BoostDurationMs = NV_WAIT_INFINITE;
	BusyHints[2].BusyAttribute = NV_TRUE;
	BusyHints[2].BoostKHz = 150000; // APB

	BusyHints[3].ClockId = NvRmDfsClockId_Cpu;
	BusyHints[3].BoostDurationMs = NV_WAIT_INFINITE;
	BusyHints[3].BusyAttribute = NV_TRUE;
	BusyHints[3].BoostKHz = 600000;

	NvRmPowerBusyHintMulti(s_hRmGlobal, pShimSpi->RmPowerClientId,
		BusyHints, 4, NvRmDfsBusyHintSyncMode_Async);

    	if (NvRmDfsRunState_ClosedLoop == NvRmDfsGetState(s_hRmGlobal)) {
		int wait_count = 500;
		/* Wait for the clcok to stabilize */
		while ((NvRmPrivDfsGetCurrentKHz(NvRmDfsClockId_Emc) < 150000)
			&& wait_count--)
			msleep(1);
		BUG_ON(wait_count == 0);
	}

	LOCK(pShimSpi->lock);

	while (!list_empty(&pShimSpi->msg_queue)) {
		m = container_of(pShimSpi->msg_queue.next,
			struct spi_slave_message, queue);
		if (!m->spi) {
			status = -EINVAL;
			break;
		}
		UNLOCK(pShimSpi->lock);
		status = tegra_spi_slave_transaction(pShimSpi, m);
		LOCK(pShimSpi->lock);
		list_del_init(&m->queue);
		m->status = status;
		m->complete(m->context);
	}

	UNLOCK(pShimSpi->lock);

	/* Set the clocks to the low corner */
	BusyHints[0].BoostKHz = 0; // Emc
	BusyHints[1].BoostKHz = 0; // Ahb
	BusyHints[2].BoostKHz = 0; // Apb
	BusyHints[3].BoostKHz = 0; // Cpu

	NvRmPowerBusyHintMulti(s_hRmGlobal, pShimSpi->RmPowerClientId,
		BusyHints, 4, NvRmDfsBusyHintSyncMode_Async);
}

static int  tegra_spi_slave_probe(struct platform_device *pdev)
{
	struct spi_slave	*pSpi;
	struct NvSpiSlave	*pShimSpi;
	struct tegra_spi_platform_data *pdata = pdev->dev.platform_data;
	int			status= 0;
	NvError			err;
	char			name[64];

	TEGRA_SPI_SLAVE_INFO("%s\n", __func__);
	pSpi = spi_alloc_slave(&pdev->dev, sizeof *pShimSpi);
	if (pSpi == NULL) {
		dev_dbg(&pdev->dev, "slave allocation failed\n");
		return -ENOMEM;
	}

	pSpi->setup = tegra_spi_slave_setup;
	pSpi->transfer = tegra_spi_slave_transfer;
	pSpi->cleanup = tegra_spi_slave_cleanup;
	pSpi->num_chipselect = 1;
	pSpi->bus_num = pdev->id;

	dev_set_drvdata(&pdev->dev, pSpi);
	pShimSpi = spi_slave_get_devdata(pSpi);
/* TODO: pass from platform device alloc
	pShimSpi->IoModuleID = pdata->IoModuleID;
	pShimSpi->Instance = pdata->Instance;
	pShimSpi->PinMuxConfig = pdata->PinMuxConfig;
*/
	
	pShimSpi->IoModuleID = NvOdmIoModule_Spi;
        pShimSpi->Instance = 0;

	pShimSpi->hOdmSpi =
		 NvOdmSpiSlaveOpen(pShimSpi->IoModuleID, 0);
	if (pShimSpi->hOdmSpi == NULL) {
		dev_dbg(&pdev->dev, "NvOdmSpiSlaveOpen failed (%d)\n", err);
		goto spi_open_failed;
	}

	snprintf(name, sizeof(name), "%s_%d", "tegra_spis_wq",
		pdata->IoModuleID || pdata->Instance << 16);
	pShimSpi->WorkQueue = create_singlethread_workqueue(name);
	if (pShimSpi->WorkQueue == NULL) {
		dev_err(&pdev->dev, "Failed to create work queue\n");
		goto workQueueCreate_failed;
	}

	pShimSpi->RmPowerClientId = NVRM_POWER_CLIENT_TAG('S','P','I','S');
	if (NvRmPowerRegister(s_hRmGlobal, NULL, &pShimSpi->RmPowerClientId)) {
		dev_err(&pdev->dev, "Failed to create power client ID\n");
		goto workQueueCreate_failed;
	}

	INIT_WORK(&pShimSpi->work, tegra_spi_slave_workerthread);

	CREATELOCK(pShimSpi->lock);
	INIT_LIST_HEAD(&pShimSpi->msg_queue);

	status = spi_register_slave(pSpi);
	if (status < 0) {
		dev_err(&pdev->dev, "spi_register_slave failed %d\n", status);
		goto spi_register_failed;
	}

	dev_info(&pdev->dev,
		"Registered spi slave device for mod,inst,mux (%d,%d,%d)\n",
		pShimSpi->IoModuleID, pShimSpi->Instance,
		pShimSpi->PinMuxConfig);

	return status;

spi_register_failed:
	destroy_workqueue(pShimSpi->WorkQueue);
workQueueCreate_failed:
	NvOdmSpiClose(pShimSpi->hOdmSpi);
spi_open_failed:
	spi_slave_put(pSpi);
	return status;
}

static int __exit tegra_spi_slave_remove(struct platform_device *pdev)
{
	struct spi_slave	*pSpi;
	struct NvSpiSlave	*pShimSpi;

	pSpi = dev_get_drvdata(&pdev->dev);
	pShimSpi = spi_slave_get_devdata(pSpi);

	spi_unregister_slave(pSpi);
	NvRmPowerUnRegister(s_hRmGlobal, pShimSpi->RmPowerClientId);
	NvOdmSpiClose(pShimSpi->hOdmSpi);
	destroy_workqueue(pShimSpi->WorkQueue);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_spi_slave_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct spi_slave *pSpi;
	struct NvSpiSlave *pShimSpi;

	TEGRA_SPI_SLAVE_INFO("%s\n", __func__);
	pSpi = dev_get_drvdata(&pdev->dev);
	pShimSpi = spi_slave_get_devdata(pSpi);

	LOCK(pShimSpi->lock);

        if (!list_empty(&pShimSpi->msg_queue)) {
		UNLOCK(pShimSpi->lock);
		TEGRA_SPI_SLAVE_INFO("%s: Still has data to send, refuse suspend\n", __func__);
		return -1;
	}

	UNLOCK(pShimSpi->lock);

	return 0;
}

static int tegra_spi_slave_resume(struct platform_device *pdev)
{
	struct spi_slave *pSpi;
	struct NvSpiSlave *pShimSpi;

	TEGRA_SPI_SLAVE_INFO("%s\n", __func__);
	pSpi = dev_get_drvdata(&pdev->dev);
	pShimSpi = spi_slave_get_devdata(pSpi);

	return 0;
}
#else
#define	tegra_spi_slave_suspend NULL
#define	tegra_spi_slave_resume NULL
#endif

static struct platform_driver tegra_spi_slave_driver = {
	.driver	= {
		.name	= "tegra_spi_slave",
		.owner	= THIS_MODULE,
	},
	.suspend = tegra_spi_slave_suspend,
	.resume	= tegra_spi_slave_resume,
	.remove	= __exit_p(tegra_spi_slave_remove),
};

static int __init tegra_spi_slave_init(void)
{
	return platform_driver_probe(&tegra_spi_slave_driver, tegra_spi_slave_probe);
}
module_init(tegra_spi_slave_init);

static void __exit tegra_spi_slave_exit(void)
{
	platform_driver_unregister(&tegra_spi_slave_driver);
}
module_exit(tegra_spi_slave_exit);




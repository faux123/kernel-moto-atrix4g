/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/
/*!
	\file	spibusdrv.c

	\brief	spi bus driver module

	This file contains implementation of the spi bus driver.
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

#include "smscoreapi.h"
#include "smsdbg_prn.h"
#include "smsspicommon.h"
#include "smsspiphy.h"

#ifdef SMS1186_MORTABLE_BOARD
#define SMS_INTR_PIN			19  /* 0 for nova sip, 26 for vega in the default, 19 in the reality */
#else
#define SMS_INTR_PIN			4   /* GPIO4 for Sms2186*/
#endif

#define TX_BUFFER_SIZE			0x200
#define RX_BUFFER_SIZE			(0x1000 + SPI_PACKET_SIZE + 0x100)
#define NUM_RX_BUFFERS			72

u32 g_Sms_Int_Counter = 0;
u32 g_Sms_MsgFound_Counter = 0;

struct _spi_device_st {
	struct _spi_dev dev;
	void *phy_dev;

	struct completion write_operation;
	struct list_head tx_queue;
	int allocatedPackets;
	int padding_allowed;
	char *rxbuf;
	dma_addr_t rxbuf_phy_addr;

	struct smscore_device_t *coredev;
	struct list_head txqueue;
	char *txbuf;
	dma_addr_t txbuf_phy_addr;
};

struct _smsspi_txmsg {
	struct list_head node;	/*! internal management */
	void *buffer;
	size_t size;
	int alignment;
	int add_preamble;
	struct completion completion;
	void (*prewrite) (void *);
	void (*postwrite) (void *);
};

struct _Msg {
	struct SmsMsgHdr_ST hdr;
	u32 data[3];
};
struct _spi_device_st *spi_dev;

static int spi_resume_fail = 0;
static int spi_suspended   = 0;

static DEFINE_MUTEX(queue_mutex);

static void spi_worker_thread(void *arg);

//===changed for seperate work queue from system main work queue======
//static DECLARE_WORK(spi_work_queue, (void *)spi_worker_thread);
static struct workqueue_struct *spi_work_queue;
static struct work_struct spi_work;
//====================================================================

static u8 smsspi_preamble[] = { 0xa5, 0x5a, 0xe7, 0x7e };
static u8 smsspi_startup[] = { 0, 0, 0xde, 0xc1, 0xa5, 0x51, 0xf1, 0xed };

#ifdef SMS1186_MORTABLE_BOARD
	static u32 default_type = SMS_VENICE;
#else
	static u32 default_type = SMS_MING;
#endif

static u32 intr_pin = SMS_INTR_PIN;

module_param(default_type, int, 0644);
MODULE_PARM_DESC(default_type, "default board type.");

module_param(intr_pin, int, 0644);
MODULE_PARM_DESC(intr_pin, "interrupt pin number.");

/******************************************/
static void spi_worker_thread(void *arg)
{
	struct _spi_device_st *spi_device = spi_dev;
	struct _smsspi_txmsg *msg = NULL;
	struct _spi_msg txmsg;
	//int i=0;
	mutex_lock(&queue_mutex);

	PDEBUG("worker start\n");
	do {
		/* do we have a msg to write ? */
		if (!msg && !list_empty(&spi_device->txqueue))
			msg = (struct _smsspi_txmsg *)list_entry(spi_device->txqueue.next, struct _smsspi_txmsg, node);

		if (msg) {
			if (msg->add_preamble) {
				txmsg.len = min(msg->size + sizeof(smsspi_preamble), (size_t)TX_BUFFER_SIZE);
				txmsg.buf = spi_device->txbuf;
				txmsg.buf_phy_addr = spi_device->txbuf_phy_addr;

				memcpy(txmsg.buf, smsspi_preamble, sizeof(smsspi_preamble));
				memcpy(&txmsg.buf[sizeof(smsspi_preamble)], msg->buffer,
				       txmsg.len - sizeof(smsspi_preamble));

				msg->add_preamble = 0;
				msg->buffer = (char*)msg->buffer + txmsg.len - sizeof(smsspi_preamble);
				msg->size -= txmsg.len - sizeof(smsspi_preamble);
				/* zero out the rest of aligned buffer */
				memset(&txmsg.buf[txmsg.len], 0, TX_BUFFER_SIZE - txmsg.len);

				if(spi_resume_fail||spi_suspended)
					printk(KERN_EMERG " SMSx186: spi failed\n") ;           
				else
					smsspi_common_transfer_msg(&spi_device->dev, &txmsg, 1);                                
			} else {
				txmsg.len = min(msg->size, (size_t)TX_BUFFER_SIZE);
				txmsg.buf = spi_device->txbuf;
				txmsg.buf_phy_addr = spi_device->txbuf_phy_addr;
				memcpy(txmsg.buf, msg->buffer, txmsg.len);

				msg->buffer = (char*)msg->buffer + txmsg.len;
				msg->size -= txmsg.len;
				/* zero out the rest of aligned buffer */
				memset(&txmsg.buf[txmsg.len], 0, TX_BUFFER_SIZE - txmsg.len);
				if(spi_resume_fail||spi_suspended) 
					printk(KERN_EMERG " SMSx186: spi failed\n") ;           
				else
					smsspi_common_transfer_msg(&spi_device->dev, &txmsg, 0);                               
			}

		} else {
 			if(spi_resume_fail||spi_suspended) 
				printk(KERN_EMERG " SMSx186: spi failed\n") ;           
			else
				smsspi_common_transfer_msg(&spi_device->dev, NULL, 1);
		}

		/* if there was write, have we finished ? */
		if (msg && !msg->size) {
			/* call postwrite call back */
			if (msg->postwrite)
				msg->postwrite(spi_device);
			list_del(&msg->node);
			complete(&msg->completion);
			msg = NULL;
		}
		/* if there was read, did we read anything ? */


		//eladr - check if we lost msg, if so, recover
		if(g_Sms_MsgFound_Counter<g_Sms_Int_Counter)
		{
			//printk("eladr - we lost msg, probably becouse dma time out\n");
			//for(i=0; i<16; i++)
			//{
				//smsspi_common_transfer_msg(&spi_device->dev, NULL, 1);
			//}
			g_Sms_MsgFound_Counter = g_Sms_Int_Counter;
		}

	} while (!list_empty(&spi_device->txqueue) || msg);
	mutex_unlock(&queue_mutex);

//	PDEBUG("worker end\n");

}

unsigned int u_msgres_count = 0;

static void msg_found(void *context, void *buf, int offset, int len)
{
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	struct smscore_buffer_t *cb = (struct smscore_buffer_t *)(container_of(buf, struct smscore_buffer_t, p));

	g_Sms_MsgFound_Counter++;

	u_msgres_count++;
	sms_info("Msg_found count = %u, len = %d", u_msgres_count, len);
        if(len > RX_BUFFER_SIZE || offset >RX_BUFFER_SIZE )
        {
           printk("SMSx186: msg rx over,len=0x%x,offset=0x%x\n",len,offset ) ;
           printk("SMSx186: cb->p = [0x%x]\n",(unsigned int) cb->p) ;
           printk("SMSx186: cb->phys=[0x%x]\n",(unsigned int) cb->phys) ;
           
        } 
//	PDEBUG("entering\n");
	cb->offset = offset;
	cb->size = len;
	/* PERROR ("buffer %p is sent back to core databuf=%p,
		offset=%d.\n", cb, cb->p, cb->offset); */
	smscore_onresponse(spi_device->coredev, cb);

//	PDEBUG("exiting\n");
}

static void smsspi_int_handler(void *context)
{
//	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
//	printk("eladr - smsspi_int_handler %d\n",g_Sms_Int_Counter);

        g_Sms_Int_Counter++;

	if(spi_resume_fail || spi_suspended) 
        {
            printk(KERN_EMERG " SMSx186: spi failed\n");
            return ;                                   
        }  
//	PREPARE_WORK(&spi_work_queue, (void *)spi_worker_thread);
//	spi_device->padding_allowed = 1;

//===changed for seperate work queue from system main queue======
	//schedule_work(&spi_work_queue);
	queue_work(spi_work_queue, &spi_work);
//===============================================================
}

static int smsspi_queue_message_and_wait(struct _spi_device_st *spi_device, struct _smsspi_txmsg *msg)
{
	init_completion(&msg->completion);
	list_add_tail(&msg->node, &spi_device->txqueue);
//===changed for seperate work queue from system main queue======
	//schedule_work(&spi_work_queue);
	queue_work(spi_work_queue, &spi_work);
//===============================================================
	wait_for_completion(&msg->completion);
	return 0;
}

static int smsspi_preload(void *context)
{
	struct _smsspi_txmsg msg;
	//struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	struct _Msg Msg = {
		{MSG_SMS_SPI_INT_LINE_SET_REQ, 0, HIF_TASK, sizeof(struct _Msg), 0}, 
		{0, intr_pin, 0}
	};
	int rc;

	//prepareForFWDnl(spi_device->phy_dev);	// nothing done in the function now
	PDEBUG("Sending SPI init sequence\n");
	msg.buffer = smsspi_startup;
	msg.size = sizeof(smsspi_startup);
	msg.alignment = 4;
	msg.add_preamble = 0;
	msg.prewrite = NULL;	/* smsspiphy_reduce_clock; */
	msg.postwrite = NULL;

	rc = smsspi_queue_message_and_wait(context, &msg);
	if (rc < 0) {
		sms_err("smsspi_queue_message_and_wait error, rc = %d\n", rc);
		return rc;
	}

	sms_debug("sending MSG_SMS_SPI_INT_LINE_SET_REQ");
	PDEBUG("Sending SPI Set Interrupt command sequence\n");
	msg.buffer = &Msg;
	msg.size = sizeof(Msg);
	msg.alignment = SPI_PACKET_SIZE;
	msg.add_preamble = 1;

	rc = smsspi_queue_message_and_wait(context, &msg);
	if (rc < 0) {
		sms_err("set interrupt line failed, rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int smsspi_postload(void *context)
{
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	int mode = smscore_registry_getmode(spi_device->coredev->devpath);
	if ( (mode != DEVICE_MODE_ISDBT) && (mode != DEVICE_MODE_ISDBT_BDA) ) 
	{
		fwDnlComplete(spi_device->phy_dev, 0);		
	}
	//eladr-
	g_Sms_Int_Counter = 0;

	return 0;
}


static int smsspi_write(void *context, void *txbuf, size_t len)
{
	struct _smsspi_txmsg msg;

	msg.buffer = txbuf;
	msg.size = len;
	msg.prewrite = NULL;
	msg.postwrite = NULL;

//sms_info("Msg_write type = %d, len = %d, msg addr: %p, pre_node addr: %p", *((unsigned short*)txbuf), (int)len, &msg, msg.node.prev);	

	if (len > 0x1000) {
		/* The FW is the only long message. Do not add preamble, and do not padd it */
		msg.alignment = 4;
		msg.add_preamble = 0;
		msg.prewrite = smschipreset;
	} else {
		msg.alignment = SPI_PACKET_SIZE;
		msg.add_preamble = 1;
	}
/*
	sms_info("Writing message to  SPI.\n");
	sms_info("msg hdr: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x.",
	       ((u8 *) txbuf)[0], ((u8 *) txbuf)[1], ((u8 *) txbuf)[2],
	       ((u8 *) txbuf)[3], ((u8 *) txbuf)[4], ((u8 *) txbuf)[5],
	       ((u8 *) txbuf)[6], ((u8 *) txbuf)[7]);
	sms_info("msg addr to write to: 0x%x, 0x%x, 0x%x, 0x%x.",
	       ((u8 *) txbuf)[8], ((u8 *) txbuf)[9], ((u8 *) txbuf)[10],
	       ((u8 *) txbuf)[11]);
	sms_info("msg contents for first 4 bytes: 0x%x, 0x%x, 0x%x, 0x%x.",
	       ((u8 *) txbuf)[12], ((u8 *) txbuf)[13], ((u8 *) txbuf)[14],
	       ((u8 *) txbuf)[15]);
*/

	return smsspi_queue_message_and_wait(context, &msg);
}

struct _rx_buffer_st *allocate_rx_buf(void *context, int size)
{
	struct smscore_buffer_t *buf;
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	if (size > RX_BUFFER_SIZE) {
		PERROR("Requested size is bigger than max buffer size.\n");
		return NULL;
	}
	buf = smscore_getbuffer(spi_device->coredev);
	PDEBUG("Recieved Rx buf %p physical 0x%x (contained in %p)\n", buf->p,
	       buf->phys, buf);

	/* note: this is not mistake! the rx_buffer_st is identical to part of
	   smscore_buffer_t and we return the address of the start of the
	   identical part */

	//eladR - smscore_getbuffer return null, lets also return null
	if(NULL == buf)
	{
		return NULL;
	}

	return (struct _rx_buffer_st *) &buf->p;
}

static void free_rx_buf(void *context, struct _rx_buffer_st *buf)
{
	struct _spi_device_st *spi_device = (struct _spi_device_st *) context;
	struct smscore_buffer_t *cb = (struct smscore_buffer_t *)(container_of(((void *)buf), struct smscore_buffer_t, p));
	PDEBUG("buffer %p is released.\n", cb);
	smscore_putbuffer(spi_device->coredev, cb);
}

/*! Release device STUB

\param[in]	dev:		device control block
\return		void
*/
static void smsspi_release(struct device *dev)
{
	PDEBUG("nothing to do\n");
	/* Nothing to release */
}

static int smsspi_driver_probe(struct platform_device *pdev)
{
    PDEBUG("smsspi_probe\n") ;
    return 0 ;
}

extern void smschar_reset_device(void) ;
extern void smschar_set_suspend(int suspend_on);

extern int sms_suspend_count ;


#ifdef CONFIG_PM
static int smsspi_driver_suspend(struct platform_device *dev,pm_message_t state)
{
    
    spi_resume_fail = 0 ;
    spi_suspended = 1 ;

  printk(KERN_INFO "smsmdtv: in smsspi_suspend \n") ; 
   
    smschar_reset_device();
    smschar_set_suspend(1); // power suspended 
/*
    cpcap_enable_cmmb_core(0);
    cpcap_enable_cmmb_io(0);

    omap_writel(0x01040104, 0x480025DC);
    omap_writel(0x01040104, 0x480025E0);

    omap_set_gpio_direction(MTV_RST_N,1);
    omap_set_gpio_direction(MTV_PWDN,0);

    printk("Set the MTV_RST_N and MTV_PWDN to low when suspend\n");
    omap_set_gpio_dataout(MTV_RST_N,0);
    msleep(30);
    omap_set_gpio_dataout(MTV_PWDN,0);
    msleep(30);

    omap_set_gpio_direction(MTV_SPI_SIMO,1);
    omap_set_gpio_direction(MTV_SPI_SOMI,1);
    omap_set_gpio_direction(MTV_CS0,1);
    omap_set_gpio_direction(MTV_CLK,1);
    omap_set_gpio_direction(MTV_RST_N,1);
    omap_set_gpio_direction(MTV_PWDN,1);
    msleep(100) ;
    //smsspibus_ssp_suspend(spi_dev->phy_dev) ;
*/
    sms_suspend_count++;
    return 0 ;
}

static int smsspi_driver_resume(struct platform_device *dev)
{
//    int ret ;
    printk(KERN_INFO "smsmdtv: in smsspi_resume\n") ;
/*    ret = smsspibus_ssp_resume(spi_dev->phy_dev) ;
    msleep(100) ;
    if( ret== -1)
    {
       printk(KERN_INFO "smsspibus_ssp_resume failed\n") ;
       spi_resume_fail = 1 ;
    }else 
    {
       spi_resume_fail = 0 ;
    }     
*/
    spi_suspended = 0 ;
    smschar_set_suspend(0) ; // power wakeup
    return 0 ;
}
#else 

#define smsspi_driver_resume NULL 
#define smsspi_driver_suspend NULL 

#endif 

static struct platform_device smsspi_device = {
	.name = "smsspi",
	.id = 1,
	.dev = {
		.release = smsspi_release,
		},
};


static struct platform_driver smsspi_driver = {
    .probe   = smsspi_driver_probe,
    .suspend = smsspi_driver_suspend,
    .resume  = smsspi_driver_resume,
    .driver  = {
    .name = "smsspi",
		},
};

int smsspi_register(void)
{
	struct smsdevice_params_t params;
	int ret;
	struct _spi_device_st *spi_device;
	struct _spi_dev_cb_st common_cb;

	spi_device = kmalloc(sizeof(struct _spi_device_st), GFP_KERNEL);
        if(!spi_device)
        {
          printk("spi_device is null smsspi_register\n") ;
	  return 0;
        }
//===changed for seperate work queue from system main queue======
		/* Create the spi workqueue */
		spi_work_queue = create_singlethread_workqueue("sms_spi_work");
		if (!spi_work_queue) {
			return 0;
		}

		INIT_WORK(&spi_work, (void *)spi_worker_thread);
//===============================================================
	
	spi_dev = spi_device;

	INIT_LIST_HEAD(&spi_device->txqueue);

	ret = platform_device_register(&smsspi_device);
	if (ret < 0) {
		PERROR("platform_device_register failed\n");
		return ret;
	}

    // IKAROWANA-3449: add suspend state monitor
    platform_driver_register(&smsspi_driver);

	spi_device->txbuf =
	    dma_alloc_coherent(NULL, TX_BUFFER_SIZE/*max(TX_BUFFER_SIZE,PAGE_SIZE)*/,
			       &spi_device->txbuf_phy_addr,
			       GFP_KERNEL | GFP_DMA);
	if (!spi_device->txbuf) {
		printk(KERN_INFO "%s dma_alloc_coherent(...) failed\n", __func__);
		ret = -ENOMEM;
		goto txbuf_error;
	}

	sms_info("spi_device->txbuf = 0x%x  spi_device->txbuf_phy_addr= 0x%x",
                        (unsigned int)spi_device->txbuf, spi_device->txbuf_phy_addr);

	spi_device->phy_dev = smsspiphy_init(NULL, smsspi_int_handler, spi_device);
	if (spi_device->phy_dev == 0) {
		printk(KERN_INFO "%s smsspiphy_init(...) failed\n", __func__);
		goto phy_error;
	}

	common_cb.allocate_rx_buf = allocate_rx_buf;
	common_cb.free_rx_buf = free_rx_buf;
	common_cb.msg_found_cb = msg_found;
	common_cb.transfer_data_cb = smsspibus_xfer;

	ret = smsspicommon_init(&spi_device->dev, spi_device, spi_device->phy_dev, &common_cb);
	if (ret) {
		printk(KERN_INFO "%s smsspiphy_init(...) failed\n", __func__);
		goto common_error;
	}

	/* register in smscore */
	memset(&params, 0, sizeof(params));
	params.context = spi_device;
	params.device = &smsspi_device.dev;
	params.buffer_size = RX_BUFFER_SIZE;
	params.num_buffers = NUM_RX_BUFFERS;
	params.flags = SMS_DEVICE_NOT_READY;
	params.sendrequest_handler = smsspi_write;
	strcpy(params.devpath, "spi");
	params.device_type = default_type;

	if (0) {
		/* device family */
		/* params.setmode_handler = smsspi_setmode; */
	} else {
		params.flags =
		    SMS_DEVICE_FAMILY2 | SMS_DEVICE_NOT_READY |
		    SMS_ROM_NO_RESPONSE;
		params.preload_handler = smsspi_preload;
		params.postload_handler = smsspi_postload;
	}


	ret = smscore_register_device(&params, &spi_device->coredev);
	if (ret < 0) {
		printk(KERN_INFO "%s smscore_register_device(...) failed\n", __func__);
		goto reg_device_error;
	}

	ret = smscore_start_device(spi_device->coredev);
	if (ret < 0) {
		printk(KERN_INFO "%s smscore_start_device(...) failed\n", __func__);
		goto start_device_error;
	}
        spi_resume_fail = 0 ;
        spi_suspended = 0 ;

	PDEBUG("exiting\n");
	return 0;

start_device_error:
	smscore_unregister_device(spi_device->coredev);

reg_device_error:

common_error:
	smsspiphy_deinit(spi_device->phy_dev);

phy_error:
	dma_free_coherent(NULL, TX_BUFFER_SIZE, spi_device->txbuf,
			  spi_device->txbuf_phy_addr);

txbuf_error:
	platform_device_unregister(&smsspi_device);

	PDEBUG("exiting error %d\n", ret);

	return ret;
}

void smsspi_unregister(void)
{
	struct _spi_device_st *spi_device = spi_dev;
	printk(KERN_INFO "smsmdtv: in smsspi_unregister\n") ;
	PDEBUG("entering\n");

	/* stop interrupts */
	smsspiphy_deinit(spi_device->phy_dev);
	smscore_unregister_device(spi_device->coredev);

	dma_free_coherent(NULL, TX_BUFFER_SIZE, spi_device->txbuf,
			  spi_device->txbuf_phy_addr);

	platform_device_unregister(&smsspi_device);

//===changed for seperate work queue from system main queue======
sms_info("Destroy work queue");
	destroy_workqueue(spi_work_queue);
//===============================================================

	PDEBUG("exiting\n");
}

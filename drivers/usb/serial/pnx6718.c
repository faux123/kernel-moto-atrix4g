/*
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 * Revision History:
 *
 * Date         Author    Comment
 * ----------   --------  ----------------------------
 * 10/21/2010   Motorola  Initial version 
 * 11/23/2010   Motorola  add bplog support
 * 11/23/2010   Motorola  add throttled control
 * 11/23/2010   Motorola  change rx tasklet to wq
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/irq.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#ifdef CONFIG_MOT_FEAT_TD_BP_CTRL
#include <linux/td_bp_ctrl.h>
#endif
#define WDR_TIMEOUT       (HZ * 5)
#define MODEM_NO_TRAFFIC_TIME (HZ*2)
#define TIMEOUT_AFTERRX (HZ*2 + HZ/2)
#define MODEM_WAKELOCK_TIME (HZ/2)
#define MODEM_INTERFACE_NUM 2

#define BP_MODEM_STATUS 0x20a1
#define BP_RSP_AVAIL 0x01a1
#define BP_SPEED_CHANGE 0x2aa1

#define BP_CAR 0x01
#define BP_DSR 0x02
#define BP_BREAK 0x04
#define BP_RNG 0x08

#define BP_STATUS_AP 89
#define AP_STATUS_BP 88
#define AP_NW  16
#define AP_NR  16

//#define PNX_DBG 0

/* for bp logger */
#define ARGETVS     _IOR('T',0x70, unsigned int)
#define ARSETVS     _IOW('T',0x71, unsigned int)

int td_modem_debug;

struct ap_wblog {
	unsigned char *buf;
	int len;
};

struct ap_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb *urb;
	struct modem_port *instance;
};

struct ap_rb {
	struct list_head list;
	int size;
	unsigned char *base;
	dma_addr_t dma;
};

struct ap_ru {
	struct list_head list;
	struct ap_rb *buffer;
	struct urb *urb;
	struct modem_port *instance;
};

struct modem_port {
	__u16 modem_status;	/* only used for data modem port */
	__u8  wakeup_gpio;
	struct ap_ru ru[AP_NR];
	struct ap_rb rb[AP_NR];
	struct ap_wb wb[AP_NW];
	struct ap_wblog wbl;
	struct ap_wb *delayed_wb;
	int rx_buflimit;
	int rx_endpoint;
	unsigned int susp_count;
	unsigned int resuming;
	struct usb_serial_port *port;
	spinlock_t read_lock;
	spinlock_t write_lock;
	atomic_t wakeup_flag;
	spinlock_t last_traffic_lock;
	unsigned long last_traffic;
	unsigned int readsize;
	unsigned int writesize;
	struct list_head spare_read_urbs;
	struct list_head spare_read_bufs;
	struct list_head filled_read_bufs;
	int processing;
	int sending;
	unsigned int port_closing;
	struct work_struct wake_and_write;
	struct workqueue_struct *sendto_tty_q;
	struct workqueue_struct *bpwk_q;
	struct work_struct sendto_tty;
	struct work_struct bpwk;
	int irq;
	struct wake_lock modem_wakelock;
};
static struct modem_port *mdmpt;

static struct usb_device_id id_table[] = {
	{USB_DEVICE(0x04cc, 0x225c)},	/* Arowana PNX6718 BP modem */
	{},
};

MODULE_DEVICE_TABLE(usb, id_table);

#ifdef CONFIG_MOT_FEAT_TD_BP_CTRL
/* We will not do bad things when disconnection is because
 * of a normal power off procedure
 */
static atomic_t td_power_off = ATOMIC_INIT(0);

static void prior_power_off(void)
{
	atomic_set(&td_power_off,1);
}

static struct td_bp_ext_interface td_usb_ipc_intf = {
	.name = "usb_ipc_ctrl",
	.prior_shutdown = prior_power_off,
};
#endif /*CONFIG_MOT_FEAT_TD_BP_CTRL*/

static int modem_wb_alloc(struct modem_port *modem_ptr)
{
	int i;
	struct ap_wb *wb;

	for (i = 0; i < AP_NW; i++) {
		wb = &modem_ptr->wb[i];
		if (!wb->use) {
			wb->use = 1;
			return i;
		}
	}

	return -1;
}

static void modem_write_buffers_free(
		struct modem_port *modem_ptr,
		struct usb_serial *serial)
{
	int i;
	struct ap_wb *wb;
	struct usb_device *usb_dev = serial->dev;

	for (wb = &modem_ptr->wb[0], i = 0; i < AP_NW; i++, wb++)
		usb_buffer_free(usb_dev, modem_ptr->writesize,
				wb->buf, wb->dmah);
}

static int modem_write_buffers_alloc(
		struct modem_port *modem_ptr,
		struct usb_serial *serial)
{
	int i;
	struct ap_wb *wb;

	for (wb = &modem_ptr->wb[0], i = 0; i < AP_NW; i++, wb++) {
		wb->buf = usb_buffer_alloc(serial->dev, modem_ptr->writesize,
					GFP_KERNEL, &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_buffer_free(serial->dev,
					modem_ptr->writesize,
					wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}


static void mark_latest_traffic_time(struct modem_port *modem_port_ptr)
{
	unsigned long flags;

	spin_lock_irqsave(&modem_port_ptr->last_traffic_lock, flags);
	modem_port_ptr->last_traffic = jiffies;
	spin_unlock_irqrestore(&modem_port_ptr->last_traffic_lock, flags);
}

static void stop_data_traffic(struct modem_port *modem_port_ptr)
{
	int i;
	struct usb_serial_port *port =  modem_port_ptr->port;

	if (port == NULL)
		return;
	if (td_modem_debug)
		dev_info(&port->dev, "%s() port %d\n",
			__func__, port->number);

	for (i = 0; i < AP_NW; i++)
		usb_kill_urb(modem_port_ptr->wb[i].urb);
	for (i = 0; i < modem_port_ptr->rx_buflimit; i++)
		usb_kill_urb(modem_port_ptr->ru[i].urb);

	usb_kill_urb(modem_port_ptr->port->interrupt_in_urb);

	cancel_work_sync(&modem_port_ptr->sendto_tty);
	cancel_work_sync(&modem_port_ptr->bpwk);
	cancel_work_sync(&port->work);
}

static void modem_read_buffers_free(
		struct modem_port *modem_ptr,
		struct usb_serial *serial)
{
	struct usb_device *usb_dev = serial->dev;
	int i;
	int n = modem_ptr->rx_buflimit;

	for (i = 0; i < n; i++)
		usb_buffer_free(usb_dev, modem_ptr->readsize,
				modem_ptr->rb[i].base,
				modem_ptr->rb[i].dma);
}

static int modem_dtr_control(struct usb_serial *serial, int ctrl)
{
	struct modem_port *modem_port_ptr =
		usb_get_serial_data(serial);
	uint8_t bRequesttype =
		(USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT);
	uint16_t wLength = 0;
	uint8_t bRequest = 0x22;
	uint16_t wValue = ctrl;
	uint16_t wIndex = MODEM_INTERFACE_NUM;
	unsigned int pipe;
	int status;

	status = usb_autopm_get_interface(serial->interface);
	if (status < 0) {
		dev_err(&serial->dev->dev, "%s %s autopm failed %d",
			dev_driver_string
			(&serial->interface->dev),
			dev_name(&serial->interface->dev), status);
		return status;
	}

	pipe = usb_sndctrlpipe(serial->dev, 0);
	status = usb_control_msg(serial->dev, pipe,
			bRequest, bRequesttype,
			wValue, wIndex, NULL, wLength,
			WDR_TIMEOUT);
	usb_autopm_put_interface(serial->interface);
	if (modem_port_ptr)
		mark_latest_traffic_time(modem_port_ptr);

	return status;
}

static int modem_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct usb_serial_port *port = tty->driver_data;
	struct modem_port *modem_port_ptr = usb_get_serial_data(port->serial);

	if (modem_port_ptr == NULL)
		return 0;

	return (int)modem_port_ptr->modem_status;
}

static int modem_tiocmset(struct tty_struct *tty, struct file *file,
					unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	int status = 0;

	if (td_modem_debug)
		dev_info(&port->dev, "%s: Enter. clear is %d, set is %d\n",
			__func__, clear, set);

	if (port->number == MODEM_INTERFACE_NUM) {

		if (clear & TIOCM_DTR)
			status = modem_dtr_control(port->serial, 0);

		if (set & TIOCM_DTR)
			status = modem_dtr_control(port->serial, 1);
	}

	if (td_modem_debug)
		dev_info(&port->dev, "%s: Exit. Status %d\n",
			__func__, status);

	return status;
}

static void modem_read_bulk_callback(struct urb *urb)
{
	struct ap_rb *buf;
	struct ap_ru *rcv = urb->context;
	struct modem_port *modem_port_ptr;
	int status = urb->status;
	unsigned long flags;

	modem_port_ptr = rcv->instance;
	if (modem_port_ptr == NULL)
		return;

	if (modem_port_ptr->port == NULL)
		return;

	mark_latest_traffic_time(modem_port_ptr);
	buf = rcv->buffer;
	buf->size = urb->actual_length;

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	list_add_tail(&rcv->list, &modem_port_ptr->spare_read_urbs);

	if (likely(status == 0)) {
		modem_port_ptr->processing++;
		list_add_tail(&buf->list, &modem_port_ptr->filled_read_bufs);
	} else {
		if (td_modem_debug)
			dev_info(&modem_port_ptr->port->dev,
				 "%s: bulk rx err %d\n", __func__, status);
		/* we drop the buffer due to an error */
		list_add(&buf->list, &modem_port_ptr->spare_read_bufs);
		/* nevertheless the tasklet must be kicked unconditionally
		so the queue cannot dry up */
	}

	if (likely(modem_port_ptr->susp_count == 0) &&
	   (modem_port_ptr->port_closing != 1)
	  )
		queue_work(modem_port_ptr->sendto_tty_q, &modem_port_ptr->sendto_tty);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

}

static void modem_update_modem_status(struct usb_serial_port *port,
						__u8 modem_status)
{
	struct modem_port *modem_port_ptr;

	if (port->number == MODEM_INTERFACE_NUM) {
		modem_port_ptr = usb_get_serial_data(port->serial);
		if (modem_port_ptr == NULL) {
			dev_err(&port->dev,
				"%s: null modem port pointer.\n",
				__func__);
			return;
		}

		if (modem_status & BP_CAR)
			modem_port_ptr->modem_status |= TIOCM_CAR;
		else
			modem_port_ptr->modem_status &= ~TIOCM_CAR;

		if (modem_status & BP_DSR)
			modem_port_ptr->modem_status |= TIOCM_DSR;
		else
			modem_port_ptr->modem_status &= ~TIOCM_DSR;

		if (modem_status & BP_RNG)
			modem_port_ptr->modem_status |= TIOCM_RNG;
		else
			modem_port_ptr->modem_status &= ~TIOCM_RNG;

		if (td_modem_debug)
			dev_info(&port->dev, "%s: modem status is now %d\n",
				__func__,
				modem_port_ptr->modem_status);
	}
}

static void modem_interrupt_callback(struct urb *urb)
{
	int status = urb->status;
	uint16_t request_and_type;
	uint8_t modem_status;
	uint8_t *data;
	int length;
	int retval;
	unsigned long flags;

	struct usb_serial_port *port = (struct usb_serial_port *)urb->context;
	struct modem_port *modem_port_ptr =
		usb_get_serial_data(port->serial);

	if (modem_port_ptr->port == NULL)
		return;

	if (port->number != MODEM_INTERFACE_NUM) {
		if (td_modem_debug)
			dev_info(&port->dev,
				"%s: Not Modem port.\n", __func__);
		goto exit;
	}

	switch (status) {
	case 0:
		if (td_modem_debug)
			dev_info(&port->dev, "%s: usb -inter_cbk\n", __func__);
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		if (td_modem_debug)
			dev_info(&port->dev, "%s: urb shutting down\n",
					__func__);
		return;
	default:
		dev_err(&port->dev, "%s: nonzero urb status, %d.\n",
			__func__, status);
		goto exit;
	}

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	modem_port_ptr->processing++;
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

	length = urb->actual_length;
	data = (__u8 *) urb->transfer_buffer;

	request_and_type = *((__u16 *) data);

	if (td_modem_debug)
		dev_info(&port->dev, "%s: request and type is %d\n",
			__func__, request_and_type);

	switch (request_and_type) {
	case BP_MODEM_STATUS:
		modem_status = data[8];
		if (td_modem_debug)
			dev_info(&port->dev, "%s: MODEM status %d\n",
				__func__, modem_status);
		modem_update_modem_status(port, modem_status);
		break;

	case BP_RSP_AVAIL:
		if (td_modem_debug)
			dev_info(&port->dev, "%s: BP_RSP_AVAIL\n",
				__func__);
		break;

	case BP_SPEED_CHANGE:
		if (td_modem_debug)
			dev_info(&port->dev, "%s: BP_SPEED_CHANGE\n",
				__func__);
		break;

	default:
		if (td_modem_debug)
			dev_info(&port->dev,
				"%s: undefined BP request type %d\n",
				__func__, request_and_type);
		break;
	}

exit:
	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	if (modem_port_ptr->susp_count == 0) {
		retval = usb_submit_urb(urb, GFP_ATOMIC);
		if (retval) {
			dev_err(&port->dev,
				"%s:  submit int usb failed. ret = %d\n",
				__func__, retval);
		}
	}
	modem_port_ptr->processing--;
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
	mark_latest_traffic_time(modem_port_ptr);
	return;
}

static int modem_open(struct tty_struct *tty,
		struct usb_serial_port *port)
{
	struct modem_port *modem_port_ptr =
		usb_get_serial_data(port->serial);
	int retval = 0;
	int i;
	unsigned long flags;

	if (td_modem_debug)
		dev_info(&port->dev, "%s: Enter. Open Port %d\n",
				 __func__, port->number);

	tty->low_latency = 1;
	/* clear the throttle flags */
	port->throttled = 0;
	port->throttle_req = 0;

	if (modem_port_ptr == NULL) {
		dev_err(&port->dev,
			 "%s: null modem port pointer.\n",
			 __func__);
		return -ENODEV;
	}

	port->serial->interface->needs_remote_wakeup = 1;

	modem_port_ptr->port_closing = 0;
	modem_port_ptr->port = port;

	INIT_LIST_HEAD(&modem_port_ptr->spare_read_urbs);
	INIT_LIST_HEAD(&modem_port_ptr->spare_read_bufs);
	INIT_LIST_HEAD(&modem_port_ptr->filled_read_bufs);

	for (i = 0; i < modem_port_ptr->rx_buflimit; i++) {
		list_add(&(modem_port_ptr->ru[i].list),
			 &modem_port_ptr->spare_read_urbs);
	}

	for (i = 0; i < modem_port_ptr->rx_buflimit; i++) {
		list_add(&(modem_port_ptr->rb[i].list),
			 &modem_port_ptr->spare_read_bufs);
	}

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	if (modem_port_ptr->susp_count == 0)
		queue_work(modem_port_ptr->sendto_tty_q, &modem_port_ptr->sendto_tty);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

	if (port->number == MODEM_INTERFACE_NUM) {
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		if (modem_port_ptr->susp_count == 0) {
			if (port->interrupt_in_urb) {
				/* start to read INT EP data */
				port->interrupt_in_urb->dev = port->serial->dev;
				retval =
					usb_submit_urb(port->interrupt_in_urb,
							GFP_ATOMIC);
				if (retval) {
					usb_kill_urb(port->interrupt_in_urb);
					dev_err(&port->dev,
							"%s: retval is %d\n",
							__func__, retval);
				}
			} else {
				dev_err(&port->dev,
						"%s: no interrupt endpoint\n",
						__func__);
			}

		}
		spin_unlock_irqrestore(&modem_port_ptr->read_lock,
				flags);
	}
	/* clean up the modem status data */
	modem_port_ptr->modem_status = 0;

	/*  pm interface is taken at
	 *  serial_open() at usb-serial.c.
	 *  For data modem port: the pm count needs to be put back here
	 *  to support the auto-suspend/auto-resume.
	 *  For other test command port: the pm count will be put back at
	 *  the time when port is closed.
	 */
	usb_autopm_put_interface(port->serial->interface);

	if (td_modem_debug)
		dev_info(&port->dev, "%s: Exit. retval = %d\n",
			 __func__, retval);

	return retval;
}

static void modem_rx_wq(struct work_struct *work)
{
	struct modem_port *modem_port_ptr = 
		container_of(work, struct modem_port, sendto_tty);
	struct ap_rb *buf;
	struct tty_struct *tty;
	struct usb_serial_port *port;
	struct ap_ru *rcv;
	unsigned long flags;
	unsigned char throttled;
	int i;

	if (!modem_port_ptr)
		return;

	port = modem_port_ptr->port;
	if (!port)
		return;

	tty = port->port.tty;
	if (!tty)
		return;

	wake_lock_timeout(&modem_port_ptr->modem_wakelock,
			TIMEOUT_AFTERRX);
next_buffer:
	spin_lock_irqsave(&modem_port_ptr->port->lock, flags);
	throttled = modem_port_ptr->port->throttle_req;
	spin_unlock_irqrestore(&modem_port_ptr->port->lock, flags);
	if (throttled) {
/*		dev_err(&port->dev, "%s: throttled.\n", __func__);	*/
		return;
	}

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	if (list_empty(&modem_port_ptr->filled_read_bufs)) {
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
		goto urbs;
	}
	buf = list_entry(modem_port_ptr->filled_read_bufs.next,
			 struct ap_rb, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

	if(tty_buffer_request_room(tty, buf->size) < buf->size) {
		printk("tty_buffer_request_room failed! wait for free.\n");
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		list_add(&buf->list, &modem_port_ptr->filled_read_bufs);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
		return;
	}
#ifdef PNX_DBG 
	printk("rx%d\n", buf->size);
//	for(i=0; i<buf->size; i++)
//		printk("%x ", buf->base[i]);
//	printk("\n");
#endif

	tty_insert_flip_string(tty, buf->base, buf->size);
	tty_flip_buffer_push(tty);

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	list_add(&buf->list, &modem_port_ptr->spare_read_bufs);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
	goto next_buffer;

urbs:
	while (!list_empty(&modem_port_ptr->spare_read_bufs)) {
		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		if (list_empty(&modem_port_ptr->spare_read_urbs)) {
			modem_port_ptr->processing = 0;
			spin_unlock_irqrestore(&modem_port_ptr->read_lock,
					       flags);
			if (td_modem_debug)
				dev_info(&port->dev,
					 "%s: no urb to create.\n", __func__);
			return;
		}
		rcv = list_entry(modem_port_ptr->spare_read_urbs.next,
				 struct ap_ru, list);
		list_del(&rcv->list);

		buf = list_entry(modem_port_ptr->spare_read_bufs.next,
				 struct ap_rb, list);
		list_del(&buf->list);

		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

		rcv->buffer = buf;

		usb_fill_bulk_urb(rcv->urb, modem_port_ptr->port->serial->dev,
				  modem_port_ptr->rx_endpoint,
				  buf->base,
				  modem_port_ptr->readsize,
				  modem_read_bulk_callback, rcv);
		rcv->urb->transfer_dma = buf->dma;
		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		if ((modem_port_ptr->susp_count > 0) ||
		    usb_submit_urb(rcv->urb, GFP_ATOMIC) < 0) {
			list_add(&buf->list, &modem_port_ptr->spare_read_bufs);
			list_add(&rcv->list, &modem_port_ptr->spare_read_urbs);
			modem_port_ptr->processing = 0;
			dev_err(&port->dev, "%s: submit bulk in  urb failed.\n",
				__func__);
			spin_unlock_irqrestore(&modem_port_ptr->read_lock,
						flags);
			return;
		} else {
			spin_unlock_irqrestore(&modem_port_ptr->read_lock,
						flags);
		}
	}
	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	modem_port_ptr->processing = 0;
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

}

static void modem_close(struct usb_serial_port *port)
{
	struct modem_port *modem_port_ptr;

	if (td_modem_debug)
		dev_info(&port->dev, "%s: Enter. Close Port %d\n",
			 __func__, port->number);

	modem_port_ptr = usb_get_serial_data(port->serial);
	if (!modem_port_ptr) {
		dev_err(&port->dev,
			 "%s: null modem port pointer.\n",
			 __func__);
		return;
	}

	modem_port_ptr->port_closing = 1;

	/*  For the data modem port, the pm interface needs to be get here
	 *  and will be put back at serial_close() of usb-serial.c
	 */

	usb_autopm_get_interface(port->serial->interface);

	stop_data_traffic(modem_port_ptr);
	cancel_work_sync(&modem_port_ptr->wake_and_write);
	modem_port_ptr->port = 0;
	modem_port_ptr->modem_status = 0;
	if (modem_port_ptr->delayed_wb)
		modem_port_ptr->delayed_wb->use = 0;

	if (td_modem_debug)
		dev_info(&port->dev, "%s: Exit.\n", __func__);
}

/* caller hold modem_port_ptr->write_lock */
static void modem_write_done(struct modem_port *modem_port_ptr,
				struct ap_wb *wb)
{
	wb->use = 0;
	modem_port_ptr->sending--;
}

static int modem_start_wb(struct modem_port *modem_port_ptr,
				struct ap_wb *wb)
{
	int result = 0;
	struct usb_serial_port *port =  modem_port_ptr->port;
	unsigned long flags;

	if (port == NULL)
		return -ENODEV;

	spin_lock_irqsave(&modem_port_ptr->write_lock, flags);
	modem_port_ptr->sending++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = modem_port_ptr->port->serial->dev;

	result = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (result < 0) {
		dev_err(&port->dev,
			"%s: Submit bulk out URB failed. ret = %d\n",
			__func__, result);
		modem_write_done(modem_port_ptr, wb);
	}

	spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags);
	mark_latest_traffic_time(modem_port_ptr);

	return result;
}

static void modem_wake_and_write(struct work_struct *work)
{
	struct modem_port *modem_port_ptr =
		container_of(work, struct modem_port, wake_and_write);
	struct usb_serial *serial;
	struct usb_serial_port *port =  modem_port_ptr->port;
	int result;

	if (modem_port_ptr->port == NULL)
		return;

	serial = modem_port_ptr->port->serial;

	down(&serial->interface->dev.sem);
	if (serial->interface->dev.power.status >= DPM_OFF ||
			serial->interface->dev.power.status == DPM_RESUMING) {
		printk("[%s]: in suspend or resume, exit\n", __func__);
		up(&serial->interface->dev.sem);
		modem_port_ptr->resuming = 0;
		return;
	}

	result = usb_autopm_get_interface(serial->interface);
	if (result < 0) {
		dev_err(&port->dev, "%s: autopm failed. result = %d \n",
			__func__, result);
		up(&serial->interface->dev.sem);
		modem_port_ptr->resuming = 0;
		return;
	}
	if (modem_port_ptr->delayed_wb) {
		modem_start_wb(modem_port_ptr, modem_port_ptr->delayed_wb);
		modem_port_ptr->delayed_wb = NULL;
	}

	usb_autopm_put_interface(serial->interface);
	up(&serial->interface->dev.sem);
}

static void modem_write_bulk_callback(struct urb *urb)
{
	struct ap_wb *wb = urb->context;
	int status = urb->status;
	struct modem_port *modem_port_ptr = wb->instance;
	struct usb_serial_port *port = modem_port_ptr->port;
	unsigned long flags;

	if (port == NULL)
		return;

	spin_lock_irqsave(&modem_port_ptr->write_lock, flags);
	modem_write_done(modem_port_ptr, wb);
	spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags);
	if (status) {
		dev_err(&port->dev, "%s: status non-zero. status = %d\n",
			 __func__, status);
		return;
	}
	usb_serial_port_softint(port);
}

static int modem_write(struct tty_struct *tty,
				 struct usb_serial_port *port,
				 const unsigned char *buf, int count)
{
	struct usb_serial *serial = port->serial;
	int result, wbn;
	struct ap_wb *wb;
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(port->serial);
	int i;
	unsigned long flags;

	if (count == 0) {
		if (td_modem_debug)
			dev_info(&port->dev, "%s: Exit1: %s count = 0\n",
				 __func__, dev_name(&port->dev));
		return 0;
	}

#ifdef PNX_DBG
	printk("[%s]: cnt = %d \n", __func__, count);
//	for(i=0; i<count; i++)
//		printk("%x ", buf[i]);
//	printk("\n");
#endif

	wake_lock_timeout(&modem_port_ptr->modem_wakelock,
			TIMEOUT_AFTERRX);

	if (serial->num_bulk_out) {
		spin_lock_irqsave(&modem_port_ptr->write_lock, flags);

		if ((modem_port_ptr->susp_count > 0) &&
			(modem_port_ptr->resuming != 0)) {
			spin_unlock_irqrestore(&modem_port_ptr->write_lock,
						flags);
			printk("pnx6718 suspend num error!!!\n");
			return -EBUSY;
		}

		wbn = modem_wb_alloc(modem_port_ptr);
		if (wbn < 0) {
			spin_unlock_irqrestore(&modem_port_ptr->write_lock,
						flags);
			if (td_modem_debug)
				dev_info(&port->dev,
					"%s: all buffers busy!\n", __func__);
			return -ENOMEM;
		}
		wb = &modem_port_ptr->wb[wbn];

		count = min((int)(modem_port_ptr->writesize), count);

		if (td_modem_debug)
			dev_info(&port->dev, "%s: Get %d bytes.\n",
				__func__, count);
		memcpy(wb->buf, buf, count);
		wb->len = count;

		/* start sending */
		if (modem_port_ptr->susp_count > 0) {
			modem_port_ptr->resuming = 1;
			modem_port_ptr->delayed_wb = wb;
			spin_unlock_irqrestore(&modem_port_ptr->write_lock,
						flags);

			/* for the data modem, add wakelock to bypass the issue
			 * caused by skip_sys_resume for FS USB
			 */
			wake_lock_timeout(&modem_port_ptr->modem_wakelock,
					MODEM_WAKELOCK_TIME);
			if (td_modem_debug)
				dev_info(&modem_port_ptr->port->dev,
						"%s: add wakelock\n", __func__);
			schedule_work(&modem_port_ptr->wake_and_write);
			return count;
		}
		spin_unlock_irqrestore(&modem_port_ptr->write_lock, flags);
		result = modem_start_wb(modem_port_ptr, wb);
		if (result >= 0)
			result = count;
		return result;
	}

	/* no bulk out, so return 0 bytes written */
	return 0;
}

#ifdef CONFIG_PM
static void modem_usb_wkup_work(struct work_struct *work)
{
	struct modem_port *modem_port_ptr =
		container_of(work, struct modem_port, bpwk);
	struct usb_serial *serial;
	int result;

	if (modem_port_ptr->port == 0) {
		printk("modem_port_ptr->port == 0! maybe reenum?\n");
		return;
	}

	serial = modem_port_ptr->port->serial;
	down(&serial->interface->dev.sem);
	if (serial->interface->dev.power.status >= DPM_OFF ||
			serial->interface->dev.power.status == DPM_RESUMING) {
		printk("[%s]: in suspend or resume, exit\n", __func__);
		up(&serial->interface->dev.sem);
		return;
	}
	if ((modem_port_ptr->port != 0) &&
			!(atomic_cmpxchg(&modem_port_ptr->wakeup_flag, 0, 1))) {
		/* for the data modem, add wakelock to bypass the issue
		 * caused by skip_sys_resume for FS USB
		 */
		if (td_modem_debug)
			dev_info(&modem_port_ptr->port->dev,
					"%s: add wakelock\n", __func__);
		wake_lock_timeout(&modem_port_ptr->modem_wakelock, MODEM_WAKELOCK_TIME);
		result = usb_autopm_get_interface(serial->interface);
		if (result < 0) {
			atomic_set(&modem_port_ptr->wakeup_flag, 0);
			dev_err(&modem_port_ptr->port->dev,
					"%s: autopm failed. result = %d \n",
					__func__, result);
			up(&serial->interface->dev.sem);
			return;
		}

		if (td_modem_debug)
			dev_info(&modem_port_ptr->port->dev,
					"%s: woke up interface\n", __func__);
		usb_autopm_put_interface_async(serial->interface);
	}
	up(&serial->interface->dev.sem);
}


static int modem_suspend(struct usb_interface *intf,
				   pm_message_t message)
{
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(serial);
	struct usb_serial_port *port;
	unsigned long flags;
	unsigned long threshold_time;
	int tmp;

	if (modem_port_ptr == NULL) {
		dev_err(&intf->dev, " NULL modem_port ptr \n");
		return 0;
	}

	if (td_modem_debug)
		dev_info(&intf->dev, "%s +++ \n", __func__);

	port = modem_port_ptr->port;

	if (port == NULL) {
		if (td_modem_debug)
			dev_info(&intf->dev,
				 "%s: port not open yet \n",
				 __func__);
		modem_port_ptr->susp_count++;
		return 0;
	}

	if (td_modem_debug)
		dev_info(&intf->dev, "%s: Suspend Port  num %d.\n",
			 __func__, port->number);

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	spin_lock(&modem_port_ptr->write_lock);
	tmp = modem_port_ptr->processing + modem_port_ptr->sending;
	spin_unlock(&modem_port_ptr->write_lock);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

	if (tmp) {
		if (td_modem_debug)
			dev_info(&intf->dev,
				 "%s:  sending = %d, receiving = %d.\n",
				 __func__, modem_port_ptr->sending,
				 modem_port_ptr->processing);
		return -EBUSY;
	}

	threshold_time = modem_port_ptr->last_traffic + MODEM_NO_TRAFFIC_TIME;

	if (time_before(jiffies, threshold_time)) {
		if (td_modem_debug)
			dev_info(&intf->dev,
				 "%s: busy. suspend failed. %u %u %u \n", __func__
				 , threshold_time , jiffies, MODEM_NO_TRAFFIC_TIME);
		return -EBUSY;
	}

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	spin_lock(&modem_port_ptr->write_lock);
	modem_port_ptr->susp_count++;
	spin_unlock(&modem_port_ptr->write_lock);
	spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

	stop_data_traffic(modem_port_ptr);

	atomic_set(&modem_port_ptr->wakeup_flag, 0);

	if (td_modem_debug)
		dev_info(&intf->dev, "%s: Port  num %d.suspended\n",
			 __func__, port->number);

	return 0;
}

/* bp need additional gpio to judge bus 
   signal is a resume or noise. 
 */
void modem_preresume(void)
{
	if (td_modem_debug)
		printk("[%s]: \n", __func__);
	gpio_set_value(AP_STATUS_BP, 1);
}

void modem_presuspend(void)
{
	if (td_modem_debug)
		printk("[%s]: \n", __func__);
	gpio_set_value(AP_STATUS_BP, 0);
}

static int modem_resume(struct usb_interface *intf)
{
	struct usb_serial *serial = usb_get_intfdata(intf);
	struct modem_port *modem_port_ptr =
		usb_get_serial_data(serial);
	struct usb_serial_port *port;
	unsigned long flags;
	int retval;

	if (modem_port_ptr == NULL) {
		dev_err(&intf->dev, "%s: null modem port pointer. \n",
				__func__);
		return 0;
	}
	if (td_modem_debug)
		dev_info(&intf->dev, "%s +++ \n", __func__);

	port = modem_port_ptr->port;

	if (port == NULL) {
		if (td_modem_debug)
			dev_info(&intf->dev,
					"%s: port not open yet \n",
					__func__);
		modem_port_ptr->susp_count--;
		return 0;

	}

	spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
	spin_lock(&modem_port_ptr->write_lock);
	if (modem_port_ptr->susp_count > 0) {
		modem_port_ptr->susp_count--;
		spin_unlock(&modem_port_ptr->write_lock);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);

		modem_port_ptr->resuming = 0;

		if (td_modem_debug)
			dev_info(&intf->dev, "%s: port %d is resumed here \n",
					__func__, port->number);

		spin_lock_irqsave(&modem_port_ptr->read_lock, flags);
		if (port->interrupt_in_urb) {
			port->interrupt_in_urb->dev = port->serial->dev;
			retval =
				usb_submit_urb(port->interrupt_in_urb,
						GFP_ATOMIC);
			if (retval) {
				usb_kill_urb(port->interrupt_in_urb);
				dev_err(&port->dev,
						"%s: retval is %d \n",
						__func__, retval);
			}
		} else {
			if (td_modem_debug)
				dev_err(&port->dev,
						"%s: no interrupt endpoint \n",
						__func__);
		}
		spin_unlock_irqrestore(&modem_port_ptr->read_lock,
				flags);

		queue_work(modem_port_ptr->sendto_tty_q, &modem_port_ptr->sendto_tty);
	} else {
		spin_unlock(&modem_port_ptr->write_lock);
		spin_unlock_irqrestore(&modem_port_ptr->read_lock, flags);
	}

	if (td_modem_debug)
		dev_info(&intf->dev, "%s: Port  num %d.resumed\n",
				__func__, port->number);

	return 0;
}

static int modem_reset_resume(struct usb_interface *intf)
{
	int ret = 0;

	if (td_modem_debug)
		dev_info(&intf->dev,
		"%s: Enter \n", __func__);

	ret = modem_resume(intf);

	if (td_modem_debug)
		dev_info(&intf->dev,
		"%s: Exit ret is %d \n", __func__, ret);

	return ret;
}

static int modem_pre_reset(struct usb_interface *intf)
{
	return 0;
}

static int modem_post_reset(struct usb_interface *intf)
{
	return 0;
}

#endif /* CONFIG_PM */

static irqreturn_t bp_status_ap_irq_handler(int irq, void *data)
{
	struct modem_port *modem_port_ptr = data;
	int bp_status_ap;

	bp_status_ap = gpio_get_value(BP_STATUS_AP);
	printk("%s,BP_STATUS_AP interrupt get value %d\n",
		__FUNCTION__, bp_status_ap);
	queue_work(modem_port_ptr->bpwk_q, &modem_port_ptr->bpwk);

	return IRQ_HANDLED;
}

void wkup_aftlp0(void)
{
	if(mdmpt) {
		printk("[%s]: wake lock 2500ms!!\n", __func__);
		wake_lock_timeout(&mdmpt->modem_wakelock,
				TIMEOUT_AFTERRX);
	} else
		printk("[%s]: mdmpt is NULL, maybe disconnect\n", __func__);
}

static int modem_startup(struct usb_serial *serial)
{
	struct usb_serial_port *port = serial->port[0];
	struct modem_port *modem_port_ptr = NULL;
	struct usb_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_host_interface *iface_desc;
	int readsize;
	int num_rx_buf;
	int i;
	int ret;
	struct usb_device *udev = NULL;

	interface = serial->interface;
	iface_desc = interface->cur_altsetting;
	udev = interface_to_usbdev(interface);
#ifdef CONFIG_MOT_FEAT_TD_BP_CTRL
	atomic_set(&td_power_off,0);
#endif
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		if (usb_endpoint_is_bulk_in(endpoint))
			epread = endpoint;
		if (usb_endpoint_is_bulk_out(endpoint))
			epwrite = endpoint;
	}

	if (epread == NULL) {
		dev_err(&serial->dev->dev,
			 "%s: No Bulk In Endpoint for this Interface\n",
			 __func__);
		return -EPERM;
	}
	if (epwrite == NULL) {
		dev_err(&serial->dev->dev,
			 "%s: No Bulk Out Endpoint for this Interface\n",
			 __func__);
		return -EPERM;
	}

	num_rx_buf = AP_NR;
	readsize = le16_to_cpu(epread->wMaxPacketSize) * 1;

	/* setup a buffer to store interface data */
	modem_port_ptr =
	    kzalloc(sizeof(struct modem_port), GFP_KERNEL);
	if (modem_port_ptr == NULL) {
		dev_err(&serial->dev->dev,
			 "%s: error -- no memory on start up.\n",
			 __func__);
		return -ENOMEM;
	}

	/* init queue work for rx processing */
	modem_port_ptr->sendto_tty_q = 
		create_singlethread_workqueue("usb-ipc-receive-queue");
	if(!modem_port_ptr->sendto_tty_q)
		goto err_status_queue;
	modem_port_ptr->rx_buflimit = num_rx_buf;
	modem_port_ptr->rx_endpoint =
		usb_rcvbulkpipe(serial->dev, port->bulk_in_endpointAddress);
	spin_lock_init(&modem_port_ptr->read_lock);
	spin_lock_init(&modem_port_ptr->write_lock);
	spin_lock_init(&modem_port_ptr->last_traffic_lock);

	atomic_set(&modem_port_ptr->wakeup_flag, 0);
	modem_port_ptr->susp_count = 0;
	modem_port_ptr->resuming = 0;
	modem_port_ptr->port = 0;
	modem_port_ptr->last_traffic = jiffies;
	modem_port_ptr->readsize = readsize;
	modem_port_ptr->writesize = le16_to_cpu(epwrite->wMaxPacketSize) * 20;

	INIT_WORK(&modem_port_ptr->sendto_tty, modem_rx_wq);
	INIT_WORK(&modem_port_ptr->bpwk, modem_usb_wkup_work);
	INIT_WORK(&modem_port_ptr->wake_and_write, modem_wake_and_write);

	if (modem_write_buffers_alloc(modem_port_ptr, serial) < 0) {
		dev_err(&serial->dev->dev,
			"%s: out of memory\n", __func__);
		goto alloc_write_buf_fail;
	}

	/* allocate multiple receive urb pool */
	for (i = 0; i < num_rx_buf; i++) {
		struct ap_ru *rcv = &(modem_port_ptr->ru[i]);

		rcv->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (rcv->urb == NULL) {
			dev_err(&serial->dev->dev,
				"%s: out of memory\n", __func__);
			goto alloc_rb_urb_fail;
		}

		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		rcv->instance = modem_port_ptr;
	}

	/* allocate multiple receive buffer */
	for (i = 0; i < num_rx_buf; i++) {
		struct ap_rb *rb = &(modem_port_ptr->rb[i]);

		rb->base = usb_buffer_alloc(serial->dev, readsize,
					GFP_KERNEL, &rb->dma);
		if (!rb->base) {
			dev_err(&serial->dev->dev,
				 "%s : out of memory\n",
				__func__);
			goto alloc_rb_buffer_fail;
		}
	}
	for (i = 0; i < AP_NW; i++) {
		struct ap_wb *snd = &(modem_port_ptr->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!snd->urb) {
			dev_err(&serial->dev->dev, "%s : out of memory "
				"(write urbs usb_alloc_urb)\n", __func__);
			goto alloc_wb_urb_fail;
		}
		usb_fill_bulk_urb(snd->urb, serial->dev,
				usb_sndbulkpipe(serial->dev,
					epwrite->bEndpointAddress),
				NULL, modem_port_ptr->writesize,
				modem_write_bulk_callback, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = modem_port_ptr;
	}

	modem_port_ptr->bpwk_q = 
		create_singlethread_workqueue("pnx6718_usbremote_wq");
	wake_lock_init(&modem_port_ptr->modem_wakelock, WAKE_LOCK_SUSPEND,
			"ap20_usb_modem");
	modem_port_ptr->modem_status = 0;

	/* install serial private data */
	usb_set_serial_data(serial, modem_port_ptr);
	if(iface_desc->desc.bInterfaceNumber == 0)
		mdmpt = modem_port_ptr;
	modem_port_ptr->irq = gpio_to_irq(BP_STATUS_AP);
	ret = request_irq(modem_port_ptr->irq, bp_status_ap_irq_handler,
			IRQ_TYPE_EDGE_RISING, "bp_status_ap_irq", modem_port_ptr);
	if(ret < 0) {
		printk("irq bp_status_ap_irq have been requested!\n", ret);
	}
	/* change usb PM level from on to auto */
	udev->autosuspend_disabled = 0;
	udev->autoresume_disabled = 0;

	return 0;

alloc_wb_urb_fail:
	for (i = 0; i < AP_NW; i++)
		usb_free_urb(modem_port_ptr->wb[i].urb);
alloc_rb_buffer_fail:
	modem_read_buffers_free(modem_port_ptr, serial);
alloc_rb_urb_fail:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(modem_port_ptr->ru[i].urb);
alloc_write_buf_fail:
	modem_write_buffers_free(modem_port_ptr, serial);
	if (modem_port_ptr != NULL) {
		kfree(modem_port_ptr);
		usb_set_serial_data(serial, NULL);
	}
err_status_queue:
	printk("couldn't create usb ipc receive workqueue\n");
	return -ENOMEM;
}

static void modem_disconnect(struct usb_serial *serial)
{
	struct modem_port *modem_port_ptr =
	    usb_get_serial_data(serial);

	uint8_t interface_num =
		serial->interface->cur_altsetting->desc.bInterfaceNumber;

	if (td_modem_debug)
		dev_info(&serial->dev->dev,
			 "%s: Disconnect Interface %d\n", __func__,
			interface_num);

	stop_data_traffic(modem_port_ptr);
	cancel_work_sync(&modem_port_ptr->wake_and_write);
#ifdef CONFIG_MOT_FEAT_TD_BP_CTRL
	if( !atomic_read(&td_power_off) ){
		dev_err(&serial->dev->dev,
				"%s:TD USB IPC Disconnect!!!\n", __func__);
	}
#endif
	free_irq(modem_port_ptr->irq, modem_port_ptr);
	mdmpt = NULL;
}

static void modem_release(struct usb_serial *serial)
{
	struct modem_port *modem_port_ptr =
	usb_get_serial_data(serial);
	int i;

	modem_write_buffers_free(modem_port_ptr, serial);
	modem_read_buffers_free(modem_port_ptr, serial);

	for (i = 0; i < AP_NW; i++)
		usb_free_urb(modem_port_ptr->wb[i].urb);

	for (i = 0; i < modem_port_ptr->rx_buflimit; i++)
		usb_free_urb(modem_port_ptr->ru[i].urb);

	if (modem_port_ptr) {
		/* free private structure allocated for serial device */
		printk("[%s]: ready to free irq\n", __func__);
		if(!mdmpt)
			free_irq(modem_port_ptr->irq, modem_port_ptr);
		else
			printk("[%s]: myfunc\n", __func__);
		wake_lock_destroy(&modem_port_ptr->modem_wakelock);
		kfree(modem_port_ptr);
		mdmpt = NULL;
		usb_set_serial_data(serial, NULL);
	}
}

void bplogvs_callback(struct urb* urb)
{
	char *buf = urb->context;

	kfree(buf);
	usb_free_urb(urb);
}

int bplog_vs(struct modem_port *modem_port_ptr, 
	struct usb_device *udev, char *buf, int len)
{
	struct urb * urb;
	int ret;
	char vs_ctrl[12]={0};
	int i;
	struct usb_serial *serial = modem_port_ptr->port->serial;

	if((modem_port_ptr->wbl.buf=kmalloc(len, GFP_KERNEL)) < 0) {
		printk("[%s]: Failed to kmalloc vs_buf!\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(vs_ctrl, buf, len) < 0) {
		printk("[%s]: Failed to copy arg from user!\n", __func__);
		return -EFAULT;
	}
	memcpy(modem_port_ptr->wbl.buf, vs_ctrl+8, 4);
#ifdef PNX_DBG 
	printk("vendor string is:  ");
	for(i=0; i<len; i++) {
		printk("0x%2x  ", vs_ctrl[i]);
	}
	printk("\n");
#endif

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb){
		kfree(modem_port_ptr->wbl.buf);
		printk("alloc urb error\n");
		return -ENOMEM;
	}
	usb_fill_control_urb(urb, udev, usb_sndctrlpipe(udev, 0), 
		vs_ctrl, modem_port_ptr->wbl.buf, 4, 
		bplogvs_callback, (void *)modem_port_ptr->wbl.buf);

	ret = usb_autopm_get_interface(serial->interface);
	if (ret < 0) {
		dev_err(&serial->dev->dev, "%s %s autopm failed %d",
			dev_driver_string
			(&serial->interface->dev),
			dev_name(&serial->interface->dev), ret);
		kfree(modem_port_ptr->wbl.buf);
		usb_free_urb(urb);
		return ret;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if(ret) {
		printk("fail to write vendor string, URB->error=%d\n", ret);
		kfree(modem_port_ptr->wbl.buf);
		usb_free_urb(urb);
		usb_autopm_put_interface(serial->interface);
		return ret;
	}
	usb_autopm_put_interface(serial->interface);
	mark_latest_traffic_time(modem_port_ptr);
	return 0;
}

static int modem_ioctl(struct tty_struct *tty, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;
	int result = 0;
	struct modem_port *modem_port_ptr = usb_get_serial_data(port->serial);

#ifdef PNX_DBG 
	printk("%s - port %d, cmd %x   my is  %x %x", 
		__func__, port->number, cmd, ARGETVS, ARSETVS);
#endif

	switch (cmd) {
		case ARGETVS:
			printk("cmd is ARGETVS\n");
		break;
		case ARSETVS:
			printk("cmd is ARSETVS\n");
			result = bplog_vs(modem_port_ptr, port->serial->dev, arg, 12);
		break;
		default:
			printk("No this tty ioctl cmd!!!!\n");    
			return -ENOIOCTLCMD;
		break;
	}

	return result;
}

void modem_throttle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	unsigned long flags;

	/* Set the throttle request flag. It will be picked up
	 * by usb_serial_generic_read_bulk_callback(). */
	spin_lock_irqsave(&port->lock, flags);
	port->throttle_req = 1;
	spin_unlock_irqrestore(&port->lock, flags);
}

void modem_unthrottle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct modem_port *modem_port_ptr = usb_get_serial_data(port->serial);
	unsigned long flags;

	/* Clear the throttle flags */
	spin_lock_irqsave(&port->lock, flags);
	port->throttle_req = 0;
	spin_unlock_irqrestore(&port->lock, flags);

/*	printk("re run rx_qw\n");	*/
	queue_work(modem_port_ptr->sendto_tty_q, &modem_port_ptr->sendto_tty);
}

static struct usb_driver modem_driver = {
	.name = "cdma-modem",
	.probe = usb_serial_probe,
	.disconnect = usb_serial_disconnect,
	.id_table = id_table,
	.no_dynamic_id = 1,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
	.suspend = modem_suspend,
	.resume = modem_resume,
	.reset_resume = modem_reset_resume,
	.pre_reset = modem_pre_reset,
	.post_reset = modem_post_reset,
#endif
};

static struct usb_serial_driver modem_device = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "pnx6718-modem",
		   },
	.description = "PNX6718 Modem Driver",
	.usb_driver = &modem_driver,
	.id_table = id_table,
	.num_ports = 1,
	.write = modem_write,
	.write_bulk_callback = modem_write_bulk_callback,
	.read_int_callback = modem_interrupt_callback,
	.tiocmset = modem_tiocmset,
	.tiocmget = modem_tiocmget,
	.open = modem_open,
	.close = modem_close,
	.attach = modem_startup,
	.disconnect = modem_disconnect,
	.release = modem_release,
	.ioctl = modem_ioctl,
	.throttle = modem_throttle,
	.unthrottle = modem_unthrottle,
};

#define BP_USB_VDD_CTRL 97 
int init_ipc_gpio(void)
{
	int ret;

	if((ret=gpio_request(BP_STATUS_AP, "BP Status AP")) < 0) {
		printk("gpio_request BP_STATUS_AP failed!\n");
		goto err3;
	}
	if((ret=gpio_direction_input(BP_STATUS_AP)) < 0) {
		printk("gpio_direction_input BP_STATUS_AP failed!\n");
		goto err2;
	}
	if((ret=gpio_request(AP_STATUS_BP, "AP Status BP")) < 0) {
		printk("gpio_request AP_STATUS_BP failed!\n");
		goto err2;
	}
	if((ret=gpio_direction_output(AP_STATUS_BP, 0)) < 0) {
		printk("gpio_direction_output AP_STATUS_BP failed!\n");
		goto err1;
	}


	return 0;

err1:
	gpio_free(AP_STATUS_BP);
err2:
	gpio_free(BP_STATUS_AP);
err3:
	return ret;
}

void free_ipc_gpio(void)
{
	gpio_free(BP_STATUS_AP);
	gpio_free(AP_STATUS_BP);
}


static void __exit modem_exit(void)
{
	usb_deregister(&modem_driver);
	usb_serial_deregister(&modem_device);
	free_ipc_gpio();
}

static int __init modem_init(void)
{
	int retval;

	retval = init_ipc_gpio();
	if (retval)
		return retval;
	modem_preresume();
	retval = usb_serial_register(&modem_device);
	if (retval) {
		free_ipc_gpio();
		return retval;
	}
	retval = usb_register(&modem_driver);
	if (retval) {
		free_ipc_gpio();
		usb_serial_deregister(&modem_device);
	}
#ifdef CONFIG_MOT_FEAT_TD_BP_CTRL
	retval = td_bp_register_ext_interface(&td_usb_ipc_intf);
	if (retval)
		printk(KERN_WARNING "\n[USB_IPC] register interface to td_ctrl failed.");
#endif
	return retval;
}

module_init(modem_init);
module_exit(modem_exit);

MODULE_DESCRIPTION("USB IPC Driver for PNX6718");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

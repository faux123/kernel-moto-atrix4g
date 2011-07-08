/* include this C file to test some static functions and variables */

#ifndef TS27010_UART_RETRAN
#define TS27010_UART_RETRAN
#endif

#include "ts27010_mux.c"

#include "ut_ts27010.h"

/*
#define TEST_PATTERN_SIZE 4

struct tty_struct *ts27010mux_usb_tty;
#ifdef QUEUE_SELF
struct workqueue_struct *g_mux_usb_queue;
#endif

static const u8 tty2dlci[TS0710_MAX_MUX] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};

static const u8 dlci2tty[TS0710_MAX_CHN] = {
	0xFF, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
};

static unsigned long s_mux_kill_flags;
static u8 crctable[256];
static struct ts0710_con ts0710_connection;

extern struct tty_struct *ts27010_usb_tty_table[TS0710_MAX_MUX];

static int ts0710_valid_dlci(u8 dlci)
{
	int ret;
	FUNC_ENTER();
	if ((dlci < TS0710_MAX_CHN) && (dlci >= 0))
		ret = 1;
	else
		ret = 0;
	FUNC_EXIT();
	return ret;
}

static void ts0710_crc_create_table(u8 table[])
{
	int i, j;

	u8 data;
	u8 code_word = 0xe0;
	u8 sr = 0;
	FUNC_ENTER();

	for (j = 0; j < 256; j++) {
		data = (u8) j;

		for (i = 0; i < 8; i++) {
			if ((data & 0x1) ^ (sr & 0x1)) {
				sr >>= 1;
				sr ^= code_word;
			} else {
				sr >>= 1;
			}

			data >>= 1;
			sr &= 0xff;
		}

		table[j] = sr;
		sr = 0;
	}
	FUNC_EXIT();
}

static u8 ts0710_crc_start(void)
{
	return 0xff;
}

static u8 ts0710_crc_calc(u8 fcs, u8 c)
{
	return crctable[fcs ^ c];
}

static u8 ts0710_crc_end(u8 fcs)
{
	return 0xff - fcs;
}

#ifdef TS27010_UART_RETRAN
static int ts0710_crc_check(u8 fcs)
{
	return fcs == CRC_VALID;
}
#endif

u8 ts0710_usb_crc_data(u8 *data, int length)
{
	u8 fcs = ts0710_crc_start();
	u8 ret;
	FUNC_ENTER();

	while (length--)
		fcs = ts0710_crc_calc(fcs, *data++);

	ret = ts0710_crc_end(fcs);
	FUNC_EXIT();
	return ret;
}

static void ts0710_pkt_set_header(u8 *data, int len, int addr_ea,
					 int addr_cr, int addr_dlci,
					 int control)
{
	struct short_frame *pkt
		= (struct short_frame *)(data + ADDRESS_OFFSET);
	FUNC_ENTER();

	pkt->h.addr.ea = addr_ea;
	pkt->h.addr.cr = addr_cr;
	pkt->h.addr.dlci = addr_dlci;
	pkt->h.control = control;

#ifdef TS27010_UART_RETRAN
	if (addr_dlci == CTRL_CHAN || control != UIH)
		pkt->h.sn = INDIFFERENT_SN;
#endif

	if ((len) > SHORT_PAYLOAD_SIZE) {
		struct long_frame *long_pkt
			= (struct long_frame *)(data + ADDRESS_OFFSET);
		long_pkt->h.length.ea = 0;
		long_pkt->h.length.l_len = len & 0x7F;
		long_pkt->h.length.h_len = (len >> 7) & 0xFF;
	} else {
		pkt->h.length.ea = 1;
		pkt->h.length.len = len;
	}
	FUNC_EXIT();
}

static void *ts0710_pkt_data(u8 *data)
{
	struct short_frame *pkt
		= (struct short_frame *)(data + ADDRESS_OFFSET);
	void *ret;
	FUNC_ENTER();

	if (pkt->h.length.ea == 1)
		ret = pkt->data;
	else
		ret = pkt->data+1;
	FUNC_EXIT();
	return ret;
}

static int ts0710_pkt_send(struct ts0710_con *ts0710, u8 *data)
{
	struct short_frame *pkt
		= (struct short_frame *)(data + ADDRESS_OFFSET);
	u8 *d;
	int len;
	int header_len;
	int res;
	FUNC_ENTER();

	if (pkt->h.length.ea == 1) {
		len = pkt->h.length.len;
		d = pkt->data;
		header_len = sizeof(*pkt);
	} else {
		struct long_frame *long_pkt =
			(struct long_frame *)(data + ADDRESS_OFFSET);
		len = (long_pkt->h.length.h_len << 7) |
			long_pkt->h.length.l_len;
		d = long_pkt->data;
		header_len = sizeof(*long_pkt);
	}

	data[0] = TS0710_BASIC_FLAG;
#ifdef TS27010_UART_RETRAN
	if (CLR_PF(pkt->h.control) == UIH) {
		d[len] = ts0710_usb_crc_data(data + ADDRESS_OFFSET,
			TS0710_FRAME_SIZE(len) - FCS_SIZE - FLAG_SIZE);
	} else {
#endif
		d[len] = ts0710_usb_crc_data(
			data + ADDRESS_OFFSET, header_len);
#ifdef TS27010_UART_RETRAN
	}
#endif
	d[len + 1] = TS0710_BASIC_FLAG;

	if (!ts27010mux_usb_tty) {
		mux_print(MSG_WARNING, "ldisc closed. discarding %d bytes\n",
			   TS0710_FRAME_SIZE(len));
		return TS0710_FRAME_SIZE(len);
	}

	if (CLR_PF(data[CONTROL_OFFSET]) != UIH
		|| (data[ADDRESS_OFFSET] >> 2 == 0)) {
		res = ts27010_usb_control_send(
			ts0710, data, TS0710_FRAME_SIZE(len));
	} else {
		res = ts27010_usb_uih_send(
			ts0710, data, TS0710_FRAME_SIZE(len));
	}
	if (res < 0) {
		mux_print(MSG_ERROR, "pkt write error %d\n", res);
		return res;
	} else if (res != TS0710_FRAME_SIZE(len)) {
		mux_print(MSG_ERROR, "short write %d < %d\n", res,
		       TS0710_FRAME_SIZE(len));
		return -EIO;
	} else {
		mux_print(MSG_DEBUG, "send %d successfully\n", res);
		res = 0;
	}

	FUNC_EXIT();
	return res;
}

static void ts0710_reset_dlci_data(struct dlci_struct *d)
{
	FUNC_ENTER();

	d->state = DISCONNECTED;
	d->flow_control = 0;
	d->clients = 0;
	d->mtu = DEF_TS0710_MTU;
	d->initiator = 0;

	FUNC_EXIT();
}

static void ts0710_reset_dlci(struct dlci_struct *d)
{
	FUNC_ENTER();

	ts0710_reset_dlci_data(d);
	init_waitqueue_head(&d->open_wait);
	init_waitqueue_head(&d->close_wait);
	init_waitqueue_head(&d->mux_write_wait);
	mutex_init(&d->lock);

	FUNC_EXIT();
}

static void ts0710_reset_channels(struct ts0710_con *ts0710)
{
	int j;
	FUNC_ENTER();

	for (j = 0; j < TS0710_MAX_CHN; j++)
		ts0710_reset_dlci(&ts0710->dlci[j]);
	FUNC_EXIT();
}

static void ts0710_init(struct ts0710_con *ts0710)
{
	int j;

	FUNC_ENTER();

	ts0710_crc_create_table(crctable);
	ts0710_reset_channels(ts0710);
	init_waitqueue_head(&ts0710->test_wait);
	ts0710->test_errs = 0;
	ts0710->be_testing = 0;

	for (j = 0; j < TS0710_MAX_MUX; j++)
		mutex_init(&ts0710->chan[j].write_lock);

	FUNC_EXIT();
}

static void ts0710_upon_disconnect(struct ts0710_con *ts0710)
{
	int j;
	FUNC_ENTER();

	for (j = 0; j < TS0710_MAX_CHN; j++) {
		ts0710->dlci[j].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[j].open_wait);
		wake_up_interruptible(&ts0710->dlci[j].close_wait);
	}
	ts0710_reset_channels(ts0710);
	FUNC_EXIT();
}

static int ts27010_send_cmd(struct ts0710_con *ts0710, u8 dlci, u8 cmd)
{
	u8 frame[TS0710_FRAME_SIZE(0)];
	int ret;
	FUNC_ENTER();

	ts0710_pkt_set_header(frame, 0, 1,
		ts0710->dlci[dlci].initiator & 0x01,
		dlci, SET_PF(cmd));
	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}


static int ts27010_send_ua(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending UA packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, UA);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_dm(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending DM packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, DM);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_sabm(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending SABM packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, SABM);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_disc(struct ts0710_con *ts0710, u8 dlci)
{
	int ret;
	FUNC_ENTER();
	mux_print(MSG_INFO, "sending DISC packet to DLCI %d\n", dlci);
	ret = ts27010_send_cmd(ts0710, dlci, DISC);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_uih(struct ts0710_con *ts0710, u8 dlci,
			    u8 *frame, const u8 *data, int len)
{
	int ret;

	FUNC_ENTER();
	mux_print(MSG_DEBUG, "sending %d bytes UIH to DLCI %d\n",
		 len, dlci);
	ts0710_pkt_set_header(frame,
		len,
		1,
		MCC_CMD,
		dlci,
		CLR_PF(UIH));

	memcpy(ts0710_pkt_data(frame), data, len);
	ret = ts0710_pkt_send(ts0710, frame);

	FUNC_EXIT();
	return ret;
}

static void ts27010_mcc_set_header(u8 *frame, int len, int cr, int cmd)
{
	struct mcc_short_frame *mcc_pkt;
	FUNC_ENTER();

	ts0710_pkt_set_header(frame, sizeof(struct mcc_short_frame) + len,
			      1, MCC_CMD, CTRL_CHAN, CLR_PF(UIH));

	mcc_pkt = ts0710_pkt_data(frame);
	mcc_pkt->h.type.ea = EA;
	mcc_pkt->h.type.cr = cr;
	mcc_pkt->h.type.type = cmd;
	mcc_pkt->h.length.ea = EA;
	mcc_pkt->h.length.len = len;
	FUNC_EXIT();
}

static void *ts27010_mcc_data(u8 *frame)
{
	return ((struct mcc_short_frame *)ts0710_pkt_data(frame))->value;
}

static int ts27010_send_fcon(struct ts0710_con *ts0710, int cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(0)];
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending FCON MCC\n");
	ts27010_mcc_set_header(frame, 0, cr, FCON);

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_fcoff(struct ts0710_con *ts0710, int cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(0)];
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending FCOFF MCC\n");
	ts27010_mcc_set_header(frame, 0, cr, FCOFF);

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_pn(struct ts0710_con *ts0710, u8 prior, int frame_size,
			   u8 credit_flow, u8 credits, u8 dlci, u8 cr)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct pn_msg_data))];
	struct pn_msg_data *pn;
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending PN MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct pn_msg_data), cr, PN);

	pn = ts27010_mcc_data(frame);
	pn->res1 = 0;
	pn->res2 = 0;
	pn->dlci = dlci;
	pn->frame_type = 0;
	pn->credit_flow = credit_flow;
	pn->prior = prior;

#ifdef TS27010_UART_RETRAN
	pn->ack_timer = RETRAN_TIMEOUT;
	pn->max_nbrof_retrans = MAX_RETRAN_TIMES;
#else
	pn->ack_timer = 0;
	pn->max_nbrof_retrans = 0;
#endif
	pn->frame_sizel = frame_size & 0xff;
	pn->frame_sizeh = frame_size >> 8;
	pn->credits = credits;

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_nsc(struct ts0710_con *ts0710, u8 type, int cr)
{
	int ret;
	FUNC_ENTER();
#if 1
	mux_print(MSG_INFO, "Received Non supported command response\n");
	ret = 0;
#else
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct mcc_type))];
	struct mcc_type *t;

	mux_print(MSG_DEBUG, "sending NSC MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct mcc_type), cr, NSC);

	t = ts27010_mcc_data(frame);
	t->ea = 1;
	t->cr = mcc_is_cmd(type);
	t->type = type >> 2;

	ret = ts0710_pkt_send(ts0710, frame);
#endif
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_test_msg(struct ts0710_con *ts0710, u8 *d_buf,
	int size, u8 cr)
{
	int ret;
	u8 frame[TS0710_MCC_FRAME_SIZE(size)];
	u8 *t_data;
	FUNC_ENTER();

	mux_print(MSG_DEBUG, "sending TEST MCC\n");
	ts27010_mcc_set_header(frame, size, cr, TEST);
#ifdef TS27010_UART_RETRAN
	((struct short_frame *)frame)->h.sn = INDIFFERENT_SN;
#endif

	t_data = ts27010_mcc_data(frame);

	memcpy(t_data, d_buf, size);

	ret = ts0710_pkt_send(ts0710, frame);
	if (ret == TS0710_MCC_FRAME_SIZE(size))
		ret = 0;
	FUNC_EXIT();
	return ret;
}

static int ts27010_send_msc(struct ts0710_con *ts0710,
			    u8 value, int cr, u8 dlci)
{
	u8 frame[TS0710_MCC_FRAME_SIZE(sizeof(struct msc_msg_data))];
	struct msc_msg_data *msc;
	int ret;
	FUNC_ENTER();

	mux_print(MSG_INFO, "sending MSC MCC\n");
	ts27010_mcc_set_header(frame, sizeof(struct msc_msg_data), cr, MSC);

	msc = ts27010_mcc_data(frame);

	msc->dlci.ea = 1;
	msc->dlci.cr = 1;
	msc->dlci.dlci = dlci;

	msc->v24_sigs = value;

	ret = ts0710_pkt_send(ts0710, frame);
	FUNC_EXIT();
	return ret;
}

static void ts27010_handle_test(struct ts0710_con *ts0710, u8 type,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	FUNC_ENTER();

	mux_print(MSG_INFO, "test command received\n");
	if (mcc_is_cmd(type)) {
		u8 *data;
		int i;

		data = kmalloc(len, GFP_KERNEL);
		if (!data) {
			mux_print(MSG_WARNING,
				"not enough memory for test data: %d\n", len);
			return;
		}
		for (i = 0; i < len; i++)
			data[i] = ts27010_ringbuf_peek(rbuf, data_idx + i);

		ts27010_send_test_msg(ts0710, data, len, MCC_RSP);
		kfree(data);
	} else {
		if (ts0710->be_testing) {
			int i;
			u8 data[TEST_PATTERN_SIZE];
			if (len != TEST_PATTERN_SIZE) {
				mux_print(MSG_ERROR,
					"reveived test on length:%d != %d\n",
					len, TEST_PATTERN_SIZE);
				ts0710->test_errs = TEST_PATTERN_SIZE;
				return;
			}

			for (i = 0; i < TEST_PATTERN_SIZE; i++)
				data[i] = ts27010_ringbuf_peek(
					rbuf, data_idx + i);

			ts0710->test_errs = 0;
			for (i = 0; i < TEST_PATTERN_SIZE; i++) {
				if (data[i] != (i & 0xFF))
					ts0710->test_errs++;
			}
			ts0710->be_testing = 0;
			wake_up_interruptible(&ts0710->test_wait);
		} else {
			mux_print(MSG_ERROR, "Err: shouldn't or late "
				"to get test cmd response\n");
		}
	}
	FUNC_EXIT();
}

static void ts27010_handle_msc(struct ts0710_con *ts0710, u8 type,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	u8 dlci;
	u8 v24_sigs;
	struct dlci_struct *d;
	FUNC_ENTER();

	dlci = ts27010_ringbuf_peek(rbuf, data_idx) >> 2;
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}
	v24_sigs = ts27010_ringbuf_peek(rbuf, data_idx + 1);

	d = &ts0710->dlci[dlci];
	if ((d->state != CONNECTED)
	    && (d->state != FLOW_STOPPED)) {
		mux_print(MSG_ERROR, "DLCI %d is not connected: %d\n",
			dlci, d->state);
		ts27010_send_dm(ts0710, dlci);
		return;
	}

	if (mcc_is_cmd(type)) {
		mux_print(MSG_INFO, "received modem status command\n");
		if (v24_sigs & FC) {
			if (d->state == CONNECTED) {
				mux_print(MSG_WARNING,
					"flow off on dlci %d\n", dlci);
				d->state = FLOW_STOPPED;
			} else {
				mux_print(MSG_WARNING,
					"flow off on dlci %d but state: %d\n",
					dlci, d->state);
			}
		} else {
			if (d->state == FLOW_STOPPED) {
				d->state = CONNECTED;
				mux_print(MSG_WARNING,
					"flow on on dlci %d\n", dlci);
				wake_up_interruptible(&d->mux_write_wait);
			} else {
				mux_print(MSG_WARNING,
					"flow on on dlci %d but state: %d\n",
					dlci, d->state);
			}
		}
		ts27010_send_msc(ts0710, v24_sigs, MCC_RSP, dlci);
	} else {
		mux_print(MSG_INFO, "received modem status response\n");

		if (v24_sigs & FC)
			mux_print(MSG_INFO, "flow off accepted\n");
		else
			mux_print(MSG_INFO, "flow on accepted\n");
	}
	FUNC_EXIT();
}

static void ts27010_handle_pn(struct ts0710_con *ts0710, u8 type,
			      struct ts27010_ringbuf *rbuf,
			      int data_idx, int len)
{
	u8 dlci;
	u16 frame_size;
	struct pn_msg_data pn;
	int i;
	FUNC_ENTER();

	if (len != 8) {
		mux_print(MSG_ERROR, "reveived pn on length:%d != 8\n", len);
		return;
	}

	for (i = 0; i < 8; i++)
		((u8 *)&pn)[i] = ts27010_ringbuf_peek(rbuf, data_idx + i);

	dlci = pn.dlci;
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}
	frame_size = pn.frame_sizel | (pn.frame_sizeh << 8);

	if (mcc_is_cmd(type)) {
		mux_print(MSG_INFO,
			"received PN command with frame size %d\n",
			frame_size);

		frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
		ts27010_send_pn(ts0710, pn.prior, frame_size,
				0, 0, dlci, MCC_RSP);
		ts0710->dlci[dlci].mtu = frame_size;

		mux_print(MSG_INFO, "mtu set to %d on dlci%d\n",
			 frame_size, dlci);
	} else {
		mux_print(MSG_INFO,
			"received PN response with frame size %d\n",
			frame_size);

		frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
		ts0710->dlci[dlci].mtu = frame_size;

		mux_print(MSG_INFO, "mtu set to %d on dlci%d\n",
			 frame_size, dlci);

		if (ts0710->dlci[dlci].state == NEGOTIATING) {
			mutex_lock(&ts0710->dlci[dlci].lock);
			ts0710->dlci[dlci].state = CONNECTING;
			mutex_unlock(&ts0710->dlci[dlci].lock);
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		}
	}
	FUNC_EXIT();
}

static void ts27010_handle_mcc(struct ts0710_con *ts0710, u8 control,
			       struct ts27010_ringbuf *rbuf,
			       int data_idx, int len)
{
	u8 type;
	u8 mcc_len;
	u8 j;
	FUNC_ENTER();

	type = ts27010_ringbuf_peek(rbuf, data_idx++);
	len--;
	mcc_len = ts27010_ringbuf_peek(rbuf, data_idx++);
	len--;
	mcc_len >>= 1;

	if (mcc_len != len) {
		mux_print(MSG_WARNING, "handle_mcc: mcc_len:%d != len:%d\n",
			   mcc_len, len);
	}

	switch (type >> 2) {
	case TEST:
		ts27010_handle_test(ts0710, type, rbuf, data_idx, len);
		break;

	case FCON:
		mux_print(MSG_WARNING,
			"received all channels flow control on\n");
		if ((type & 0x2) >> 1 == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++)
				ts0710->dlci[j].state = CONNECTED;
			ts27010_send_fcon(ts0710, MCC_RSP);
		}
		break;

	case FCOFF:
		mux_print(MSG_WARNING,
			"received all channels flow control off\n");
		if ((type & 0x2) >> 1 == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++)
				ts0710->dlci[j].state = FLOW_STOPPED;
			ts27010_send_fcoff(ts0710, MCC_RSP);
		}
		break;

	case MSC:
		ts27010_handle_msc(ts0710, type, rbuf, data_idx, len);
		break;

	case PN:
		ts27010_handle_pn(ts0710, type, rbuf, data_idx, len);
		break;

	case NSC:
		mux_print(MSG_WARNING,
			"received non supported cmd response\n");
		break;

	default:
		mux_print(MSG_WARNING, "received a non supported command\n");
		ts27010_send_nsc(ts0710, type, MCC_RSP);
		break;
	}
	FUNC_EXIT();
}

static void ts27010_flow_on(u8 dlci, struct ts0710_con *ts0710)
{
	int i;
	u8 tty_idx;
	struct tty_struct *tty;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	tty_idx = dlci2tty[dlci] - TS27010MUX_NAME_BASE;
	WARN_ON(tty_idx >= TS0710_MAX_MUX || tty_idx < 0);

	tty = ts27010_usb_tty_table[tty_idx];
	if (!tty) {
		mux_print(MSG_ERROR,
			"DLCI %d no tty used to set throttle\n", dlci);
		return;
	}
	if (test_bit(TTY_THROTTLED, &tty->flags)) {
		mux_print(MSG_WARNING,
			"DLCI %d tty throttle set, can't unthrottle\n",
			dlci);
		return;
	}

	if ((ts0710->dlci[0].state != CONNECTED)
		&& (ts0710->dlci[0].state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "BP is not connected: %d\n",
			ts0710->dlci[0].state);
		return;
	} else if ((d->state != CONNECTED) && (d->state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "DLCI %d is not connected: %d\n",
			dlci, d->state);
		return;
	}

	if (!(d->flow_control)) {
		mux_print(MSG_WARNING,
			"DLCI %d is not in AP flow_control\n", dlci);
		return;
	}

	for (i = 0; i < 3; i++) {
		int ret;
		ret = ts27010_send_msc(ts0710,
			EA | RTC | RTR | DV, MCC_CMD, dlci);
		if (ret < 0) {
			mux_print(MSG_WARNING,
				"send flow on failed on dlci %d: %d\n",
				dlci, ret);
			continue;
		} else {
			mux_print(MSG_INFO,
				"send flow on on dlci %d to BP successfully\n",
				dlci);
			d->flow_control = 0;
			break;
		}
	}
	FUNC_EXIT();
}

static void ts27010_flow_off(struct tty_struct *tty, u8 dlci,
			    struct ts0710_con *ts0710)
{
	int i;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	if (!tty) {
		mux_print(MSG_ERROR,
			"DLCI %d no tty used to set throttle\n", dlci);
		return;
	}
	if (test_bit(TTY_THROTTLED, &tty->flags) == 0) {
		mux_print(MSG_WARNING,
			"DLCI %d no tty throttle set\n", dlci);
		return;
	}

	if ((ts0710->dlci[0].state != CONNECTED)
		&& (ts0710->dlci[0].state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "BP is not connected: %d\n",
			ts0710->dlci[0].state);
		return;
	} else if ((d->state != CONNECTED) && (d->state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING, "DLCI %d is not connected: %d\n",
			dlci, d->state);
		return;
	}

	if (d->flow_control) {
		mux_print(MSG_WARNING,
			"DLCI %d has been in AP flow_control\n", dlci);
		return;
	}

	for (i = 0; i < 3; i++) {
		int ret;
		ret = ts27010_send_msc(ts0710,
			EA | FC | RTC | RTR | DV, MCC_CMD, dlci);
		if (ret < 0) {
			mux_print(MSG_WARNING,
				"send flow off failed on dlci %d: %d\n",
				dlci, ret);
			continue;
		} else {
			mux_print(MSG_INFO,
				"send flow off on dlci %d successfully\n",
				dlci);
			d->flow_control = 1;
			break;
		}
	}
	FUNC_EXIT();
}

static void ts27010_handle_sabm(
	struct ts0710_con *ts0710, u8 control, int dlci)
{
	FUNC_ENTER();
	mux_print(MSG_INFO, "SABM received on dlci %d\n", dlci);

	if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];
		ts27010_send_ua(ts0710, dlci);

		d->state = CONNECTED;
		wake_up_interruptible(&d->open_wait);
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d.\n", dlci);
	}
	FUNC_EXIT();
}

static void ts27010_handle_ua(struct ts0710_con *ts0710, u8 control, int dlci)
{
	FUNC_ENTER();
	mux_print(MSG_INFO, "UA packet received on dlci %d\n", dlci);

	if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];

		if (d->state == CONNECTING) {
			mutex_lock(&d->lock);
			d->state = CONNECTED;
			mutex_unlock(&d->lock);
			wake_up_interruptible(&d->open_wait);
		} else if (d->state == DISCONNECTING) {
			if (dlci == 0) {
				ts0710_upon_disconnect(ts0710);
			} else {
				mutex_lock(&d->lock);
				d->state = DISCONNECTED;
				mutex_unlock(&d->lock);
				wake_up_interruptible(&d->open_wait);
				wake_up_interruptible(&d->close_wait);
			}
			mux_print(MSG_INFO, "dlci %d disconnected\n", dlci);
		} else {
			if (dlci != 0 && test_bit(dlci, &s_mux_kill_flags)) {
				mutex_lock(&d->lock);
				d->state = DISCONNECTING;
				mutex_unlock(&d->lock);
				clear_bit(dlci, &s_mux_kill_flags);
				ts27010_send_disc(ts0710, dlci);
			}
			mux_print(MSG_WARNING,
				"invalid UA packet on dlci: %x state: %d\n",
				dlci, d->state);
		}
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d\n", dlci);
	}
	FUNC_EXIT();
}

static void ts27010_handle_dm(struct ts0710_con *ts0710, u8 control, int dlci)
{
	int oldstate;
	FUNC_ENTER();

	mux_print(MSG_INFO, "DM packet received on dlci %d\n", dlci);

	if (dlci == 0) {
		oldstate = ts0710->dlci[0].state;
		ts0710_upon_disconnect(ts0710);
		mutex_lock(&ts0710->dlci[0].lock);
		if (oldstate == CONNECTING)
			ts0710->dlci[0].state = REJECTED;
		mutex_unlock(&ts0710->dlci[0].lock);
	} else if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];

		mutex_lock(&d->lock);
		if (d->state == CONNECTING)
			d->state = REJECTED;
		else
			d->state = DISCONNECTED;
		mutex_unlock(&d->lock);

		wake_up_interruptible(&d->open_wait);
		wake_up_interruptible(&d->close_wait);
		ts0710_reset_dlci_data(d);
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d\n", dlci);
	}
	FUNC_EXIT();
}

static void ts27010_handle_disc(
	struct ts0710_con *ts0710, u8 control, int dlci)
{
	FUNC_ENTER();
	mux_print(MSG_INFO, "DISC packet received on dlci %d\n", dlci);

	if (!dlci) {
		ts27010_send_ua(ts0710, dlci);
		mux_print(MSG_INFO, "sending back UA\n");

		ts0710_upon_disconnect(ts0710);
	} else if (ts0710_valid_dlci(dlci)) {
		struct dlci_struct *d = &ts0710->dlci[dlci];
		ts27010_send_ua(ts0710, dlci);
		mux_print(MSG_INFO, "sending back UA\n");

		d->state = DISCONNECTED;
		wake_up_interruptible(&d->open_wait);
		wake_up_interruptible(&d->close_wait);
	} else {
		mux_print(MSG_WARNING, "invalid dlci %d\n", dlci);
	}
	FUNC_EXIT();
}

static void ts27010_handle_uih(struct ts0710_con *ts0710, u8 control, int dlci,
	struct ts27010_ringbuf *rbuf, int data_idx, int len)
{
	int tty_idx;
	FUNC_ENTER();

	if ((dlci >= TS0710_MAX_CHN)) {
		mux_print(MSG_ERROR, "invalid dlci %d\n", dlci);
		return;
	}

	if (GET_PF(control)) {
		mux_print(MSG_WARNING,
			"dlci %d: uih packet with P/F set, discarding %d\n",
				dlci, len);
		return;
	}

	if ((ts0710->dlci[dlci].state != CONNECTED)
		&& (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		mux_print(MSG_WARNING,
			"uih: dlci %d not connected(%d), discarding %d.\n",
			dlci, ts0710->dlci[dlci].state, len);
		ts27010_send_dm(ts0710, dlci);
		return;
	}

	if (dlci == 0) {
		mux_print(MSG_INFO, "handle mcc on DLCI 0\n");
		ts27010_handle_mcc(ts0710, control, rbuf, data_idx, len);
		return;
	}
	mux_print(MSG_DEBUG,
		"receive 0x%x bytes UIH from DLCI %d\n", len, dlci);

	if (len > ts0710->dlci[dlci].mtu) {
		mux_print(MSG_WARNING, "dlci %d: uih_len:%d "
			   "is bigger than mtu:%d, discarding.\n",
			    dlci, len, ts0710->dlci[dlci].mtu);
		len = ts0710->dlci[dlci].mtu;
	}
	if (len == 0) {
		mux_print(MSG_WARNING, "dlci %d: uih_len is 0.\n", dlci);
		return;
	}
	tty_idx = dlci2tty[dlci];
	mux_print(MSG_DEBUG, "receive data on DLCI %d, /dev/usb%d\n",
		 dlci, tty_idx);
	tty_idx -= TS27010MUX_NAME_BASE;
	ts27010_tty_usb_send_rbuf(tty_idx, rbuf, data_idx, len);
	FUNC_EXIT();
}

#ifdef TS27010_UART_RETRAN
static void ts27010_handle_retran_frame(struct ts0710_con *ts0710,
	struct ts27010_ringbuf *rbuf, u8 addr, u8 sn,
	u8 control, int data_idx, int len)
{
	FUNC_ENTER();

	if (CLR_PF(control) == ACK) {
		ts27010_usb_process_ack(ts0710, sn);
	} else {
		if (sn != INDIFFERENT_SN) {
			if (!ts27010_usb_check_sequence_number(ts0710, sn))
				return;
		}
		ts27010_handle_frame(
			ts0710, rbuf, addr, control, data_idx, len);
	}
	FUNC_EXIT();
}
#endif

static int ts27010_wait_for_close(struct ts0710_con *ts0710, int dlci)
{
	int ret;
	struct dlci_struct *d = &ts0710->dlci[dlci];

	mutex_unlock(&d->lock);
	ret = wait_event_interruptible_timeout(d->close_wait,
		d->state != DISCONNECTING, TS0710MUX_TIME_OUT);

	mutex_lock(&d->lock);
	if (ret == 0) {
		mux_print(MSG_INFO,
			"DLCI %d Wait for disconnecting timeout!\n", dlci);
		return -ETIMEDOUT;
	}

	if (ret == -ERESTARTSYS) {
		mux_print(MSG_ERROR,
			"DLCI %d Send DISC got signal! but I send 2 again\n",
			dlci);
		msleep(20);
		ts27010_send_disc(ts0710, dlci);
		msleep(30);
		ts27010_send_disc(ts0710, dlci);
		return -EAGAIN;
	}

	if (d->state != DISCONNECTED) {
		mux_print(MSG_ERROR,
			"DLCI %d wait for disconnected "
			"got invalid state: %d!\n",
			dlci, d->state);
		return -ENODEV;
	}

	mux_print(MSG_INFO, "DLCI %d disconnected!\n", dlci);
	return 0;
}

static int ts27010_close_channel(u8 dlci)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	int try;
	int retval;
	FUNC_ENTER();

	mux_print(MSG_INFO, "closing dlci %d\n", dlci);

	mutex_lock(&d->lock);
	if (d->state == DISCONNECTED || d->state == REJECTED) {
		mux_print(MSG_WARNING, "DLCI %d has been closed!\n", dlci);
		if (d->clients > 0)
			d->clients--;
		retval = 0;
		goto EXIT;
	} else if (d->state == DISCONNECTING) {
		mux_print(MSG_WARNING,
			"DLCI %d is being disconnected!\n", dlci);

		try = 8;
		while (try--) {
			retval = ts27010_wait_for_close(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (!retval && d->clients > 0)
			d->clients--;
		goto EXIT;
	}

	if ((d->clients > 1)
		&& (d->state == CONNECTED || d->state == FLOW_STOPPED)) {
		mux_print(MSG_WARNING,
			"DLCI %d not closed clients: %d!\n",
			dlci, d->clients);
		d->clients--;
		retval = 0;
		goto EXIT;
	}

	WARN_ON(d->clients != 1);
	d->state = DISCONNECTING;
	try = 10;
	while (try--) {
		retval = ts27010_send_disc(ts0710, dlci);
		if (retval) {
			mux_print(MSG_ERROR, "DLCI %d send DISC failed\n",
				dlci);
			msleep_interruptible(20);
			continue;
		}

		retval = ts27010_wait_for_close(ts0710, dlci);
		if (retval == -ETIMEDOUT)
			continue;
		else if (!retval)
			mux_print(MSG_INFO, "DLCI %d closed\n", dlci);
		break;
	}

	if (try < 0 && d->state != DISCONNECTED)
		retval = -EIO;

	if (d->state != DISCONNECTED) {
		if (dlci == 0) {
			ts0710_upon_disconnect(ts0710);
		} else {
			ts0710_reset_dlci_data(d);
			wake_up_interruptible(&d->close_wait);
		}
	}
	d->state = DISCONNECTED;
	d->clients = 0;
	if (dlci != 0) {
		mutex_lock(&ts0710->dlci[0].lock);
		ts0710->dlci[0].clients--;
		mutex_unlock(&ts0710->dlci[0].lock);
		mux_print(MSG_DEBUG, "Dec control ref %d\n",
			ts0710->dlci[0].clients);
	}

EXIT:
	mutex_unlock(&d->lock);
	FUNC_EXIT();

	return retval;
}

void ts27010_mux_usb_line_close(int line)
{
	int dlci;
	int closeCTRL = 1;
	int j;
	FUNC_ENTER();

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return;
	}
	ts27010_close_channel(dlci);

	for (j = 1; j < TS0710_MAX_CHN; j++) {
		struct dlci_struct *d = &ts0710_connection.dlci[j];
		if (d->state != DISCONNECTED) {
			closeCTRL = 0;
			break;
		}
	}
	if (closeCTRL) {
		mux_print(MSG_INFO, "All mux devices closed, "
			"will close control channel.\n");
		ts27010_close_channel(0);
	}
	FUNC_EXIT();
}

static int ts27010_wait_for_open(struct ts0710_con *ts0710, int dlci)
{
	int ret;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	mutex_unlock(&d->lock);
	ret = wait_event_interruptible_timeout(d->open_wait,
		d->state != CONNECTING, TS0710MUX_TIME_OUT);
	mutex_lock(&d->lock);
	if (ret == 0) {
		mux_print(MSG_INFO,
			"DLCI %d Wait for connecting timeout!\n", dlci);
		return -ETIMEDOUT;
	}

	if (ret == -ERESTARTSYS) {
		mux_print(MSG_ERROR,
			"DLCI %d Wait for connecting got signal!\n", dlci);
		set_bit(dlci, &s_mux_kill_flags);
		return -EAGAIN;
	}

	if (d->state == REJECTED) {
		mux_print(MSG_ERROR,
			"DLCI %d Wait for connecting got rejected!\n", dlci);
		return -EREJECTED;
	}

	if (d->state != CONNECTED && d->state != FLOW_STOPPED) {
		mux_print(MSG_ERROR,
			"DLCI %d Wait for connecting got invalid state: %d!\n",
			dlci, d->state);
		return -ENODEV;
	}
	mux_print(MSG_INFO, "DLCI %d connected!\n", dlci);
	FUNC_EXIT();

	return 0;
}

static int ts27010_wait_for_negotiated(struct ts0710_con *ts0710, int dlci)
{
	int ret;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	FUNC_ENTER();

	mutex_unlock(&d->lock);
	ret = wait_event_interruptible_timeout(d->open_wait,
		d->state != NEGOTIATING, TS0710MUX_TIME_OUT);

	mutex_lock(&d->lock);
	if (ret == 0) {
		mux_print(MSG_INFO,
			"DLCI %d Wait for negotiated timeout!\n", dlci);
		return -ETIMEDOUT;
	}

	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		mux_print(MSG_INFO,
			"DLCI %d connected!\n", dlci);
		return 0;
	}

	if (d->state == CONNECTING) {
		mux_print(MSG_INFO,
			"DLCI %d negotiated!\n", dlci);
		FUNC_EXIT();
		return 0;
	}

	if (ret == -ERESTARTSYS) {
		mux_print(MSG_ERROR,
			"DLCI %d Wait for negotiated got signal!\n", dlci);
		set_bit(dlci, &s_mux_kill_flags);
		return -EAGAIN;
	}

	if (d->state == REJECTED) {
		mux_print(MSG_ERROR,
			"DLCI %d wait for negotiated got rejected!\n", dlci);
		return -EREJECTED;
	} else {
		mux_print(MSG_ERROR,
			"DLCI %d wait for negotiated got invalid state: %d!\n",
			dlci, d->state);
		return -ENODEV;
	}
}

static int ts27010_open_channel(u8 dlci)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = NULL;
	int try;
	int retval = -ENODEV;
	FUNC_ENTER();

	mux_print(MSG_DEBUG, "will open dlci %d\n", dlci);

	d = &ts0710->dlci[dlci];

	mutex_lock(&d->lock);
	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		mux_print(MSG_WARNING,
			"DLCI %d has been opened clients: %d!\n",
			dlci, d->clients);
		WARN_ON(d->clients < 1);
		d->clients++;
		retval = 0;
		goto EXIT;
	}
	if (d->state == NEGOTIATING) {
		try = 8;
		while (try--) {
			retval = ts27010_wait_for_negotiated(
				ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
			d->clients++;
			retval = 0;
			goto EXIT;
		}
	}
	if (d->state == CONNECTING) {
		try = 8;
		while (try--) {
			retval = ts27010_wait_for_open(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
			d->clients++;
			retval = 0;
		}
		goto EXIT;
	}
	if (d->clients > 0) {
		mux_print(MSG_ERROR, "DLCI %d invalid state: %d, clients: %d!\n",
			dlci, d->state, d->clients);
		retval = -EREJECTED;
		goto EXIT;
	}
	if ((d->state != DISCONNECTED) && (d->state != REJECTED)) {
		mux_print(MSG_ERROR,
			"DLCI %d state is invalid: %d!\n", dlci, d->state);
		retval = -ENODEV;
		goto EXIT;
	}

	WARN_ON(d->clients != 0);
	d->state = NEGOTIATING;
	d->initiator = 1;
	try = 10;
	while (try--) {
		retval = ts27010_send_pn(ts0710, 7, d->mtu, 0, 0, dlci, 1);
		if (retval) {
			mux_print(MSG_ERROR,
				"send pn to open channel 0x%x failed: %d\n",
				dlci, retval);
			msleep_interruptible(20);
			continue;
		}
		mux_print(MSG_INFO, "wait for 0x%x negotiated\n", dlci);

		retval = ts27010_wait_for_negotiated(ts0710, dlci);
		if (retval == -ETIMEDOUT)
			continue;
		break;
	}

	if (d->state == CONNECTING) {
		try = 10;
		while (try--) {
			retval = ts27010_send_sabm(ts0710, dlci);
			if (retval) {
				mux_print(MSG_ERROR,
					"send sabm to open channel 0x%x "
					"failed: %d\n",
					dlci, retval);
				msleep_interruptible(20);
				continue;
			}
			mux_print(MSG_INFO, "wait for 0x%x opened\n", dlci);

			retval = ts27010_wait_for_open(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			break;
		}
	}

	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		mux_print(MSG_INFO, "DLCI %d connected!\n", dlci);
		d->clients++;
	} else {
		mux_print(MSG_ERROR, "open DLCI %d failed: %d\n",
			dlci, d->state);
		d->state = REJECTED;
		retval = -ENODEV;
	}

	wake_up_interruptible(&d->open_wait);

EXIT:
	mutex_unlock(&d->lock);
	FUNC_EXIT();
	return retval;
}

static int ts27010_open_ctrl_channel(int dlci)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	struct dlci_struct *d = &ts0710->dlci[dlci];
	int try;
	int retval = -ENODEV;
	FUNC_ENTER();

	mutex_lock(&d->lock);
	if (d->state == CONNECTED || d->state == FLOW_STOPPED) {
		mux_print(MSG_INFO,
			"DLCI %d has been opened clients: %d!\n",
			dlci, d->clients);
		d->clients++;
		retval = 0;
		goto EXIT;
	} else if (d->state == CONNECTING) {
		try = 8;
		while (try--) {
			retval = ts27010_wait_for_open(ts0710, dlci);
			if (retval == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (!retval)
			d->clients++;
		goto EXIT;
	} else if (d->clients > 0) {
		mux_print(MSG_ERROR,
			"DLCI %d state invalid: %d, clients: %d!\n",
			dlci, d->state, d->clients);
		retval = -EREJECTED;
		goto EXIT;
	}

	if ((d->state != DISCONNECTED) && (d->state != REJECTED)) {
		mux_print(MSG_ERROR,
			"DLCI %d state invalid: %d!\n", dlci, d->state);
		retval = -ENODEV;
		goto EXIT;
	}

	WARN_ON(d->clients != 0);
	ts0710->initiator = 1;
	d->initiator = 1;
	d->state = CONNECTING;
	try = 10;
	while (try--) {
		retval = ts27010_send_sabm(ts0710, dlci);
		if (retval) {
			mux_print(MSG_ERROR,
				"send sabm to open control 0 failed: %d\n",
				retval);
			msleep_interruptible(20);
			continue;
		}
		mux_print(MSG_DEBUG, "wait for DLCI %d opened\n", dlci);
		retval = ts27010_wait_for_open(ts0710, dlci);
		if (retval == -ETIMEDOUT)
			continue;
		else if (!retval) {
			mux_print(MSG_INFO, "DLCI %d connected!\n", dlci);
			d->clients++;
			break;
		} else
			break;
	}

	if (try < 0 && d->state != CONNECTED && d->state != FLOW_STOPPED) {
		mux_print(MSG_ERROR, "open DLCI 0 failed: %d\n",
			d->state);
		retval = -ENODEV;
	}

	if (d->state == CONNECTING)
		d->state = DISCONNECTED;

	wake_up_interruptible(&d->open_wait);

EXIT:
	mutex_unlock(&d->lock);
	FUNC_EXIT();

	return retval;
}

int ts27010_mux_usb_line_open(int line)
{
	int dlci;
	int retval;
	FUNC_ENTER();

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return -EINVAL;
	}

	retval = ts27010_open_ctrl_channel(0);
	if (retval != 0) {
		mux_print(MSG_ERROR, "Can't open control DLCI 0!\n");
		return retval;
	}

	retval = ts27010_open_channel(dlci);
	FUNC_EXIT();
	return retval;
}

int ts27010_mux_usb_line_write(int line, const unsigned char *buf, int count)
{
	struct ts0710_con *ts0710 = &ts0710_connection;
	int err = -EINVAL;
	int dlci;
	int correct_ttyidx;
	int mtu;
	int sent = 0;

	FUNC_ENTER();

	if (count <= 0)
		return 0;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		return -EIO;
	}

	correct_ttyidx = dlci2tty[dlci] - TS27010MUX_NAME_BASE;
	if (line != correct_ttyidx) {
		mux_print(MSG_ERROR,
			"Discard data sending to channel 0x%x, "
			"correct_ttyidx=%d, line =%d\n",
			dlci, correct_ttyidx, line);
		return -EINVAL;
	}

	mtu = ts0710->dlci[dlci].mtu;
	if (mtu > TS0710MUX_SEND_BUF_SIZE) {
		mux_print(MSG_ERROR, "buffer overflow %d/%d\n",
			mtu, TS0710MUX_SEND_BUF_SIZE);
		BUG_ON(0);
	}

	while (sent < count) {
		if (ts0710->dlci[dlci].state == HOLD_CONNECTED
			|| ts0710->dlci[dlci].state == HOLD_FLOW_STOPPED) {
			mux_print(MSG_INFO,
				"/dev/usb%d is being held\n",
				line + TS27010MUX_NAME_BASE);
			return -EDISCONNECTED;
		}
		if (ts0710->dlci[0].state == FLOW_STOPPED) {
			mux_print(MSG_INFO, "Flow stopped on all channels, "
				"including /dev/usb%d\n",
				line + TS27010MUX_NAME_BASE);
			wait_event_interruptible(
				ts0710->dlci[0].mux_write_wait,
				ts0710->dlci[0].state != FLOW_STOPPED);
			if (signal_pending(current)) {
				mux_print(MSG_WARNING, "/dev/usb%d(DLCI:%d) "
					"Wait for writing got signal!\n",
					line + TS27010MUX_NAME_BASE, 0);
				return -EAGAIN;
			} else if (ts0710->dlci[0].state != CONNECTED) {
				mux_print(MSG_WARNING,
					"write on DLCI %d "
					"while not connected\n", 0);
				return -EDISCONNECTED;
			}
		}
		if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
			mux_print(MSG_INFO,
				"Flow stopped on /dev/usb%d\n",
				line + TS27010MUX_NAME_BASE);
			wait_event_interruptible(
				ts0710->dlci[dlci].mux_write_wait,
				ts0710->dlci[dlci].state != FLOW_STOPPED);
			if (signal_pending(current)) {
				mux_print(MSG_WARNING, "/dev/usb%d(DLCI:%d) "
					"Wait for writing got signal!\n",
					line + TS27010MUX_NAME_BASE, dlci);
				return -EAGAIN;
			} else if (ts0710->dlci[dlci].state != CONNECTED) {
				mux_print(MSG_WARNING,
					"write on DLCI %d "
					"while not connected\n", dlci);
				return -EDISCONNECTED;
			}
		}
		if (ts0710->dlci[dlci].state == CONNECTED) {
			int n = ((count - sent) > mtu) ? mtu : (count - sent);
			mux_print(MSG_DEBUG, "preparing to write %d bytes "
				"to /dev/usb%d\n",
				n, line + TS27010MUX_NAME_BASE);
			mutex_lock(&ts0710->chan[line].write_lock);

			err = ts27010_send_uih(
				ts0710, dlci, ts0710->chan[line].buf,
				buf + sent, n);
			if (err < 0)
				goto ERR;

			sent += n;

			mutex_unlock(&ts0710->chan[line].write_lock);
		} else {
			mux_print(MSG_WARNING,
				"write on DLCI %d while not connected\n",
				dlci);
			return -EDISCONNECTED;
		}
	}
	do_tty_write_wakeup(ts0710, tty2dlci[line],
		ts27010_usb_tty_table[line]);
	mux_print(MSG_DEBUG,
		"write %d bytes to DLCI %d successfully\n", count, dlci);

	FUNC_EXIT();
	return count;

ERR:
	mutex_unlock(&ts0710->chan[line].write_lock);

	return err;
}

#define TS0710MUX_MAX_CHARS_IN_BUF 65535

int ts27010_mux_usb_line_chars_in_buffer(int line)
{
#if 1
	struct ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	u8 dlci;

	retval = TS0710MUX_MAX_CHARS_IN_BUF;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		goto out;
	}
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING, "Flow stopped on all channels,"
			"returning MAX chars in buffer\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING,
			"Flow stopped, returning MAX chars in buffer\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		mux_print(MSG_ERROR, "DLCI %d not connected\n", dlci);
		goto out;
	}

	retval = 0;

out:
	return retval;
#else
	struct ts0710_con *ts0710 = &ts0710_connection;
	int ret;
	FUNC_ENTER();

	if (mutex_is_locked(&ts0710->chan[line].write_lock))
		ret = TS0710MUX_SERIAL_BUF_SIZE;
	else
		ret = 0;
	FUNC_EXIT();
	return ret;
#endif
}

int ts27010_mux_usb_line_write_room(int line)
{
#if 1
	struct ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	u8 dlci;
	FUNC_ENTER();

	retval = 0;

	dlci = tty2dlci[line];
	if (!ts0710_valid_dlci(dlci)) {
		mux_print(MSG_ERROR, "invalid DLCI %d\n", dlci);
		goto out;
	}
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING,
			"Flow stopped on all channels, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		mux_print(MSG_WARNING, "Flow stopped, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		mux_print(MSG_ERROR, "DLCI %d not connected\n", dlci);
		goto out;
	}

	retval = ts0710->dlci[dlci].mtu;

out:
	FUNC_EXIT();
	return retval;
#else
	struct ts0710_con *ts0710 = &ts0710_connection;
	int ret;
	FUNC_ENTER();

	if (mutex_is_locked(&ts0710->chan[line].write_lock))
		ret = 0;
	else
		ret = TS0710MUX_SERIAL_BUF_SIZE;
	FUNC_EXIT();
	return ret;
#endif
}
*/

#define UT_RBUF_SIZE 40
static int set_mux_correct_data(u8 *buf, u8 *data, int len)
{
	int data_idx;

	--len; /* since sizeof("ERROR") is 6(including '\0' */

	buf[0] = TS0710_BASIC_FLAG;
	buf[1] = 7 << 2 | 0x3;/* addr */
	buf[2] = CLR_PF(UIH);/* control */
#ifdef TS27010_UART_RETRAN
	buf[3] = 0x80;/* sn */
	if (len < 128) {
		buf[4] = len << 1 | 0x1;/* len */
		data_idx = 5;
	} else {
		buf[4] = (u8)(len << 1 | 0xFE);/* len */
		buf[5] = len >> 7;/* len2 */
		data_idx = 6;
	}
#else
	if (len < 128) {
		buf[3] = len << 1 | 0x1;/* len */
		data_idx = 4;
	} else {
		buf[3] = (u8)(len << 1 & 0xFE);/* len */
		buf[4] = len >> 7;/* len2 */
		data_idx = 5;
	}
#endif
	memcpy(buf + data_idx, data, len);

#ifdef TS27010_UART_RETRAN
	buf[data_idx + len] = ts0710_usb_crc_data(buf + ADDRESS_OFFSET,
		TS0710_FRAME_SIZE(len) - FCS_SIZE - FLAG_SIZE);
#else
	buf[data_idx + len] = 0xFC;
#endif
	buf[data_idx + len + 1] = TS0710_BASIC_FLAG;
	return data_idx + len + 1 + 1;
}

static void dump_buf(u8 *buf, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		printf("(%d) %02x ", i + 1, buf[i]);
		if ((i + 1) % 16 == 0)
			printf("\n");
	}
	printf("\n\n");
}

static void test_ts27010_mux_recv_correct(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 24, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 36) == 36);
	UT_ASSERT(rbuf->head == 36);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 36);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 36);
	/* dump_buf(buf, 36); */
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 29, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* if recv successfully, will empty rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_mux_recv_no_start(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 24, "12345", sizeof("12345"))
		== 12);
	buf[24] = 0x77;/* remove the third start BASIC_FLAG */
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 36) == 36);
	UT_ASSERT(rbuf->head == 36);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 36);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 36);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 29, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* since missing the third start BASIC_FLAG, 1 bytes left in rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 1);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 1);
	UT_ASSERT(set_mux_correct_data(buf, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 12) == 12);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 13);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 13 - 1);
	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_mux_recv_invalid_addr(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 24, "12345", sizeof("12345"))
		== 12);
	buf[25] = 0xF3;
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 36) == 36);
	UT_ASSERT(rbuf->head == 36);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 36);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 36);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 29, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* since invalid the third address, 1 bytes left in rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 1);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 1);
	UT_ASSERT(set_mux_correct_data(buf, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 12) == 12);
	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}
	
static void test_ts27010_mux_recv_invalid_control(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 24, "12345", sizeof("12345"))
		== 12);
	buf[26] = SABM + 1;
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 36) == 36);
	UT_ASSERT(rbuf->head == 36);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 36);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 36);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 29, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* since invalid the third control, 1 bytes left in rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 1);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 1);
	UT_ASSERT(set_mux_correct_data(buf, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 12) == 12);
	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}

#ifdef TS27010_UART_RETRAN
static void test_ts27010_mux_recv_invalid_sn(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 24, "12345", sizeof("12345"))
		== 12);
	buf[27] |= 0x40;
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 36) == 36);
	UT_ASSERT(rbuf->head == 36);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 36);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 36);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 29, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* since invalid the third sn, 1 bytes left in rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 1);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 1);
	UT_ASSERT(set_mux_correct_data(buf, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 12) == 12);
	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}
#endif

static void test_ts27010_mux_recv_len2(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	buf[24] = TS0710_BASIC_FLAG;
	buf[25] = 7 << 2 | 0x3;
	buf[26] = CLR_PF(UIH);
#ifdef TS27010_UART_RETRAN
	buf[27] = 0x12;
	buf[28] = 5 << 1 & 0xFE;
	buf[29] = 0;
	memcpy(buf + 30, "12345", 5);
	buf[35] = ts0710_usb_crc_data(buf + 24 + ADDRESS_OFFSET,
		TS0710_FRAME_SIZE(5) + 1 - FCS_SIZE - FLAG_SIZE);
	buf[36] = TS0710_BASIC_FLAG;
#else
	buf[27] = 5 << 1 & 0xFE;
	buf[28] = 0;
	memcpy(buf + 29, "12345", 5);
	buf[34] = 0xFC;
	buf[35] = TS0710_BASIC_FLAG;
#endif
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 37) == 37);
	UT_ASSERT(rbuf->head == 37);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 37);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 37);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 30, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_mux_recv_invalid_len(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	buf[24] = TS0710_BASIC_FLAG;
	buf[25] = 7 << 2 | 0x3;
	buf[26] = CLR_PF(UIH);
#ifdef TS27010_UART_RETRAN
	buf[27] = 0x12;
	buf[28] = 5 << 1 & 0xFE;
	buf[29] = 0x7F;
	memcpy(buf + 30, "12345", 5);
	buf[35] = ts0710_usb_crc_data(buf + 24 + ADDRESS_OFFSET,
		TS0710_FRAME_SIZE(5) + 1 - FCS_SIZE - FLAG_SIZE);
	buf[36] = TS0710_BASIC_FLAG;
#else
	buf[27] = 5 << 1 & 0xFE;
	buf[28] = 0x7F;
	memcpy(buf + 29, "12345", 5);
	buf[34] = 0xFC;
	buf[35] = TS0710_BASIC_FLAG;
#endif
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 37) == 37);
	UT_ASSERT(rbuf->head == 37);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 37);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 37);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 30, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* since unintegrated frame, 1 bytes left in rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 1);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 1);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf + 12, 12) == 12);
	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_mux_recv_unintegrated_frame(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 24, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 31) == 31);
	UT_ASSERT(rbuf->head == 31);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 31);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 31);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 29, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) != 0);
	UT_ASSERT(memcmp(buf, "12", 2) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* since unintegrated frame, 7 bytes left in rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 7);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 7 - 1);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf + 31, 5) == 5);
	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_mux_recv_no_end(void)
{
	u8 buf[UT_RBUF_SIZE];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(
		UT_RBUF_SIZE);
	UT_ASSERT(rbuf != NULL);
	
	/* test parsing correct frame */
	UT_ASSERT(set_mux_correct_data(buf, "ERROR", sizeof("ERROR"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 12, "ABCDE", sizeof("ABCDE"))
		== 12);
	UT_ASSERT(set_mux_correct_data(buf + 24, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 36) == 36);
	buf[35] = 0;
	UT_ASSERT(rbuf->head == 36);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 36);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1 - 36);
	/* dump_buf(buf, 36); */

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ERROR", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 17, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "ABCDE", 5) == 0);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 29, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	ts27010_mux_usb_recv(rbuf);
	/* since missing the third end BASIC_FLAG, none bytes left in rbuf */
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 0 - 1);
	UT_ASSERT(set_mux_correct_data(buf, "12345", sizeof("12345"))
		== 12);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, buf, 12) == 12);
	ts27010_mux_usb_recv(rbuf);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == UT_RBUF_SIZE - 1);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_mux_recv(void)
{
	test_ts27010_mux_recv_correct();
	test_ts27010_mux_recv_no_start();
	test_ts27010_mux_recv_invalid_addr();
	test_ts27010_mux_recv_invalid_control();
#ifdef TS27010_UART_RETRAN
	test_ts27010_mux_recv_invalid_sn();
#endif
	test_ts27010_mux_recv_len2();
	test_ts27010_mux_recv_invalid_len();
	test_ts27010_mux_recv_unintegrated_frame();
	test_ts27010_mux_recv_no_end();
}

void test_ts27010_usb_mux(void)
{
	test_ts27010_mux_recv();
}

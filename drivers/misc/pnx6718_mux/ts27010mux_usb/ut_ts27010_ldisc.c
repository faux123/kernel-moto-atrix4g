/* include this C file to test some static functions and variables */
#include "ts27010_ldisc.c"

#include "ut_ts27010.h"

static void test_ts27010_ldisc_open_close(void)
{
	struct tty_struct *tty;
	tty = kmalloc(sizeof(*tty), GFP_KERNEL);
	UT_ASSERT(tty);
	
	tty->driver = kmalloc(sizeof(struct tty_driver), GFP_KERNEL);
	UT_ASSERT(tty->driver);
	tty->driver->name = "MUX_USB_UT";
	tty->driver->driver_name = "mux_usb_ut";
	tty->disc_data = NULL;
	ts27010mux_usb_tty = NULL;

	UT_ASSERT(atomic_read(&s_ld->ref_count) == 0);
	ts27010_ldisc_close();
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 0);
	UT_ASSERT(tty->disc_data == NULL);
	UT_ASSERT(ts27010mux_usb_tty == NULL);

	UT_ASSERT(ts27010_ldisc_open(tty) == 0);
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 1);
	UT_ASSERT(tty->disc_data == s_ld);
	UT_ASSERT(ts27010mux_usb_tty == tty);
	ts27010_ldisc_close();
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 0);
	UT_ASSERT(tty->disc_data == NULL);
	UT_ASSERT(ts27010mux_usb_tty == NULL);

	UT_ASSERT(ts27010_ldisc_open(tty) == 0);
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 1);
	UT_ASSERT(tty->disc_data == s_ld);
	UT_ASSERT(ts27010mux_usb_tty == tty);
	UT_ASSERT(ts27010_ldisc_open(tty) == 0);
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 2);
	UT_ASSERT(tty->disc_data == s_ld);
	UT_ASSERT(ts27010mux_usb_tty == tty);
	ts27010_ldisc_close();
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 1);
	UT_ASSERT(tty->disc_data == s_ld);
	UT_ASSERT(ts27010mux_usb_tty == tty);
	ts27010_ldisc_close();
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 0);
	UT_ASSERT(tty->disc_data == NULL);
	UT_ASSERT(ts27010mux_usb_tty == NULL);

	kfree(tty->driver);
	kfree(tty);
}

static void test_ts27010_ldisc_usb_receive(void)
{
	int room = ts27010_ringbuf_room(s_ld->rbuf);
	ts27010_ldisc_usb_receive(NULL, "1", NULL, 0);
	UT_ASSERT(ts27010_ringbuf_level(s_ld->rbuf) == 0);
	UT_ASSERT(s_bFC_USB == 0);
	ts27010_ldisc_usb_receive(NULL, "1", NULL, 100);
	UT_ASSERT(ts27010_ringbuf_level(s_ld->rbuf) == 100);
	UT_ASSERT(s_bFC_USB == 0);
	ts27010_ldisc_usb_receive(NULL, "2", NULL, room - 200);
	UT_ASSERT(ts27010_ringbuf_level(s_ld->rbuf) == (room - 100));
	UT_ASSERT(s_bFC_USB == 1);
	ts27010_ldisc_usb_receive(NULL, "3", NULL, 101);
	UT_ASSERT(ts27010_ringbuf_level(s_ld->rbuf) == room);
	UT_ASSERT(s_bFC_USB == 1);
}

static void test_ts27010_ldisc_usb_init(void)
{
	UT_ASSERT(ts27010_ldisc_usb_init() == 0);
	UT_ASSERT(s_mux_recv_flags == 0);
	UT_ASSERT(s_bFC_USB == 0);
	UT_ASSERT(s_ld);
	UT_ASSERT(s_ld->rbuf);
	UT_ASSERT(atomic_read(&s_ld->ref_count) == 0);
}

static void test_ts27010_usb_ldisc(void)
{
	test_ts27010_ldisc_usb_init();
	test_ts27010_ldisc_usb_receive();
	test_ts27010_ldisc_open_close();
	ts27010_ldisc_usb_remove();
}

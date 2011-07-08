#ifdef MUX_USB_UT

#include "ut_ts27010_ringbuf.c"
#include "ut_ts27010_ldisc.c"
#include "ut_ts27010_misc.c"
#include "ut_ts27010_td.c"
#include "ut_ts27010_tty.c"
#include "ut_ts27010_mux.c"
/*
#include "ut_ts27010_logger.c"
*/

void test_ts27010_usb(void)
{
	test_ts27010_usb_ringbuf();
	test_ts27010_usb_mux();
	test_ts27010_usb_ldisc();
	test_ts27010_usb_tty();
	test_ts27010_usb_td();
	test_ts27010_usb_misc();
	test_ts27010_usb_logger();
}
#endif


#ifndef __UT_TS27010_H__
#define __UT_TS27010_H__

#define UT_ASSERT(cond) do { \
	if (!(cond)) { \
		printk(KERN_ERR "MUX_UT failed %s at %s %s:%d\n", \
			#cond, __func__, __FILE__, __LINE__); \
	} \
} while(0)

/*
void test_ts27010_usb_ringbuf(void);
void test_ts27010_usb_td(void);
void test_ts27010_usb_tty(void);
void test_ts27010_usb_mux(void);
void test_ts27010_usb_misc(void);
void test_ts27010_usb_logger(void);
void test_ts27010_usb_ldisc(void);
*/

#endif /* __UT_TS27010_H__ */

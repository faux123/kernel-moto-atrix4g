/*
 * simple ring buffer
 *
 * supports a concurrent reader and writer without locking
 */

#include "ts27010_ringbuf.h"
#include "ut_ts27010.h"

static void test_ts27010_ringbuf_alloc(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(20);
	UT_ASSERT(rbuf != NULL);

	UT_ASSERT(rbuf->len == 20);
	UT_ASSERT(rbuf->head == 0);
	UT_ASSERT(rbuf->tail == 0);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_ringbuf_free(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(20);
	UT_ASSERT(rbuf != NULL);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_ringbuf_level(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(10);
	UT_ASSERT(rbuf != NULL);

	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 9);
	
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "12345", 5) == 5);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 5);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 4);
	
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "6789", 4) == 4);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 9);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 0);

	UT_ASSERT(ts27010_ringbuf_write(rbuf, "6789", 4) == 0);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_ringbuf_room(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(10);
	UT_ASSERT(rbuf != NULL);

	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 9);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "1234", 4) == 4);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 5);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 4);
	
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "56789", 5) == 5);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 9);
	
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "56789", 5) == 0);

	ts27010_ringbuf_free(rbuf);
}

/* get ith datum, datum still in rbuf */
/*
static inline u8 ts27010_ringbuf_peek(struct ts27010_ringbuf *rbuf, int i)
{
	return rbuf->buf[(rbuf->tail + i) % rbuf->len];
}
*/
static void test_ts27010_ringbuf_peek(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(10);
	UT_ASSERT(rbuf != NULL);

	UT_ASSERT(ts27010_ringbuf_write(rbuf, "123456789", 9) == 9);
	UT_ASSERT(ts27010_ringbuf_peek(rbuf, 0) == '1');
	UT_ASSERT(ts27010_ringbuf_peek(rbuf, 4) == '5');
	UT_ASSERT(ts27010_ringbuf_peek(rbuf, 8) == '9');

	UT_ASSERT(ts27010_ringbuf_peek(rbuf, 9) == '1');
	UT_ASSERT(ts27010_ringbuf_peek(rbuf, 13) == '5');
	UT_ASSERT(ts27010_ringbuf_peek(rbuf, 17) == '9');

	ts27010_ringbuf_free(rbuf);
}

/* get len data from idx, data still in rbuf */
/*
static inline int ts27010_ringbuf_read(
	struct ts27010_ringbuf *rbuf, int idx, u8* buf, int len)
{
#if 1
	int buf_level = ts27010_ringbuf_level(rbuf);
	int i;
	FUNC_ENTER();

	if (!buf)
		return -ENOMEM;

	if (idx > buf_level || idx < 0) {
		mux_print(MSG_ERROR,
			"invalid argument idx: %d while level = %d\n",
			idx, buf_level);
		return -EINVAL;
	}

	if ((len + idx) > buf_level)
		mux_print(MSG_WARNING,
			"only can read part data, len: %d, level: %d\n",
			len + idx, buf_level);

	for (i = 0; i < len; i++)
		buf[i] = ts27010_ringbuf_peek(rbuf, i + idx);

	FUNC_EXIT();
	return len;
#else
	int buf_level = ts27010_ringbuf_level(rbuf);
	int count;
	FUNC_ENTER();

	if (!buf)
		return -ENOMEM;

	if (idx > buf_level || idx < 0) {
		mux_print(MSG_ERROR,
			"invalid argument idx: %d while level = %d\n",
			idx, buf_level);
		return -EINVAL;
	}

	if ((len + idx) > buf_level)
		mux_print(MSG_WARNING,
			"only can read part data, len: %d, level: %d\n",
			len + idx, buf_level);

	count = (len <= (buf_level - idx)) ? len : (buf_level - idx);
	if (count == 0) {
		mux_print(MSG_ERROR, "rbuf empty: head %d, tail %d, len %d\n",
			rbuf->head, rbuf->tail, rbuf->len);
	} else if (rbuf->head > rbuf->tail) {
		memcpy(buf, rbuf->buf + rbuf->tail + idx, count);
	} else {
		if (rbuf->tail + idx + count >= rbuf->len) {
			int rest = rbuf->len - rbuf->tail - idx;
			memcpy(buf, rbuf->buf + rbuf->tail + idx, rest);
			memcpy(buf + rest, rbuf->buf, count - rest);
		} else {
			memcpy(buf, rbuf->buf + rbuf->tail + idx, count);
		}
	}
	return count;
#endif
}
*/
static void test_ts27010_ringbuf_read(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(10);
	u8 buf[10];

	UT_ASSERT(rbuf != NULL);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "123456789", 9) == 9);

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 0, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 1, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "23456", 5) == 0);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 4, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "56789", 5) == 0);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 5, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "67891", 5) == 0);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 6, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "78912", 5) == 0);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 8, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "91234", 5) == 0);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 9, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 10, buf, 5) == -EINVAL);
	UT_ASSERT(ts27010_ringbuf_read(rbuf, -1, buf, 5) == -EINVAL);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_ringbuf_consume(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(10);
	u8 buf[10];
	UT_ASSERT(rbuf != NULL);

	UT_ASSERT(ts27010_ringbuf_write(rbuf, "123456789", 9) == 9);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 9);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 0);
	UT_ASSERT(rbuf->head == 9);
	UT_ASSERT(rbuf->tail == 0);

	UT_ASSERT(ts27010_ringbuf_consume(rbuf, 5) == 5);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 4);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 5);
	UT_ASSERT(rbuf->head == 9);
	UT_ASSERT(rbuf->tail == 5);

	UT_ASSERT(ts27010_ringbuf_consume(rbuf, 5) == 4);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 0);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 9);
	UT_ASSERT(rbuf->head == 9);
	UT_ASSERT(rbuf->tail == 9);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_ringbuf_push(void)
{
	int i;
	u8 buf[10];
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(10);
	UT_ASSERT(rbuf != NULL);

	for (i = 0; i < 9; i++) {
		UT_ASSERT(ts27010_ringbuf_push(rbuf, '1' + i) == 1);
		UT_ASSERT(ts27010_ringbuf_level(rbuf) == (1 + i));
		UT_ASSERT(ts27010_ringbuf_room(rbuf) == (8 - i));
	}
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 0, buf, 9) == 9);
	UT_ASSERT(memcmp(buf, "123456789", 9) == 0);
	
	for (i = 0; i < 9; i++)
		UT_ASSERT(ts27010_ringbuf_push(rbuf, 'A' + i) == 0);
	for (i = 0; i < 9; i++)
		UT_ASSERT(ts27010_ringbuf_peek(rbuf, i) == ('1' + i));

/*
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 0, buf, 9) == 9);
	UT_ASSERT(memcmp(buf, "123456789", 9) == 0);
*/
	
	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_ringbuf_write(void)
{
	struct ts27010_ringbuf * rbuf = ts27010_ringbuf_alloc(10);
	u8 buf[10];
	UT_ASSERT(rbuf != NULL);

	UT_ASSERT(ts27010_ringbuf_write(rbuf, "12345", 5) == 5);
	UT_ASSERT(rbuf->head == 5);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "67890", 5) == 4);
	UT_ASSERT(rbuf->head == 9);
	UT_ASSERT(rbuf->tail == 0);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 9);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 0);

	UT_ASSERT(ts27010_ringbuf_write(rbuf, "123456789", 9) == 0);
	UT_ASSERT(rbuf->head == 9);
	UT_ASSERT(rbuf->tail == 0);

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 0, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "12345", 5) == 0);

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 0, buf, 9) == 9);
	UT_ASSERT(memcmp(buf, "123456789", 9) == 0);

	UT_ASSERT(ts27010_ringbuf_consume(rbuf, 5) == 5);
	UT_ASSERT(rbuf->head == 9);
	UT_ASSERT(rbuf->tail == 5);
	UT_ASSERT(ts27010_ringbuf_write(rbuf, "6789", 4) == 4);
	UT_ASSERT(ts27010_ringbuf_level(rbuf) == 8);
	UT_ASSERT(ts27010_ringbuf_room(rbuf) == 1);
	UT_ASSERT(rbuf->head == 3);
	UT_ASSERT(rbuf->tail == 5);
	
	UT_ASSERT(ts27010_ringbuf_read(rbuf, 0, buf, 5) == 5);
	UT_ASSERT(memcmp(buf, "67896", 5) == 0);

	UT_ASSERT(ts27010_ringbuf_read(rbuf, 0, buf, 8) == 8);
	UT_ASSERT(memcmp(buf, "67896789", 8) == 0);

	ts27010_ringbuf_free(rbuf);
}

static void test_ts27010_usb_ringbuf(void)
{
	test_ts27010_ringbuf_alloc();
	test_ts27010_ringbuf_free();
	test_ts27010_ringbuf_level();
	test_ts27010_ringbuf_room();
	test_ts27010_ringbuf_peek();
	test_ts27010_ringbuf_read();
	test_ts27010_ringbuf_consume();
	test_ts27010_ringbuf_push();
	test_ts27010_ringbuf_write();
}

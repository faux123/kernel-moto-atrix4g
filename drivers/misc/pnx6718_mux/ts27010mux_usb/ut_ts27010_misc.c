#ifndef TS27010_UART_RETRAN
#define TS27010_UART_RETRAN
#endif

/* include this C file to test some static functions and variables */
#include "ts27010_misc.c"

#include "ut_ts27010.h"

#ifdef TS27010_UART_RETRAN
/***************************** sequence number ********************************/
static void test_ts27010_sequence_number(void)
{
	int i;
	struct sequence_number_t *sn = ts27010_sequence_number_alloc();
	UT_ASSERT(sn != NULL);
	UT_ASSERT(sn->m_sn == 0);
	ts27010_sequence_number_inc(sn);
	ts27010_sequence_number_inc(sn);
	UT_ASSERT(ts27010_sequence_number_get(sn) == 2);

	ts27010_sequence_number_lock(sn);
	UT_ASSERT(mutex_is_locked(&sn->m_lock))
	ts27010_sequence_number_lock(sn);
	UT_ASSERT(mutex_is_locked(&sn->m_lock))

	ts27010_sequence_number_unlock(sn);
	UT_ASSERT(mutex_is_locked(&sn->m_lock))
	ts27010_sequence_number_unlock(sn);
	UT_ASSERT(!mutex_is_locked(&sn->m_lock))
	ts27010_sequence_number_unlock(sn);
	UT_ASSERT(!mutex_is_locked(&sn->m_lock))

	for (i = 0; i < MAX_TRANS_SN; i++) {
		ts27010_sequence_number_inc(sn);
	}
	UT_ASSERT(ts27010_sequence_number_get(sn) == 2);
	ts27010_sequence_number_free(sn);
	UT_ASSERT(sn == NULL);
}

/***************************** timer param ********************************/
static void timer_para_func(struct work_struct *work)
{
}

static void test_ts27010_timer_para(void)
{
	int i;
	u8 buf[SLIDE_WINDOW_SIZE_AP];
	struct ts27010_timer_para_t *tp = ts27010_alloc_timer_para(timer_para_func);
	UT_ASSERT(tp);
	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		UT_ASSERT(tp->m_cur_index[i] == 0);
	}
	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		ts27010_store_timer_para(tp, i);
		ts27010_store_timer_para(tp, i);
		UT_ASSERT(tp->m_cur_index[i] == 2);
	}
	ts27010_get_timer_para(tp, buf);
	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		UT_ASSERT(tp->m_cur_index[i] == 0);
		UT_ASSERT(buf[i] == 2);
	}
	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		ts27010_store_timer_para(tp, i);
		ts27010_store_timer_para(tp, i);
		UT_ASSERT(tp->m_cur_index[i] == 2);
	}
	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		UT_ASSERT(tp->m_cur_index[i] == 2);
		ts27010_clear_timer_para(tp, i);
		UT_ASSERT(tp->m_cur_index[i] == 0);
	}

	ts27010_timer_para_free(tp);
	UT_ASSERT(tp == NULL);
}

/***************************** retran info ********************************/
/*
struct ts27010_retran_info_t {
	struct timer_list m_tl; // retran timer
	u8 m_data[TS0710MUX_SEND_BUF_SIZE]; // frame data
	u16 m_length; // frame length
	u8 m_counter;
	u8 m_sn;
};

inline void ts27010_mod_retran_timer(
	struct ts27010_retran_info_t *retran_info)
{
	mod_timer(&retran_info->m_tl, (jiffies + RETRAN_TIMEOUT));
}

inline void ts27010_stop_retran_timer(
	struct ts27010_retran_info_t *retran_info)
{
	del_timer_sync(&retran_info->m_tl);
}

inline u8 *ts27010_get_retran_data(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_data;
}

inline u16 ts27010_get_retran_len(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_length;
}

inline u8 ts27010_get_retran_count(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_counter;
}

inline u8 ts27010_inc_retran_count(struct ts27010_retran_info_t *retran_info)
{
	return ++retran_info->m_counter;
}

inline void ts27010_clean_retran_count(
	struct ts27010_retran_info_t *retran_info)
{
	del_timer_sync(&retran_info->m_tl);
	memset(retran_info->m_data, 0, sizeof(u8) * TS0710MUX_SEND_BUF_SIZE);
	retran_info->m_length = 0;
	retran_info->m_counter = 0;
	retran_info->m_sn = 0xFF;
}

inline u8 ts27010_get_retran_sn(struct ts27010_retran_info_t *retran_info)
{
	return retran_info->m_sn;
}
*/

/***************************** slide window ********************************/
/*
struct ts27010_slide_window_t {
	struct mutex m_lock; // slide window r/w lock
	// if slide window full, sleep on this wait queue
	wait_queue_head_t m_retran_wait;
	struct ts27010_retran_info_t m_retran_info[SLIDE_WINDOWS_SIZE_AP];
	u8 m_head, m_tail;
};

void ts27010_clear_retran_counter(struct ts27010_slide_window_t *slide_window,
	struct ts27010_timer_para_t *timer_para, u8 index, int include)
{
	u8 j;
	FUNC_ENTER();

	for (j = slide_window->m_tail; j != index; ) {
		ts27010_clean_retran_count(
			ts27010_slidewindow_peek(slide_window, j));
		// need lock it?
		timer_para->m_cur_index[j] = 0;
		j = (j + 1) % SLIDE_WINDOWS_SIZE_AP;
	}
	if (include) {
		ts27010_clean_retran_count(
			ts27010_slidewindow_peek(slide_window, index));
		// need lock it?
		timer_para->m_cur_index[index] = 0;
	}
	FUNC_EXIT();
}
*/
#endif /* TS27010_UART_RETRAN */

static void test_timer_func(unsigned long para)
{
}

static void test_ts27010_slidewindow_funcs(void)
{
	int i;
	u8 buf[TS0710_FRAME_SIZE(1)];
	struct ts27010_retran_info_t *retran_info;
	struct ts27010_slide_window_t *sw = ts27010_slidewindow_alloc(
		test_timer_func);

	g_ap_send_sn = ts27010_sequence_number_alloc();
	UT_ASSERT(g_ap_send_sn);

	UT_ASSERT(sw);
	UT_ASSERT(sw->m_head == 0);
	UT_ASSERT(sw->m_tail == 0);
	UT_ASSERT(ts27010_slidewindow_level(sw) == 0);
	UT_ASSERT(ts27010_slidewindow_room(sw)
		== (SLIDE_WINDOWS_SIZE_AP - 1));
	/* head == tail */
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 0) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 1) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 1) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP) == 0);

	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 0) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 1) == 0xFF);

	/* data size illegal */
	UT_ASSERT(ts27010_slidewindo_store(
		sw, buf, TS0710_FRAME_SIZE(0) - 1) == 0xFF);

	for (i = 0; i < 3; i++) {
		/* set data to i */
		buf[TS0710_FRAME_SIZE(1) - 1 - 1 - 1] = i;
		buf[1 + 1 + 1] = CLR_PF(UIH);/* control */
		buf[1 + 1 + 1 + 1] = 0x09;/* length */
		UT_ASSERT(ts27010_slidewindo_store(
			sw, buf, TS0710_FRAME_SIZE(1)) == i);

		UT_ASSERT(ts27010_slidewindow_level(sw) == (i + 1));
		UT_ASSERT(ts27010_slidewindow_room(sw)
			== (SLIDE_WINDOWS_SIZE_AP - 1 - i));
		UT_ASSERT(sw->m_head == (i + 1));
		UT_ASSERT(sw->m_tail == 0);
		UT_ASSERT(ts27010_slidewindow_head(sw) == (i + 1));
		UT_ASSERT(ts27010_slidewindow_tail(sw) == 0);
		UT_ASSERT(ts27010_slidewindow_peek(sw, i)
			== (sw->m_retran_info + i));
	}
	/* head > tail */
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 3) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 1) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 0) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 1) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 2) == 1);

	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 0) == 0);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 1) == 1);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 2) == 2);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 3) == 0xFE);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 4) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 5) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 6) == 0xFF);

	for (; i < (SLIDE_WINDOWS_SIZE_AP - 1); i++) {
		/* set data to i */
		buf[TS0710_FRAME_SIZE(1) - 1 - 1 - 1] = i;
		buf[1 + 1 + 1] = CLR_PF(UIH);/* control */
		buf[1 + 1 + 1 + 1] = 0x09;/* length */
		UT_ASSERT(ts27010_slidewindo_store(
			sw, buf, TS0710_FRAME_SIZE(1)) == i);

		UT_ASSERT(ts27010_slidewindow_level(sw) == (i + 1));
		UT_ASSERT(ts27010_slidewindow_room(sw)
			== (SLIDE_WINDOWS_SIZE_AP - 1 - i));
		UT_ASSERT(sw->m_head == (i + 1));
		UT_ASSERT(sw->m_tail == 0);
		UT_ASSERT(ts27010_slidewindow_head(sw) == (i + 1));
		UT_ASSERT(ts27010_slidewindow_tail(sw) == 0);
		UT_ASSERT(ts27010_slidewindow_peek(sw, i)
			== (sw->m_retran_info + i));
	}
	/* slidewindow full */
	UT_ASSERT(ts27010_slidewindo_store(
		sw, buf, TS0710_FRAME_SIZE(1)) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 0) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 1) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 2) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 3) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 4) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 2) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 1) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP) == 0);
	
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 0) == 0);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 1) == 1);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 2) == 2);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 3) == 3);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 4) == 4);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 5) == 0xFE);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 6) == 0xFF);

	UT_ASSERT(ts27010_slidewindow_consume(sw, 3) == 3);

	UT_ASSERT(ts27010_slidewindow_level(sw)
		== (SLIDE_WINDOWS_SIZE_AP - 1) - 3);
	UT_ASSERT(ts27010_slidewindow_room(sw) == 3);
	UT_ASSERT(sw->m_head == (SLIDE_WINDOWS_SIZE_AP - 1));
	UT_ASSERT(sw->m_tail == 3);
	UT_ASSERT(ts27010_slidewindow_head(sw)
		== (SLIDE_WINDOWS_SIZE_AP - 1));
	UT_ASSERT(ts27010_slidewindow_tail(sw) == 3);

	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 0) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 1) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 2) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 3) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 4) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 2) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 1) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP) == 0);

	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 0) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 1) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 2) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 3) == 3);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 4) == 4);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 5) == 0xFE);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 6) == 0xFF);

	for (i = 0; i < 2; i++) {
		/* set data to i */
		buf[TS0710_FRAME_SIZE(1) - 1 - 1 - 1]
			= i + (SLIDE_WINDOWS_SIZE_AP - 1);
		buf[1 + 1 + 1] = CLR_PF(UIH);/* control */
		buf[1 + 1 + 1 + 1] = 0x09;/* length */
		UT_ASSERT(ts27010_slidewindo_store(
			sw, buf, TS0710_FRAME_SIZE(1)) == i);
	}
	UT_ASSERT(ts27010_slidewindow_level(sw)
		== (SLIDE_WINDOWS_SIZE_AP - 2));
	UT_ASSERT(ts27010_slidewindow_room(sw) == 1);
	UT_ASSERT(sw->m_head == 1);
	UT_ASSERT(sw->m_tail == 3);
	UT_ASSERT(ts27010_slidewindow_head(sw) == 1);
	UT_ASSERT(ts27010_slidewindow_tail(sw) == 3);
	/* head < tail */
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 0) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 1) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 2) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 3) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 4) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 2) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP - 1) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(
		sw, SLIDE_WINDOWS_SIZE_AP) == 0);

	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 0) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 1) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 2) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 3) == 3);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 4) == 4);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 5) == 0);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 6) == 1);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 7) == 0xFE);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 8) == 0xFF);

	/* test ts27010_slidewindow_set_tail */
	UT_ASSERT(ts27010_slidewindow_set_tail(sw, 5) == 5);
	UT_ASSERT(ts27010_slidewindow_level(sw) == 2);
	UT_ASSERT(ts27010_slidewindow_room(sw) == 3);
	UT_ASSERT(sw->m_head == 1);
	UT_ASSERT(sw->m_tail == 5);
	UT_ASSERT(ts27010_slidewindow_head(sw) == 1);
	UT_ASSERT(ts27010_slidewindow_tail(sw) == 5);
	/* head < tail */
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 0) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 1) == 1);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 2) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 3) == 0);
	UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, 4) == 0);

	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 2) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 3) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 4) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 5) == 0);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 6) == 1);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 7) == 0xFE);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 8) == 0xFF);

	ts27010_slidewindow_clear(sw);
	UT_ASSERT(sw->m_head == 0);
	UT_ASSERT(sw->m_tail == 0);
	for (i = 0; i <= SLIDE_WINDOWS_SIZE_AP; i++)
		UT_ASSERT(ts27010_slidewindow_is_idx_in(sw, i) == 0);

	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 0) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 1) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 2) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 3) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 4) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 5) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 6) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 7) == 0xFF);
	UT_ASSERT(ts27010_slidewindow_is_sn_in(sw, 8) == 0xFF);

	UT_ASSERT(ts27010_slidewindow_level(sw) == 0);
	UT_ASSERT(ts27010_slidewindow_room(sw)
		== (SLIDE_WINDOWS_SIZE_AP - 1));

	ts27010_slidewindow_free(sw);
	UT_ASSERT(sw == NULL);
	ts27010_sequence_number_free(g_ap_send_sn);
	UT_ASSERT(g_ap_send_sn == NULL);
}

static void test_ts27010_slidewindow_alloc_free(void)
{
	int i;
	struct ts27010_retran_info_t *retran_info;
	struct ts27010_slide_window_t *sw = ts27010_slidewindow_alloc(
		test_timer_func);
	UT_ASSERT(sw);
	UT_ASSERT(sw->m_head == 0);
	UT_ASSERT(sw->m_tail == 0);
	for (i = 0; i < SLIDE_WINDOWS_SIZE_AP; i++) {
		retran_info = sw->m_retran_info + i;

		UT_ASSERT(retran_info->m_tl.function == test_timer_func);
		UT_ASSERT(retran_info->m_tl.data == (unsigned long)i);
		UT_ASSERT(retran_info->m_length == 0);
		UT_ASSERT(retran_info->m_counter == 0);
		UT_ASSERT(retran_info->m_sn == 0xFF);
	}

	ts27010_slidewindow_lock(sw);
	UT_ASSERT(mutex_is_locked(&sw->m_lock))
	ts27010_slidewindow_lock(sw);
	UT_ASSERT(mutex_is_locked(&sw->m_lock))

	ts27010_slidewindow_unlock(sw);
	UT_ASSERT(mutex_is_locked(&sw->m_lock))
	ts27010_slidewindow_unlock(sw);
	UT_ASSERT(!mutex_is_locked(&sw->m_lock))
	ts27010_slidewindow_unlock(sw);
	UT_ASSERT(!mutex_is_locked(&sw->m_lock))
	
	ts27010_slidewindow_free(sw);
	UT_ASSERT(sw == NULL);
}

void test_ts27010_usb_misc(void)
{
	test_ts27010_sequence_number();
	test_ts27010_timer_para();
	test_ts27010_slidewindow_alloc_free();
	test_ts27010_slidewindow_funcs();
}


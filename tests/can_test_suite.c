/**
 * @file can_test_suite.c
 * @brief Comprehensive test suite for the LPC1768 CAN driver
 *
 * Week 4 — Tests every API function using self-test (loopback) mode.
 * No external hardware required.
 *
 * Test categories:
 *   1. Init / Deinit
 *   2. Transmit / Receive (loopback)
 *   3. Acceptance Filter (individual + group)
 *   4. Status query
 *   5. Mode switching
 *   6. Callbacks
 *   7. Ring buffer stress
 *   8. Diagnostics
 *   9. Gateway routing
 *  10. Sleep / Wake
 *  11. Error handling / edge cases
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include "can_buffer.h"
#include "can_gateway.h"
#include "can_test.h"
#include <string.h>

/* ── Forward declarations ──────────────────────────────────── */
void run_all_tests(test_suite_t *suite);

/* ── Shared state for callbacks ────────────────────────────── */
static volatile bool     g_rx_flag;
static volatile uint32_t g_rx_id;
static volatile uint8_t  g_rx_data[8];
static volatile uint8_t  g_rx_dlc;
static volatile bool     g_err_flag;
static volatile can_error_t g_last_err;

static void cb_rx(can_channel_t ch, can_message_t *msg)
{
    (void)ch;
    g_rx_flag = true;
    g_rx_id   = msg->id;
    g_rx_dlc  = msg->dlc;
    memcpy((void *)g_rx_data, msg->data, 8);
}

static void cb_err(can_channel_t ch, can_error_t err)
{
    (void)ch;
    g_err_flag = true;
    g_last_err = err;
}

/* ── Helper: init CAN1 in self-test ────────────────────────── */
static int helper_init_ch1(void)
{
    can_config_t cfg;
    cfg.channel         = CAN_CHANNEL_1;
    cfg.baudrate        = CAN_BAUD_500K;
    cfg.mode            = CAN_MODE_SELFTEST;
    cfg.rx_callback     = cb_rx;
    cfg.error_callback  = cb_err;
    cfg.enable_timestamp = true;
    return can_init(&cfg);
}

/* ── Helper: send + receive one standard frame ─────────────── */
static int helper_loopback(can_channel_t ch, uint32_t id,
                           const uint8_t *data, uint8_t dlc,
                           can_message_t *out)
{
    can_message_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.id         = id;
    tx.frame_type = CAN_FRAME_STANDARD;
    tx.dlc        = dlc;
    if (data) memcpy(tx.data, data, dlc);

    int rc = can_transmit(ch, &tx);
    if (rc != CAN_OK) return rc;

    /* Small spin for loopback to complete */
    volatile uint32_t spin = 50000;
    while (spin--) {}

    return can_receive(ch, out, 50);
}

/* ── Helper: blocking delay ────────────────────────────────── */
static void tdelay(uint32_t ms)
{
    volatile uint32_t i;
    for (; ms > 0; ms--)
        for (i = 0; i < 10000; i++);
}

/* ================================================================
 *  Test Groups
 * ================================================================ */

/* ─── 1. Init / Deinit ─────────────────────────────────────── */
static void test_init_deinit(test_suite_t *s)
{
    /* 1a. Valid init */
    TEST_BEGIN(s, "init: CAN1 self-test 500K");
    TEST_ASSERT_EQ(s, helper_init_ch1(), CAN_OK);

    /* 1b. Double init (should succeed — re-init) */
    TEST_BEGIN(s, "init: double init OK");
    TEST_ASSERT_EQ(s, helper_init_ch1(), CAN_OK);

    /* 1c. Deinit */
    TEST_BEGIN(s, "deinit: CAN1");
    TEST_ASSERT_EQ(s, can_deinit(CAN_CHANNEL_1), CAN_OK);

    /* 1d. Deinit when not init */
    TEST_BEGIN(s, "deinit: not-init returns ERR");
    TEST_ASSERT_EQ(s, can_deinit(CAN_CHANNEL_1), CAN_ERR_NOT_INIT);

    /* 1e. NULL config */
    TEST_BEGIN(s, "init: NULL config returns ERR");
    TEST_ASSERT_EQ(s, can_init(0), CAN_ERR_INVALID_PARAM);

    /* Re-init for remaining tests */
    helper_init_ch1();
}

/* ─── 2. Transmit / Receive ────────────────────────────────── */
static void test_tx_rx(test_suite_t *s)
{
    can_message_t rx;
    uint8_t pat1[] = {0xDE, 0xAD, 0xBE, 0xEF};

    /* 2a. Basic loopback */
    TEST_BEGIN(s, "TX/RX: basic 4-byte loopback");
    int rc = helper_loopback(CAN_CHANNEL_1, 0x123, pat1, 4, &rx);
    TEST_ASSERT(s, rc == CAN_OK && rx.id == 0x123 && rx.dlc == 4
                && rx.data[0] == 0xDE && rx.data[3] == 0xEF);

    /* 2b. Zero-length frame */
    TEST_BEGIN(s, "TX/RX: DLC=0 frame");
    rc = helper_loopback(CAN_CHANNEL_1, 0x001, 0, 0, &rx);
    TEST_ASSERT(s, rc == CAN_OK && rx.id == 0x001 && rx.dlc == 0);

    /* 2c. Full 8-byte frame */
    uint8_t pat8[] = {1,2,3,4,5,6,7,8};
    TEST_BEGIN(s, "TX/RX: full 8-byte frame");
    rc = helper_loopback(CAN_CHANNEL_1, 0x7FF, pat8, 8, &rx);
    TEST_ASSERT(s, rc == CAN_OK && rx.id == 0x7FF && rx.dlc == 8
                && rx.data[7] == 8);

    /* 2d. Transmit when not init */
    can_deinit(CAN_CHANNEL_2);
    can_message_t dummy;
    memset(&dummy, 0, sizeof(dummy));
    TEST_BEGIN(s, "TX: not-init returns ERR");
    TEST_ASSERT_EQ(s, can_transmit(CAN_CHANNEL_2, &dummy),
                   CAN_ERR_NOT_INIT);

    /* 2e. Receive timeout */
    TEST_BEGIN(s, "RX: timeout returns ERR");
    /* drain buffer first */
    while (can_receive(CAN_CHANNEL_1, &rx, 0) == CAN_OK) {}
    TEST_ASSERT_EQ(s, can_receive(CAN_CHANNEL_1, &rx, 10),
                   CAN_ERR_TIMEOUT);

    /* 2f. NULL message pointer */
    TEST_BEGIN(s, "TX: NULL msg returns ERR");
    TEST_ASSERT_EQ(s, can_transmit(CAN_CHANNEL_1, 0),
                   CAN_ERR_INVALID_PARAM);
}

/* ─── 3. Acceptance Filter ─────────────────────────────────── */
static void test_filters(test_suite_t *s)
{
    /* 3a. Individual filter */
    TEST_BEGIN(s, "filter: set individual SFF");
    TEST_ASSERT_EQ(s, can_set_filter(CAN_CHANNEL_1, 0x200, 0x7FF,
                                      CAN_FRAME_STANDARD), CAN_OK);

    /* 3b. Group filter */
    TEST_BEGIN(s, "filter: set group SFF 0x300-0x3FF");
    TEST_ASSERT_EQ(s, can_set_group_filter(CAN_CHANNEL_1, 0x300, 0x3FF,
                                            CAN_FRAME_STANDARD), CAN_OK);

    /* 3c. Clear filters */
    TEST_BEGIN(s, "filter: clear all -> bypass");
    TEST_ASSERT_EQ(s, can_clear_filters(CAN_CHANNEL_1), CAN_OK);

    /* 3d. Verify bypass works (any ID) */
    can_message_t rx;
    uint8_t d[] = {0xAA};
    TEST_BEGIN(s, "filter: bypass accepts any ID");
    int rc = helper_loopback(CAN_CHANNEL_1, 0x555, d, 1, &rx);
    TEST_ASSERT(s, rc == CAN_OK && rx.id == 0x555);
}

/* ─── 4. Status ────────────────────────────────────────────── */
static void test_status(test_suite_t *s)
{
    can_status_t st;

    /* 4a. Get status */
    TEST_BEGIN(s, "status: read OK");
    TEST_ASSERT_EQ(s, can_get_status(CAN_CHANNEL_1, &st), CAN_OK);

    /* 4b. No bus-off in self-test */
    TEST_BEGIN(s, "status: not bus-off");
    TEST_ASSERT(s, st.bus_off == false);

    /* 4c. Error counts near zero */
    TEST_BEGIN(s, "status: low error counts");
    TEST_ASSERT(s, st.tx_error_count < 10 && st.rx_error_count < 10);

    /* 4d. NULL pointer */
    TEST_BEGIN(s, "status: NULL returns ERR");
    TEST_ASSERT_EQ(s, can_get_status(CAN_CHANNEL_1, 0),
                   CAN_ERR_INVALID_PARAM);
}

/* ─── 5. Mode switching ───────────────────────────────────── */
static void test_modes(test_suite_t *s)
{
    /* 5a. Switch to reset */
    TEST_BEGIN(s, "mode: set RESET");
    TEST_ASSERT_EQ(s, can_set_mode(CAN_CHANNEL_1, CAN_MODE_RESET), CAN_OK);

    /* 5b. Back to self-test */
    TEST_BEGIN(s, "mode: set SELFTEST");
    TEST_ASSERT_EQ(s, can_set_mode(CAN_CHANNEL_1, CAN_MODE_SELFTEST), CAN_OK);

    /* 5c. Listen-only */
    TEST_BEGIN(s, "mode: set LISTEN");
    TEST_ASSERT_EQ(s, can_set_mode(CAN_CHANNEL_1, CAN_MODE_LISTEN), CAN_OK);

    /* Restore to self-test for remaining tests */
    can_set_mode(CAN_CHANNEL_1, CAN_MODE_SELFTEST);

    /* 5d. Verify loopback still works after mode cycling */
    can_message_t rx;
    uint8_t d[] = {0x42};
    TEST_BEGIN(s, "mode: loopback works after cycle");
    TEST_ASSERT_EQ(s, helper_loopback(CAN_CHANNEL_1, 0x050, d, 1, &rx),
                   CAN_OK);
}

/* ─── 6. Callbacks ─────────────────────────────────────────── */
static void test_callbacks(test_suite_t *s)
{
    /* 6a. RX callback fires */
    g_rx_flag = false;
    g_rx_id   = 0;
    can_register_rx_callback(CAN_CHANNEL_1, cb_rx);

    can_message_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.id = 0x0AA; tx.dlc = 2;
    tx.data[0] = 0x11; tx.data[1] = 0x22;
    tx.frame_type = CAN_FRAME_STANDARD;

    can_transmit(CAN_CHANNEL_1, &tx);
    tdelay(20);

    TEST_BEGIN(s, "callback: RX fires on loopback");
    TEST_ASSERT(s, g_rx_flag && g_rx_id == 0x0AA);

    /* 6b. Callback data correct */
    TEST_BEGIN(s, "callback: RX data matches");
    TEST_ASSERT(s, g_rx_data[0] == 0x11 && g_rx_data[1] == 0x22);

    /* 6c. Register NULL callback (disable) */
    TEST_BEGIN(s, "callback: NULL deregisters OK");
    TEST_ASSERT_EQ(s, can_register_rx_callback(CAN_CHANNEL_1, 0), CAN_OK);
}

/* ─── 7. Ring buffer stress ────────────────────────────────── */
static void test_buffer_stress(test_suite_t *s)
{
    can_message_t tx, rx;
    memset(&tx, 0, sizeof(tx));
    tx.frame_type = CAN_FRAME_STANDARD;
    tx.dlc = 1;

    /* 7a. Burst 16 messages */
    TEST_BEGIN(s, "buffer: burst 16 TX");
    int ok = 1;
    for (uint8_t i = 0; i < 16; i++) {
        tx.id = 0x100 + i;
        tx.data[0] = i;
        if (can_transmit(CAN_CHANNEL_1, &tx) != CAN_OK) { ok = 0; break; }
    }
    TEST_ASSERT(s, ok);

    tdelay(100);  /* let loopback complete */

    /* 7b. Receive all 16 back */
    TEST_BEGIN(s, "buffer: receive 16 back");
    int count = 0;
    while (can_receive(CAN_CHANNEL_1, &rx, 5) == CAN_OK) count++;
    TEST_ASSERT(s, count >= 16);
}

/* ─── 8. Diagnostics ───────────────────────────────────────── */
static void test_diagnostics(test_suite_t *s)
{
    can_diag_t diag;

    /* 8a. Read diagnostics */
    TEST_BEGIN(s, "diag: read OK");
    TEST_ASSERT_EQ(s, can_get_diag(CAN_CHANNEL_1, &diag), CAN_OK);

    /* 8b. TX count > 0 (we've sent many messages by now) */
    TEST_BEGIN(s, "diag: tx_count > 0");
    TEST_ASSERT(s, diag.tx_count > 0);

    /* 8c. RX count > 0 */
    TEST_BEGIN(s, "diag: rx_count > 0");
    TEST_ASSERT(s, diag.rx_count > 0);

    /* 8d. Uptime > 0 */
    TEST_BEGIN(s, "diag: uptime > 0");
    TEST_ASSERT(s, diag.uptime_ms > 0);

    /* 8e. Reset diagnostics */
    TEST_BEGIN(s, "diag: reset clears counts");
    can_reset_diag(CAN_CHANNEL_1);
    can_get_diag(CAN_CHANNEL_1, &diag);
    TEST_ASSERT(s, diag.tx_count == 0 && diag.rx_count == 0);

    /* 8f. NULL pointer */
    TEST_BEGIN(s, "diag: NULL returns ERR");
    TEST_ASSERT_EQ(s, can_get_diag(CAN_CHANNEL_1, 0),
                   CAN_ERR_INVALID_PARAM);
}

/* ─── 9. Gateway ───────────────────────────────────────────── */
static void test_gateway(test_suite_t *s)
{
    /* Need both channels for gateway. Use self-test on both. */
    can_config_t cfg2;
    cfg2.channel         = CAN_CHANNEL_2;
    cfg2.baudrate        = CAN_BAUD_500K;
    cfg2.mode            = CAN_MODE_SELFTEST;
    cfg2.rx_callback     = 0;
    cfg2.error_callback  = 0;
    cfg2.enable_timestamp = false;
    can_init(&cfg2);

    /* 9a. Gateway init */
    TEST_BEGIN(s, "gateway: init OK");
    TEST_ASSERT_EQ(s, can_gateway_init(), CAN_OK);

    /* 9b. Add route */
    can_route_t r;
    r.src = CAN_CHANNEL_1; r.dst = CAN_CHANNEL_2;
    r.id_min = 0x100; r.id_max = 0x1FF;
    r.frame_type = CAN_FRAME_STANDARD; r.active = true;
    TEST_BEGIN(s, "gateway: add route OK");
    int idx = can_gateway_add_route(&r);
    TEST_ASSERT(s, idx >= 0);

    /* 9c. Add invalid route (src == dst) */
    r.dst = CAN_CHANNEL_1;
    TEST_BEGIN(s, "gateway: src==dst returns ERR");
    TEST_ASSERT_EQ(s, can_gateway_add_route(&r), CAN_ERR_INVALID_PARAM);

    /* 9d. Send message in range, check forwarding */
    can_gateway_reset_fwd_count();
    can_message_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.id = 0x150; tx.frame_type = CAN_FRAME_STANDARD;
    tx.dlc = 2; tx.data[0] = 0xAB; tx.data[1] = 0xCD;
    can_transmit(CAN_CHANNEL_1, &tx);
    tdelay(50);
    can_gateway_process();
    tdelay(50);

    TEST_BEGIN(s, "gateway: fwd_count >= 1");
    TEST_ASSERT(s, can_gateway_get_fwd_count() >= 1);

    /* 9e. Remove route */
    TEST_BEGIN(s, "gateway: remove route OK");
    TEST_ASSERT_EQ(s, can_gateway_remove_route((uint8_t)idx), CAN_OK);

    /* 9f. Deinit gateway */
    can_gateway_deinit();
    can_deinit(CAN_CHANNEL_2);
}

/* ─── 10. Sleep / Wake ─────────────────────────────────────── */
static void test_sleep_wake(test_suite_t *s)
{
    /* 10a. Sleep */
    TEST_BEGIN(s, "sleep: CAN1 OK");
    TEST_ASSERT_EQ(s, can_sleep(CAN_CHANNEL_1), CAN_OK);

    /* 10b. Wake */
    TEST_BEGIN(s, "wake: CAN1 OK");
    TEST_ASSERT_EQ(s, can_wake(CAN_CHANNEL_1), CAN_OK);

    /* 10c. Loopback still works after sleep/wake */
    can_message_t rx;
    uint8_t d[] = {0x99};
    TEST_BEGIN(s, "sleep/wake: loopback works after");
    TEST_ASSERT_EQ(s, helper_loopback(CAN_CHANNEL_1, 0x077, d, 1, &rx),
                   CAN_OK);

    /* 10d. Sleep on not-init channel */
    TEST_BEGIN(s, "sleep: not-init returns ERR");
    TEST_ASSERT_EQ(s, can_sleep(CAN_CHANNEL_2), CAN_ERR_NOT_INIT);
}

/* ─── 11. Edge cases ───────────────────────────────────────── */
static void test_edge_cases(test_suite_t *s)
{
    /* 11a. Max standard ID */
    can_message_t rx;
    uint8_t d[] = {0xFF};
    TEST_BEGIN(s, "edge: max SFF ID 0x7FF");
    TEST_ASSERT_EQ(s, helper_loopback(CAN_CHANNEL_1, 0x7FF, d, 1, &rx),
                   CAN_OK);

    /* 11b. ID = 0 */
    TEST_BEGIN(s, "edge: ID = 0x000");
    TEST_ASSERT_EQ(s, helper_loopback(CAN_CHANNEL_1, 0x000, d, 1, &rx),
                   CAN_OK);

    /* 11c. Timestamp non-zero */
    TEST_BEGIN(s, "edge: timestamp > 0");
    int rc = helper_loopback(CAN_CHANNEL_1, 0x010, d, 1, &rx);
    TEST_ASSERT(s, rc == CAN_OK && rx.timestamp > 0);

    /* 11d. Repeated init-deinit cycle (leak check) */
    TEST_BEGIN(s, "edge: 10x init/deinit cycle");
    int ok = 1;
    for (int i = 0; i < 10; i++) {
        if (helper_init_ch1() != CAN_OK) { ok = 0; break; }
        if (can_deinit(CAN_CHANNEL_1) != CAN_OK) { ok = 0; break; }
    }
    TEST_ASSERT(s, ok);

    /* Re-init for safety */
    helper_init_ch1();
}

/* ================================================================
 *  Run everything
 * ================================================================ */

void run_all_tests(test_suite_t *suite)
{
    test_suite_init(suite);

    test_init_deinit(suite);
    test_tx_rx(suite);
    test_filters(suite);
    test_status(suite);
    test_modes(suite);
    test_callbacks(suite);
    test_buffer_stress(suite);
    test_diagnostics(suite);
    test_gateway(suite);
    test_sleep_wake(suite);
    test_edge_cases(suite);
}

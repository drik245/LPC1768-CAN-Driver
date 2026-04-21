/**
 * @file   can_loopback_test.ino
 * @brief  ESP32 CAN Self-Test (loopback)
 *
 * >>> CONNECT GPIO 5 TO GPIO 4 WITH A JUMPER WIRE <<<
 * That's it. No transceiver, no resistor. Just one wire.
 * 
 * TX pin (GPIO 5) feeds directly into RX pin (GPIO 4),
 * so the TWAI controller reads back its own transmission.
 */

#include "driver/twai.h"

#define CAN_TX_PIN   GPIO_NUM_5
#define CAN_RX_PIN   GPIO_NUM_4

static int tests_passed = 0;
static int tests_failed = 0;

/* ── Recover from bus-off ──────────────────────────────────── */
void bus_recover() {
    twai_status_info_t status;
    twai_get_status_info(&status);

    if (status.state == TWAI_STATE_BUS_OFF) {
        Serial.println("  [WARN] Bus-off, recovering...");
        twai_initiate_recovery();
        /* Wait until recovery completes */
        for (int i = 0; i < 50; i++) {
            delay(10);
            twai_get_status_info(&status);
            if (status.state == TWAI_STATE_STOPPED) {
                twai_start();
                break;
            }
        }
        delay(50);
    }

    /* Flush stale RX messages */
    twai_message_t dummy;
    while (twai_receive(&dummy, pdMS_TO_TICKS(5)) == ESP_OK) {}
}

/* ── Send + verify one message ─────────────────────────────── */
bool run_test(const char *name, uint32_t id, bool extended,
              const uint8_t *data, uint8_t len) {
    Serial.printf("  [TEST] %-35s ... ", name);

    bus_recover();

    twai_message_t tx_msg = {};
    tx_msg.identifier       = id;
    tx_msg.data_length_code = len;
    tx_msg.extd             = extended ? 1 : 0;
    tx_msg.self             = 1;     // Self-reception (internal + physical)
    memcpy(tx_msg.data, data, len);

    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(500));
    if (err != ESP_OK) {
        Serial.printf("FAIL (tx err=%d)\n", err);
        tests_failed++;
        return false;
    }

    twai_message_t rx_msg;
    err = twai_receive(&rx_msg, pdMS_TO_TICKS(500));
    if (err != ESP_OK) {
        Serial.printf("FAIL (rx err=%d)\n", err);
        tests_failed++;
        return false;
    }

    bool ok = (rx_msg.identifier == id) &&
              (rx_msg.data_length_code == len) &&
              (memcmp(rx_msg.data, data, len) == 0);

    if (ok) {
        Serial.println("PASS");
        tests_passed++;
    } else {
        Serial.printf("FAIL (id=0x%X dlc=%d)\n",
                      rx_msg.identifier, rx_msg.data_length_code);
        tests_failed++;
    }
    return ok;
}

/* ──────────────────────────────────────────────────────────── */
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  ESP32 CAN Loopback Self-Test");
    Serial.println("  Wire GPIO 5 --> GPIO 4 (one jumper)");
    Serial.println("===========================================\n");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[FAIL] TWAI install failed");
        while (1) delay(1000);
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[FAIL] TWAI start failed");
        while (1) delay(1000);
    }

    Serial.println("[OK] TWAI started\n");
    delay(100);   // Let controller stabilize

    /* ── Test Suite ────────────────────────────────────────── */
    Serial.println("--- Standard Frames ---");
    {
        uint8_t d[] = { 0xCA, 0xFE, 0xBA, 0xBE };
        run_test("ID=0x150 (CAFEBABE)", 0x150, false, d, 4);
    }
    {
        uint8_t d[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04 };
        run_test("ID=0x250 (8 bytes)", 0x250, false, d, 8);
    }
    {
        uint8_t d[] = { 0xFF, 0x00 };
        run_test("ID=0x400 (2 bytes)", 0x400, false, d, 2);
    }
    {
        uint8_t d[] = {};
        run_test("ID=0x100 (0 bytes)", 0x100, false, d, 0);
    }
    {
        uint8_t d[] = { 0x55, 0xAA, 0x55, 0xAA };
        run_test("Max ID=0x7FF", 0x7FF, false, d, 4);
    }

    Serial.println("\n--- Extended Frames ---");
    {
        uint8_t d[] = { 0x11, 0x22, 0x33, 0x44 };
        run_test("ExtID=0x1ABCDEF", 0x1ABCDEF, true, d, 4);
    }
    {
        uint8_t d[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };
        run_test("ExtID=0x12345678 (8B)", 0x12345678, true, d, 8);
    }

    Serial.println("\n--- Burst (8 msgs) ---");
    for (uint8_t i = 0; i < 8; i++) {
        char name[40];
        snprintf(name, sizeof(name), "Burst %d  ID=0x%03X", i, 0x100 + i);
        uint8_t d[] = { 0xBB, i, (uint8_t)(7 - i), 0x00 };
        run_test(name, 0x100 + i, false, d, 4);
    }

    /* ── Results ───────────────────────────────────────────── */
    Serial.println();
    Serial.println("===========================================");
    Serial.printf("  %d PASSED  |  %d FAILED\n", tests_passed, tests_failed);
    Serial.println("===========================================");
    if (tests_failed == 0) {
        Serial.println("  ALL PASS - ESP32 TWAI is working!");
    } else {
        Serial.println("  FAILED - check GPIO 5 is wired to GPIO 4");
    }

    twai_stop();
    twai_driver_uninstall();
}

void loop() {
    delay(10000);
}

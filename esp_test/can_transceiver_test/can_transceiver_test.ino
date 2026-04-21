/**
 * @file   can_transceiver_test.ino
 * @brief  Test CAN transceiver with a single ESP32
 *
 * Uses self=1 so the controller receives its own message.
 * The signal STILL goes through the physical transceiver pins.
 * If the transceiver is broken, TX will fail with bit errors
 * (because RXD won't match TXD during transmission).
 *
 * Wiring:
 *   ESP32 GPIO 5  → CTX
 *   ESP32 GPIO 4  → CRX
 *   ESP32 3.3V    → 3V3
 *   ESP32 GND     → GND
 *   Wire or resistor between CANH and CANL
 */

#include "driver/twai.h"

#define CAN_TX_PIN   GPIO_NUM_5
#define CAN_RX_PIN   GPIO_NUM_4

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=============================================");
    Serial.println("  CAN Transceiver Test (FIXED)");
    Serial.println("  Uses self-reception through physical pins");
    Serial.println("=============================================\n");

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
    Serial.println("[OK] TWAI started at 500 kbps\n");
    delay(100);

    /* ── Test: send through transceiver, receive back ──────── */
    int passed = 0;
    int failed = 0;

    struct {
        const char *name;
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
    } tests[] = {
        { "CAFEBABE (Route1)",  0x150, {0xCA,0xFE,0xBA,0xBE},             4 },
        { "DEADBEEF (Route2)",  0x250, {0xDE,0xAD,0xBE,0xEF,1,2,3,4},     8 },
        { "Short frame",       0x400, {0xFF,0x00},                        2 },
        { "Max ID 0x7FF",      0x7FF, {0x55,0xAA,0x55,0xAA},             4 },
        { "Single byte",       0x100, {0x42},                             1 },
    };
    int num_tests = sizeof(tests) / sizeof(tests[0]);

    for (int i = 0; i < num_tests; i++) {
        Serial.printf("  [%d/%d] %-25s ... ", i+1, num_tests, tests[i].name);

        twai_message_t tx = {};
        tx.identifier       = tests[i].id;
        tx.data_length_code = tests[i].len;
        tx.self             = 1;    // Self-reception — BUT still goes through
                                    // physical TX/RX pins. If transceiver is
                                    // broken, TX fails with bit error.
        memcpy(tx.data, tests[i].data, tests[i].len);

        esp_err_t err = twai_transmit(&tx, pdMS_TO_TICKS(500));
        if (err != ESP_OK) {
            Serial.printf("TX FAIL (err=%d) — transceiver issue!\n", err);
            failed++;
            continue;
        }

        twai_message_t rx;
        err = twai_receive(&rx, pdMS_TO_TICKS(500));
        if (err != ESP_OK) {
            Serial.printf("RX FAIL (err=%d)\n", err);
            failed++;
            continue;
        }

        bool ok = (rx.identifier == tests[i].id) &&
                  (rx.data_length_code == tests[i].len) &&
                  (memcmp(rx.data, tests[i].data, tests[i].len) == 0);

        if (ok) {
            Serial.println("PASS");
            passed++;
        } else {
            Serial.println("DATA MISMATCH");
            failed++;
        }
    }

    /* ── Results ───────────────────────────────────────────── */
    Serial.println();
    Serial.println("=============================================");
    Serial.printf("  %d PASSED  |  %d FAILED\n", passed, failed);
    Serial.println("=============================================");

    if (failed == 0) {
        Serial.println("  TRANSCEIVER IS WORKING!");
        Serial.println("  Signal path: ESP32 TX -> SN65HVD230 -> Bus -> SN65HVD230 -> ESP32 RX");
    } else {
        Serial.println("  Check transceiver wiring.");
    }

    twai_stop();
    twai_driver_uninstall();
    Serial.println("\n[DONE]");
}

void loop() {
    delay(10000);
}

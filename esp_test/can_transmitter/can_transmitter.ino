/**
 * @file   can_transmitter.ino
 * @brief  ESP32 CAN Transmitter — test companion for LPC1768 CAN Driver
 *
 * Sends CAN messages that the LPC1768 driver should receive.
 * Uses the ESP32's built-in TWAI (CAN 2.0B) controller.
 *
 * Hardware:
 *   ESP32 GPIO 5  → CAN Transceiver TX  (e.g. SN65HVD230)
 *   ESP32 GPIO 4  → CAN Transceiver RX
 *   Transceiver CANH/CANL → LPC1768 CAN Bus
 *   120 Ω termination resistors at both ends of the bus
 *
 * Baud: 500 kbps (matches LPC1768 cfg1/cfg2 in main.c)
 */

#include "driver/twai.h"

/* ── Pin Configuration ─────────────────────────────────────── */
#define CAN_TX_PIN   GPIO_NUM_5
#define CAN_RX_PIN   GPIO_NUM_4

/* ── Message IDs (matching LPC1768 gateway routes) ─────────── */
#define TEST_ID_ROUTE1   0x150   // Falls in Route 1: 0x100–0x1FF (CAN1→CAN2)
#define TEST_ID_ROUTE2   0x250   // Falls in Route 2: 0x200–0x2FF (CAN2→CAN1)
#define TEST_ID_OUTSIDE  0x400   // Outside any route — should NOT be forwarded
#define HEARTBEAT_ID     0x700   // Heartbeat / status message

/* ── Test Configuration ────────────────────────────────────── */
#define SEND_INTERVAL_MS  1000   // Interval between test messages
#define BAUD_RATE_500K    TWAI_TIMING_CONFIG_500KBITS()

/* ── Counters ──────────────────────────────────────────────── */
static uint32_t tx_count     = 0;
static uint32_t tx_err_count = 0;
static uint32_t msg_sequence = 0;

/* ──────────────────────────────────────────────────────────── */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  ESP32 CAN Transmitter");
    Serial.println("  Target: LPC1768 CAN Driver Test");
    Serial.println("  Baud: 500 kbps");
    Serial.println("===========================================");

    /* TWAI (CAN) configuration */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN,
                                                                  CAN_RX_PIN,
                                                                  TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = BAUD_RATE_500K;
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* Install and start TWAI driver */
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("[ERROR] TWAI driver install failed!");
        while (1) delay(1000);
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[ERROR] TWAI start failed!");
        while (1) delay(1000);
    }

    Serial.println("[OK] TWAI driver started. Transmitting...\n");
}

/* ──────────────────────────────────────────────────────────── */
/*  Helper: transmit one CAN frame                             */
/* ──────────────────────────────────────────────────────────── */
bool send_can_msg(uint32_t id, const uint8_t *data, uint8_t len) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.data_length_code = len;
    msg.extd             = 0;   // Standard frame (11-bit ID)
    msg.rtr              = 0;
    memcpy(msg.data, data, len);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        tx_count++;
        return true;
    } else {
        tx_err_count++;
        Serial.printf("[TX ERR] ID=0x%03X err=%d\n", id, err);
        return false;
    }
}

/* ──────────────────────────────────────────────────────────── */
/*  Test 1: Send message in Route 1 range (0x100–0x1FF)        */
/*  LPC1768 gateway should forward CAN1→CAN2                   */
/* ──────────────────────────────────────────────────────────── */
void test_route1_message() {
    uint8_t data[4] = { 0xCA, 0xFE, 0xBA, 0xBE };   // Same as LPC1768 demo
    if (send_can_msg(TEST_ID_ROUTE1, data, 4)) {
        Serial.printf("[TX] Route1 msg  ID=0x%03X  data=CA FE BA BE\n",
                      TEST_ID_ROUTE1);
    }
}

/* ──────────────────────────────────────────────────────────── */
/*  Test 2: Send message in Route 2 range (0x200–0x2FF)        */
/*  LPC1768 gateway should forward CAN2→CAN1                   */
/* ──────────────────────────────────────────────────────────── */
void test_route2_message() {
    uint8_t data[8] = {
        (uint8_t)(msg_sequence >> 24),
        (uint8_t)(msg_sequence >> 16),
        (uint8_t)(msg_sequence >>  8),
        (uint8_t)(msg_sequence),
        0xDE, 0xAD, 0xBE, 0xEF
    };
    if (send_can_msg(TEST_ID_ROUTE2, data, 8)) {
        Serial.printf("[TX] Route2 msg  ID=0x%03X  seq=%lu\n",
                      TEST_ID_ROUTE2, msg_sequence);
    }
}

/* ──────────────────────────────────────────────────────────── */
/*  Test 3: Send message OUTSIDE route range                    */
/*  LPC1768 gateway should NOT forward this                     */
/* ──────────────────────────────────────────────────────────── */
void test_out_of_range() {
    uint8_t data[2] = { 0xFF, 0x00 };
    if (send_can_msg(TEST_ID_OUTSIDE, data, 2)) {
        Serial.printf("[TX] Out-of-range  ID=0x%03X  (should be dropped)\n",
                      TEST_ID_OUTSIDE);
    }
}

/* ──────────────────────────────────────────────────────────── */
/*  Test 4: Heartbeat — periodic status frame                   */
/* ──────────────────────────────────────────────────────────── */
void send_heartbeat() {
    uint8_t data[4] = {
        0x01,                                    // node ID
        (uint8_t)(millis() / 1000),              // uptime (seconds, lower byte)
        (uint8_t)(tx_count & 0xFF),              // TX count lower byte
        (uint8_t)(tx_err_count & 0xFF)           // error count lower byte
    };
    if (send_can_msg(HEARTBEAT_ID, data, 4)) {
        Serial.printf("[TX] Heartbeat    ID=0x%03X  uptime=%lus  tx=%lu  err=%lu\n",
                      HEARTBEAT_ID, millis() / 1000, tx_count, tx_err_count);
    }
}

/* ──────────────────────────────────────────────────────────── */
/*  Test 5: Burst — send multiple messages rapidly              */
/*  Stresses the LPC1768 RX buffer and ISR                      */
/* ──────────────────────────────────────────────────────────── */
void test_burst(uint8_t count) {
    Serial.printf("[TX] --- BURST x%d ---\n", count);
    for (uint8_t i = 0; i < count; i++) {
        uint8_t data[4] = { 0xBB, i, (uint8_t)(count - i), 0x00 };
        uint32_t id = 0x100 + i;   // All within Route 1 range
        send_can_msg(id, data, 4);
        delay(2);                  // Small gap so bus isn't constantly dominant
    }
    Serial.printf("[TX] --- BURST DONE ---\n");
}

/* ──────────────────────────────────────────────────────────── */
/*  Main Loop — cycles through all tests                        */
/* ──────────────────────────────────────────────────────────── */
void loop() {
    msg_sequence++;
    uint8_t test_phase = (msg_sequence - 1) % 5;

    switch (test_phase) {
    case 0:
        test_route1_message();
        break;
    case 1:
        test_route2_message();
        break;
    case 2:
        test_out_of_range();
        break;
    case 3:
        send_heartbeat();
        break;
    case 4:
        test_burst(8);
        break;
    }

    Serial.printf("      [stats] total_tx=%lu  errors=%lu\n\n",
                  tx_count, tx_err_count);

    delay(SEND_INTERVAL_MS);
}

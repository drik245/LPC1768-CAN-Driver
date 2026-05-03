/**
 * ESP32_Demo_TX.ino
 * Transmits CAN frames with different IDs for LPC1768 demos
 *
 * Sends frames with 6 different IDs in rotation:
 *   0x080 — CRITICAL priority
 *   0x100 — HIGH priority     (passes LPC filter)
 *   0x150 — HIGH priority     (REJECTED by filter)
 *   0x200 — MEDIUM priority   (passes LPC filter)
 *   0x2FF — MEDIUM priority   (REJECTED by filter)
 *   0x300 — LOW priority      (REJECTED by filter)
 *
 * Wiring:
 *   GPIO5 → CAN TXD
 *   GPIO4 → CAN RXD
 */

#include "driver/twai.h"

#define CAN_TX  GPIO_NUM_5
#define CAN_RX  GPIO_NUM_4

struct FrameDef {
    uint32_t    id;
    const char *label;
    const char *filter_status;
    uint8_t     priority;
};

FrameDef frames[] = {
    { 0x080, "CRITICAL", "FILTER:REJECT", 1 },
    { 0x100, "HIGH",     "FILTER:PASS",   2 },
    { 0x150, "HIGH",     "FILTER:REJECT", 2 },
    { 0x200, "MEDIUM",   "FILTER:PASS",   3 },
    { 0x2FF, "MEDIUM",   "FILTER:REJECT", 3 },
    { 0x300, "LOW",      "FILTER:REJECT", 4 },
};

const int NUM_FRAMES = sizeof(frames) / sizeof(frames[0]);
uint32_t tx_count = 0;
uint32_t fail_count = 0;
uint8_t counter = 0;

/* ── Bus-off recovery ──────────────────────────────────── */
void check_and_recover()
{
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        if (status.state == TWAI_STATE_BUS_OFF) {
            Serial.println("[WARN] Bus-off! Recovering...");
            twai_initiate_recovery();
            delay(200);
            /* Restart driver */
            twai_stop();
            twai_driver_uninstall();
            delay(100);

            twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
                CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
            g.tx_queue_len = 5;
            twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
            twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
            twai_driver_install(&g, &t, &f);
            twai_start();
            Serial.println("[OK] Recovered from bus-off");
            fail_count = 0;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   ESP32 CAN Demo TX — Multi-ID Sender    ║");
    Serial.println("║   For LPC1768 Filter & Priority Demos    ║");
    Serial.println("╚══════════════════════════════════════════╝");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    g.tx_queue_len = 5;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK) {
        Serial.println("[OK] CAN bus active at 500 kbps\n");
    } else {
        Serial.println("[FAIL] CAN init failed!");
        while (1) delay(1000);
    }

    Serial.println("Frame Schedule:");
    Serial.println("  ID=0x080 CRITICAL  -> LPC filter: REJECT");
    Serial.println("  ID=0x100 HIGH      -> LPC filter: PASS");
    Serial.println("  ID=0x150 HIGH      -> LPC filter: REJECT");
    Serial.println("  ID=0x200 MEDIUM    -> LPC filter: PASS");
    Serial.println("  ID=0x2FF MEDIUM    -> LPC filter: REJECT");
    Serial.println("  ID=0x300 LOW       -> LPC filter: REJECT");
    Serial.println();
    Serial.println("--------------------------------------------");
}

void loop()
{
    for (int i = 0; i < NUM_FRAMES; i++)
    {
        /* Check for bus-off before each TX */
        check_and_recover();

        twai_message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.identifier = frames[i].id;
        msg.data_length_code = 4;
        msg.data[0] = counter;
        msg.data[1] = frames[i].priority;
        msg.data[2] = (uint8_t)(frames[i].id >> 8);
        msg.data[3] = (uint8_t)(frames[i].id & 0xFF);

        esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(500));
        tx_count++;

        if (r == ESP_OK) {
            fail_count = 0;
            Serial.printf("[TX #%3d] ID=0x%03X  %-8s  %s  Data=[%02X %02X %02X %02X]  OK\n",
                tx_count, frames[i].id, frames[i].label,
                frames[i].filter_status,
                msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
        } else {
            fail_count++;
            Serial.printf("[TX #%3d] ID=0x%03X  FAILED (0x%X)\n",
                tx_count, frames[i].id, r);
            /* If too many failures, recover */
            if (fail_count >= 3) {
                check_and_recover();
            }
        }

        delay(500);  /* 500ms gap — give bus time to stabilize */
    }

    Serial.println("--- Round complete ---\n");
    counter++;
    delay(1000);
}

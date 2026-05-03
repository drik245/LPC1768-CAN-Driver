/**
 * ESP32_CAN_Transceiver.ino
 * Both transmits AND receives CAN frames.
 *
 * - Receives LPC1768 frames (ID=0x100) and prints them
 * - Transmits test frames (ID=0x200) every 2 seconds
 *
 * Wiring:
 *   GPIO5 → SN65HVD230 TXD
 *   GPIO4 ← SN65HVD230 RXD
 */

#include "Arduino.h"
#include "driver/twai.h"

#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4
#define LED_PIN     2

uint8_t tx_counter = 0;
uint32_t rx_count = 0;
uint32_t tx_count = 0;
uint32_t last_tx_ms = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);
    pinMode(LED_PIN, OUTPUT);

    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║  ESP32 CAN Transceiver (TX + RX)     ║");
    Serial.println("║  RX: listens for all frames          ║");
    Serial.println("║  TX: sends ID=0x200 every 2s         ║");
    Serial.println("╚══════════════════════════════════════╝");

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g.rx_queue_len = 10;
    g.tx_queue_len = 5;

    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK ||
        twai_start() != ESP_OK) {
        Serial.println("[FATAL] TWAI init failed!");
        while (1) delay(1000);
    }
    Serial.println("[OK] CAN bus active. Listening + transmitting...\n");
    last_tx_ms = millis();
}

void loop()
{
    /* ── RX: Check for incoming frames (50ms timeout) ───── */
    twai_message_t rx_msg;
    esp_err_t rx_result = twai_receive(&rx_msg, pdMS_TO_TICKS(50));

    if (rx_result == ESP_OK)
    {
        rx_count++;
        digitalWrite(LED_PIN, HIGH);

        Serial.printf("\n[RX #%lu] ID=0x%03X DLC=%d  ",
            rx_count, rx_msg.identifier, rx_msg.data_length_code);

        /* Print hex data */
        Serial.print("HEX: ");
        for (int i = 0; i < rx_msg.data_length_code; i++)
            Serial.printf("%02X ", rx_msg.data[i]);

        /* Parse our protocol if ID=0x100 */
        if (rx_msg.identifier == 0x100 &&
            rx_msg.data_length_code == 8 &&
            rx_msg.data[0] == 0xAB && rx_msg.data[1] == 0xCD)
        {
            uint8_t counter = rx_msg.data[2];
            uint16_t sensor = ((uint16_t)rx_msg.data[4] << 8) | rx_msg.data[5];
            uint8_t crc_ok = (rx_msg.data[6] == (rx_msg.data[2] ^ rx_msg.data[4]));
            Serial.printf("\n       Counter=%d Sensor=%d CRC=%s",
                counter, sensor, crc_ok ? "OK" : "FAIL");
        }
        Serial.println();
        digitalWrite(LED_PIN, LOW);
    }

    /* ── TX: Send a test frame every 2 seconds ──────────── */
    if (millis() - last_tx_ms >= 2000)
    {
        last_tx_ms = millis();

        twai_message_t tx_msg;
        memset(&tx_msg, 0, sizeof(tx_msg));
        tx_msg.identifier = 0x200;
        tx_msg.data_length_code = 4;
        tx_msg.data[0] = 0xBE;
        tx_msg.data[1] = 0xEF;
        tx_msg.data[2] = tx_counter;
        tx_msg.data[3] = tx_counter ^ 0xFF;

        esp_err_t tx_result = twai_transmit(&tx_msg, pdMS_TO_TICKS(200));
        if (tx_result == ESP_OK) {
            tx_count++;
            Serial.printf("[TX #%lu] ID=0x200 Counter=%d  OK\n",
                tx_count, tx_counter);
        } else {
            Serial.printf("[TX] FAILED (0x%X) - is LPC1768 on the bus?\n",
                tx_result);
        }
        tx_counter++;
    }

    /* ── Bus health check ─────────────────────────────── */
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        if (status.state == TWAI_STATE_BUS_OFF) {
            Serial.println("[WARN] Bus-off! Recovering...");
            twai_initiate_recovery();
            delay(500);
        }
    }
}

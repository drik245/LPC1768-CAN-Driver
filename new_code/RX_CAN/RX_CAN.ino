/**
 * ESP32_CAN_Receiver.ino
 * TWAI (CAN) receiver @ 500kbps
 * Prints received frames to Serial Monitor
 *
 * Pin connections:
 *   GPIO5 (TX) → SN65HVD230 TXD
 *   GPIO4 (RX) ← SN65HVD230 RXD
 *
 * Board: ESP32 WROOM DevKit
 * Framework: Arduino (ESP-IDF TWAI driver via arduino-esp32)
 *
 * Install ESP32 boards:
 *   File → Preferences → Additional Boards Manager URLs:
 *   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 */

#include "Arduino.h"
#include "driver/twai.h"

/* ─── Pin Definitions ──────────────────────────────── */
#define CAN_TX_PIN  GPIO_NUM_5   /* ESP32 → SN65HVD230 TXD */
#define CAN_RX_PIN  GPIO_NUM_4   /* SN65HVD230 RXD → ESP32  */

/* ─── LED for visual indication ────────────────────── */
#define LED_BUILTIN_PIN  2       /* ESP32 WROOM onboard LED */

/* ─── Message Statistics ──────────────────────────── */
static uint32_t msg_received   = 0;
static uint32_t msg_errors     = 0;
static uint32_t last_print_ms  = 0;

/* ─── Forward Declarations ─────────────────────────── */
bool twai_init_bus(void);
void print_can_frame(const twai_message_t &msg);
void print_statistics(void);
void blink_led(int times, int on_ms, int off_ms);

/* ──────────────────────────────────────────────────── */
void setup()
{
    Serial.begin(115200);
    delay(1000);   /* Wait for serial monitor to open */

    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW);

    Serial.println("╔══════════════════════════════════╗");
    Serial.println("║   ESP32 CAN (TWAI) Receiver      ║");
    Serial.println("║   500kbps, Standard Frame        ║");
    Serial.println("╚══════════════════════════════════╝");
    Serial.printf("TX GPIO: %d   RX GPIO: %d\n", CAN_TX_PIN, CAN_RX_PIN);

    /* ── Initialize TWAI (CAN) driver ────────────────── */
    if (!twai_init_bus()) {
        Serial.println("[ERROR] TWAI initialization FAILED. Halting.");
        blink_led(10, 100, 100);  /* Fast blink = fatal error */
        while (1) { delay(1000); }
    }

    Serial.println("[OK] TWAI driver started. Listening for frames...");
    Serial.println("─────────────────────────────────────────────────");
    blink_led(3, 200, 200);  /* 3 slow blinks = boot OK */
}

/* ──────────────────────────────────────────────────── */
void loop()
{
    twai_message_t message;

    /* ── Wait for incoming frame (100ms timeout) ─────── */
    esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(100));

    if (result == ESP_OK)
    {
        msg_received++;
        digitalWrite(LED_BUILTIN_PIN, HIGH);

        /* ── Print the received frame ─────────────────── */
        print_can_frame(message);

        /* ── Simple data validation for our protocol ───── */
        if (message.identifier == 0x100 &&
            message.data_length_code == 8 &&
            message.data[0] == 0xAB &&
            message.data[1] == 0xCD)
        {
            uint8_t counter   = message.data[2];
            uint8_t status    = message.data[3];
            uint16_t sensor   = ((uint16_t)message.data[4] << 8) | message.data[5];
            uint8_t crc_rx    = message.data[6];
            uint8_t crc_calc  = message.data[2] ^ message.data[4];

            Serial.printf("  ► Parsed: Counter=%3d  Status=%02X  Sensor=%5d",
                          counter, status, sensor);

            if (crc_rx == crc_calc)
                Serial.println("  [CRC OK]");
            else
                Serial.printf("  [CRC FAIL: got %02X exp %02X]\n", crc_rx, crc_calc);
        }

        digitalWrite(LED_BUILTIN_PIN, LOW);
    }
    else if (result == ESP_ERR_TIMEOUT)
    {
        /* ── No frame received in 100ms — check bus health */
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            if (status.state == TWAI_STATE_BUS_OFF) {
                Serial.println("[WARN] Bus-off detected! Recovering...");
                twai_initiate_recovery();
                delay(500);
            }
            else if (status.rx_error_counter > 96 || status.tx_error_counter > 96) {
                Serial.printf("[WARN] High error counters: RX=%d TX=%d\n",
                              status.rx_error_counter, status.tx_error_counter);
            }
        }
    }
    else
    {
        msg_errors++;
        Serial.printf("[ERROR] Receive error: 0x%X\n", result);
    }

    /* ── Print statistics every 10 seconds ───────────── */
    if (millis() - last_print_ms >= 10000UL) {
        print_statistics();
        last_print_ms = millis();
    }
}

/* ──────────────────────────────────────────────────── */
/*  TWAI Bus Initialization                             */
/* ──────────────────────────────────────────────────── */
bool twai_init_bus(void)
{
    /* ── General Config ───────────────────────────────── */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN,        /* TX GPIO */
        CAN_RX_PIN,        /* RX GPIO */
        TWAI_MODE_NORMAL   /* Normal operation (not Listen-Only) */
    );
    /* Increase RX queue depth for burst messages */
    g_config.rx_queue_len = 10;
    g_config.tx_queue_len = 5;

    /* ── Timing Config ────────────────────────────────── */
    /* TWAI_TIMING_CONFIG_500KBITS() is a built-in macro:
     * BRP=8, TSEG1=15, TSEG2=4, SJW=3
     * = 500kbps @ 80MHz APB clock                       */
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    /* ── Filter Config ────────────────────────────────── */
    /* Accept ALL messages — we'll filter in software     */
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* To accept ONLY ID 0x100 (standard frame):
     * twai_filter_config_t f_config = {
     *     .acceptance_code = (0x100 << 21),
     *     .acceptance_mask = ~(0x7FF << 21),
     *     .single_filter   = true
     * };
     */

    /* ── Install Driver ──────────────────────────────── */
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] twai_driver_install: %s\n", esp_err_to_name(err));
        return false;
    }

    /* ── Start Driver ───────────────────────────────── */
    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[ERROR] twai_start: %s\n", esp_err_to_name(err));
        twai_driver_uninstall();
        return false;
    }

    return true;
}

/* ──────────────────────────────────────────────────── */
/*  Print CAN Frame to Serial Monitor                   */
/* ──────────────────────────────────────────────────── */
void print_can_frame(const twai_message_t &msg)
{
    Serial.printf("\n[%6lu ms] ", millis());
    Serial.printf("ID: 0x%03X  ", msg.identifier);
    Serial.printf("DLC: %d  ", msg.data_length_code);
    Serial.printf("%s  ", msg.extd ? "[EXT]" : "[STD]");
    Serial.printf("%s\n",  msg.rtr  ? "[RTR]" : "[DATA]");

    if (!msg.rtr && msg.data_length_code > 0)
    {
        Serial.print("  HEX: ");
        for (int i = 0; i < msg.data_length_code; i++)
            Serial.printf("%02X ", msg.data[i]);

        Serial.print("\n  DEC: ");
        for (int i = 0; i < msg.data_length_code; i++)
            Serial.printf("%-4d ", msg.data[i]);

        Serial.print("\n  CHR: ");
        for (int i = 0; i < msg.data_length_code; i++)
            Serial.printf("%-4c ", isprint(msg.data[i]) ? msg.data[i] : '.');

        Serial.println();
    }
}

/* ──────────────────────────────────────────────────── */
/*  Print Bus Statistics                                */
/* ──────────────────────────────────────────────────── */
void print_statistics(void)
{
    twai_status_info_t status;
    Serial.println("\n──── Bus Statistics ────────────────────");
    Serial.printf("  Messages received : %lu\n", msg_received);
    Serial.printf("  Receive errors    : %lu\n", msg_errors);

    if (twai_get_status_info(&status) == ESP_OK) {
        Serial.printf("  RX error counter  : %d\n",  status.rx_error_counter);
        Serial.printf("  TX error counter  : %d\n",  status.tx_error_counter);
        Serial.printf("  Msgs pending (RX) : %lu\n", status.msgs_to_rx);
        Serial.printf("  Bus state         : ");
        switch (status.state) {
            case TWAI_STATE_STOPPED:    Serial.println("STOPPED");    break;
            case TWAI_STATE_RUNNING:    Serial.println("RUNNING");    break;
            case TWAI_STATE_BUS_OFF:    Serial.println("BUS OFF ⚠"); break;
            case TWAI_STATE_RECOVERING: Serial.println("RECOVERING"); break;
            default:                    Serial.println("UNKNOWN");    break;
        }
    }
    Serial.println("────────────────────────────────────────\n");
}

/* ──────────────────────────────────────────────────── */
/*  LED Blink Helper                                    */
/* ──────────────────────────────────────────────────── */
void blink_led(int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_BUILTIN_PIN, HIGH); delay(on_ms);
        digitalWrite(LED_BUILTIN_PIN, LOW);  delay(off_ms);
    }
}
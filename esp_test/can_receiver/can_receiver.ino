/**
 * @file   can_receiver.ino
 * @brief  ESP32 CAN Receiver — test companion for LPC1768 CAN Driver
 *
 * Listens on the CAN bus and prints every received frame.
 * Use this to verify that the LPC1768 driver is transmitting correctly,
 * and that the gateway is forwarding messages as expected.
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

/* ── Known Message ID Ranges ──────────────────────────────── */
#define ROUTE1_MIN  0x100
#define ROUTE1_MAX  0x1FF
#define ROUTE2_MIN  0x200
#define ROUTE2_MAX  0x2FF
#define GW_OFFSET   0x400    // Gateway transform adds this offset

/* ── Counters ──────────────────────────────────────────────── */
static uint32_t rx_count         = 0;
static uint32_t rx_route1_count  = 0;
static uint32_t rx_route2_count  = 0;
static uint32_t rx_gw_fwd_count  = 0;
static uint32_t rx_other_count   = 0;
static uint32_t rx_err_count     = 0;

/* ──────────────────────────────────────────────────────────── */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  ESP32 CAN Receiver");
    Serial.println("  Target: LPC1768 CAN Driver Test");
    Serial.println("  Baud: 500 kbps");
    Serial.println("===========================================");

    /* TWAI (CAN) configuration — listen-only to avoid interfering */
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN,
                                                                  CAN_RX_PIN,
                                                                  TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
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

    Serial.println("[OK] TWAI driver started (LISTEN-ONLY). Waiting for messages...\n");
    Serial.println("ID       | Type | DLC | Data                     | Route");
    Serial.println("---------+------+-----+--------------------------+------------------");
}

/* ──────────────────────────────────────────────────────────── */
/*  Classify and label received message                         */
/* ──────────────────────────────────────────────────────────── */
const char* classify_message(uint32_t id) {
    if (id >= ROUTE1_MIN && id <= ROUTE1_MAX) {
        rx_route1_count++;
        return "Route1 (0x100-1FF)";
    }
    if (id >= ROUTE2_MIN && id <= ROUTE2_MAX) {
        rx_route2_count++;
        return "Route2 (0x200-2FF)";
    }
    /* Gateway with transform adds 0x400 offset */
    if (id >= (ROUTE1_MIN + GW_OFFSET) && id <= (ROUTE1_MAX + GW_OFFSET)) {
        rx_gw_fwd_count++;
        return "GW Fwd (Route1+0x400)";
    }
    if (id >= (ROUTE2_MIN + GW_OFFSET) && id <= (ROUTE2_MAX + GW_OFFSET)) {
        rx_gw_fwd_count++;
        return "GW Fwd (Route2+0x400)";
    }
    rx_other_count++;
    return "Other";
}

/* ──────────────────────────────────────────────────────────── */
/*  Print a received CAN frame                                  */
/* ──────────────────────────────────────────────────────────── */
void print_frame(const twai_message_t *msg) {
    rx_count++;

    const char *route = classify_message(msg->identifier);

    /* ID */
    Serial.printf("0x%03X    | ", msg->identifier);

    /* Frame type */
    if (msg->extd)     Serial.print("EXT  | ");
    else if (msg->rtr) Serial.print("RTR  | ");
    else               Serial.print("STD  | ");

    /* DLC */
    Serial.printf("  %d | ", msg->data_length_code);

    /* Data bytes */
    char buf[26] = {0};
    int  pos = 0;
    for (int i = 0; i < msg->data_length_code && i < 8; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", msg->data[i]);
    }
    Serial.printf("%-25s| %s", buf, route);
    Serial.println();
}

/* ──────────────────────────────────────────────────────────── */
/*  Print periodic statistics                                   */
/* ──────────────────────────────────────────────────────────── */
void print_stats() {
    Serial.println();
    Serial.println("─── Statistics ────────────────────────────");
    Serial.printf("  Total RX:       %lu\n", rx_count);
    Serial.printf("  Route 1 msgs:   %lu\n", rx_route1_count);
    Serial.printf("  Route 2 msgs:   %lu\n", rx_route2_count);
    Serial.printf("  GW forwarded:   %lu\n", rx_gw_fwd_count);
    Serial.printf("  Other msgs:     %lu\n", rx_other_count);
    Serial.printf("  RX errors:      %lu\n", rx_err_count);
    Serial.println("───────────────────────────────────────────\n");
}

/* ──────────────────────────────────────────────────────────── */
/*  Main Loop — poll for received frames                        */
/* ──────────────────────────────────────────────────────────── */
static uint32_t last_stats_time = 0;

void loop() {
    twai_message_t msg;

    /* Non-blocking receive — check every 10 ms */
    esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(10));

    if (err == ESP_OK) {
        print_frame(&msg);
    } else if (err != ESP_ERR_TIMEOUT) {
        rx_err_count++;
        Serial.printf("[RX ERR] err=%d\n", err);
    }

    /* Print stats every 10 seconds */
    if (millis() - last_stats_time >= 10000) {
        last_stats_time = millis();
        print_stats();

        /* Also check TWAI controller status */
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            Serial.printf("  TWAI state:     %d\n", status.state);
            Serial.printf("  TX queued:      %lu\n", status.msgs_to_tx);
            Serial.printf("  RX queued:      %lu\n", status.msgs_to_rx);
            Serial.printf("  TX error cnt:   %lu\n", status.tx_error_counter);
            Serial.printf("  RX error cnt:   %lu\n", status.rx_error_counter);
            Serial.printf("  TX failed:      %lu\n", status.tx_failed_count);
            Serial.printf("  RX miss:        %lu\n", status.rx_missed_count);
            Serial.printf("  Arb lost:       %lu\n", status.arb_lost_count);
            Serial.printf("  Bus error:      %lu\n", status.bus_error_count);
            Serial.println();
        }
    }
}

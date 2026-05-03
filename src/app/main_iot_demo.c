/**
 * @file main_iot_demo.c
 * @brief IoT CAN Demo — Uses the CAN Driver API
 *
 * Copy to main.c and rebuild.
 *
 * Features:
 *   - Physical switch on P2.1 (mbed p25) → sends CAN frame
 *   - Receives CAN frame from ESP32 → blinks LED2→3→4
 *   - LED1 = heartbeat
 *
 * Wiring:
 *   P0.0 (RD1) → CAN transceiver RXD
 *   P0.1 (TD1) → CAN transceiver TXD
 *   P2.1 (p25) → Switch to GND (internal pull-up enabled)
 *
 * CAN Protocol:
 *   TX: ID=0x300, data[0]=0x01 → "play audio" command
 *   RX: ID=0x400, data[0]=N   → blink LED sequence
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include <string.h>

/* ── LEDs ─────────────────────────────────────────────── */
#define LED1  (1UL << 18)   /* P1.18 */
#define LED2  (1UL << 20)   /* P1.20 */
#define LED3  (1UL << 21)   /* P1.21 */
#define LED4  (1UL << 23)   /* P1.23 */
#define ALL   (LED1 | LED2 | LED3 | LED4)

/* ── Switch on P2.1 (mbed pin p25) ────────────────────── */
#define SW_PIN   (1UL << 1)   /* P2.1 = mbed p25 */

static void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    for (; ms > 0U; ms--)
        for (i = 0U; i < 100000UL; i++);
}

/* ── LED sequence animation ──────────────────────────── */
static void blink_sequence(void)
{
    uint32_t seq[] = {LED2, LED3, LED4};
    for (int round = 0; round < 2; round++) {
        for (int i = 0; i < 3; i++) {
            LPC_GPIO1->FIOSET = seq[i];
            delay_ms(150);
            LPC_GPIO1->FIOCLR = seq[i];
            delay_ms(100);
        }
    }
}

/* ── RX callback (ISR context) ────────────────────────── */
static volatile uint32_t g_rx_flag = 0;

static void on_rx(can_channel_t ch, can_message_t *msg)
{
    (void)ch; (void)msg;
    g_rx_flag++;
}

static void on_err(can_channel_t ch, can_error_t err)
{
    (void)ch; (void)err;
}

/* ── SysTick ──────────────────────────────────────────── */
void SysTick_Handler(void)
{
    can_timestamp_tick();
}

static uint8_t switch_last = 1;

int main(void)
{
    /* ── GPIO init ────────────────────────────────────── */
    LPC_GPIO1->FIODIR |= ALL;
    LPC_GPIO1->FIOCLR  = ALL;

    /* Switch input: P2.1 (p25), pull-up */
    LPC_GPIO2->FIODIR &= ~SW_PIN;
    LPC_PINCON->PINMODE4 &= ~(3UL << 2);

    /* SysTick for driver timestamps */
    SysTick_Config(SystemCoreClock / 1000);

    /* ═══════════════════════════════════════════════════
     * CAN Driver Init — using the driver API
     * ═══════════════════════════════════════════════════ */
    can_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.channel          = CAN_CHANNEL_1;
    cfg.baudrate         = CAN_BAUD_500K;
    cfg.mode             = CAN_MODE_NORMAL;
    cfg.rx_callback      = on_rx;
    cfg.error_callback   = on_err;
    cfg.enable_timestamp = true;

    if (can_init(&cfg) != CAN_OK) {
        /* Init failed — blink all LEDs */
        while (1) {
            LPC_GPIO1->FIOPIN ^= ALL;
            delay_ms(100);
        }
    }

    /* Disable error interrupts to prevent ISR starvation.
     * Keep RX + TX complete — the driver needs these for
     * can_receive() ring buffer and isr_feed_tx(). */
    LPC_CAN1->IER = (1UL << 0)    /* RIE  — RX interrupt      */
                   | (1UL << 1)    /* TIE1 — TX1 complete       */
                   | (1UL << 9)    /* TIE2 — TX2 complete       */
                   | (1UL << 10);  /* TIE3 — TX3 complete       */

    /* Boot animation */
    uint32_t leds[] = {LED1, LED2, LED3, LED4};
    for (int i = 0; i < 4; i++) {
        LPC_GPIO1->FIOSET = leds[i]; delay_ms(80);
        LPC_GPIO1->FIOCLR = leds[i];
    }

    uint32_t loop_count = 0;

    /* ═══════════════════════════════════════════════════
     * Main loop — using driver API for TX and RX
     * ═══════════════════════════════════════════════════ */
    while (1)
    {
        /* Heartbeat every ~10 loops */
        if ((loop_count % 10) == 0)
            LPC_GPIO1->FIOPIN ^= LED1;

        /* ── Switch debounce + read ───────────────────── */
        uint8_t sw_now = (LPC_GPIO2->FIOPIN & SW_PIN) ? 1 : 0;

        if (sw_now == 0 && switch_last == 1)
        {
            /* Falling edge = button pressed */
            /* TX via driver API */
            can_message_t tx_msg;
            memset(&tx_msg, 0, sizeof(tx_msg));
            tx_msg.id         = 0x300;
            tx_msg.frame_type = CAN_FRAME_STANDARD;
            tx_msg.rtr        = false;
            tx_msg.dlc        = 2;
            tx_msg.data[0]    = 0x01;   /* Command: play audio */
            tx_msg.data[1]    = 0x00;

            if (can_transmit(CAN_CHANNEL_1, &tx_msg) == CAN_OK) {
                /* Visual feedback: quick double-flash */
                LPC_GPIO1->FIOSET = LED1; delay_ms(50);
                LPC_GPIO1->FIOCLR = LED1; delay_ms(50);
                LPC_GPIO1->FIOSET = LED1; delay_ms(50);
                LPC_GPIO1->FIOCLR = LED1;
            }
        }
        switch_last = sw_now;

        /* ── RX via driver API ────────────────────────── */
        can_message_t rx_msg;
        while (can_receive(CAN_CHANNEL_1, &rx_msg, 0) == CAN_OK)
        {
            /* LED blink command from ESP32 dashboard */
            if (rx_msg.id == 0x400 && rx_msg.dlc >= 1) {
                blink_sequence();
            }
        }

        loop_count++;
        delay_ms(50);   /* 50ms loop = responsive switch */
    }

    return 0;
}

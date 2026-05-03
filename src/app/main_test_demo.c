/**
 * @file main_test_demo.c
 * @brief Demo 3: Self-test suite (no external hardware)
 *
 * Copy this file to main.c and rebuild to run the test suite.
 * No ESP32 or transceiver needed — uses internal loopback.
 *
 * LED sequence:
 *   Chase 1→2→3→4     = boot
 *   LED1 solid         = CAN init OK
 *   LED2 solid         = TX OK (self-test)
 *   LED3 solid         = RX OK (loopback received)
 *   LED4 solid         = Data verified
 *   All 4 blinking     = running full test suite
 *   All 4 solid (2s on / 0.1s off) = ALL TESTS PASSED
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include "can_test.h"
#include <string.h>

/* ── LEDs ─────────────────────────────────────────────── */
#define LED1  (1UL << 18)
#define LED2  (1UL << 20)
#define LED3  (1UL << 21)
#define LED4  (1UL << 23)
#define ALL   (LED1 | LED2 | LED3 | LED4)

static void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    for (; ms > 0U; ms--)
        for (i = 0U; i < 100000UL; i++);
}

void SysTick_Handler(void)
{
    can_timestamp_tick();
}

extern void run_all_tests(test_suite_t *suite);

static void error_halt(uint32_t led_pattern)
{
    while (1) {
        LPC_GPIO1->FIOSET = led_pattern;
        delay_ms(200);
        LPC_GPIO1->FIOCLR = led_pattern;
        delay_ms(200);
    }
}

int main(void)
{
    LPC_GPIO1->FIODIR |= ALL;
    LPC_GPIO1->FIOCLR  = ALL;
    SysTick_Config(SystemCoreClock / 1000);

    /* ── Boot chase ───────────────────────────────────── */
    uint32_t ch[] = {LED1, LED2, LED3, LED4};
    for (int r = 0; r < 2; r++)
        for (int i = 0; i < 4; i++) {
            LPC_GPIO1->FIOCLR = ALL;
            LPC_GPIO1->FIOSET = ch[i];
            delay_ms(80);
        }
    LPC_GPIO1->FIOCLR = ALL;
    delay_ms(500);

    /* ═══ STEP 1: Init CAN1 self-test ═══════════════════ */
    can_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.channel          = CAN_CHANNEL_1;
    cfg.baudrate         = CAN_BAUD_500K;
    cfg.mode             = CAN_MODE_SELFTEST;
    cfg.rx_callback      = 0;
    cfg.error_callback   = 0;
    cfg.enable_timestamp = false;

    if (can_init(&cfg) != CAN_OK)
        error_halt(LED1);

    /* Disable error interrupts (re-trigger in self-test) */
    LPC_CAN1->IER = (1UL << 0) | (1UL << 1) | (1UL << 9) | (1UL << 10);

    LPC_GPIO1->FIOSET = LED1;
    delay_ms(500);

    /* ═══ STEP 2: Transmit ══════════════════════════════ */
    can_message_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.id = 0x123;  tx.frame_type = CAN_FRAME_STANDARD;
    tx.dlc = 4;
    tx.data[0] = 0xDE; tx.data[1] = 0xAD;
    tx.data[2] = 0xBE; tx.data[3] = 0xEF;

    LPC_CAN1->IER = 0;  /* No ISR stealing the frame */

    if (can_transmit(CAN_CHANNEL_1, &tx) != CAN_OK)
        error_halt(LED2);

    LPC_GPIO1->FIOSET = LED2;

    /* ═══ STEP 3: Receive loopback (direct HW poll) ════ */
    can_message_t rx;
    memset(&rx, 0, sizeof(rx));
    int rx_ok = 0;
    for (volatile uint32_t t = 0; t < 2000000UL; t++) {
        if (LPC_CAN1->GSR & (1UL << 0)) {
            uint32_t rfs = LPC_CAN1->RFS, rid = LPC_CAN1->RID;
            uint32_t rda = LPC_CAN1->RDA, rdb = LPC_CAN1->RDB;
            LPC_CAN1->CMR = (1UL << 2);
            rx.id  = rid & 0x7FFU;
            rx.dlc = (uint8_t)((rfs >> 16) & 0x0F);
            rx.data[0]=(uint8_t)rda;     rx.data[1]=(uint8_t)(rda>>8);
            rx.data[2]=(uint8_t)(rda>>16);rx.data[3]=(uint8_t)(rda>>24);
            rx.data[4]=(uint8_t)rdb;     rx.data[5]=(uint8_t)(rdb>>8);
            rx.data[6]=(uint8_t)(rdb>>16);rx.data[7]=(uint8_t)(rdb>>24);
            rx_ok = 1; break;
        }
    }
    if (!rx_ok) error_halt(LED3);
    LPC_GPIO1->FIOSET = LED3;
    delay_ms(500);

    /* ═══ STEP 4: Verify data ══════════════════════════ */
    if (rx.id != 0x123 || rx.dlc != 4 ||
        rx.data[0] != 0xDE || rx.data[1] != 0xAD ||
        rx.data[2] != 0xBE || rx.data[3] != 0xEF)
        error_halt(LED4);

    LPC_GPIO1->FIOSET = LED4;
    delay_ms(1000);

    /* ═══ Full test suite ══════════════════════════════ */
    can_deinit(CAN_CHANNEL_1);
    for (int i = 0; i < 5; i++) {
        LPC_GPIO1->FIOSET = ALL; delay_ms(150);
        LPC_GPIO1->FIOCLR = ALL; delay_ms(150);
    }

    test_suite_t suite;
    run_all_tests(&suite);

    if (suite.failed == 0) {
        while (1) {
            LPC_GPIO1->FIOSET = ALL; delay_ms(2000);
            LPC_GPIO1->FIOCLR = ALL; delay_ms(100);
        }
    } else {
        while (1) {
            for (uint8_t f = 0; f < suite.failed && f < 15; f++) {
                LPC_GPIO1->FIOSET = LED1; delay_ms(200);
                LPC_GPIO1->FIOCLR = LED1; delay_ms(200);
            }
            delay_ms(1000);
            for (uint8_t p = 0; p < suite.passed && p < 15; p++) {
                LPC_GPIO1->FIOSET = LED4; delay_ms(200);
                LPC_GPIO1->FIOCLR = LED4; delay_ms(200);
            }
            delay_ms(2000);
        }
    }
}

/**
 * @file main.c
 * @brief CAN Driver — Week 4 Test Runner & Deployment Demo
 *
 * LED1 = P1.18 (mbed LPC1768 onboard LED1)
 *
 *   - SOLID ON 3 s     = all tests passed
 *   - <N> fast blinks   = number of failures
 *   - After report: Normal-mode gateway with heartbeat
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include "can_buffer.h"
#include "can_gateway.h"
#include "can_test.h"
#include <string.h>

extern void run_all_tests(test_suite_t *suite);

/* ── LED on P1.18 (mbed LPC1768 LED1) ─────────────────────── */
#define LED1_PIN    (1U << 18)

static test_suite_t g_suite;

static void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    for (; ms > 0; ms--)
        for (i = 0; i < 10000; i++);
}

static void led_on(void)     { LPC_GPIO1->FIOSET = LED1_PIN; }
static void led_off(void)    { LPC_GPIO1->FIOCLR = LED1_PIN; }
static void led_toggle(void) { LPC_GPIO1->FIOPIN ^= LED1_PIN; }

static void led_blink_count(uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++) {
        led_on();  delay_ms(120);
        led_off(); delay_ms(120);
    }
    delay_ms(800);
}

static void deploy_error_cb(can_channel_t ch, can_error_t err)
{
    (void)ch; (void)err;
}

void SysTick_Handler(void)
{
    can_timestamp_tick();
}

/* ── Phase 1: Run Tests ────────────────────────────────────── */
static void phase_run_tests(void)
{
    run_all_tests(&g_suite);
}

/* ── Phase 2: LED Report ───────────────────────────────────── */
static void phase_report_results(void)
{
    if (g_suite.failed == 0) {
        led_on();
        delay_ms(3000);
        led_off();
        delay_ms(500);
        for (int i = 0; i < 3; i++) {
            led_on();  delay_ms(80);
            led_off(); delay_ms(80);
        }
        delay_ms(1000);
    } else {
        for (int rep = 0; rep < 5; rep++) {
            led_blink_count(g_suite.failed);
        }
    }
}

/* ── Phase 3: Production Gateway ───────────────────────────── */
static void phase_deployment(void)
{
    can_deinit(CAN_CHANNEL_1);
    can_deinit(CAN_CHANNEL_2);

    can_config_t cfg1;
    memset(&cfg1, 0, sizeof(cfg1));
    cfg1.channel         = CAN_CHANNEL_1;
    cfg1.baudrate        = CAN_BAUD_500K;
    cfg1.mode            = CAN_MODE_NORMAL;
    cfg1.error_callback  = deploy_error_cb;
    cfg1.enable_timestamp = true;
    can_init(&cfg1);

    can_config_t cfg2;
    memset(&cfg2, 0, sizeof(cfg2));
    cfg2.channel         = CAN_CHANNEL_2;
    cfg2.baudrate        = CAN_BAUD_500K;
    cfg2.mode            = CAN_MODE_NORMAL;
    cfg2.error_callback  = deploy_error_cb;
    cfg2.enable_timestamp = true;
    can_init(&cfg2);

    can_clear_filters(CAN_CHANNEL_1);

    can_gateway_init();

    can_route_t r1;
    r1.src = CAN_CHANNEL_1;  r1.dst = CAN_CHANNEL_2;
    r1.id_min = 0x000;       r1.id_max = 0x7FF;
    r1.frame_type = CAN_FRAME_STANDARD;
    r1.active = true;
    can_gateway_add_route(&r1);

    can_route_t r2;
    r2.src = CAN_CHANNEL_2;  r2.dst = CAN_CHANNEL_1;
    r2.id_min = 0x000;       r2.id_max = 0x7FF;
    r2.frame_type = CAN_FRAME_STANDARD;
    r2.active = true;
    can_gateway_add_route(&r2);

    uint32_t heartbeat = 0;
    while (1) {
        can_gateway_process();
        if (++heartbeat >= 500) {
            led_toggle();
            heartbeat = 0;
        }
        delay_ms(1);
    }
}

/* ── Main ──────────────────────────────────────────────────── */
int main(void)
{
    LPC_GPIO1->FIODIR |= LED1_PIN;
    led_off();

    SysTick_Config(SystemCoreClock / 1000);

    /* Skip self-tests when on live bus — they block ACKs.
       Uncomment below to run tests (disconnect ESP32 first):
       phase_run_tests();
       phase_report_results();
    */

    /* Go straight to production gateway */
    phase_deployment();

    return 0;
}

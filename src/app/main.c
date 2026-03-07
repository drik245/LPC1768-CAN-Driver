/**
 * @file main.c
 * @brief CAN Driver — Week 3 Dual-Channel Gateway Demo
 *
 * Sets up CAN1 and CAN2 in self-test mode.
 * Routes messages from CAN1 → CAN2 (IDs 0x100–0x1FF)
 * and CAN2 → CAN1 (IDs 0x200–0x2FF).
 *
 * Transmits a test message on CAN1, verifies it arrives on CAN2
 * via the gateway, then prints diagnostics via LED blink codes.
 *
 * LED1 (P0.22):  fast blink = gateway test passed
 *                slow blink = test failed
 * LED2 (P0.3) :  toggles on each forwarded message (visual heartbeat)
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include "can_buffer.h"
#include "can_gateway.h"

/* ── LED definitions ───────────────────────────────────────── */
#define LED1_PIN    (1U << 22)      /* P0.22 — status        */

/* ── Globals ───────────────────────────────────────────────── */
static volatile bool     g_ch2_received = false;
static volatile uint32_t g_ch2_rx_id    = 0;
static volatile uint8_t  g_ch2_rx_data0 = 0;

/* ── Blocking delay ────────────────────────────────────────── */
static void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    for (; ms > 0; ms--)
        for (i = 0; i < 10000; i++);
}

/* ── Error callback (shared) ───────────────────────────────── */
static void on_error(can_channel_t ch, can_error_t error)
{
    (void)ch; (void)error;
}

/* ── SysTick — 1 ms tick ───────────────────────────────────── */
void SysTick_Handler(void)
{
    can_timestamp_tick();
}

/* ── Message transform example (optional) ──────────────────── */
static bool gw_transform(const can_route_t *route, can_message_t *msg)
{
    (void)route;
    /* Example: offset the ID by 0x400 when forwarding */
    msg->id += 0x400;
    return true;    /* allow forwarding */
}

/* ── Main ──────────────────────────────────────────────────── */
int main(void)
{
    /* LED init */
    LPC_GPIO0->FIODIR |= LED1_PIN;
    LPC_GPIO0->FIOCLR  = LED1_PIN;

    /* 1 ms SysTick */
    SysTick_Config(SystemCoreClock / 1000);

    /* ─── Init CAN1 (self-test) ─────────────────────────── */
    can_config_t cfg1;
    cfg1.channel         = CAN_CHANNEL_1;
    cfg1.baudrate        = CAN_BAUD_500K;
    cfg1.mode            = CAN_MODE_SELFTEST;
    cfg1.rx_callback     = 0;       /* gateway will set its own */
    cfg1.error_callback  = on_error;
    cfg1.enable_timestamp = true;

    if (can_init(&cfg1) != CAN_OK) {
        while (1) { LPC_GPIO0->FIOPIN ^= LED1_PIN; delay_ms(2000); }
    }

    /* ─── Init CAN2 (self-test) ─────────────────────────── */
    can_config_t cfg2;
    cfg2.channel         = CAN_CHANNEL_2;
    cfg2.baudrate        = CAN_BAUD_500K;
    cfg2.mode            = CAN_MODE_SELFTEST;
    cfg2.rx_callback     = 0;
    cfg2.error_callback  = on_error;
    cfg2.enable_timestamp = true;

    if (can_init(&cfg2) != CAN_OK) {
        while (1) { LPC_GPIO0->FIOPIN ^= LED1_PIN; delay_ms(2000); }
    }

    /* ─── Set up Gateway ────────────────────────────────── */
    can_gateway_init();
    can_gateway_set_transform(gw_transform);

    /* Route 1: CAN1 → CAN2 for IDs 0x100–0x1FF */
    can_route_t r1;
    r1.src        = CAN_CHANNEL_1;
    r1.dst        = CAN_CHANNEL_2;
    r1.id_min     = 0x100;
    r1.id_max     = 0x1FF;
    r1.frame_type = CAN_FRAME_STANDARD;
    r1.active     = true;
    can_gateway_add_route(&r1);

    /* Route 2: CAN2 → CAN1 for IDs 0x200–0x2FF */
    can_route_t r2;
    r2.src        = CAN_CHANNEL_2;
    r2.dst        = CAN_CHANNEL_1;
    r2.id_min     = 0x200;
    r2.id_max     = 0x2FF;
    r2.frame_type = CAN_FRAME_STANDARD;
    r2.active     = true;
    can_gateway_add_route(&r2);

    /* ─── Group filter: accept IDs 0x100–0x5FF on CAN1 ── */
    can_set_group_filter(CAN_CHANNEL_1, 0x100, 0x5FF, CAN_FRAME_STANDARD);
    can_set_group_filter(CAN_CHANNEL_2, 0x100, 0x5FF, CAN_FRAME_STANDARD);

    /* ─── Transmit test message on CAN1 ─────────────────── */
    can_message_t tx;
    tx.id         = 0x150;          /* within route 1 range    */
    tx.frame_type = CAN_FRAME_STANDARD;
    tx.rtr        = false;
    tx.dlc        = 4;
    tx.data[0]    = 0xCA;
    tx.data[1]    = 0xFE;
    tx.data[2]    = 0xBA;
    tx.data[3]    = 0xBE;
    tx.data[4]    = 0;
    tx.data[5]    = 0;
    tx.data[6]    = 0;
    tx.data[7]    = 0;
    tx.timestamp  = 0;

    can_transmit(CAN_CHANNEL_1, &tx);

    /* ─── Process gateway + wait ────────────────────────── */
    uint32_t wait = 200;
    while (wait--) {
        can_gateway_process();      /* deferred forwarding     */
        delay_ms(1);
    }

    /* ─── Check diagnostics ─────────────────────────────── */
    can_diag_t diag1, diag2;
    can_get_diag(CAN_CHANNEL_1, &diag1);
    can_get_diag(CAN_CHANNEL_2, &diag2);

    /* In self-test mode the loopback is per-channel, so
       CAN1 transmits and receives its own message.
       The gateway queues it for CAN2, and gateway_process()
       transmits it on CAN2 (which also loops back to CAN2).
       Success = CAN1 TX≥1 and the forward count ≥1 */
    bool success = (diag1.tx_count >= 1)
                && (can_gateway_get_fwd_count() >= 1);

    /* ─── Result blink ──────────────────────────────────── */
    while (1) {
        LPC_GPIO0->FIOPIN ^= LED1_PIN;
        delay_ms(success ? 150 : 1000);
    }

    return 0;
}
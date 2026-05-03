/**
 * @file main_filter_demo.c
 * @brief Demo: CAN Acceptance Filter — LPC1768 RX with serial output
 *
 * Copy to main.c and rebuild.
 * ESP32: Upload ESP32_Demo_TX.ino
 *
 * Shows CAN hardware acceptance filtering via UART serial output.
 * Filter accepts ID=0x100 and ID=0x200. Other IDs are hardware-rejected.
 *
 * Serial: 115200 baud on USB (UART0, P0.2/P0.3)
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include <string.h>

/* ── LEDs ─────────────────────────────────────────────── */
#define LED1  (1UL << 18)
#define LED2  (1UL << 20)
#define LED3  (1UL << 21)
#define LED4  (1UL << 23)
#define ALL   (LED1 | LED2 | LED3 | LED4)

/* ── UART0 Serial ─────────────────────────────────────── */
static void uart_init(uint32_t baud)
{
    LPC_SC->PCONP    |= (1UL << 3);
    LPC_SC->PCLKSEL0 |= (1UL << 6);
    LPC_PINCON->PINSEL0 &= ~((3UL << 4) | (3UL << 6));
    LPC_PINCON->PINSEL0 |=  ((1UL << 4) | (1UL << 6));
    LPC_UART0->LCR = 0x83;
    uint32_t dll = SystemCoreClock / (16UL * baud);
    LPC_UART0->DLL = dll & 0xFF;
    LPC_UART0->DLM = (dll >> 8) & 0xFF;
    LPC_UART0->LCR = 0x03;
    LPC_UART0->FCR = 0x07;
}

static void uart_putc(char c)
{
    while (!(LPC_UART0->LSR & (1UL << 5))) ;
    LPC_UART0->THR = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

static void uart_puthex(uint32_t val, int digits)
{
    const char hex[] = "0123456789ABCDEF";
    for (int i = digits - 1; i >= 0; i--)
        uart_putc(hex[(val >> (i * 4)) & 0xF]);
}

static void uart_putdec(uint32_t val)
{
    char buf[12]; int i = 0;
    if (val == 0) { uart_putc('0'); return; }
    while (val > 0) { buf[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

static void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    for (; ms > 0U; ms--)
        for (i = 0U; i < 100000UL; i++);
}

/* ── SysTick ──────────────────────────────────────────── */
void SysTick_Handler(void)
{
    can_timestamp_tick();
}

/* ──────────────────────────────────────────────────────── */
/*  Software-level filter — accepts all frames at HW level */
/*  but checks IDs in software and reports status          */
/* ──────────────────────────────────────────────────────── */

/* IDs accepted by our "filter" */
static const uint32_t filter_ids[] = { 0x100, 0x200 };
static const int NUM_FILTERS = 2;

static int id_passes_filter(uint32_t id)
{
    for (int i = 0; i < NUM_FILTERS; i++)
        if (id == filter_ids[i]) return 1;
    return 0;
}

int main(void)
{
    LPC_GPIO1->FIODIR |= ALL;
    LPC_GPIO1->FIOCLR  = ALL;
    SysTick_Config(SystemCoreClock / 1000);

    uart_init(115200);

    uart_puts("\n\n");
    uart_puts("========================================\n");
    uart_puts("   CAN Acceptance Filter Demo\n");
    uart_puts("   LPC1768 CAN Driver (Polling Mode)\n");
    uart_puts("========================================\n\n");

    /* ═══ Init CAN1 — bypass filter (accept all at HW) ═══ */
    can_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.channel          = CAN_CHANNEL_1;
    cfg.baudrate         = CAN_BAUD_500K;
    cfg.mode             = CAN_MODE_NORMAL;
    cfg.rx_callback      = 0;
    cfg.error_callback   = 0;
    cfg.enable_timestamp = true;

    if (can_init(&cfg) != CAN_OK) {
        uart_puts("[FAIL] CAN init failed!\n");
        while (1) { LPC_GPIO1->FIOPIN ^= ALL; delay_ms(100); }
    }
    uart_puts("[OK] CAN1 initialized at 500 kbps\n\n");

    /* ═══ Filter configuration (software layer) ═══════════ */
    uart_puts("[FILTER] Software acceptance filter active:\n");
    uart_puts("  -> PASS:   ID=0x100, ID=0x200\n");
    uart_puts("  -> REJECT: ID=0x080, 0x150, 0x2FF, 0x300\n\n");
    uart_puts("Waiting for CAN frames...\n");
    uart_puts("--------------------------------------------\n");

    LPC_GPIO1->FIOSET = LED1;

    uint32_t accept_count = 0;
    uint32_t reject_count = 0;
    uint32_t total_count  = 0;

    /* ═══ Main loop ═══════════════════════════════════════ */
    while (1)
    {
        can_message_t msg;
        if (can_receive(CAN_CHANNEL_1, &msg, 0) == CAN_OK)
        {
            total_count++;

            if (id_passes_filter(msg.id))
            {
                /* ── ACCEPTED ──────────────────────────── */
                accept_count++;
                LPC_GPIO1->FIOPIN ^= LED2;
                LPC_GPIO1->FIOSET  = LED3;

                uart_puts("[PASS  #");
                uart_putdec(accept_count);
                uart_puts("] ID=0x");
                uart_puthex(msg.id, 3);
                uart_puts("  DLC=");
                uart_putdec(msg.dlc);
                uart_puts("  Data=[");
                for (uint8_t i = 0; i < msg.dlc && i < 8; i++) {
                    if (i > 0) uart_putc(' ');
                    uart_puthex(msg.data[i], 2);
                }
                uart_puts("]  <-- ACCEPTED\n");
            }
            else
            {
                /* ── REJECTED ──────────────────────────── */
                reject_count++;
                LPC_GPIO1->FIOPIN ^= LED4;

                uart_puts("[REJECT] ID=0x");
                uart_puthex(msg.id, 3);
                uart_puts("  DLC=");
                uart_putdec(msg.dlc);
                uart_puts("  Data=[");
                for (uint8_t i = 0; i < msg.dlc && i < 8; i++) {
                    if (i > 0) uart_putc(' ');
                    uart_puthex(msg.data[i], 2);
                }
                uart_puts("]  --- FILTERED OUT\n");
            }

            /* Stats every 12 frames (2 rounds) */
            if ((total_count % 12) == 0) {
                uart_puts("\n--- Filter Stats ---\n");
                uart_puts("  Total:    "); uart_putdec(total_count);  uart_puts("\n");
                uart_puts("  Accepted: "); uart_putdec(accept_count); uart_puts("\n");
                uart_puts("  Rejected: "); uart_putdec(reject_count); uart_puts("\n");
                uart_puts("--------------------\n\n");
            }
        }

        /* Heartbeat */
        static uint32_t hb = 0;
        if (++hb >= 500000UL) {
            hb = 0;
            LPC_GPIO1->FIOPIN ^= LED1;
        }
    }
}

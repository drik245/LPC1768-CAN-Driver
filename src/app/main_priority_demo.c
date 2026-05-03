/**
 * @file main_priority_demo.c
 * @brief Demo: CAN Priority / Arbitration — LPC1768 RX with serial output
 *
 * Copy to main.c and rebuild.
 * ESP32: Upload ESP32_Demo_TX.ino
 *
 * Shows CAN priority: lower ID = higher priority.
 * Accepts all frames (bypass filter) and tags them with priority level.
 * Tracks statistics: frame counts per ID, reception order.
 *
 * Serial: 115200 baud on USB (UART0, P0.2/P0.3)
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include <string.h>
#include <stdio.h>

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
    char buf[12];
    int i = 0;
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

/* ── Priority label ───────────────────────────────────── */
static const char* get_priority(uint32_t id)
{
    if (id <= 0x0FF) return "CRITICAL";
    if (id <= 0x1FF) return "HIGH    ";
    if (id <= 0x2FF) return "MEDIUM  ";
    if (id <= 0x3FF) return "LOW     ";
    return "LOWEST  ";
}

static const char* get_prio_bar(uint32_t id)
{
    if (id <= 0x0FF) return "█████";
    if (id <= 0x1FF) return "████ ";
    if (id <= 0x2FF) return "███  ";
    if (id <= 0x3FF) return "██   ";
    return "█    ";
}

/* ── SysTick ──────────────────────────────────────────── */
void SysTick_Handler(void)
{
    can_timestamp_tick();
}

int main(void)
{
    LPC_GPIO1->FIODIR |= ALL;
    LPC_GPIO1->FIOCLR  = ALL;
    SysTick_Config(SystemCoreClock / 1000);

    uart_init(115200);

    uart_puts("\n\n");
    uart_puts("========================================\n");
    uart_puts("   CAN Priority / Arbitration Demo\n");
    uart_puts("   LPC1768 CAN Driver (Polling Mode)\n");
    uart_puts("========================================\n\n");
    uart_puts("  CAN Priority Rule: Lower ID = Higher Priority\n");
    uart_puts("  ┌─────────────┬────────────┬────────┐\n");
    uart_puts("  │ ID Range    │ Priority   │ Bar    │\n");
    uart_puts("  ├─────────────┼────────────┼────────┤\n");
    uart_puts("  │ 0x000-0x0FF │ CRITICAL   │ █████  │\n");
    uart_puts("  │ 0x100-0x1FF │ HIGH       │ ████   │\n");
    uart_puts("  │ 0x200-0x2FF │ MEDIUM     │ ███    │\n");
    uart_puts("  │ 0x300-0x3FF │ LOW        │ ██     │\n");
    uart_puts("  │ 0x400+      │ LOWEST     │ █      │\n");
    uart_puts("  └─────────────┴────────────┴────────┘\n\n");

    /* ═══ Init CAN1 ═══════════════════════════════════════ */
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
    uart_puts("[OK] CAN1 initialized at 500 kbps (accept all)\n");
    uart_puts("Waiting for CAN frames...\n");
    uart_puts("────────────────────────────────────────────────────\n");

    LPC_GPIO1->FIOSET = LED1;

    uint32_t rx_count = 0;
    uint32_t stat_crit = 0, stat_high = 0, stat_med = 0;
    uint32_t stat_low = 0, stat_lowest = 0;

    /* ═══ Main loop ═══════════════════════════════════════ */
    while (1)
    {
        can_message_t msg;
        if (can_receive(CAN_CHANNEL_1, &msg, 0) == CAN_OK)
        {
            rx_count++;
            LPC_GPIO1->FIOPIN ^= LED2;
            LPC_GPIO1->FIOSET  = LED3;

            /* Update stats */
            if (msg.id <= 0x0FF) stat_crit++;
            else if (msg.id <= 0x1FF) stat_high++;
            else if (msg.id <= 0x2FF) stat_med++;
            else if (msg.id <= 0x3FF) stat_low++;
            else stat_lowest++;

            /* Print frame with priority */
            uart_puts("[RX #");
            uart_putdec(rx_count);
            uart_puts("] ID=0x");
            uart_puthex(msg.id, 3);
            uart_puts("  Prio=");
            uart_puts(get_priority(msg.id));
            uart_puts("  ");
            uart_puts(get_prio_bar(msg.id));
            uart_puts("  DLC=");
            uart_putdec(msg.dlc);
            uart_puts("  [");
            for (uint8_t i = 0; i < msg.dlc && i < 8; i++) {
                if (i > 0) uart_putc(' ');
                uart_puthex(msg.data[i], 2);
            }
            uart_puts("]\n");

            /* Print stats every 10 frames */
            if ((rx_count % 10) == 0) {
                uart_puts("\n--- Stats after ");
                uart_putdec(rx_count);
                uart_puts(" frames ---\n");
                uart_puts("  CRITICAL: "); uart_putdec(stat_crit);  uart_puts("\n");
                uart_puts("  HIGH:     "); uart_putdec(stat_high);  uart_puts("\n");
                uart_puts("  MEDIUM:   "); uart_putdec(stat_med);   uart_puts("\n");
                uart_puts("  LOW:      "); uart_putdec(stat_low);   uart_puts("\n");
                uart_puts("  LOWEST:   "); uart_putdec(stat_lowest); uart_puts("\n");
                uart_puts("----------------------------\n\n");
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

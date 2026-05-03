/**
 * @file main_filter_demo.c
 * @brief Demo: CAN Acceptance Filter — LPC1768 RX with serial output
 *
 * Copy to main.c and rebuild.
 * ESP32: Upload ESP32_Demo_TX.ino
 *
 * Shows CAN hardware acceptance filtering via UART serial output.
 * Filter is set to accept only ID=0x100 and ID=0x200.
 * ESP32 sends frames with IDs: 0x080, 0x100, 0x150, 0x200, 0x2FF, 0x300
 * Only 0x100 and 0x200 should appear on LPC serial monitor.
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

/* ── UART0 Serial (USB debug port) ────────────────────── */
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

    /* UART serial init */
    uart_init(115200);

    uart_puts("\n\n");
    uart_puts("========================================\n");
    uart_puts("   CAN Acceptance Filter Demo\n");
    uart_puts("   LPC1768 CAN Driver (Polling Mode)\n");
    uart_puts("========================================\n\n");

    /* ═══ Init CAN1 via driver API ════════════════════════ */
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
    uart_puts("[OK] CAN1 initialized at 500 kbps\n");

    /* ═══ Set hardware acceptance filters ═════════════════ */
    uart_puts("[FILTER] Setting acceptance filter...\n");

    uart_puts("  -> Accept ID=0x100\n");
    can_set_filter(CAN_CHANNEL_1, 0x100, 0x7FF, CAN_FRAME_STANDARD);

    uart_puts("  -> Accept ID=0x200\n");
    can_set_filter(CAN_CHANNEL_1, 0x200, 0x7FF, CAN_FRAME_STANDARD);

    /* Flush stale frames from bypass window + clear errors */
    while (LPC_CAN1->GSR & (1UL << 0))
        LPC_CAN1->CMR = (1UL << 2);
    if (LPC_CAN1->GSR & (1UL << 1))
        LPC_CAN1->CMR = (1UL << 3);
    (void)LPC_CAN1->ICR;

    uart_puts("[FILTER] Active: only ID=0x100, 0x200 pass\n\n");

    /* ═══ CANAF Register Dump (debug) ═════════════════════ */
    uart_puts("[DEBUG] CANAF Registers:\n");
    uart_puts("  AFMR       = 0x"); uart_puthex(LPC_CANAF->AFMR, 8); uart_putc('\n');
    uart_puts("  SFF_sa     = 0x"); uart_puthex(LPC_CANAF->SFF_sa, 8); uart_putc('\n');
    uart_puts("  SFF_GRP_sa = 0x"); uart_puthex(LPC_CANAF->SFF_GRP_sa, 8); uart_putc('\n');
    uart_puts("  EFF_sa     = 0x"); uart_puthex(LPC_CANAF->EFF_sa, 8); uart_putc('\n');
    uart_puts("  EFF_GRP_sa = 0x"); uart_puthex(LPC_CANAF->EFF_GRP_sa, 8); uart_putc('\n');
    uart_puts("  ENDofTable = 0x"); uart_puthex(LPC_CANAF->ENDofTable, 8); uart_putc('\n');
    uart_puts("  AF_RAM[0]  = 0x"); uart_puthex(LPC_CANAF_RAM->mask[0], 8); uart_putc('\n');
    uart_puts("  CAN1->MOD  = 0x"); uart_puthex(LPC_CAN1->MOD, 8); uart_putc('\n');
    uart_puts("  CAN1->GSR  = 0x"); uart_puthex(LPC_CAN1->GSR, 8); uart_putc('\n');
    uart_puts("\n");

    uart_puts("Waiting for CAN frames...\n");
    uart_puts("----------------------------------------\n");

    LPC_GPIO1->FIOSET = LED1;

    uint32_t rx_count = 0;

    /* ═══ Main loop — receive and print ═══════════════════ */
    while (1)
    {
        can_message_t msg;
        if (can_receive(CAN_CHANNEL_1, &msg, 0) == CAN_OK)
        {
            rx_count++;
            LPC_GPIO1->FIOPIN ^= LED2;
            LPC_GPIO1->FIOSET  = LED3;

            /* Print received frame */
            uart_puts("[RX #");
            uart_putdec(rx_count);
            uart_puts("] ID=0x");
            uart_puthex(msg.id, 3);
            uart_puts("  DLC=");
            uart_putdec(msg.dlc);
            uart_puts("  Data=[");
            for (uint8_t i = 0; i < msg.dlc && i < 8; i++) {
                if (i > 0) uart_putc(' ');
                uart_puthex(msg.data[i], 2);
            }
            uart_puts("]  <-- ACCEPTED by filter\n");
        }

        /* Heartbeat blink */
        static uint32_t hb = 0;
        if (++hb >= 500000UL) {
            hb = 0;
            LPC_GPIO1->FIOPIN ^= LED1;
        }
    }
}

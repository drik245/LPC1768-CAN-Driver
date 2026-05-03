/**
 * LPC1768_CAN_Transmitter.c
 * CAN1 transmitter @ 500kbps
 * Sends 8-byte frames with ID=0x100 every 500ms
 *
 * Pin connections:
 *   P0.0 (RD1) ? SN65HVD230 RXD
 *   P0.1 (TD1) ? SN65HVD230 TXD
 *a
 * Toolchain: Keil MDK or GCC ARM
 * CMSIS: LPC17xx (SystemInit sets CCLK=100MHz, PCLK=25MHz)
 */

#include "LPC17xx.h"
#include <stdint.h>
#include <string.h>

/* ---------------------------------------------------
 *  BIT TIMING � 500kbps @ PCLK=25MHz
 *
 *  BRP=4  ? Prescaler=5 ? tq = 5/25MHz = 200ns
 *  SYNC_SEG = 1 tq  (fixed)
 *  TSEG1   = 7 tq  (register value = 6, i.e., 6+1=7)
 *  TSEG2   = 2 tq  (register value = 1, i.e., 1+1=2)
 *  Total   = 1+7+2 = 10 tq
 *  Bit time = 10 � 200ns = 2000ns = 1/500kHz ?
 *  Sample point = (1+7)/10 = 80% ?
 *
 *  BTR Register:
 *    BRP[9:0]   = 4        (bits  9:0)
 *    SJW[15:14] = 0        (bits 15:14, SJW=1)
 *    TSEG1[19:16] = 6      (bits 19:16)
 *    TSEG2[22:20] = 1      (bits 22:20)
 *  ? 0x00160004
 * --------------------------------------------------- */
#define CAN_BTR_500KBPS    0x00160004UL

/* LED on P1.18 (onboard LED on LPC1768 mbed board) */
#define LED_PIN            (1UL << 18)

/* CAN Frame ID for our messages */
#define CAN_TX_ID          0x100UL

/* ---------------------------------------------------- */
/*  Low-level delay (blocking, for bring-up only)       */
/*  For production: use SysTick timer                   */
/* ---------------------------------------------------- */
static void delay_ms(uint32_t ms)
{
    /* At 100MHz CCLK, each iteration � 1 cycle.
     * 100,000 iterations � 1ms. Trim as needed.       */
    volatile uint32_t i;
    for (; ms > 0U; ms--)
        for (i = 0U; i < 100000UL; i++);
}

/* ---------------------------------------------------- */
/*  CAN1 Initialisation                                 */
/* ---------------------------------------------------- */
void CAN1_Init(void)
{
    /* -- Step 1: Power on CAN1 peripheral ------------ */
    LPC_SC->PCONP |= (1UL << 13);   /* PCAN1 enable bit */

    /* -- Step 2: Set PCLK_CAN1 = CCLK/4 = 25MHz ----- */
    /* PCLKSEL0[27:26] = 00 ? CCLK/4 (default, but set explicitly) */
    LPC_SC->PCLKSEL0 &= ~(3UL << 26);

    /* -- Step 3: Configure CAN1 pins on P0.0 and P0.1 */
    /* PINSEL0[1:0] = 01 ? P0.0 = RD1 (CAN1 Receive)  */
    /* PINSEL0[3:2] = 01 ? P0.1 = TD1 (CAN1 Transmit) */
    LPC_PINCON->PINSEL0 &= ~(0xFUL << 0);   /* Clear P0.0 and P0.1 */
    LPC_PINCON->PINSEL0 |=  (0x5UL << 0);   /* 0101b = RD1 | TD1   */

    /* -- Step 4: Enter Reset Mode (required for BTR config) */
    LPC_CAN1->MOD = 0x01UL;

    /* -- Step 5: Clear error counters and flags ------- */
    LPC_CAN1->GSR = 0;

    /* -- Step 6: Set Bus Timing Register ------------ */
    LPC_CAN1->BTR = CAN_BTR_500KBPS;

    /* -- Step 7: Configure Acceptance Filter ---------- */
    /* AFMR=0x02 ? Bypass mode: accept ALL incoming IDs */
    LPC_CANAF->AFMR = 0x02UL;

    /* -- Step 8: Exit Reset Mode ? enter Normal Mode -- */
    LPC_CAN1->MOD = 0x00UL;

    /* -- Step 9: Brief settle time ------------------- */
    delay_ms(10);
}

/* ---------------------------------------------------- */
/*  CAN1 Transmit Function                              */
/*  id  : 11-bit Standard CAN ID (0x000 � 0x7FF)       */
/*  data: pointer to data bytes                         */
/*  len : number of bytes (0�8)                         */
/*  returns: 1=success, 0=timeout/error                 */
/* ---------------------------------------------------- */
uint8_t CAN1_Transmit(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint32_t TFI, TDA, TDB;
    uint32_t timeout = 1000000UL;

    /* -- Clamp DLC to max 8 -- */
    if (len > 8U) len = 8U;

    /* -- Wait for Transmit Buffer 1 to be free -------- */
    /* SR bit 2 (TBS1): 1 = buffer released (ready)     */
    while (!(LPC_CAN1->SR & (1UL << 2)) && timeout--)
        ;

    if (timeout == 0UL)
        return 0U; /* Timeout � bus may be in error */

    /* -- Build Frame Information Word (TFI1) ----------
     * Bits  [7:0]  : PRIO (priority, unused here)
     * Bits [19:16] : DLC  (Data Length Code)
     * Bit  [30]    : FF  = 0 (Standard 11-bit ID)
     * Bit  [31]    : RTR = 0 (Data frame, not Remote)
     * All other bits: 0                                 */
    TFI = (uint32_t)((len & 0x0FUL)<<16);   /* DLC, standard frame, data frame */

    /* -- Pack data bytes (little-endian) --------------- */
    TDA = 0UL;
    TDB = 0UL;

    if (len > 0U) TDA |= ((uint32_t)data[0] <<  0);
    if (len > 1U) TDA |= ((uint32_t)data[1] <<  8);
    if (len > 2U) TDA |= ((uint32_t)data[2] << 16);
    if (len > 3U) TDA |= ((uint32_t)data[3] << 24);
    if (len > 4U) TDB |= ((uint32_t)data[4] <<  0);
    if (len > 5U) TDB |= ((uint32_t)data[5] <<  8);
    if (len > 6U) TDB |= ((uint32_t)data[6] << 16);
    if (len > 7U) TDB |= ((uint32_t)data[7] << 24);

    /* -- Write to hardware Transmit Buffer 1 ------------ */
    LPC_CAN1->TFI1 = TFI;
    LPC_CAN1->TID1 = id & 0x7FFUL;   /* Only lower 11 bits valid */
    LPC_CAN1->TDA1 = TDA;
    LPC_CAN1->TDB1 = TDB;

    /* -- Issue Transmit Request via Buffer 1 ------------
     * CMR bit 0 (TR)   : Transmission Request
     * CMR bit 5 (STB1) : Select Transmit Buffer 1
     * Value = 0x21                                       */
    LPC_CAN1->CMR = 0x21UL;

    return 1U; /* Transmission initiated */
}

/* ---------------------------------------------------- */
/*  CAN Status / Error Check                            */
/*  Returns GSR (Global Status Register) for debug      */
/* ---------------------------------------------------- */
uint32_t CAN1_GetStatus(void)
{
    return LPC_CAN1->GSR;
}

uint8_t CAN1_IsError(void)
{
    uint32_t gsr = LPC_CAN1->GSR;
    /* Bit 6 (BS) = Bus-Off, Bit 5 (ES) = Error Status */
    return (uint8_t)((gsr >> 5) & 0x03UL);
}

/* ---------------------------------------------------- */
/*  LED Helpers                                         */
/* ---------------------------------------------------- */
static void LED_Init(void)
{
    LPC_GPIO1->FIODIR |= LED_PIN;   /* Output */
    LPC_GPIO1->FIOCLR  = LED_PIN;   /* OFF    */
}

static void LED_Toggle(void)
{
    LPC_GPIO1->FIOPIN ^= LED_PIN;
}

static void LED_On(void)  { LPC_GPIO1->FIOSET = LED_PIN; }
static void LED_Off(void) { LPC_GPIO1->FIOCLR = LED_PIN; }

/* ---------------------------------------------------- */
/*  Main Application                                    */
/* ---------------------------------------------------- */
int main(void)
{
    /* SystemInit() called by startup code sets:
     * CCLK = 100MHz, PCLK = 25MHz                      */

    LED_Init();
    CAN1_Init();

    /* Blink 3� to indicate boot success */
    for (int i = 0; i < 3; i++) {
        LED_On();  delay_ms(100);
        LED_Off(); delay_ms(100);
    }

    uint8_t msg_counter = 0;
    uint8_t can_data[8];

    while (1)
    {
        /* -- Build payload -----------------------------
         * Byte 0-1 : Magic header 0xAB, 0xCD
         * Byte 2   : Message counter (increments each send)
         * Byte 3   : Status byte (0x01 = normal)
         * Byte 4-5 : Simulated sensor value (16-bit, MSB first)
         * Byte 6-7 : Checksum placeholder                */
        uint16_t sensor_value = (uint16_t)(msg_counter * 13 + 200); /* Fake sensor */

        can_data[0] = 0xABU;
        can_data[1] = 0xCDU;
        can_data[2] = msg_counter;
        can_data[3] = 0x01U;                          /* Status: OK   */
        can_data[4] = (uint8_t)(sensor_value >> 8);   /* High byte    */
        can_data[5] = (uint8_t)(sensor_value & 0xFF); /* Low byte     */
        can_data[6] = (uint8_t)(can_data[2] ^ can_data[4]); /* XOR CRC */
        can_data[7] = 0xFFU;                          /* End marker   */

        /* -- Transmit ----------------------------------- */
        if (CAN1_Transmit(CAN_TX_ID, can_data, 8U))
        {
            LED_Toggle(); /* Blink on each successful transmit */
        }
        else
        {
            /* Transmission failed � fast blink error pattern */
            for (int e = 0; e < 5; e++) {
                LED_On();  delay_ms(50);
                LED_Off(); delay_ms(50);
            }

            /* Optional: Re-initialize CAN if bus-off */
            if (CAN1_IsError() & 0x02U) {
                CAN1_Init();
            }
        }

        msg_counter++;
        delay_ms(500); /* Transmit every 500ms */
    }

    return 0; /* Never reached */
}
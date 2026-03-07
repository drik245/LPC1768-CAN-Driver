/**
 * @file can_hal.c
 * @brief Hardware Abstraction Layer — clock, pin-mux, NVIC for LPC1768 CAN
 */

#include "LPC17xx.h"
#include "can_hal.h"
#include "can_reg.h"

/* ──────────────────────────────────────────────────────────── */
/*  Clock / Power                                              */
/* ──────────────────────────────────────────────────────────── */

int can_hal_init_clock(can_channel_t channel)
{
    /* Enable power for the requested CAN controller */
    if (channel == CAN_CHANNEL_1) {
        LPC_SC->PCONP |= PCONP_PCCAN1;
    } else if (channel == CAN_CHANNEL_2) {
        LPC_SC->PCONP |= PCONP_PCCAN2;
    } else {
        return CAN_ERR_INVALID_PARAM;
    }

    /*
     * Set PCLK for both CAN controllers to CCLK (100 MHz).
     * LPC1768 REQUIRES both CAN PCLK dividers to be identical.
     * PCLKSEL0 bits: 01 = CCLK, applied to both CAN1 & CAN2.
     */
    LPC_SC->PCLKSEL0 &= ~(3U << PCLKSEL0_CAN1_SHIFT);
    LPC_SC->PCLKSEL0 |=  (1U << PCLKSEL0_CAN1_SHIFT);

    LPC_SC->PCLKSEL0 &= ~(3U << PCLKSEL0_CAN2_SHIFT);
    LPC_SC->PCLKSEL0 |=  (1U << PCLKSEL0_CAN2_SHIFT);

    return CAN_OK;
}

void can_hal_deinit_clock(can_channel_t channel)
{
    if (channel == CAN_CHANNEL_1)
        LPC_SC->PCONP &= ~PCONP_PCCAN1;
    else
        LPC_SC->PCONP &= ~PCONP_PCCAN2;
}

/* ──────────────────────────────────────────────────────────── */
/*  Pin Mux                                                    */
/* ──────────────────────────────────────────────────────────── */

int can_hal_init_pins(can_channel_t channel)
{
    if (channel == CAN_CHANNEL_1) {
        /* P0.0 = CAN1 RD1  (PINSEL0 bits [1:0] = 01) */
        LPC_PINCON->PINSEL0 &= ~(3U << PINSEL_CAN1_RD_SHIFT);
        LPC_PINCON->PINSEL0 |=  (PINSEL_CAN1_RD_FUNC << PINSEL_CAN1_RD_SHIFT);

        /* P0.1 = CAN1 TD1  (PINSEL0 bits [3:2] = 01) */
        LPC_PINCON->PINSEL0 &= ~(3U << PINSEL_CAN1_TD_SHIFT);
        LPC_PINCON->PINSEL0 |=  (PINSEL_CAN1_TD_FUNC << PINSEL_CAN1_TD_SHIFT);

    } else if (channel == CAN_CHANNEL_2) {
        /* P0.4 = CAN2 RD2  (PINSEL0 bits [9:8] = 10) */
        LPC_PINCON->PINSEL0 &= ~(3U << PINSEL_CAN2_RD_SHIFT);
        LPC_PINCON->PINSEL0 |=  (PINSEL_CAN2_RD_FUNC << PINSEL_CAN2_RD_SHIFT);

        /* P0.5 = CAN2 TD2  (PINSEL0 bits [11:10] = 10) */
        LPC_PINCON->PINSEL0 &= ~(3U << PINSEL_CAN2_TD_SHIFT);
        LPC_PINCON->PINSEL0 |=  (PINSEL_CAN2_TD_FUNC << PINSEL_CAN2_TD_SHIFT);

    } else {
        return CAN_ERR_INVALID_PARAM;
    }

    return CAN_OK;
}

/* ──────────────────────────────────────────────────────────── */
/*  NVIC — shared CAN IRQ (IRQn = 25)                         */
/* ──────────────────────────────────────────────────────────── */

void can_hal_enable_irq(void)
{
    NVIC_EnableIRQ(CAN_IRQn);
}

void can_hal_disable_irq(void)
{
    NVIC_DisableIRQ(CAN_IRQn);
}

/* ──────────────────────────────────────────────────────────── */
/*  Peripheral Clock Query                                     */
/* ──────────────────────────────────────────────────────────── */

uint32_t can_hal_get_pclk(void)
{
    /* We always configure PCLK_CAN = CCLK in can_hal_init_clock() */
    return SystemCoreClock;       /* 100 MHz */
}

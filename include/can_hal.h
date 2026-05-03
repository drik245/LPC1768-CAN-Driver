/**
 * @file can_hal.h
 * @brief Hardware Abstraction Layer for LPC1768 CAN peripheral
 */

#ifndef CAN_HAL_H
#define CAN_HAL_H

#include "can_driver.h"

/**
 * @brief Enable power and set peripheral clock for CAN channel.
 *        Both CAN1 and CAN2 PCLK are set to CCLK/4 (25 MHz).
 */
int can_hal_init_clock(can_channel_t channel);

/**
 * @brief Disable power for a CAN channel.
 */
void can_hal_deinit_clock(can_channel_t channel);

/**
 * @brief Configure GPIO pin mux for CAN TX/RX pins.
 *        CAN1: P0.0 (RD1), P0.1 (TD1)
 *        CAN2: P0.4 (RD2), P0.5 (TD2)
 */
int can_hal_init_pins(can_channel_t channel);

/**
 * @brief Enable CAN IRQ in NVIC (shared IRQ for both channels).
 */
void can_hal_enable_irq(void);

/**
 * @brief Disable CAN IRQ in NVIC.
 */
void can_hal_disable_irq(void);

/**
 * @brief Get the peripheral clock frequency for CAN (Hz).
 */
uint32_t can_hal_get_pclk(void);

#endif /* CAN_HAL_H */

/**
 * @file can_driver.h
 * @brief CAN Driver API for LPC1768
 * @author Your Name
 * @date 2026
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "can_error.h"

/* -- Channel ----------------------------------------------- */
typedef enum {
    CAN_CHANNEL_1 = 0,
    CAN_CHANNEL_2 = 1
} can_channel_t;

/* -- Modes ------------------------------------------------- */
typedef enum {
    CAN_MODE_NORMAL   = 0x00,
    CAN_MODE_LISTEN   = 0x01,
    CAN_MODE_SELFTEST = 0x02,
    CAN_MODE_RESET    = 0x03
} can_mode_t;

/* -- Baud Rates -------------------------------------------- */
typedef enum {
    CAN_BAUD_125K =  125000,
    CAN_BAUD_250K =  250000,
    CAN_BAUD_500K =  500000,
    CAN_BAUD_1M   = 1000000
} can_baudrate_t;

/* -- Frame Type -------------------------------------------- */
typedef enum {
    CAN_FRAME_STANDARD = 0,   /**< 11-bit identifier */
    CAN_FRAME_EXTENDED = 1    /**< 29-bit identifier */
} can_frame_type_t;

/* -- Message Structure ------------------------------------- */
typedef struct {
    uint32_t         id;
    can_frame_type_t frame_type;
    bool             rtr;
    uint8_t          dlc;
    uint8_t          data[8];
    uint32_t         timestamp;
} can_message_t;

/* -- Status ------------------------------------------------ */
typedef struct {
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    bool    bus_off;
    bool    error_warning;
    bool    tx_pending;
    bool    rx_available;
} can_status_t;

/* -- Error Types ------------------------------------------- */
typedef enum {
    CAN_ERROR_NONE    = 0x00,
    CAN_ERROR_STUFF   = 0x01,
    CAN_ERROR_FORM    = 0x02,
    CAN_ERROR_ACK     = 0x03,
    CAN_ERROR_BIT1    = 0x04,
    CAN_ERROR_BIT0    = 0x05,
    CAN_ERROR_CRC     = 0x06,
    CAN_ERROR_BUS_OFF = 0x07
} can_error_t;

/* -- Callbacks --------------------------------------------- */
typedef void (*can_rx_callback_t)   (can_channel_t ch, can_message_t *msg);
typedef void (*can_error_callback_t)(can_channel_t ch, can_error_t error);

/* -- Config ------------------------------------------------ */
typedef struct {
    can_channel_t        channel;
    can_baudrate_t       baudrate;
    can_mode_t           mode;
    can_rx_callback_t    rx_callback;
    can_error_callback_t error_callback;
    bool                 enable_timestamp;
} can_config_t;

/* -- API (Week 2) ------------------------------------------ */
int can_init        (const can_config_t *config);
int can_deinit      (can_channel_t channel);
int can_transmit    (can_channel_t channel, const can_message_t *msg);
int can_receive     (can_channel_t channel, can_message_t *msg, uint32_t timeout_ms);
int can_set_filter  (can_channel_t channel, uint32_t id, uint32_t mask, can_frame_type_t ft);
int can_get_status  (can_channel_t channel, can_status_t *status);
int can_set_mode    (can_channel_t channel, can_mode_t mode);
int can_register_rx_callback   (can_channel_t ch, can_rx_callback_t cb);
int can_register_error_callback(can_channel_t ch, can_error_callback_t cb);

/** Call from your 1 ms timer / SysTick_Handler for timestamp support */
void can_timestamp_tick(void);

/* -- Diagnostics (Week 3) ---------------------------------- */
typedef struct {
    uint32_t    tx_count;           /**< Total frames transmitted      */
    uint32_t    rx_count;           /**< Total frames received         */
    uint32_t    tx_err_frames;      /**< TX errors detected            */
    uint32_t    rx_err_frames;      /**< RX errors detected            */
    uint32_t    bus_off_count;      /**< Bus-off events                */
    uint32_t    overrun_count;      /**< HW data-overrun events        */
    uint32_t    arb_lost_count;     /**< Arbitration-lost events       */
    can_error_t last_error;         /**< Most recent error type        */
    uint32_t    uptime_ms;          /**< Milliseconds since can_init   */
} can_diag_t;

int  can_get_diag   (can_channel_t ch, can_diag_t *diag);
void can_reset_diag (can_channel_t ch);

int can_set_group_filter(can_channel_t ch, uint32_t id_low,
                         uint32_t id_high, can_frame_type_t ft);

int can_clear_filters(can_channel_t ch);

int can_sleep(can_channel_t channel);
int can_wake (can_channel_t channel);

#endif /* CAN_DRIVER_H */
/**
 * @file can_buffer.h
 * @brief Thread-safe circular ring buffer for CAN messages
 */

#ifndef CAN_BUFFER_H
#define CAN_BUFFER_H

#include "can_driver.h"

#define CAN_TX_BUFFER_SIZE  16
#define CAN_RX_BUFFER_SIZE  32

typedef struct {
    can_message_t    messages[CAN_RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} can_rx_buffer_t;

typedef struct {
    can_message_t    messages[CAN_TX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} can_tx_buffer_t;

static inline bool can_buffer_is_empty(can_rx_buffer_t *buf) {
    return (buf->count == 0);
}

static inline bool can_buffer_is_full(can_rx_buffer_t *buf) {
    return (buf->count >= CAN_RX_BUFFER_SIZE);
}

static inline bool can_buffer_push(can_rx_buffer_t *buf, const can_message_t *msg) {
    if (can_buffer_is_full(buf)) return false;
    buf->messages[buf->head] = *msg;
    buf->head = (buf->head + 1) % CAN_RX_BUFFER_SIZE;
    buf->count++;
    return true;
}

static inline bool can_buffer_pop(can_rx_buffer_t *buf, can_message_t *msg) {
    if (can_buffer_is_empty(buf)) return false;
    *msg = buf->messages[buf->tail];
    buf->tail = (buf->tail + 1) % CAN_RX_BUFFER_SIZE;
    buf->count--;
    return true;
}

/* ── TX Buffer Operations ─────────────────────────────────── */

static inline bool can_tx_buffer_is_empty(can_tx_buffer_t *buf) {
    return (buf->count == 0);
}

static inline bool can_tx_buffer_is_full(can_tx_buffer_t *buf) {
    return (buf->count >= CAN_TX_BUFFER_SIZE);
}

static inline bool can_tx_buffer_push(can_tx_buffer_t *buf, const can_message_t *msg) {
    if (can_tx_buffer_is_full(buf)) return false;
    buf->messages[buf->head] = *msg;
    buf->head = (buf->head + 1) % CAN_TX_BUFFER_SIZE;
    buf->count++;
    return true;
}

static inline bool can_tx_buffer_pop(can_tx_buffer_t *buf, can_message_t *msg) {
    if (can_tx_buffer_is_empty(buf)) return false;
    *msg = buf->messages[buf->tail];
    buf->tail = (buf->tail + 1) % CAN_TX_BUFFER_SIZE;
    buf->count--;
    return true;
}

#endif /* CAN_BUFFER_H */
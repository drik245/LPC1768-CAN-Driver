/**
 * @file can_gateway.h
 * @brief CAN Gateway — message routing between CAN channels
 *
 * Routes received messages from one CAN channel to another,
 * with optional ID range filtering and data transformation.
 * Common pattern in automotive / industrial gateway ECUs.
 */

#ifndef CAN_GATEWAY_H
#define CAN_GATEWAY_H

#include "can_driver.h"

#define CAN_GW_MAX_ROUTES  16

/* -- Route definition -------------------------------------- */
typedef struct {
    can_channel_t    src;            /**< Source channel            */
    can_channel_t    dst;            /**< Destination channel       */
    uint32_t         id_min;         /**< Lowest ID to forward      */
    uint32_t         id_max;         /**< Highest ID to forward     */
    can_frame_type_t frame_type;     /**< SFF or EFF                */
    bool             active;         /**< Route is enabled          */
} can_route_t;

/* -- Callback for message transformation ------------------- */
/**
 * Optional hook called before forwarding.
 * Return true to allow forwarding, false to drop.
 * You may modify *msg in place (e.g. remap ID, scale data).
 */
typedef bool (*can_gw_transform_t)(const can_route_t *route,
                                    can_message_t *msg);

/* -- API --------------------------------------------------- */

/** Initialise the gateway (registers RX callbacks on both channels) */
int can_gateway_init(void);

/** Shut down gateway and deregister callbacks */
void can_gateway_deinit(void);

/** Add a routing rule. Returns route index or negative error. */
int can_gateway_add_route(const can_route_t *route);

/** Remove a route by index */
int can_gateway_remove_route(uint8_t index);

/** Set a global transform hook (called for every forwarded msg) */
void can_gateway_set_transform(can_gw_transform_t fn);

/** Get the number of messages forwarded so far */
uint32_t can_gateway_get_fwd_count(void);

/** Reset forwarded-message counter */
void can_gateway_reset_fwd_count(void);

/**
 * Process any pending messages in software.
 * Call from main loop if you prefer deferred (non-ISR) forwarding.
 * If callbacks are used, this is optional.
 */
void can_gateway_process(void);

#endif /* CAN_GATEWAY_H */

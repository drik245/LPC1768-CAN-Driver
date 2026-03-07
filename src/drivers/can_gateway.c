/**
 * @file can_gateway.c
 * @brief CAN Gateway — routes messages between CAN1 and CAN2
 *
 * Registers as an RX callback on both channels.  When a message
 * arrives, it checks every active route.  If the message ID falls
 * within [id_min .. id_max] of a matching route, the message is
 * forwarded to the destination channel (optionally transformed).
 */

#include "can_gateway.h"
#include "can_buffer.h"
#include <string.h>

/* ================================================================
 *  Internal State
 * ================================================================ */

static can_route_t        g_routes[CAN_GW_MAX_ROUTES];
static uint8_t            g_num_routes;
static can_gw_transform_t g_transform;
static volatile uint32_t  g_fwd_count;
static bool               g_gw_init;

/* Ring buffer for deferred forwarding (optional path) */
typedef struct {
    can_message_t msg;
    can_channel_t dst;
} gw_pending_t;

#define GW_PENDING_SIZE 16
static gw_pending_t      g_pending[GW_PENDING_SIZE];
static volatile uint8_t  g_pend_head;
static volatile uint8_t  g_pend_tail;
static volatile uint8_t  g_pend_count;

/* ================================================================
 *  Internal helpers
 * ================================================================ */

static void gw_pending_push(can_channel_t dst, const can_message_t *msg)
{
    if (g_pend_count >= GW_PENDING_SIZE) return;   /* drop if full */
    g_pending[g_pend_head].msg = *msg;
    g_pending[g_pend_head].dst = dst;
    g_pend_head = (g_pend_head + 1) % GW_PENDING_SIZE;
    g_pend_count++;
}

/** RX callback shared by both channels */
static void gw_rx_callback(can_channel_t ch, can_message_t *msg)
{
    uint8_t i;
    for (i = 0; i < g_num_routes; i++) {
        can_route_t *r = &g_routes[i];
        if (!r->active)          continue;
        if (r->src != ch)        continue;
        if (r->frame_type != msg->frame_type) continue;

        /* ID range check */
        if (msg->id < r->id_min || msg->id > r->id_max)
            continue;

        /* Make a copy so transform doesn't corrupt original */
        can_message_t fwd = *msg;

        /* Optional transform */
        if (g_transform) {
            if (!g_transform(r, &fwd))
                continue;   /* transform said "drop" */
        }

        /* Queue for deferred TX (safer than TX from ISR) */
        gw_pending_push(r->dst, &fwd);
        g_fwd_count++;
    }
}

/* ================================================================
 *  Public API
 * ================================================================ */

int can_gateway_init(void)
{
    memset(g_routes, 0, sizeof(g_routes));
    g_num_routes  = 0;
    g_transform   = 0;
    g_fwd_count   = 0;
    g_pend_head   = 0;
    g_pend_tail   = 0;
    g_pend_count  = 0;

    /* Register our callback on both channels */
    can_register_rx_callback(CAN_CHANNEL_1, gw_rx_callback);
    can_register_rx_callback(CAN_CHANNEL_2, gw_rx_callback);

    g_gw_init = true;
    return CAN_OK;
}

void can_gateway_deinit(void)
{
    can_register_rx_callback(CAN_CHANNEL_1, 0);
    can_register_rx_callback(CAN_CHANNEL_2, 0);
    g_gw_init = false;
}

int can_gateway_add_route(const can_route_t *route)
{
    if (!route)                         return CAN_ERR_INVALID_PARAM;
    if (g_num_routes >= CAN_GW_MAX_ROUTES) return CAN_ERR_FILTER_FULL;
    if (route->src == route->dst)       return CAN_ERR_INVALID_PARAM;

    g_routes[g_num_routes] = *route;
    g_routes[g_num_routes].active = true;
    return (int)g_num_routes++;
}

int can_gateway_remove_route(uint8_t index)
{
    if (index >= g_num_routes) return CAN_ERR_INVALID_PARAM;
    g_routes[index].active = false;
    return CAN_OK;
}

void can_gateway_set_transform(can_gw_transform_t fn)
{
    g_transform = fn;
}

uint32_t can_gateway_get_fwd_count(void)
{
    return g_fwd_count;
}

void can_gateway_reset_fwd_count(void)
{
    g_fwd_count = 0;
}

void can_gateway_process(void)
{
    while (g_pend_count > 0) {
        gw_pending_t *p = &g_pending[g_pend_tail];
        can_transmit(p->dst, &p->msg);
        g_pend_tail = (g_pend_tail + 1) % GW_PENDING_SIZE;
        g_pend_count--;
    }
}

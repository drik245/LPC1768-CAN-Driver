/**
 * @file can_driver.c
 * @brief CAN Driver implementation for LPC1768
 */

#include "LPC17xx.h"
#include "can_driver.h"
#include "can_buffer.h"
#include "can_hal.h"
#include "can_reg.h"
#include <string.h>

/*  Internal Constants */

#define CAN_NUM_CHANNELS     2
#define MAX_STD_FILTERS      16
#define MAX_EXT_FILTERS      8
#define MAX_STD_GRP_FILTERS  8
#define MAX_EXT_GRP_FILTERS  4
#define BUSOFF_RECOVERY_MS   500

/*  Internal State */

/** Diagnostic counters (per channel) */
typedef struct {
    uint32_t    tx_count;
    uint32_t    rx_count;
    uint32_t    tx_err_frames;
    uint32_t    rx_err_frames;
    uint32_t    bus_off_count;
    uint32_t    overrun_count;
    uint32_t    arb_lost_count;
    can_error_t last_error;
    uint32_t    init_tick;       /* g_tick at init time */
} can_diag_internal_t;

/** Per-channel runtime context */
typedef struct {
    bool                 initialized;
    bool                 timestamp_en;
    can_rx_callback_t    rx_cb;
    can_error_callback_t err_cb;
    can_rx_buffer_t      rx_buf;
    can_tx_buffer_t      tx_buf;
    can_diag_internal_t  diag;
    bool                 busoff_recovery;  /* auto-recovery armed     */
    uint32_t             busoff_tick;      /* tick when bus-off seen  */
} can_ctx_t;

/** Individual filter entry */
typedef struct {
    uint32_t id;
    uint8_t  channel;
    bool     active;
} can_filter_t;

/** Group (range) filter entry */
typedef struct {
    uint32_t id_low;
    uint32_t id_high;
    uint8_t  channel;
    bool     active;
} can_group_filter_t;

static can_ctx_t          g_ctx[CAN_NUM_CHANNELS];
static can_filter_t       g_sff[MAX_STD_FILTERS];
static can_filter_t       g_eff[MAX_EXT_FILTERS];
static uint8_t            g_num_sff;
static uint8_t            g_num_eff;
static can_group_filter_t g_sff_grp[MAX_STD_GRP_FILTERS];
static can_group_filter_t g_eff_grp[MAX_EXT_GRP_FILTERS];
static uint8_t            g_num_sff_grp;
static uint8_t            g_num_eff_grp;
static bool               g_filters_active;

/** Global millisecond tick — call can_timestamp_tick() from 1 ms timer */
static volatile uint32_t g_tick;

/* ================================================================
 *  Helper — get peripheral pointer
 * ================================================================ */

static LPC_CAN_TypeDef *can_periph(can_channel_t ch)
{
    return (ch == CAN_CHANNEL_1) ? LPC_CAN1 : LPC_CAN2;
}

/* ================================================================
 *  Baud-rate → BTR register value
 *
 *  PCLK = CCLK/4 = 25 MHz (set by HAL, proven working config).
 *  Auto-searches Nq (quanta per bit) from 25 down to 8,
 *  picking the first Nq that gives an exact integer BRP
 *  and keeps TSEG1 within the 4-bit hardware limit (≤ 16 Tq).
 *
 *  Target: ~80 % sample point, SJW = min(TSEG2, 4).
 *
 *  Verified for 500kbps @ 25MHz PCLK:
 *    total_q = 25000000/500000 = 50
 *    nq = 10, BRP = 5 (reg=4), TSEG1 = 7 (reg=6), TSEG2 = 2 (reg=1)
 *    BTR = 0x00160004  ← matches proven working demo
 *    Sample point = (1+7)/10 = 80%
 * ================================================================ */

static int can_calc_btr(can_baudrate_t baud, uint32_t *btr)
{
    uint32_t pclk    = can_hal_get_pclk();
    uint32_t total_q = pclk / (uint32_t)baud;   /* total quanta per bit */
    uint32_t nq, brp_val, tseg1, tseg2, sjw;
    bool     found = false;

    for (nq = 25; nq >= 8; nq--) {
        if (total_q % nq != 0) continue;         /* need exact division */
        brp_val = total_q / nq;
        if (brp_val < 1 || brp_val > 1024) continue;

        /* ~20 % of Nq for phase2 */
        tseg2 = nq / 5;
        if (tseg2 < 2) tseg2 = 2;
        if (tseg2 > 8) tseg2 = 8;
        tseg1 = nq - 1 - tseg2;                  /* SYNC = 1 Tq */
        if (tseg1 < 1 || tseg1 > 16) continue;   /* TSEG1 4-bit limit */

        found = true;
        break;
    }

    if (!found) return CAN_ERR_INVALID_PARAM;

    sjw = (tseg2 < 4) ? tseg2 : 4;

    /* Register values are (actual - 1) */
    *btr = ((brp_val - 1) & 0x3FFU)
         | ((sjw   - 1) << 14)
         | ((tseg1 - 1) << 16)
         | ((tseg2 - 1) << 20);

    return CAN_OK;
}

/* ================================================================
 *  Acceptance Filter helpers
 * ================================================================ */

/** Sort filter array by (channel, id) for AF table ordering */
static void sort_filters(can_filter_t *arr, uint8_t n)
{
    uint8_t i, j;
    for (i = 1; i < n; i++) {
        can_filter_t tmp = arr[i];
        j = i;
        while (j > 0) {
            uint32_t key_j   = ((uint32_t)arr[j-1].channel << 29) | arr[j-1].id;
            uint32_t key_tmp = ((uint32_t)tmp.channel << 29) | tmp.id;
            if (key_j <= key_tmp) break;
            arr[j] = arr[j-1];
            j--;
        }
        arr[j] = tmp;
    }
}

/** Sort group-filter array by (channel, id_low) */
static void sort_group_filters(can_group_filter_t *arr, uint8_t n)
{
    uint8_t i, j;
    for (i = 1; i < n; i++) {
        can_group_filter_t tmp = arr[i];
        j = i;
        while (j > 0) {
            uint32_t key_j   = ((uint32_t)arr[j-1].channel << 29) | arr[j-1].id_low;
            uint32_t key_tmp = ((uint32_t)tmp.channel << 29) | tmp.id_low;
            if (key_j <= key_tmp) break;
            arr[j] = arr[j-1];
            j--;
        }
        arr[j] = tmp;
    }
}

/** Rebuild the AF RAM table from all filter lists */
static void af_rebuild(void)
{
    uint32_t idx = 0;
    uint8_t  i;

    LPC_CANAF->AFMR = CAN_AFMR_ACCOFF;

    /* ── 1. Standard Individual (2 per word) ───────────── */
    /*   Bits [15:13] = SCC (CAN controller number)
     *   Bits [12:2]  = 11-bit standard ID
     *   Bit  [1]     = Disable (0 = active)
     *   Bit  [0]     = Reserved (0)                        */
    uint32_t sff_sa = idx * 4;
    sort_filters(g_sff, g_num_sff);
    for (i = 0; i < g_num_sff; i += 2) {
        uint16_t e1 = ((uint16_t)(g_sff[i].channel & 0x7) << 13)
                     | ((uint16_t)(g_sff[i].id & 0x7FF) << 2);
        uint16_t e2 = 0xFFFF;  /* Disabled / unused slot */
        if ((i + 1) < g_num_sff)
            e2 = ((uint16_t)(g_sff[i+1].channel & 0x7) << 13)
               | ((uint16_t)(g_sff[i+1].id & 0x7FF) << 2);
        LPC_CANAF_RAM->mask[idx++] = ((uint32_t)e1 << 16) | e2;
    }

    /* ── 2. Standard Group (1 pair per word) ────────────── */
    uint32_t sff_grp_sa = idx * 4;
    sort_group_filters(g_sff_grp, g_num_sff_grp);
    for (i = 0; i < g_num_sff_grp; i++) {
        uint16_t lo = ((uint16_t)(g_sff_grp[i].channel & 0x7) << 13)
                    | ((uint16_t)(g_sff_grp[i].id_low  & 0x7FF) << 2);
        uint16_t hi = ((uint16_t)(g_sff_grp[i].channel & 0x7) << 13)
                    | ((uint16_t)(g_sff_grp[i].id_high & 0x7FF) << 2);
        LPC_CANAF_RAM->mask[idx++] = ((uint32_t)lo << 16) | hi;
    }

    /* ── 3. Extended Individual (1 per word) ────────────── */
    uint32_t eff_sa = idx * 4;
    sort_filters(g_eff, g_num_eff);
    for (i = 0; i < g_num_eff; i++) {
        LPC_CANAF_RAM->mask[idx++] =
            ((uint32_t)(g_eff[i].channel & 0x7) << 29)
            | (g_eff[i].id & 0x1FFFFFFFU);
    }

    /* ── 4. Extended Group (2 words per entry) ──────────── */
    uint32_t eff_grp_sa = idx * 4;
    sort_group_filters(g_eff_grp, g_num_eff_grp);
    for (i = 0; i < g_num_eff_grp; i++) {
        LPC_CANAF_RAM->mask[idx++] =
            ((uint32_t)(g_eff_grp[i].channel & 0x7) << 29)
            | (g_eff_grp[i].id_low & 0x1FFFFFFFU);
        LPC_CANAF_RAM->mask[idx++] =
            ((uint32_t)(g_eff_grp[i].channel & 0x7) << 29)
            | (g_eff_grp[i].id_high & 0x1FFFFFFFU);
    }

    uint32_t end_of_tbl = idx * 4;

    LPC_CANAF->SFF_sa     = sff_sa;
    LPC_CANAF->SFF_GRP_sa = sff_grp_sa;
    LPC_CANAF->EFF_sa     = eff_sa;
    LPC_CANAF->EFF_GRP_sa = eff_grp_sa;
    LPC_CANAF->ENDofTable = end_of_tbl;

    LPC_CANAF->AFMR = 0;
    g_filters_active = true;
}

/* Forward declaration — defined in ISR section below */
static void busoff_recovery_check(void);

/* ================================================================
 *  Timestamp
 * ================================================================ */

void can_timestamp_tick(void)
{
    g_tick++;
    busoff_recovery_check();
}

static uint32_t get_timestamp(can_channel_t ch)
{
    return g_ctx[ch].timestamp_en ? g_tick : 0;
}

/* ================================================================
 *  PUBLIC API
 * ================================================================ */

/* ── Init ──────────────────────────────────────────────────── */

int can_init(const can_config_t *config)
{
    if (!config) return CAN_ERR_INVALID_PARAM;

    can_channel_t ch = config->channel;
    if (ch > CAN_CHANNEL_2) return CAN_ERR_INVALID_PARAM;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);
    can_ctx_t *ctx = &g_ctx[ch];

    /* HAL setup */
    int rc = can_hal_init_clock(ch);
    if (rc != CAN_OK) return rc;

    rc = can_hal_init_pins(ch);
    if (rc != CAN_OK) return rc;

    /* Enter Reset Mode (required for BTR config) */
    pCAN->MOD = CAN_MOD_RM;

    /* Clear error counters and flags (matching working demo) */
    pCAN->GSR = 0;

    /* Baud rate */
    uint32_t btr;
    rc = can_calc_btr(config->baudrate, &btr);
    if (rc != CAN_OK) return rc;
    pCAN->BTR = btr;

    /* Error Warning Limit — default 96 */
    pCAN->EWL = 96;

    /* Clear status by reading ICR */
    (void)pCAN->ICR;

    /* Polling-based driver — no interrupts. */
    pCAN->IER = 0;

    /* Default: Acceptance Filter in Bypass (accept all)
     * AFMR = 0x02 - Bypass mode, matching working demo */
    if (!g_filters_active) {
        LPC_CANAF->AFMR = CAN_AFMR_ACCBP;
    }

    /* Initialise context */
    memset(ctx, 0, sizeof(*ctx));
    ctx->rx_cb       = config->rx_callback;
    ctx->err_cb      = config->error_callback;
    ctx->timestamp_en = config->enable_timestamp;
    ctx->diag.init_tick = g_tick;
    ctx->initialized = true;

    /* Set operating mode (exits reset mode for NORMAL) */
    rc = can_set_mode(ch, config->mode);
    if (rc != CAN_OK) return rc;

    /* NVIC IRQ left disabled — polling mode.
     * Uncomment to enable ISR-driven mode:
     * can_hal_enable_irq(); */

    return CAN_OK;
}

/* ── Deinit ────────────────────────────────────────────────── */

int can_deinit(can_channel_t ch)
{
    if (ch > CAN_CHANNEL_2)       return CAN_ERR_INVALID_PARAM;
    if (!g_ctx[ch].initialized)   return CAN_ERR_NOT_INIT;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);

    pCAN->MOD = CAN_MOD_RM;      /* enter reset */
    pCAN->IER = 0;                /* disable all CAN IRQs */

    can_hal_deinit_clock(ch);
    memset(&g_ctx[ch], 0, sizeof(can_ctx_t));

    return CAN_OK;
}

/* ── Transmit ──────────────────────────────────────────────── */

int can_transmit(can_channel_t ch, const can_message_t *msg)
{
    if (ch > CAN_CHANNEL_2 || !msg)  return CAN_ERR_INVALID_PARAM;
    if (!g_ctx[ch].initialized)       return CAN_ERR_NOT_INIT;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);
    uint32_t sr = pCAN->SR;

    /* Build TFI (Frame Info) word */
    uint32_t tfi = ((uint32_t)(msg->dlc & 0x0F) << CAN_TFI_DLC_SHIFT);
    if (msg->frame_type == CAN_FRAME_EXTENDED) tfi |= CAN_TFI_FF;
    if (msg->rtr)                              tfi |= CAN_TFI_RTR;

    /* Build TID (ID) word */
    uint32_t tid = (msg->frame_type == CAN_FRAME_EXTENDED)
                 ? (msg->id & 0x1FFFFFFFU)
                 : (msg->id & 0x7FFU);

    /* Build TDA / TDB (data) */
    uint32_t tda = (uint32_t)msg->data[0]
                 | ((uint32_t)msg->data[1] << 8)
                 | ((uint32_t)msg->data[2] << 16)
                 | ((uint32_t)msg->data[3] << 24);
    uint32_t tdb = (uint32_t)msg->data[4]
                 | ((uint32_t)msg->data[5] << 8)
                 | ((uint32_t)msg->data[6] << 16)
                 | ((uint32_t)msg->data[7] << 24);

    /* Pick the first available HW TX buffer (1, 2, or 3) */
    uint32_t cmr_select;
    if (sr & CAN_SR_TBS1) {
        pCAN->TFI1 = tfi;  pCAN->TID1 = tid;
        pCAN->TDA1 = tda;  pCAN->TDB1 = tdb;
        cmr_select = CAN_CMR_STB1;
    } else if (sr & CAN_SR_TBS2) {
        pCAN->TFI2 = tfi;  pCAN->TID2 = tid;
        pCAN->TDA2 = tda;  pCAN->TDB2 = tdb;
        cmr_select = CAN_CMR_STB2;
    } else if (sr & CAN_SR_TBS3) {
        pCAN->TFI3 = tfi;  pCAN->TID3 = tid;
        pCAN->TDA3 = tda;  pCAN->TDB3 = tdb;
        cmr_select = CAN_CMR_STB3;
    } else {
        /* All HW buffers busy — queue in software TX ring buffer */
        if (!can_tx_buffer_push(&g_ctx[ch].tx_buf, msg))
            return CAN_ERR_BUFFER_FULL;
        return CAN_OK;
    }

    /* Trigger transmission (use SRR in self-test mode) */
    if (pCAN->MOD & CAN_MOD_STM)
        pCAN->CMR = CAN_CMR_SRR | cmr_select;
    else
        pCAN->CMR = CAN_CMR_TR  | cmr_select;

    return CAN_OK;
}

/* ── Receive (polling-based) ───────────────────────────────── */

static int hw_read_frame(LPC_CAN_TypeDef *pCAN, can_message_t *msg,
                         can_channel_t ch)
{
    if (!(pCAN->GSR & CAN_GSR_RBS))
        return 0;   /* No frame pending */

    uint32_t rfs = pCAN->RFS;
    uint32_t rid = pCAN->RID;
    uint32_t rda = pCAN->RDA;
    uint32_t rdb = pCAN->RDB;

    msg->frame_type = (rfs & CAN_RFS_FF) ? CAN_FRAME_EXTENDED
                                          : CAN_FRAME_STANDARD;
    msg->id  = (msg->frame_type == CAN_FRAME_EXTENDED)
             ? (rid & 0x1FFFFFFFU) : (rid & 0x7FFU);
    msg->rtr = (rfs & CAN_RFS_RTR) ? true : false;
    msg->dlc = (rfs & CAN_RFS_DLC_MASK) >> CAN_RFS_DLC_SHIFT;

    msg->data[0] = (uint8_t)(rda);
    msg->data[1] = (uint8_t)(rda >> 8);
    msg->data[2] = (uint8_t)(rda >> 16);
    msg->data[3] = (uint8_t)(rda >> 24);
    msg->data[4] = (uint8_t)(rdb);
    msg->data[5] = (uint8_t)(rdb >> 8);
    msg->data[6] = (uint8_t)(rdb >> 16);
    msg->data[7] = (uint8_t)(rdb >> 24);

    msg->timestamp = get_timestamp(ch);

    /* Release HW receive buffer */
    pCAN->CMR = CAN_CMR_RRB;

    /* Update diagnostics */
    g_ctx[ch].diag.rx_count++;

    /* Fire callback if registered */
    if (g_ctx[ch].rx_cb)
        g_ctx[ch].rx_cb(ch, msg);

    return 1;
}

int can_receive(can_channel_t ch, can_message_t *msg, uint32_t timeout_ms)
{
    if (ch > CAN_CHANNEL_2 || !msg)  return CAN_ERR_INVALID_PARAM;
    if (!g_ctx[ch].initialized)       return CAN_ERR_NOT_INIT;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);

    /* Try ring buffer first (populated if ISR mode is active) */
    if (can_buffer_pop(&g_ctx[ch].rx_buf, msg))
        return CAN_OK;

    /* Immediate hardware check — works even with timeout=0 */
    if (hw_read_frame(pCAN, msg, ch))
        return CAN_OK;

    /* If caller wants to wait, poll until timeout */
    if (timeout_ms > 0) {
        volatile uint32_t start = g_tick;
        while ((g_tick - start) < timeout_ms) {
            if (hw_read_frame(pCAN, msg, ch))
                return CAN_OK;
        }
    }

    return CAN_ERR_TIMEOUT;
}

/* ── Filter ────────────────────────────────────────────────── */

int can_set_filter(can_channel_t ch, uint32_t id, uint32_t mask,
                   can_frame_type_t ft)
{
    if (ch > CAN_CHANNEL_2)  return CAN_ERR_INVALID_PARAM;
    (void)mask;       /* HW individual entry = exact match only */

    if (ft == CAN_FRAME_STANDARD) {
        if (g_num_sff >= MAX_STD_FILTERS) return CAN_ERR_FILTER_FULL;
        g_sff[g_num_sff].id      = id & 0x7FFU;
        g_sff[g_num_sff].channel = (uint8_t)ch;
        g_sff[g_num_sff].active  = true;
        g_num_sff++;
    } else {
        if (g_num_eff >= MAX_EXT_FILTERS) return CAN_ERR_FILTER_FULL;
        g_eff[g_num_eff].id      = id & 0x1FFFFFFFU;
        g_eff[g_num_eff].channel = (uint8_t)ch;
        g_eff[g_num_eff].active  = true;
        g_num_eff++;
    }

    af_rebuild();
    return CAN_OK;
}

/* ── Status ────────────────────────────────────────────────── */

int can_get_status(can_channel_t ch, can_status_t *status)
{
    if (ch > CAN_CHANNEL_2 || !status) return CAN_ERR_INVALID_PARAM;
    if (!g_ctx[ch].initialized)         return CAN_ERR_NOT_INIT;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);
    uint32_t gsr = pCAN->GSR;

    status->tx_error_count = (uint8_t)((gsr & CAN_GSR_TXERR_MASK) >> CAN_GSR_TXERR_SHIFT);
    status->rx_error_count = (uint8_t)((gsr & CAN_GSR_RXERR_MASK) >> CAN_GSR_RXERR_SHIFT);
    status->bus_off        = (gsr & CAN_GSR_BS) ? true : false;
    status->error_warning  = (gsr & CAN_GSR_ES) ? true : false;
    status->tx_pending     = (gsr & CAN_GSR_TS) ? true : false;
    status->rx_available   = (gsr & CAN_GSR_RBS) ? true : false;

    return CAN_OK;
}

/* ── Mode ──────────────────────────────────────────────────── */

int can_set_mode(can_channel_t ch, can_mode_t mode)
{
    if (ch > CAN_CHANNEL_2) return CAN_ERR_INVALID_PARAM;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);

    switch (mode) {
    case CAN_MODE_NORMAL:
        pCAN->MOD &= ~(CAN_MOD_RM | CAN_MOD_LOM | CAN_MOD_STM);
        break;
    case CAN_MODE_LISTEN:
        pCAN->MOD |=  CAN_MOD_LOM;
        pCAN->MOD &= ~CAN_MOD_RM;
        break;
    case CAN_MODE_SELFTEST:
        pCAN->MOD |=  CAN_MOD_STM;
        pCAN->MOD &= ~CAN_MOD_RM;
        break;
    case CAN_MODE_RESET:
        pCAN->MOD |=  CAN_MOD_RM;
        break;
    default:
        return CAN_ERR_INVALID_PARAM;
    }

    return CAN_OK;
}

/* ── Callbacks ─────────────────────────────────────────────── */

int can_register_rx_callback(can_channel_t ch, can_rx_callback_t cb)
{
    if (ch > CAN_CHANNEL_2) return CAN_ERR_INVALID_PARAM;
    g_ctx[ch].rx_cb = cb;
    return CAN_OK;
}

int can_register_error_callback(can_channel_t ch, can_error_callback_t cb)
{
    if (ch > CAN_CHANNEL_2) return CAN_ERR_INVALID_PARAM;
    g_ctx[ch].err_cb = cb;
    return CAN_OK;
}

/* ================================================================
 *  ISR  — shared by CAN1 and CAN2
 * ================================================================ */

/** Read one frame from hardware RX buffer into msg struct */
static void isr_read_rx(LPC_CAN_TypeDef *pCAN, can_channel_t ch)
{
    can_ctx_t *ctx = &g_ctx[ch];

    while (pCAN->GSR & CAN_GSR_RBS) {
        can_message_t msg;

        uint32_t rfs = pCAN->RFS;
        uint32_t rid = pCAN->RID;
        uint32_t rda = pCAN->RDA;
        uint32_t rdb = pCAN->RDB;

        msg.frame_type = (rfs & CAN_RFS_FF) ? CAN_FRAME_EXTENDED
                                             : CAN_FRAME_STANDARD;
        msg.id  = (msg.frame_type == CAN_FRAME_EXTENDED)
                ? (rid & 0x1FFFFFFFU) : (rid & 0x7FFU);
        msg.rtr = (rfs & CAN_RFS_RTR) ? true : false;
        msg.dlc = (uint8_t)((rfs & CAN_RFS_DLC_MASK) >> CAN_RFS_DLC_SHIFT);

        msg.data[0] = (uint8_t)(rda);
        msg.data[1] = (uint8_t)(rda >> 8);
        msg.data[2] = (uint8_t)(rda >> 16);
        msg.data[3] = (uint8_t)(rda >> 24);
        msg.data[4] = (uint8_t)(rdb);
        msg.data[5] = (uint8_t)(rdb >> 8);
        msg.data[6] = (uint8_t)(rdb >> 16);
        msg.data[7] = (uint8_t)(rdb >> 24);

        msg.timestamp = get_timestamp(ch);

        /* Release HW RX buffer */
        pCAN->CMR = CAN_CMR_RRB;

        /* Push into ring buffer */
        can_buffer_push(&ctx->rx_buf, &msg);

        /* User callback (from ISR context) */
        if (ctx->rx_cb)
            ctx->rx_cb(ch, &msg);
    }
}

/** Drain software TX queue into any free HW TX buffer */
static void isr_feed_tx(LPC_CAN_TypeDef *pCAN, can_channel_t ch)
{
    can_ctx_t *ctx = &g_ctx[ch];
    can_message_t msg;

    while (!can_tx_buffer_is_empty(&ctx->tx_buf)) {
        uint32_t sr = pCAN->SR;

        /* Pick a free HW buffer */
        uint32_t cmr_select;
        volatile uint32_t *pTFI, *pTID, *pTDA, *pTDB;

        if (sr & CAN_SR_TBS1) {
            pTFI = &pCAN->TFI1; pTID = &pCAN->TID1;
            pTDA = &pCAN->TDA1; pTDB = &pCAN->TDB1;
            cmr_select = CAN_CMR_STB1;
        } else if (sr & CAN_SR_TBS2) {
            pTFI = &pCAN->TFI2; pTID = &pCAN->TID2;
            pTDA = &pCAN->TDA2; pTDB = &pCAN->TDB2;
            cmr_select = CAN_CMR_STB2;
        } else if (sr & CAN_SR_TBS3) {
            pTFI = &pCAN->TFI3; pTID = &pCAN->TID3;
            pTDA = &pCAN->TDA3; pTDB = &pCAN->TDB3;
            cmr_select = CAN_CMR_STB3;
        } else {
            break;  /* no free buffer */
        }

        can_tx_buffer_pop(&ctx->tx_buf, &msg);

        /* Load frame */
        uint32_t tfi = ((uint32_t)(msg.dlc & 0x0F) << CAN_TFI_DLC_SHIFT);
        if (msg.frame_type == CAN_FRAME_EXTENDED) tfi |= CAN_TFI_FF;
        if (msg.rtr) tfi |= CAN_TFI_RTR;

        *pTFI = tfi;
        *pTID = (msg.frame_type == CAN_FRAME_EXTENDED)
              ? (msg.id & 0x1FFFFFFFU) : (msg.id & 0x7FFU);
        *pTDA = (uint32_t)msg.data[0]
              | ((uint32_t)msg.data[1] << 8)
              | ((uint32_t)msg.data[2] << 16)
              | ((uint32_t)msg.data[3] << 24);
        *pTDB = (uint32_t)msg.data[4]
              | ((uint32_t)msg.data[5] << 8)
              | ((uint32_t)msg.data[6] << 16)
              | ((uint32_t)msg.data[7] << 24);

        if (pCAN->MOD & CAN_MOD_STM)
            pCAN->CMR = CAN_CMR_SRR | cmr_select;
        else
            pCAN->CMR = CAN_CMR_TR  | cmr_select;
    }
}

/** Map ICR error-code capture bits to can_error_t */
static can_error_t isr_decode_error(uint32_t icr)
{
    uint32_t errc = (icr & CAN_ICR_ERRC_MASK) >> CAN_ICR_ERRC_SHIFT;
    switch (errc) {
    case 0: return CAN_ERROR_BIT0;
    case 1: return CAN_ERROR_FORM;
    case 2: return CAN_ERROR_STUFF;
    default: return CAN_ERROR_CRC;
    }
}

/** Process all pending interrupts for one channel */
static void isr_process_channel(LPC_CAN_TypeDef *pCAN, can_channel_t ch)
{
    uint32_t icr = pCAN->ICR;
    can_ctx_t *ctx = &g_ctx[ch];

    if (!ctx->initialized) return;

    /* ── Receive ───────────────────────── */
    if (icr & CAN_ICR_RI) {
        isr_read_rx(pCAN, ch);
        ctx->diag.rx_count++;
    }

    /* ── TX complete — feed queued msgs ── */
    if (icr & (CAN_ICR_TI1 | CAN_ICR_TI2 | CAN_ICR_TI3)) {
        ctx->diag.tx_count++;
        isr_feed_tx(pCAN, ch);
    }

    /* ── Data overrun ──────────────────── */
    if (icr & CAN_ICR_DOI) {
        pCAN->CMR = CAN_CMR_CDO;
        ctx->diag.overrun_count++;
    }

    /* ── Arbitration lost ──────────────── */
    if (icr & CAN_ICR_ALI) {
        ctx->diag.arb_lost_count++;
    }

    /* ── Bus error ─────────────────────── */
    if (icr & CAN_ICR_BEI) {
        can_error_t err = isr_decode_error(icr);
        ctx->diag.last_error = err;
        ctx->diag.tx_err_frames++;
        if (ctx->err_cb)
            ctx->err_cb(ch, err);
    }

    /* ── Error Warning / Passive / Bus-off ─ */
    if (icr & (CAN_ICR_EI | CAN_ICR_EPI)) {
        if (pCAN->GSR & CAN_GSR_BS) {
            ctx->diag.bus_off_count++;
            ctx->diag.last_error = CAN_ERROR_BUS_OFF;
            /* Arm bus-off auto-recovery */
            ctx->busoff_recovery = true;
            ctx->busoff_tick     = g_tick;
            if (ctx->err_cb)
                ctx->err_cb(ch, CAN_ERROR_BUS_OFF);
        } else if (pCAN->GSR & CAN_GSR_ES) {
            ctx->diag.rx_err_frames++;
            if (ctx->err_cb)
                ctx->err_cb(ch, CAN_ERROR_ACK);
        }
    }
}

/* ── Bus-off auto-recovery (called from timestamp tick) ───── */

static void busoff_recovery_check(void)
{
    uint8_t i;
    for (i = 0; i < CAN_NUM_CHANNELS; i++) {
        can_ctx_t *ctx = &g_ctx[i];
        if (!ctx->busoff_recovery) continue;
        if ((g_tick - ctx->busoff_tick) >= BUSOFF_RECOVERY_MS) {
            LPC_CAN_TypeDef *pCAN = can_periph((can_channel_t)i);
            /* Toggle reset mode to re-join bus */
            pCAN->MOD |=  CAN_MOD_RM;
            pCAN->MOD &= ~CAN_MOD_RM;
            ctx->busoff_recovery = false;
        }
    }
}

/* ── CAN_IRQHandler — called by NVIC for both CAN1 & CAN2 ── */

void CAN_IRQHandler(void)
{
    isr_process_channel(LPC_CAN1, CAN_CHANNEL_1);
    isr_process_channel(LPC_CAN2, CAN_CHANNEL_2);
}

/* ================================================================
 *  Week 3 — Advanced API
 * ================================================================ */

/* ── Diagnostics ───────────────────────────────────────────── */

int can_get_diag(can_channel_t ch, can_diag_t *diag)
{
    if (ch > CAN_CHANNEL_2 || !diag) return CAN_ERR_INVALID_PARAM;
    if (!g_ctx[ch].initialized)       return CAN_ERR_NOT_INIT;

    can_diag_internal_t *d = &g_ctx[ch].diag;
    diag->tx_count       = d->tx_count;
    diag->rx_count       = d->rx_count;
    diag->tx_err_frames  = d->tx_err_frames;
    diag->rx_err_frames  = d->rx_err_frames;
    diag->bus_off_count  = d->bus_off_count;
    diag->overrun_count  = d->overrun_count;
    diag->arb_lost_count = d->arb_lost_count;
    diag->last_error     = d->last_error;
    diag->uptime_ms      = g_tick - d->init_tick;
    return CAN_OK;
}

void can_reset_diag(can_channel_t ch)
{
    if (ch > CAN_CHANNEL_2) return;
    memset(&g_ctx[ch].diag, 0, sizeof(can_diag_internal_t));
    g_ctx[ch].diag.init_tick = g_tick;
}

/* ── Group Filters ─────────────────────────────────────────── */

int can_set_group_filter(can_channel_t ch, uint32_t id_low,
                         uint32_t id_high, can_frame_type_t ft)
{
    if (ch > CAN_CHANNEL_2)       return CAN_ERR_INVALID_PARAM;
    if (id_low > id_high)          return CAN_ERR_INVALID_PARAM;

    if (ft == CAN_FRAME_STANDARD) {
        if (g_num_sff_grp >= MAX_STD_GRP_FILTERS) return CAN_ERR_FILTER_FULL;
        g_sff_grp[g_num_sff_grp].id_low  = id_low  & 0x7FFU;
        g_sff_grp[g_num_sff_grp].id_high = id_high & 0x7FFU;
        g_sff_grp[g_num_sff_grp].channel = (uint8_t)ch;
        g_sff_grp[g_num_sff_grp].active  = true;
        g_num_sff_grp++;
    } else {
        if (g_num_eff_grp >= MAX_EXT_GRP_FILTERS) return CAN_ERR_FILTER_FULL;
        g_eff_grp[g_num_eff_grp].id_low  = id_low  & 0x1FFFFFFFU;
        g_eff_grp[g_num_eff_grp].id_high = id_high & 0x1FFFFFFFU;
        g_eff_grp[g_num_eff_grp].channel = (uint8_t)ch;
        g_eff_grp[g_num_eff_grp].active  = true;
        g_num_eff_grp++;
    }

    af_rebuild();
    return CAN_OK;
}

int can_clear_filters(can_channel_t ch)
{
    (void)ch;  /* clears all filters (AF is shared) */
    g_num_sff     = 0;
    g_num_eff     = 0;
    g_num_sff_grp = 0;
    g_num_eff_grp = 0;
    g_filters_active = false;
    LPC_CANAF->AFMR = CAN_AFMR_ACCBP;   /* back to bypass */
    return CAN_OK;
}

/* ── Sleep / Wake ──────────────────────────────────────────── */

int can_sleep(can_channel_t ch)
{
    if (ch > CAN_CHANNEL_2)     return CAN_ERR_INVALID_PARAM;
    if (!g_ctx[ch].initialized) return CAN_ERR_NOT_INIT;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);

    /* Enable wake-up interrupt so bus activity wakes us */
    pCAN->IER |= CAN_IER_WUIE;
    pCAN->MOD |= CAN_MOD_SM;
    return CAN_OK;
}

int can_wake(can_channel_t ch)
{
    if (ch > CAN_CHANNEL_2)     return CAN_ERR_INVALID_PARAM;
    if (!g_ctx[ch].initialized) return CAN_ERR_NOT_INIT;

    LPC_CAN_TypeDef *pCAN = can_periph(ch);
    pCAN->MOD &= ~CAN_MOD_SM;
    return CAN_OK;
}

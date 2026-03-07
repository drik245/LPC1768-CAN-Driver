/**
 * @file can_reg.h
 * @brief LPC1768 CAN register bit definitions (internal use)
 */

#ifndef CAN_REG_H
#define CAN_REG_H

/* ── MOD Register ─────────────────────────────────────────── */
#define CAN_MOD_RM          (1U << 0)   /* Reset Mode            */
#define CAN_MOD_LOM         (1U << 1)   /* Listen Only Mode      */
#define CAN_MOD_STM         (1U << 2)   /* Self Test Mode        */
#define CAN_MOD_TPM         (1U << 3)   /* TX Priority Mode      */
#define CAN_MOD_SM          (1U << 4)   /* Sleep Mode            */
#define CAN_MOD_RPM         (1U << 5)   /* RX Polarity Mode      */
#define CAN_MOD_TM          (1U << 7)   /* Test Mode             */

/* ── CMR Register (write-only) ────────────────────────────── */
#define CAN_CMR_TR          (1U << 0)   /* Transmission Request  */
#define CAN_CMR_AT          (1U << 1)   /* Abort Transmission    */
#define CAN_CMR_RRB         (1U << 2)   /* Release RX Buffer     */
#define CAN_CMR_CDO         (1U << 3)   /* Clear Data Overrun    */
#define CAN_CMR_SRR         (1U << 4)   /* Self Reception Req    */
#define CAN_CMR_STB1        (1U << 5)   /* Select TX Buffer 1    */
#define CAN_CMR_STB2        (1U << 6)   /* Select TX Buffer 2    */
#define CAN_CMR_STB3        (1U << 7)   /* Select TX Buffer 3    */

/* ── GSR Register ─────────────────────────────────────────── */
#define CAN_GSR_RBS         (1U << 0)   /* RX Buffer Status      */
#define CAN_GSR_DOS         (1U << 1)   /* Data Overrun Status   */
#define CAN_GSR_TBS         (1U << 2)   /* TX Buffer Status      */
#define CAN_GSR_TCS         (1U << 3)   /* TX Complete Status    */
#define CAN_GSR_RS          (1U << 4)   /* Receiving             */
#define CAN_GSR_TS          (1U << 5)   /* Transmitting          */
#define CAN_GSR_ES          (1U << 6)   /* Error Status          */
#define CAN_GSR_BS          (1U << 7)   /* Bus-off Status        */
#define CAN_GSR_RXERR_SHIFT 16
#define CAN_GSR_RXERR_MASK  (0xFFU << 16)
#define CAN_GSR_TXERR_SHIFT 24
#define CAN_GSR_TXERR_MASK  (0xFFU << 24)

/* ── ICR Register (read-only, read clears) ────────────────── */
#define CAN_ICR_RI          (1U << 0)   /* RX Interrupt          */
#define CAN_ICR_TI1         (1U << 1)   /* TX Buffer 1 Int       */
#define CAN_ICR_EI          (1U << 2)   /* Error Warning Int     */
#define CAN_ICR_DOI         (1U << 3)   /* Data Overrun Int      */
#define CAN_ICR_WUI         (1U << 4)   /* Wake-Up Interrupt     */
#define CAN_ICR_EPI         (1U << 5)   /* Error Passive Int     */
#define CAN_ICR_ALI         (1U << 6)   /* Arbitration Lost Int  */
#define CAN_ICR_BEI         (1U << 7)   /* Bus Error Interrupt   */
#define CAN_ICR_IDI         (1U << 8)   /* ID Ready Interrupt    */
#define CAN_ICR_TI2         (1U << 9)   /* TX Buffer 2 Int       */
#define CAN_ICR_TI3         (1U << 10)  /* TX Buffer 3 Int       */
#define CAN_ICR_ERRC_SHIFT  22
#define CAN_ICR_ERRC_MASK   (3U << 22)

/* ── IER Register ─────────────────────────────────────────── */
#define CAN_IER_RIE         (1U << 0)   /* RX Int Enable         */
#define CAN_IER_TIE1        (1U << 1)   /* TX1 Int Enable        */
#define CAN_IER_EIE         (1U << 2)   /* Error Warning Enable  */
#define CAN_IER_DOIE        (1U << 3)   /* Data Overrun Enable   */
#define CAN_IER_WUIE        (1U << 4)   /* Wake-Up Enable        */
#define CAN_IER_EPIE        (1U << 5)   /* Error Passive Enable  */
#define CAN_IER_ALIE        (1U << 6)   /* Arb Lost Enable       */
#define CAN_IER_BEIE        (1U << 7)   /* Bus Error Enable      */
#define CAN_IER_IDIE        (1U << 8)   /* ID Ready Enable       */
#define CAN_IER_TIE2        (1U << 9)   /* TX2 Int Enable        */
#define CAN_IER_TIE3        (1U << 10)  /* TX3 Int Enable        */

/* ── SR Register (per-buffer status) ──────────────────────── */
#define CAN_SR_RBS1         (1U << 0)
#define CAN_SR_DOS1         (1U << 1)
#define CAN_SR_TBS1         (1U << 2)
#define CAN_SR_TCS1         (1U << 3)
#define CAN_SR_RS1          (1U << 4)
#define CAN_SR_TS1          (1U << 5)
#define CAN_SR_ES1          (1U << 6)
#define CAN_SR_BS1          (1U << 7)
#define CAN_SR_TBS2         (1U << 10)
#define CAN_SR_TCS2         (1U << 11)
#define CAN_SR_TBS3         (1U << 18)
#define CAN_SR_TCS3         (1U << 19)

/* ── TFI Register (TX Frame Info) ─────────────────────────── */
#define CAN_TFI_DLC_SHIFT   16
#define CAN_TFI_DLC_MASK    (0xFU << 16)
#define CAN_TFI_RTR         (1U << 30)
#define CAN_TFI_FF          (1U << 31)  /* 0=SFF 1=EFF          */

/* ── RFS Register (RX Frame Status) ──────────────────────── */
#define CAN_RFS_ID_SHIFT    0
#define CAN_RFS_BP          (1U << 4)
#define CAN_RFS_DLC_SHIFT   16
#define CAN_RFS_DLC_MASK    (0xFU << 16)
#define CAN_RFS_RTR         (1U << 30)
#define CAN_RFS_FF          (1U << 31)  /* 0=SFF 1=EFF          */

/* ── Acceptance Filter (AFMR) ─────────────────────────────── */
#define CAN_AFMR_ACCOFF     (1U << 0)   /* AF Off                */
#define CAN_AFMR_ACCBP      (1U << 1)   /* AF Bypass             */
#define CAN_AFMR_EFCAN      (1U << 2)   /* FullCAN Mode          */

/* ── Power / Clock ─────────────────────────────────────────── */
#define PCONP_PCCAN1        (1U << 13)
#define PCONP_PCCAN2        (1U << 14)
#define PCLKSEL0_CAN1_SHIFT 26
#define PCLKSEL0_CAN2_SHIFT 28

/* ── Pin Mux ───────────────────────────────────────────────── */
/* CAN1: RD1 = P0.0 (PINSEL0[1:0]=01), TD1 = P0.1 (PINSEL0[3:2]=01) */
/* CAN2: RD2 = P0.4 (PINSEL0[9:8]=10), TD2 = P0.5 (PINSEL0[11:10]=10) */
#define PINSEL_CAN1_RD_SHIFT  0
#define PINSEL_CAN1_RD_FUNC   0x01U
#define PINSEL_CAN1_TD_SHIFT  2
#define PINSEL_CAN1_TD_FUNC   0x01U
#define PINSEL_CAN2_RD_SHIFT  8
#define PINSEL_CAN2_RD_FUNC   0x02U
#define PINSEL_CAN2_TD_SHIFT  10
#define PINSEL_CAN2_TD_FUNC   0x02U

#endif /* CAN_REG_H */

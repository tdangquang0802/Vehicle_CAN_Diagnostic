/*
 * bringup_ecu.c
 *
 *  Created on: Nov 18, 2026
 *      Author: Quang Trung
 */

#include "main.h"
#include "can_ids.h"
#include "can_frames.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern CAN_HandleTypeDef  hcan1;
extern UART_HandleTypeDef huart2;

/* Select the stage, rebuild, flash. */
#define STAGE_CLOCK     1   /* A4: print clocks, blink LED */
#define STAGE_LOOPBACK  2   /* A5: CAN self-test (CubeMX Operating Mode = Loopback) */
#define STAGE_TX        3   /* A6: send 0x100 every 20 ms (Operating Mode = Normal) */
#define STAGE_RX        4   /* Day 2: receive 0x200/0x201 from SIM (Operating Mode = Normal) */
#define BRINGUP_STAGE   STAGE_CLOCK

#define LED_TOGGLE()    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

/* ---------------------------------------------------------------- debug print */
static void dbg_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)sizeof buf - 1) n = (int)sizeof buf - 1;
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
    }
}

/* bxCAN error status register: TEC bits 23:16, REC bits 31:24, BOFF bit 2, EPVF bit 1. */
static uint32_t can_tec(void)  { return (hcan1.Instance->ESR >> 16) & 0xFFu; }
static uint32_t can_rec(void)  { return (hcan1.Instance->ESR >> 24) & 0xFFu; }
static uint32_t can_boff(void) { return (hcan1.Instance->ESR >> 2) & 1u; }

static void bringup_fail(const char *what)
{
    dbg_printf("[ECU] FAIL: %s\r\n", what);
    for (;;) { LED_TOGGLE(); HAL_Delay(100); }          /* fast blink = failure */
}

/* ---------------------------------------------------------------- CAN filters (4.6) */
static HAL_StatusTypeDef ecu_can_filters(void)
{
    CAN_FilterTypeDef f;
    memset(&f, 0, sizeof f);

    /* Bank 0: 32-bit ID-list mode holds 2 IDs: 0x200 and 0x201 -> FIFO0.
     * A standard ID sits in bits 31:21 of the 32-bit register, so shift by 5 in the high half. */
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDLIST;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = (uint16_t)(CANID_SIM_MOTION << 5);   /* first ID  */
    f.FilterIdLow          = 0;
    f.FilterMaskIdHigh     = (uint16_t)(CANID_SIM_ANALOG << 5);   /* second ID (list mode) */
    f.FilterMaskIdLow      = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = CAN_FILTER_ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) return HAL_ERROR;

    /* Bank 1: mask mode, mask = 0 -> accept every ID -> FIFO1 (OBD, FC, USB-CAN frames).
     * List mode wins over mask mode, so 0x200/0x201 never land here. */
    f.FilterBank           = 1;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterIdHigh         = 0;
    f.FilterIdLow          = 0;
    f.FilterMaskIdHigh     = 0;
    f.FilterMaskIdLow      = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    return HAL_CAN_ConfigFilter(&hcan1, &f);
}

/* ---------------------------------------------------------------- TX, latest-wins (AD-04) */
typedef struct { uint32_t mbx; } tx_slot_t;   /* mailbox mask of the last frame, 0 = none */
static uint32_t g_abort_cnt, g_tx_fail;

static HAL_StatusTypeDef can_send_latest(tx_slot_t *slot, uint16_t id, const uint8_t d[8])
{
    CAN_TxHeaderTypeDef h;
    memset(&h, 0, sizeof h);
    h.StdId = id;
    h.IDE   = CAN_ID_STD;
    h.RTR   = CAN_RTR_DATA;
    h.DLC   = 8;
    h.TransmitGlobalTime = DISABLE;

    /* Old frame of this ID still waiting (no ACK, lost arbitration): drop it, send the new one. */
    if (slot->mbx != 0u && HAL_CAN_IsTxMessagePending(&hcan1, slot->mbx)) {
        HAL_CAN_AbortTxRequest(&hcan1, slot->mbx);
        g_abort_cnt++;
    }
    uint32_t mbx = 0;
    HAL_StatusTypeDef st = HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)d, &mbx);
    slot->mbx = (st == HAL_OK) ? mbx : 0u;
    if (st != HAL_OK) g_tx_fail++;
    return st;
}

/* ---------------------------------------------------------------- A5: loopback self-test */
static void loopback_selftest(void)
{
    static const struct { uint16_t id; uint32_t fifo; uint32_t other; } tc[2] = {
        { CANID_SIM_MOTION, CAN_RX_FIFO0, CAN_RX_FIFO1 },   /* must hit the list bank  */
        { CANID_ECU_MOTION, CAN_RX_FIFO1, CAN_RX_FIFO0 },   /* must hit the mask bank  */
    };
    int pass = 1;

    for (int i = 0; i < 2; i++) {
        uint8_t tx[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, (uint8_t)i };
        CAN_TxHeaderTypeDef h;
        memset(&h, 0, sizeof h);
        h.StdId = tc[i].id; h.IDE = CAN_ID_STD; h.RTR = CAN_RTR_DATA; h.DLC = 8;
        uint32_t mbx;
        if (HAL_CAN_AddTxMessage(&hcan1, &h, tx, &mbx) != HAL_OK) { pass = 0; dbg_printf("  tx %lu fail\r\n", (unsigned long)i); continue; }

        uint32_t t0 = HAL_GetTick();
        while (HAL_CAN_GetRxFifoFillLevel(&hcan1, tc[i].fifo) == 0u && (HAL_GetTick() - t0) < 10u) { }

        CAN_RxHeaderTypeDef rh;
        uint8_t rx[8] = { 0 };
        int ok = (HAL_CAN_GetRxMessage(&hcan1, tc[i].fifo, &rh, rx) == HAL_OK)
              && rh.StdId == tc[i].id && rh.DLC == 8 && memcmp(rx, tx, 8) == 0
              && HAL_CAN_GetRxFifoFillLevel(&hcan1, tc[i].other) == 0u;
        dbg_printf("  id=0x%03lX expect FIFO%lu: %s\r\n", (unsigned long)tc[i].id,
                   (unsigned long)tc[i].fifo, ok ? "PASS" : "FAIL");
        if (!ok) pass = 0;
    }
    dbg_printf("[ECU] loopback selftest: %s\r\n", pass ? "PASS" : "FAIL");
    if (!pass) bringup_fail("loopback");
}

/* ---------------------------------------------------------------- Day 2: RX statistics */
typedef struct {
    uint32_t n, last_ms, dt_min, dt_max, alive_jumps;
    uint8_t  last_alive, started, data[8];
} rx_stat_t;

static rx_stat_t g_rx[2];            /* [0] = 0x200, [1] = 0x201 */
static uint32_t  g_other_rx;         /* frames seen in FIFO1 (anything but 0x200/0x201) */

static void rx_stat_reset_window(rx_stat_t *s) { s->dt_min = 0xFFFFFFFFu; s->dt_max = 0; }

static void rx_drain(uint32_t fifo)  /* called from ISR: keep it short */
{
    CAN_RxHeaderTypeDef rh;
    uint8_t d[8];
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, fifo) > 0u) {
        if (HAL_CAN_GetRxMessage(&hcan1, fifo, &rh, d) != HAL_OK) break;
        uint32_t now = HAL_GetTick();
        int i = (rh.StdId == CANID_SIM_MOTION) ? 0 : (rh.StdId == CANID_SIM_ANALOG) ? 1 : -1;
        if (i < 0 || rh.IDE != CAN_ID_STD) { g_other_rx++; continue; }

        rx_stat_t *s = &g_rx[i];
        uint8_t alive = d[7] & CANF_ALIVE_MASK;
        if (s->started) {
            uint32_t dt = now - s->last_ms;
            if (dt < s->dt_min) s->dt_min = dt;
            if (dt > s->dt_max) s->dt_max = dt;
            if (alive != canf_alive_next(s->last_alive)) s->alive_jumps++;
        }
        s->started = 1; s->last_ms = now; s->last_alive = alive; s->n++;
        memcpy(s->data, d, 8);
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h) { (void)h; rx_drain(CAN_RX_FIFO0); }
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *h) { (void)h; rx_drain(CAN_RX_FIFO1); }

static void rx_print(void)
{
    rx_stat_t s[2];
    __disable_irq();
    s[0] = g_rx[0]; s[1] = g_rx[1];
    rx_stat_reset_window(&g_rx[0]); rx_stat_reset_window(&g_rx[1]);   /* min/max per 1 s window */
    __enable_irq();

    if (s[0].n == 0u) dbg_printf("[0x200] waiting\r\n");
    else {
        canf_sim_motion_t m; canf_sim_motion_unpack(&m, s[0].data);
        dbg_printf("[0x200] n=%lu dt=%lu..%lu ms jumps=%lu spd=%u.%u rpm=%u thr=%u.%u pulse=%u phase=%u eng=%u alive=%u\r\n",
                   (unsigned long)s[0].n, (unsigned long)s[0].dt_min, (unsigned long)s[0].dt_max,
                   (unsigned long)s[0].alive_jumps, m.speed_x10 / 10, m.speed_x10 % 10, m.rpm,
                   m.throttle_x2 / 2, (m.throttle_x2 % 2) * 5, m.wheel_pulse, m.phase, m.engine_state, m.alive);
    }
    if (s[1].n == 0u) dbg_printf("[0x201] waiting\r\n");
    else {
        canf_sim_analog_t a; canf_sim_analog_unpack(&a, s[1].data);
        dbg_printf("[0x201] n=%lu dt=%lu..%lu ms jumps=%lu coolant_raw=%u (%d C) vbat_raw=%u (%lu mV) alive=%u\r\n",
                   (unsigned long)s[1].n, (unsigned long)s[1].dt_min, (unsigned long)s[1].dt_max,
                   (unsigned long)s[1].alive_jumps, a.coolant_raw,
                   (int)(a.coolant_raw / CANF_COOLANT_RAW_PER_C) - CANF_COOLANT_OFFSET_C,
                   a.vbat_raw, (unsigned long)a.vbat_raw * CANF_VBAT_MV_PER_LSB, a.alive);
    }
    dbg_printf("[CAN] TEC=%lu REC=%lu BOFF=%lu other_rx=%lu\r\n",
               (unsigned long)can_tec(), (unsigned long)can_rec(), (unsigned long)can_boff(),
               (unsigned long)g_other_rx);
}

/* ---------------------------------------------------------------- entry points */
static uint32_t g_next_led, g_next_print, g_next_tx;
static tx_slot_t g_slot_motion;
static uint8_t  g_alive_motion;

void bringup_init(void)
{
    dbg_printf("\r\n[ECU] boot stage=%d SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu\r\n", BRINGUP_STAGE,
               (unsigned long)SystemCoreClock, (unsigned long)HAL_RCC_GetHCLKFreq(),
               (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
    /* Expected: 168000000 / 168000000 / 42000000 / 84000000 */

#if BRINGUP_STAGE >= STAGE_LOOPBACK
    if (ecu_can_filters() != HAL_OK) bringup_fail("filter config");
    if (HAL_CAN_Start(&hcan1) != HAL_OK) bringup_fail("HAL_CAN_Start");
#endif
#if BRINGUP_STAGE == STAGE_LOOPBACK
    loopback_selftest();
#endif
#if BRINGUP_STAGE == STAGE_RX
    rx_stat_reset_window(&g_rx[0]); rx_stat_reset_window(&g_rx[1]);
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
        bringup_fail("notification");
#endif
    uint32_t now = HAL_GetTick();
    g_next_led = now + 500; g_next_print = now + 1000; g_next_tx = now + CANPER_ECU_MOTION_MS;
}

void bringup_poll(void)
{
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - g_next_led) >= 0) { g_next_led += 500; LED_TOGGLE(); }   /* 1 Hz blink */

#if BRINGUP_STAGE == STAGE_TX
    if ((int32_t)(now - g_next_tx) >= 0) {
        g_next_tx += CANPER_ECU_MOTION_MS;
        canf_ecu_motion_t m = { 500, 2550, 50, CANF_ENG_RUN, g_alive_motion };   /* 50.0 km/h, 2550 rpm, 25 % */
        uint8_t d[8];
        canf_ecu_motion_pack(&m, d);
        (void)can_send_latest(&g_slot_motion, CANID_ECU_MOTION, d);
        g_alive_motion = canf_alive_next(g_alive_motion);
    }
    if ((int32_t)(now - g_next_print) >= 0) {
        g_next_print += 1000;
        dbg_printf("[TX] 0x100 aborts=%lu tx_fail=%lu TEC=%lu REC=%lu BOFF=%lu\r\n",
                   (unsigned long)g_abort_cnt, (unsigned long)g_tx_fail,
                   (unsigned long)can_tec(), (unsigned long)can_rec(), (unsigned long)can_boff());
    }
#endif
#if BRINGUP_STAGE == STAGE_RX
    if ((int32_t)(now - g_next_print) >= 0) { g_next_print += 1000; rx_print(); }
#endif
}

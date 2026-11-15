/*
 * can_frame.h
 *
 *  Created on: Nov 15, 2026
 *      Author: Quang Trung
 */

#ifndef COMMON_CAN_FRAME_H_
#define COMMON_CAN_FRAME_H_

#include <stdint.h>

/* ---- Conventions (4.1) ---- */
#define CANF_NA_U8              0xFFu
#define CANF_NA_U16             0xFFFFu
#define CANF_ALIVE_MASK         0x0Fu
#define CANF_OBD_PAD            0xAAu

static inline uint8_t canf_alive_next(uint8_t a) { return (uint8_t)((a + 1u) & CANF_ALIVE_MASK); }

/* ---- Scaling ---- */
#define CANF_SPEED_LSB_KMH_X10  1u      /* 0.1 km/h per bit */
#define CANF_THROTTLE_PER_PCT   2u      /* 0.5 % per bit: raw = percent * 2 */
#define CANF_COOLANT_OFFSET_C   40      /* 0x101 byte 0 = degC + 40 */
#define CANF_VBAT_MV_PER_LSB    4u      /* mV = vbat_raw * 4 */
#define CANF_COOLANT_RAW_PER_C  25u     /* byte = coolant_raw / 25 */

/* ---- Enumerations ---- */
enum { CANF_PHASE_IDLE = 0, CANF_PHASE_ACCEL = 1, CANF_PHASE_CRUISE = 2, CANF_PHASE_DECEL = 3 };
enum { CANF_ENG_IDLE = 1, CANF_ENG_RUN = 2 };

/* 0x200 flags (bits 7:4 of byte 7) */
#define CANF_SIM_FLAG_WHEEL_FAULT   0x1u   /* bit 4 overall: display only */

/* 0x101 status bits */
#define CANF_ST_MIL             (1u << 0)
#define CANF_ST_MOTION_VALID    (1u << 1)
#define CANF_ST_ANALOG_VALID    (1u << 2)
#define CANF_ST_NVM_FAULT       (1u << 3)

/* 0x101 DTC bitmask */
#define CANF_DTC_P0217          (1u << 0)
#define CANF_DTC_P0118          (1u << 1)
#define CANF_DTC_P0562          (1u << 2)
#define CANF_DTC_C0035          (1u << 3)

/* ---- Little-endian helpers ---- */
static inline void     canf_put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline uint16_t canf_get_u16(const uint8_t *p)       { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

/* ---- 0x200 SIM_MOTION ---- */
typedef struct {
    uint16_t speed_x10;     /* 0.1 km/h */
    uint16_t rpm;
    uint8_t  throttle_x2;   /* 0.5 %/bit, 0..200 */
    uint8_t  wheel_pulse;   /* cumulative, wraps at 256 */
    uint8_t  phase;         /* CANF_PHASE_* */
    uint8_t  engine_state;  /* CANF_ENG_* */
    uint8_t  flags;         /* 4 bits */
    uint8_t  alive;         /* 4 bits */
} canf_sim_motion_t;

static inline void canf_sim_motion_pack(const canf_sim_motion_t *m, uint8_t d[8])
{
    canf_put_u16(&d[0], m->speed_x10);
    canf_put_u16(&d[2], m->rpm);
    d[4] = m->throttle_x2;
    d[5] = m->wheel_pulse;
    d[6] = (uint8_t)(((m->phase & 0x0Fu) << 4) | (m->engine_state & 0x0Fu));
    d[7] = (uint8_t)(((m->flags & 0x0Fu) << 4) | (m->alive & CANF_ALIVE_MASK));
}
static inline void canf_sim_motion_unpack(canf_sim_motion_t *m, const uint8_t d[8])
{
    m->speed_x10    = canf_get_u16(&d[0]);
    m->rpm          = canf_get_u16(&d[2]);
    m->throttle_x2  = d[4];
    m->wheel_pulse  = d[5];
    m->phase        = (uint8_t)(d[6] >> 4);
    m->engine_state = (uint8_t)(d[6] & 0x0Fu);
    m->flags        = (uint8_t)(d[7] >> 4);
    m->alive        = (uint8_t)(d[7] & CANF_ALIVE_MASK);
}

/* ---- 0x201 SIM_ANALOG ---- */
typedef struct {
    uint16_t coolant_raw;   /* 12-bit ADC after filter */
    uint16_t vbat_raw;      /* 12-bit ADC after filter */
    uint8_t  alive;
} canf_sim_analog_t;

static inline void canf_sim_analog_pack(const canf_sim_analog_t *a, uint8_t d[8])
{
    canf_put_u16(&d[0], a->coolant_raw);
    canf_put_u16(&d[2], a->vbat_raw);
    d[4] = 0; d[5] = 0; d[6] = 0;
    d[7] = (uint8_t)(a->alive & CANF_ALIVE_MASK);
}
static inline void canf_sim_analog_unpack(canf_sim_analog_t *a, const uint8_t d[8])
{
    a->coolant_raw = canf_get_u16(&d[0]);
    a->vbat_raw    = canf_get_u16(&d[2]);
    a->alive       = (uint8_t)(d[7] & CANF_ALIVE_MASK);
}

/* ---- 0x100 ECU_MOTION ---- */
typedef struct {
    uint16_t speed_x10;     /* NA = 0xFFFF */
    uint16_t rpm;           /* NA = 0xFFFF */
    uint8_t  throttle_x2;   /* NA = 0xFF */
    uint8_t  engine_state;  /* NA = 0xFF */
    uint8_t  alive;
} canf_ecu_motion_t;

static inline void canf_ecu_motion_pack(const canf_ecu_motion_t *m, uint8_t d[8])
{
    canf_put_u16(&d[0], m->speed_x10);
    canf_put_u16(&d[2], m->rpm);
    d[4] = m->throttle_x2;
    d[5] = m->engine_state;
    d[6] = 0;
    d[7] = (uint8_t)(m->alive & CANF_ALIVE_MASK);
}
static inline void canf_ecu_motion_unpack(canf_ecu_motion_t *m, const uint8_t d[8])
{
    m->speed_x10    = canf_get_u16(&d[0]);
    m->rpm          = canf_get_u16(&d[2]);
    m->throttle_x2  = d[4];
    m->engine_state = d[5];
    m->alive        = (uint8_t)(d[7] & CANF_ALIVE_MASK);
}

/* ---- 0x101 ECU_BODY ---- */
typedef struct {
    uint8_t  coolant;       /* degC + 40, NA = 0xFF */
    uint16_t vbat_mv;       /* NA = 0xFFFF */
    uint8_t  status;        /* CANF_ST_* */
    uint8_t  dtc_count;
    uint8_t  dtc_confirmed; /* CANF_DTC_* bitmask */
    uint8_t  dtc_pending;
    uint8_t  alive;
} canf_ecu_body_t;

static inline void canf_ecu_body_pack(const canf_ecu_body_t *b, uint8_t d[8])
{
    d[0] = b->coolant;
    canf_put_u16(&d[1], b->vbat_mv);
    d[3] = b->status;
    d[4] = b->dtc_count;
    d[5] = b->dtc_confirmed;
    d[6] = b->dtc_pending;
    d[7] = (uint8_t)(b->alive & CANF_ALIVE_MASK);
}
static inline void canf_ecu_body_unpack(canf_ecu_body_t *b, const uint8_t d[8])
{
    b->coolant       = d[0];
    b->vbat_mv       = canf_get_u16(&d[1]);
    b->status        = d[3];
    b->dtc_count     = d[4];
    b->dtc_confirmed = d[5];
    b->dtc_pending   = d[6];
    b->alive         = (uint8_t)(d[7] & CANF_ALIVE_MASK);
}



#endif /* COMMON_CAN_FRAME_H_ */

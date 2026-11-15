/*
 * can_ids.h
 *
 *  Created on: Nov 15, 2026
 *      Author: Quang Trung
 */

#ifndef COMMON_CAN_IDS_H_
#define COMMON_CAN_IDS_H_


/* Bus */
#define CANSYN_BITRATE_BPS      500000u

/* Identifiers (CAN 2.0A, 11-bit, DLC always 8) */
#define CANID_ECU_MOTION        0x100u  /* ECU -> bus, 20 ms  */
#define CANID_ECU_BODY          0x101u  /* ECU -> bus, 100 ms */
#define CANID_SIM_MOTION        0x200u  /* SIM -> ECU, 20 ms, offset 0 */
#define CANID_SIM_ANALOG        0x201u  /* SIM -> ECU, 50 ms, offset 5 ms */
#define CANID_OBD_FUNC_REQ      0x7DFu  /* DIAG -> ECU (functional request) */
#define CANID_OBD_PHYS_REQ      0x7E0u  /* DIAG -> ECU (flow control, physical) */
#define CANID_OBD_RESP          0x7E8u  /* ECU -> DIAG */

/* Periods and offsets in ms */
#define CANPER_ECU_MOTION_MS    20u
#define CANPER_ECU_BODY_MS      100u
#define CANPER_SIM_MOTION_MS    20u
#define CANPER_SIM_ANALOG_MS    50u
#define CANOFS_SIM_MOTION_MS    0u
#define CANOFS_SIM_ANALOG_MS    5u

/* Supervision timeouts used by the ECU (section 7.5) */
#define CANTO_SIM_MOTION_MS     100u
#define CANTO_SIM_ANALOG_MS     200u



#endif /* COMMON_CAN_IDS_H_ */

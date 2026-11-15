/* common/test_frames.c : host-side check of the frame layout. Build: gcc -std=c99 -I. test_frames.c */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "can_ids.h"
#include "can_frames.h"

int main(void)
{
    uint8_t d[8];

    /* Architecture 4.5 example: 50.0 km/h, 2550 rpm, 25 %, RUN, alive 5 */
    canf_ecu_motion_t m = { 500, 2550, 50, CANF_ENG_RUN, 5 };
    canf_ecu_motion_pack(&m, d);
    const uint8_t exp[8] = { 0xF4, 0x01, 0xF6, 0x09, 0x32, 0x02, 0x00, 0x05 };
    assert(memcmp(d, exp, 8) == 0);

    /* 0x200 round trip, phase/state and flags/alive nibbles */
    canf_sim_motion_t s = { 900, 3950, 120, 250, CANF_PHASE_CRUISE, CANF_ENG_RUN, 1, 15 }, s2;
    canf_sim_motion_pack(&s, d);
    canf_sim_motion_unpack(&s2, d);
    assert(memcmp(&s, &s2, sizeof s) == 0 && d[6] == 0x22 && d[7] == 0x1F);

    /* Nominal analog values from Architecture 6.4: 90 degC <-> 3250, 13.8 V <-> 3450 */
    assert(3250 / CANF_COOLANT_RAW_PER_C == 90 + CANF_COOLANT_OFFSET_C);
    assert(3450 * CANF_VBAT_MV_PER_LSB == 13800);
    assert(canf_alive_next(15) == 0);

    puts("ALL OK");
    return 0;
}
/* Host self-check for the FC frame parser/formatter.
 *   gcc -I../Core/Inc test_telemetry.c ../Core/Src/telemetry.c -o t && ./t
 * Not part of the STM32 build (only Core/ and Drivers/ are source folders). */
#include "telemetry.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    const char *fc = "$,011,125,42,0.10|-0.20|9.81,1.00|2.00|3.00,10.50|20.50|30.50,1.50|2.50|180.00,"
                     "512.30,948.12,27.40,0.30|0.40|-5.20,41400.0,18.520430,73.856743,560.0,9,0.30|0.40,"
                     "0.45,7.90,83.0,3,1A,2,0,#\r\n";
    FC_tlm t;
    char out[TLM_LINE_MAX];

    assert(tlm_parse(fc, &t) == 0);
    assert(strcmp(t.team_id, "011") == 0);
    assert(t.mission_time == 125 && t.packet_count == 42);
    assert(t.acc[2] > 9.8f && t.eul[2] == 180.0f && t.sats == 9 && t.state == 3 && t.e_flag == 0x1A);
    assert(t.prop_deploy == 2 && t.rw_status == 0);
    assert(t.lat > 18.52042 && t.lat < 18.52044);

    LoRa_tlm l;
    tlm_copy(&l, &t, 7);
    int n = tlm_format(&l, out, sizeof out);
    assert(n > 0 && strcmp(out, "$,011,125,7,0.10|-0.20|9.81,1.00|2.00|3.00,10.50|20.50|30.50,1.50|2.50|180.00,"
                                "512.30,948.12,27.40,0.30|0.40|-5.20,41400.0,18.520430,73.856743,560.0,9,0.30|0.40,"
                                "0.45,7.90,83.0,3,1A,2,0,#\r\n") == 0);

    assert(tlm_parse("$,011,125,42,0.1|0.2,#", &t) == -1);        /* truncated */
    assert(tlm_parse("$,011,125,42,0.10|-0.20|9.81,1.00|2.00|3.00,10.50|20.50|30.50,1.50|2.50|180.00,"
                     "512.30,948.12,27.40,0.30|0.40|-5.20,41400.0,18.520430,73.856743,560.0,9,0.30|0.40,"
                     "0.45,7.90,83.0,3,1A,#\r\n", &t) == -1);   /* old frame without the two new fields */
    assert(tlm_parse("$IMG,3,0,10,135,AAAA*4F", &t) == -1);         /* image link line */
    assert(tlm_parse("STATUS", &t) == -1);
    assert(tlm_format(&l, out, 50) == -1);                         /* too small */
    puts("telemetry ok");
    return 0;
}

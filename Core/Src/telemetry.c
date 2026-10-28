#include "telemetry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Each macro consumes one number and the separator that must follow it. */
#define NUM(dst, conv, sep) do { dst = conv; if (*end != (sep)) return -1; p = end + 1; } while (0)
#define F(dst, sep)  NUM(dst, strtof(p, &end), sep)
#define D(dst, sep)  NUM(dst, strtod(p, &end), sep)
#define I(dst, sep)  NUM(dst, (int)strtol(p, &end, 10), sep)
#define U(dst, sep)  NUM(dst, strtoul(p, &end, 10), sep)
#define F3(a)        do { F(a[0], '|'); F(a[1], '|'); F(a[2], ','); } while (0)

int tlm_parse(const char *line, FC_tlm *out)
{
    FC_tlm r;
    const char *p = strchr(line, '$');
    char *end;

    if (!p || p[1] != ',') return -1;
    p += 2;
    memset(&r, 0, sizeof r);

    end = strchr(p, ',');
    if (!end || (size_t)(end - p) >= sizeof r.team_id) return -1;
    memcpy(r.team_id, p, end - p);
    p = end + 1;

    U(r.mission_time, ',');
    U(r.packet_count, ',');
    F3(r.acc);
    F3(r.gyr);
    F3(r.mag);
    F3(r.eul);
    F(r.alt, ',');
    F(r.pres, ',');
    F(r.temp, ',');
    F3(r.vel);
    F(r.gnss_time, ',');
    D(r.lat, ',');
    D(r.lon, ',');
    F(r.gnss_alt, ',');
    I(r.sats, ',');
    F(r.gnss_vel[0], '|');
    F(r.gnss_vel[1], ',');
    F(r.current, ',');
    F(r.volt, ',');
    F(r.batt_pct, ',');
    I(r.state, ',');
    NUM(r.e_flag, strtoul(p, &end, 16), ',');
    I(r.prop_deploy, ',');
    r.rw_status = (int)strtol(p, &end, 10);
    if (end[0] != ',' || end[1] != '#') return -1;

    *out = r;
    return 0;
}

int tlm_format(const FC_tlm *t, char *out, size_t n)
{
    int len = snprintf(out, n,
        "$,%s,%lu,%lu,%.2f|%.2f|%.2f,%.2f|%.2f|%.2f,%.2f|%.2f|%.2f,%.2f|%.2f|%.2f,"
        "%.2f,%.2f,%.2f,%.2f|%.2f|%.2f,%.1f,%.6f,%.6f,%.1f,%d,%.2f|%.2f,%.2f,%.2f,%.1f,%d,%lX,%d,%d,#\r\n",
        t->team_id, (unsigned long)t->mission_time, (unsigned long)t->packet_count,
        t->acc[0], t->acc[1], t->acc[2],
        t->gyr[0], t->gyr[1], t->gyr[2],
        t->mag[0], t->mag[1], t->mag[2],
        t->eul[0], t->eul[1], t->eul[2],
        t->alt, t->pres, t->temp,
        t->vel[0], t->vel[1], t->vel[2],
        t->gnss_time, t->lat, t->lon, t->gnss_alt, t->sats,
        t->gnss_vel[0], t->gnss_vel[1],
        t->current, t->volt, t->batt_pct, t->state, (unsigned long)t->e_flag,
        t->prop_deploy, t->rw_status);
    return (len > 0 && (size_t)len < n) ? len : -1;
}

void tlm_copy(FC_tlm *dst, const FC_tlm *src, uint32_t packet_count)
{
    *dst = *src;
    dst->packet_count = packet_count;
}

/*
 * telemetry.h - flight computer telemetry packet
 *
 * FC line on USART3 (921600, 10 Hz), see FC README "Telemetry packet":
 * $,TEAM_ID,TIME,PKT,ACC(x|y|z),GYR(x|y|z),MAG(x|y|z),EUL(r|p|y),ALT,PRES,TEMP,
 *   VEL(x|y|z),GNSS_TIME,LAT,LON,GNSS_ALT,SATS,GNSS_VEL(x|y),CURRENT,VOLT,BATT%,STATE,EFLAG,#\r\n
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stddef.h>

#define TLM_LINE_MAX 320   /* FC frame is ~250 chars */

typedef struct {
    char     team_id[16];
    uint32_t mission_time;   /* s */
    uint32_t packet_count;
    float    acc[3];         /* m/s^2, x|y|z */
    float    gyr[3];         /* deg/s */
    float    mag[3];         /* uT */
    float    eul[3];         /* deg, roll|pitch|yaw */
    float    alt;            /* m, barometric */
    float    pres;           /* hPa */
    float    temp;           /* degC */
    float    vel[3];         /* m/s, east|north|vertical */
    float    gnss_time;      /* seconds of day */
    double   lat, lon;       /* decimal degrees */
    float    gnss_alt;       /* m */
    int      sats;
    float    gnss_vel[2];    /* m/s, east|north */
    float    current;        /* A */
    float    volt;           /* V */
    float    batt_pct;
    int      state;
    uint32_t e_flag;         /* hex bitfield */
} FC_tlm;

/* Per-radio packets: every FC_tlm field, but packet_count is that radio's own
 * transmit counter (see tlm_copy). Same layout as FC_tlm on purpose. */
typedef FC_tlm LoRa_tlm;
typedef FC_tlm Xbee_tlm;
typedef FC_tlm gsm_tlm;

/* Parse one FC line into *out. Returns 0 on success, -1 if the line is not a
 * complete telemetry frame (then *out is untouched). */
int tlm_parse(const char *line, FC_tlm *out);

/* Write the frame (same layout as the FC sends, with CRLF) into out.
 * Returns bytes written, or -1 if it does not fit. */
int tlm_format(const FC_tlm *t, char *out, size_t n);

/* dst = src, with dst->packet_count = packet_count. */
void tlm_copy(FC_tlm *dst, const FC_tlm *src, uint32_t packet_count);

#endif

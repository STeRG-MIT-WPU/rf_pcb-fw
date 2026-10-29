/* Host self-check for the telecommand filter in freertos.c (FC TELECOMMAND_HANDOFF.md rules).
 *   gcc -Istubs -I../Core/Inc test_telecommand.c ../Core/Src/telemetry.c -o t && ./t
 * Not part of the STM32 build. */
#include "../Core/Src/freertos.c"
#include <assert.h>

UART_HandleTypeDef huart2, huart3, huart4, huart5;
DMA_HandleTypeDef hdma_usart3_rx;
static char uart2_log[256];                     /* everything written to the XBee, concatenated */
static bool xbee_sim = true;                    /* fake XBee: answers in command mode when true */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *p, uint16_t n, uint32_t t)
{
    if (h != &huart2) return HAL_OK;
    strncat(uart2_log, (const char *)p, n);
    if (xbee_sim && xbee_at_state == XAT_RUNNING)
        xbee_at_collect((const uint8_t *)(n == 5 && !memcmp(p, "ATVR\r", 5) ? "3001\r" : "OK\r"), n == 5 && !memcmp(p, "ATVR\r", 5) ? 5 : 3);
    return HAL_OK;
}

static const char *run(const char *in, const char *want_line)
{
    tc_msg_t m;
    m.len = (uint8_t)strlen(in);
    memcpy(m.data, in, m.len);
    const char *st = tc_filter(&m);
    assert(m.len == strlen(want_line) && memcmp(m.data, want_line, m.len) == 0);
    return st;
}

int main(void)
{
    assert(!strcmp(run("buzz on\r\n", "BUZZ ON"), "OK"));                 /* upper-case, CRLF stripped */
    assert(!strcmp(run("  servo   1500 ", "SERVO 1500"), "OK"));          /* spaces collapsed */
    assert(!strcmp(run("CMD,011,prop heading,90", "PROP HEADING 90"), "OK")); /* ground CMD format translated */
    assert(!strcmp(run("TI\x08ME 1 2 3\x7f", "TIME 1 2 3"), "OK"));       /* non-printable dropped */
    assert(!strcmp(run("$,011,1,#", "$,011,1,#"), "BAD"));                /* never a '$' line */
    assert(!strcmp(run("\r\n", ""), "BAD"));
    assert(!strcmp(run("RESET", "RESET"), "ARM?"));                       /* irreversible, not armed */
    assert(!strcmp(run("SIM,1,100,0,0", "SIM,1,100,0,0"), "ARM?"));       /* commas kept on raw lines */
    assert(!strcmp(run("arm", "ARM"), "ARMED"));
    assert(!strcmp(run("BURN 500", "BURN 500"), "OK"));
    stub_tick += TC_ARM_MS;
    assert(!strcmp(run("DEPLOY", "DEPLOY"), "ARM?"));                      /* window closed */
    assert(!strcmp(run("CAPTURE", "CAPTURE"), "OK"));
    assert(!strcmp(run("PARA", "PARA"), "ARM?"));                          /* parachute release needs ARM too */
    assert(!strcmp(run("img link mqtt", "IMG LINK MQTT"), "BAD"));        /* 13.8 KB lines would not fit the ring */
    assert(!strcmp(run("IMG LINK XBEE", "IMG LINK XBEE"), "OK"));
    assert(tc_hold("GYRO CAL\r\n") == 13000 && tc_hold("SD CLEAR\r\n") == 3000 && tc_hold("SD\r\n") == 0);
    /* EC200U GNSS: AT+QGPSLOC=2 line -> GNSS fields; used when it sees more satellites than the FC's L89 */
    FC_tlm g = {0}, snap;
    assert(gnss_parse("\r\n+QGPSLOC: 061951.500,18.520430,73.856743,1.2,560.0,3,90.00,3.60,1.94,261026,12\r\n\r\nOK\r\n", &g));
    assert(g.sats == 12 && g.gnss_time == 22791.5f && g.lat > 18.52 && g.lon > 73.85 && g.gnss_alt == 560.0f);
    assert(g.gnss_vel[0] > 0.99f && g.gnss_vel[0] < 1.01f && g.gnss_vel[1] > -0.01f && g.gnss_vel[1] < 0.01f); /* 3.6 km/h east */
    assert(!gnss_parse("\r\n+CME ERROR: 516\r\n", &g));
    assert(!gnss_parse("+QGPSLOC: 061951.500,18.52", &g));
    fc_valid = true; fc_tlm.sats = 9; fc_tlm.lat = 1.0;
    gsm_gnss = g; gsm_gnss.sats = 9;
    assert(tlm_snapshot(&snap, 1) && snap.lat == 1.0 && snap.sats == 9);     /* tie: FC fix kept */
    gsm_gnss.sats = 10;
    assert(tlm_snapshot(&snap, 1) && snap.sats == 10 && snap.lat > 18.52 && snap.gnss_alt == 560.0f);
    gsm_gnss.sats = 0;                                                       /* modem lost its fix */
    assert(tlm_snapshot(&snap, 1) && snap.lat == 1.0);
    /* SMS telecommand: +CMT URC from SMS_CMD_FROM is queued, from anyone else dropped, both consumed */
    strcpy(gsm_acc, "\r\n+CMT: \"" SMS_CMD_FROM "\",,\"26/10/26,10:00:00+22\"\r\nbuzz on\r\n"
                    "\r\n+CMT: \"+910000000000\",,\"26/10/26,10:00:01+22\"\r\nRESET\r\n");
    gsm_acc_len = strlen(gsm_acc);
    stub_q[0] = 0;
    assert(!gsm_handle_urc());
    assert(((tc_msg_t *)stub_q)->len == 9 && memcmp(((tc_msg_t *)stub_q)->data, "buzz on\r\n", 9) == 0);
    assert(strstr(gsm_acc, "+CMT:") == NULL && strstr(gsm_acc, "RESET") == NULL);
    strcpy(gsm_acc, "\r\n+CMT: \"" SMS_CMD_FROM "\",,\"26/10/26,10:00:02+22\"\r\nSER");    /* text still arriving */
    gsm_acc_len = strlen(gsm_acc);
    stub_q[0] = 0;
    assert(!gsm_handle_urc() && stub_q[0] == 0 && strstr(gsm_acc, "+CMT:") != NULL);
    /* XBee AT telecommands (rf 4 reference): an "AT" line on USART2 configures the XBee, nothing goes to the FC */
    assert(!xbee_at_queue("BUZZ ON\r\n", 9) && xbee_at_state == XAT_IDLE);
    assert(xbee_at_queue("\r\nATCH 11\r\n", 11) && xbee_at_state == XAT_PENDING && !strcmp(xbee_at_cmd, "ATCH 11"));
    assert(xbee_at_queue("ATID\r", 5) && !strcmp(xbee_at_cmd, "ATCH 11"));            /* busy: dropped */
    uart2_log[0] = 0; xbee_at_run();                                                   /* RF change: early OK, then ATWR */
    assert(!strcmp(uart2_log, "RESP|ATCH|OK|ACCEPTED\r\n+++ATCH 11\rATWR\rATCN\r") && xbee_at_state == XAT_IDLE);
    assert(xbee_at_queue("ATVR\r", 5)); uart2_log[0] = 0; xbee_at_run();               /* query: no ATWR, value in RESP */
    assert(!strcmp(uart2_log, "+++ATVR\rATCN\rRESP|ATVR|OK|3001\r\n"));
    assert(xbee_at_queue("ATID\r", 5)); uart2_log[0] = 0; xbee_at_run();               /* RF register query: early reply, no ATWR (reference quirk) */
    assert(!strcmp(uart2_log, "RESP|ATID|OK|ACCEPTED\r\n+++ATID\rATCN\r"));
    xbee_sim = false;                                                                  /* module silent: abort, no ATCN */
    assert(xbee_at_queue("ATWR\r", 5)); uart2_log[0] = 0; xbee_at_run();
    assert(!strcmp(uart2_log, "+++RESP|ATWR|ERROR|NO_CMD_MODE\r\n") && xbee_at_state == XAT_IDLE);
    puts("telecommand ok");
    return 0;
}

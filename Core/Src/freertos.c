/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "telemetry.h"
#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---------------- tunables ---------------- */
#define LORA_PERIOD_MS      3000    /* E22 air rate 2.4 kbps: one ~280 B frame is ~1.15 s on air (half duplex); the rest is the telecommand window */
#define XBEE_PERIOD_MS      200
#define GSM_PERIOD_MS       66      /* ~15 Hz; see note in gsm_task */
#define SMS_PERIOD_MS       60000
#define GSM_BOOT_MS         5000    /* EC200U power-up before the first AT */
#define GSM_PUB_TIMEOUT_MS  200     /* wait for +QMTPUBEX; a slow ack is not a link failure */
#define GSM_MAX_FAILS       8       /* consecutive publish failures -> reconnect */
#define MQTT_RETRY_MS       3000    /* pause between broker connect attempts (never gives up) */
#define MQTT_RETRIES_PER_NET 5      /* after this many broker misses, redo the network bring-up as well */
#define SMS_TIMEOUT_MS      8000    /* +CMGS can take seconds; publishing pauses meanwhile */
#define LORA_CHUNK          240     /* E22 UART sub-packet: send 240, pause, send the rest (rf_txtest build) */
#define LORA_CHUNK_GAP_MS   1000    /* first chunk must be fully on air before the tail (was 500 at 62.5 kbps) */
#define XBEE_CTS_WAIT_MS    50
#define XBEE_PIECE          256     /* bytes per XBee write between nCTS checks (one RF packet) */
#define XBEE_IMG_MARGIN_MS  25      /* stop relaying image lines this close to the next telemetry slot */
#define XBEE_AT_MAX         64      /* longest XBee AT telecommand, e.g. "ATCH 11" (rf 4 reference values) */
#define XBEE_AT_GUARD_MS    1100    /* silence required before "+++" */
#define XBEE_AT_RESP_MS     2000    /* wait for each XBee reply */
#define XBEE_AT_SETTLE_MS   100     /* pause after ATCN before relaying again */
#define GSM_IMG_PER_PERIOD  1       /* image lines published per GSM period (~15 lines/s) */
#define IMG_RING            192     /* buffered $IMG lines: 270 KB static in RAM3 (384 KB), ~190 KB of image */
#define IMG_LINE_MAX        1408    /* $IMG line of 1024 raw bytes (IMG LINK XBEE, FC default) is ~1400 chars.
                                       IMG LINK MQTT lines (~13.8 KB) do not fit: that command is blocked */

#define GSM_APN         "airtelgprs.com"
#define MQTT_BROKER     "broker.hivemq.com"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "cansat_rf_011"
#define MQTT_TOPIC_TLM  "cansat/011/tlm"
#define MQTT_TOPIC_CMD  "cansat/011/cmd"
#define MQTT_TOPIC_IMG  "cansat/011/img"
#define MQTT_TOPIC_ACK  "cansat/011/ack"   /* "OK <cmd>" when forwarded, "ARMED", "ARM? <cmd>", "BAD <cmd>" */
#define SMS_NUMBER      "+916375605728"
#define SMS_CMD_FROM    "+919373986194"  /* only SMS from this number are telecommands; "" accepts any sender */

#define TC_MAX          120     /* longest telecommand line incl. CRLF (FC limit 127) */
#define TC_GAP_MS       20      /* FC asks for >= 20 ms between command lines */
#define TC_ARM_MS       60000   /* ARM opens this window for RESET/BURN/DEPLOY/PARA/STATE/PERSIST/SD/BENCH/DIAG/SIM */
#define FC_DMA_BUF      2048    /* 1.4 KB $IMG lines arrive back to back at 921600 (10 ms apart) */
#define GSM_ACC_MAX     1024
#define GNSS_PERIOD_MS  1000    /* AT+QGPSLOC poll of the EC200U GNSS; blocks the gsm loop ~100 ms each time */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* RF board application tasks live here (see RF_Tasks_Init at the bottom):
 *   fc_comms  USART3 (DMA)  parse FC telemetry, queue $IMG lines, relay ground-station commands to the FC
 *   lora      UART5         E22-900T22D, telemetry every 3 s
 *   xbee      USART2        XBee Pro, telemetry at 5 Hz + image lines in between
 *   gsm       UART4         EC200U, MQTT at 15 Hz + image lines + SMS every 60 s
 */
extern UART_HandleTypeDef huart2, huart3, huart4, huart5;
extern DMA_HandleTypeDef  hdma_usart3_rx;

/* ---------------- shared state ---------------- */
static FC_tlm        fc_tlm;              /* latest packet from the FC */
static bool          fc_valid;
static FC_tlm        gsm_gnss;            /* EC200U fix, GNSS fields only; sats 0 = no fix. Guarded by fc_mutex */
static osMutexId_t   fc_mutex;

typedef struct { uint8_t len; char data[TC_MAX]; } tc_msg_t;
static osMessageQueueId_t tc_q;           /* radios/ISRs -> fc_comms -> FC */
static osMessageQueueId_t ack_q;          /* fc_comms -> gsm -> MQTT_TOPIC_ACK */
static uint32_t           tc_armed_until; /* tick until which irreversible commands pass */

static osThreadId_t fc_task_h, gsm_task_h;
enum { FLAG_RX = 1, FLAG_RESET = 2 };

/* Image relay ring: fc_comms writes $IMGSTART/$IMG/$IMGEND lines verbatim (+CRLF), xbee and gsm
 * each read at their own pace. Sequence numbers, slot = seq % IMG_RING; full when the slowest
 * reader is IMG_RING behind. IMGACK,<id>,OK goes to the FC once both readers passed $IMGEND. */
typedef struct { uint16_t len; char data[IMG_LINE_MAX]; } img_line_t;
static img_line_t        img_ring[IMG_RING];
static volatile uint32_t img_w, img_r_xbee, img_r_gsm;
static bool              img_drop;             /* a line of the current image did not fit */
static int               img_ack_id = -1;      /* image waiting for its OK ack, or -1 */
static uint32_t          img_ack_seq;          /* readers must reach this seq before the OK */

static uint8_t fc_dma_buf[FC_DMA_BUF];
static uint8_t lora_rx[TC_MAX], xbee_rx[TC_MAX], gsm_rx[128];

/* XBee AT telecommand (ported from the rf 4 reference main.c): a USART2 line starting with "AT" configures
 * the XBee itself instead of going to the FC. The ISR queues it (XAT_PENDING); xbee_task runs the sequence
 * (XAT_RUNNING: USART2 RX is the module's reply and goes to xbee_at_acc, not to the FC). One at a time. */
enum { XAT_IDLE, XAT_PENDING, XAT_RUNNING };
static volatile uint8_t  xbee_at_state;
static char              xbee_at_cmd[XBEE_AT_MAX];
static char              xbee_at_acc[64];
static volatile uint8_t  xbee_at_acc_len;

/* EC200U response accumulator: ISR appends, gsm task reads/clears under a critical section */
static char              gsm_acc[GSM_ACC_MAX];
static volatile uint16_t gsm_acc_len;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* ---------------- helpers ---------------- */
/* Fixed-rate sleep. Resyncs instead of bursting if a cycle overran. */
static void sleep_until(uint32_t *next, uint32_t period)
{
    *next += period;
    int32_t left = (int32_t)(*next - osKernelGetTickCount());
    if (left > 0) osDelay(left);
    else *next = osKernelGetTickCount();
}

static bool tlm_snapshot(FC_tlm *dst, uint32_t packet_count)
{
    if (!fc_valid) return false;
    osMutexAcquire(fc_mutex, osWaitForever);
    tlm_copy(dst, &fc_tlm, packet_count);
    if (gsm_gnss.sats > dst->sats) {       /* ponytail: more satellites wins; compare HDOP too if both fixes are weak */
        dst->gnss_time = gsm_gnss.gnss_time;
        dst->lat = gsm_gnss.lat;
        dst->lon = gsm_gnss.lon;
        dst->gnss_alt = gsm_gnss.gnss_alt;
        dst->sats = gsm_gnss.sats;
        dst->gnss_vel[0] = gsm_gnss.gnss_vel[0];
        dst->gnss_vel[1] = gsm_gnss.gnss_vel[1];
    }
    osMutexRelease(fc_mutex);
    return true;
}

/* Queue a telecommand for the FC. Strips CR/LF, appends CRLF (FC console ends lines on Enter).
 * ISR-safe (timeout 0). */
static void tc_push(const char *s, size_t len)
{
    tc_msg_t m;
    while (len && (s[len - 1] == '\r' || s[len - 1] == '\n')) len--;
    if (!len || len > TC_MAX - 2) return;
    memcpy(m.data, s, len);
    m.data[len++] = '\r';
    m.data[len++] = '\n';
    m.len = len;
    osMessageQueuePut(tc_q, &m, 0, 0);
}

/* ---------------- telecommands (FC TELECOMMAND_HANDOFF.md, sections 2, 4, 7) ----------------
 * Normalise a ground-station line for the FC console and decide whether to forward it. Accepts the
 * raw console words ("BUZZ ON") or CMD,<team>,<verb>,<arg,...>. Upper-cases, keeps printable ASCII
 * only, collapses spaces, refuses '$' lines. Irreversible words, DIAG and SIM need an ARM line first
 * (ARM is consumed here, never sent to the FC). Leaves the clean line in m (no CRLF) and returns the
 * ack word for the ground: "OK" = forwarded, "ARMED", "ARM?" = blocked, "BAD" = unusable. */
static const char *tc_filter(tc_msg_t *m)
{
    /* ponytail: whole-word check, so read-only STATE / PERSIST / SD also need ARM; they only print on USB anyway */
    static const char *const armed_only[] = { "RESET", "BURN", "PAYLOAD", "DEPLOY", "STATE", "PERSIST",
                                              "SD", "BENCH", "DIAG", "SIM", "PARA", NULL };
    char *d = m->data;
    uint8_t n = 0, i = 0;
    bool cmd_fmt = m->len > 4 && strncmp(d, "CMD,", 4) == 0;

    if (cmd_fmt) {                                  /* CMD,<team>,VERB,ARG -> VERB ARG */
        int commas = 0;
        while (i < m->len && commas < 2) if (d[i++] == ',') commas++;
    }
    for (; i < m->len; i++) {
        char c = d[i];
        if (cmd_fmt && c == ',') c = ' ';
        if (c < 0x20 || c > 0x7e) continue;
        if (c == ' ' && (n == 0 || d[n - 1] == ' ')) continue;
        d[n++] = (char)toupper((unsigned char)c);
    }
    while (n && d[n - 1] == ' ') n--;
    m->len = n;
    if (n == 0 || d[0] == '$' || n > TC_MAX - 2) return "BAD";
    d[n] = 0;                                       /* queue payloads carry no terminator */
    if (strcmp(d, "IMG LINK MQTT") == 0) return "BAD";  /* its ~13.8 KB $IMG lines exceed IMG_LINE_MAX */

    size_t w = strcspn(d, " ,");                    /* SIM,t,alt,... has no space after the word */
    if (w == 3 && strncmp(d, "ARM", 3) == 0) {
        tc_armed_until = osKernelGetTickCount() + TC_ARM_MS;
        return "ARMED";
    }
    for (int k = 0; armed_only[k]; k++)
        if (strlen(armed_only[k]) == w && strncmp(d, armed_only[k], w) == 0 &&
            (int32_t)(tc_armed_until - osKernelGetTickCount()) <= 0)
            return "ARM?";
    return "OK";
}

/* Queue "<status> <line>" for the gsm task to publish on MQTT_TOPIC_ACK (handoff section 8, option A). */
/* The FC's command task blocks on these (handoff 4.4, 10); hold our queue meanwhile so nothing
 * piles into its 512-byte RX ring. Returns the extra ms to wait after sending the line. */
static uint32_t tc_hold(const char *line)
{
    if (strncmp(line, "GYRO CAL", 8) == 0) return 13000;   /* waits for gyro status 3, then averages 3 s */
    if (strncmp(line, "SD CLEAR", 8) == 0) return 3000;    /* a few seconds with a full card */
    return 0;
}

static void tc_ack(const char *status, const tc_msg_t *m)
{
    tc_msg_t a;
    int n = snprintf(a.data, sizeof a.data, "%s %.*s", status, (int)m->len, m->data);
    a.len = (uint8_t)(n < (int)sizeof a.data ? n : (int)sizeof a.data - 1);
    osMessageQueuePut(ack_q, &a, 0, 0);
}

/* ---------------- XBee AT telecommands, ISR side ---------------- */
/* "AT..." line -> xbee_at_cmd (no CR/LF), true. Anything else -> false, it is for the FC.
 * While a command is queued or running, further AT lines are dropped (reference: "busy, dropped"). */
static bool xbee_at_queue(const char *s, size_t n)
{
    size_t len = 0;
    while (n && (*s == '\r' || *s == '\n' || *s == ' ')) s++, n--;
    if (n < 2 || s[0] != 'A' || s[1] != 'T') return false;
    if (xbee_at_state != XAT_IDLE) return true;
    while (len < n && len < XBEE_AT_MAX - 1 && s[len] != '\r' && s[len] != '\n') len++;
    memcpy(xbee_at_cmd, s, len);
    xbee_at_cmd[len] = 0;
    xbee_at_state = XAT_PENDING;
    return true;
}

static void xbee_at_collect(const uint8_t *s, size_t n)
{
    if (xbee_at_acc_len + n < sizeof xbee_at_acc) {
        memcpy(xbee_at_acc + xbee_at_acc_len, s, n);
        xbee_at_acc_len += n;
        xbee_at_acc[xbee_at_acc_len] = 0;
    }
}

/* ---------------- UART callbacks ---------------- */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size)
{
    if (h == &huart3) {                     /* circular DMA keeps running; wake the parser */
        if (fc_task_h) osThreadFlagsSet(fc_task_h, FLAG_RX);
    } else if (h == &huart4) {
        if (gsm_acc_len + size < GSM_ACC_MAX) {
            memcpy(gsm_acc + gsm_acc_len, gsm_rx, size);
            gsm_acc_len += size;
            gsm_acc[gsm_acc_len] = 0;
        }
        if (gsm_task_h) osThreadFlagsSet(gsm_task_h, FLAG_RX);
        HAL_UARTEx_ReceiveToIdle_IT(&huart4, gsm_rx, sizeof gsm_rx);
    } else if (h == &huart5) {              /* LoRa: transparent mode, payload is the command */
        tc_push((char *)lora_rx, size);     /* ponytail: a command split across two IDLE events becomes two lines */
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, lora_rx, sizeof lora_rx);
    } else if (h == &huart2) {              /* XBee: transparent mode; "AT..." lines configure the XBee itself */
        if (xbee_at_state == XAT_RUNNING) xbee_at_collect(xbee_rx, size);
        else if (!xbee_at_queue((char *)xbee_rx, size)) tc_push((char *)xbee_rx, size);
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, xbee_rx, sizeof xbee_rx);
    }
}

/* Overrun/framing errors abort HAL reception: re-arm, never let a port go deaf. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    __HAL_UART_CLEAR_OREFLAG(h);
    if (h == &huart3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, fc_dma_buf, sizeof fc_dma_buf);
        if (fc_task_h) osThreadFlagsSet(fc_task_h, FLAG_RESET);
    } else if (h == &huart4) HAL_UARTEx_ReceiveToIdle_IT(&huart4, gsm_rx, sizeof gsm_rx);
    else if (h == &huart5)   HAL_UARTEx_ReceiveToIdle_IT(&huart5, lora_rx, sizeof lora_rx);
    else if (h == &huart2)   HAL_UARTEx_ReceiveToIdle_IT(&huart2, xbee_rx, sizeof xbee_rx);
}

/* ---------------- image relay ring ---------------- */
static uint32_t img_min_read(void)
{
    uint32_t a = img_r_xbee, b = img_r_gsm;
    return a < b ? a : b;
}

/* Next unsent line for a reader, or NULL. The reader bumps *r after it has sent the line. */
static const img_line_t *img_peek(volatile uint32_t *r)
{
    return *r != img_w ? &img_ring[*r % IMG_RING] : NULL;
}

static void img_ack(int id, bool ok)
{
    char s[24];
    int n = snprintf(s, sizeof s, "IMGACK,%d,%s\r\n", id, ok ? "OK" : "FAIL");
    HAL_UART_Transmit(&huart3, (uint8_t *)s, n, 50);
}

static void img_push(const char *line)
{
    size_t n = strlen(line);
    if (strncmp(line, "$IMGSTART,", 10) == 0) { img_drop = false; img_ack_id = -1; }

    if (n + 2 > IMG_LINE_MAX || img_w - img_min_read() >= IMG_RING) {
        img_drop = true;                                   /* radios too slow for this image: it will FAIL */
    } else {
        img_line_t *l = &img_ring[img_w % IMG_RING];
        memcpy(l->data, line, n);
        l->data[n] = '\r';
        l->data[n + 1] = '\n';
        l->len = n + 2;
        __DMB();                                           /* data before the readers see the new seq */
        img_w++;
    }

    if (strncmp(line, "$IMGEND,", 8) == 0) {
        int id = (int)strtoul(line + 8, NULL, 10);
        if (img_drop) img_ack(id, false);                  /* FC resends the image */
        else { img_ack_id = id; img_ack_seq = img_w; }     /* OK once both radios drained it */
    }
}

/* ---------------- 1. fc_comms ---------------- */
static void fc_handle_line(const char *line)
{
    FC_tlm t;
    if (strncmp(line, "$IMG", 4) == 0) { img_push(line); return; }
    if (line[0] != '$' || line[1] != ',') return;   /* console echo etc.: not handled here */
    if (tlm_parse(line, &t) != 0) return;
    osMutexAcquire(fc_mutex, osWaitForever);
    fc_tlm = t;
    fc_valid = true;
    osMutexRelease(fc_mutex);
}

static void fc_comms_task(void *arg)
{
    static char line[IMG_LINE_MAX];
    uint16_t tail = 0, ll = 0;
    uint32_t tc_last = 0;
    tc_msg_t m;

    fc_task_h = osThreadGetId();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, fc_dma_buf, sizeof fc_dma_buf);

    for (;;) {
        uint32_t f = osThreadFlagsWait(FLAG_RX | FLAG_RESET, osFlagsWaitAny, 20);
        if (!(f & 0x80000000u) && (f & FLAG_RESET)) { tail = 0; ll = 0; }

        uint16_t head = FC_DMA_BUF - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        while (tail != head) {
            char c = (char)fc_dma_buf[tail];
            tail = (tail + 1) % FC_DMA_BUF;
            if (c == '\n') {
                line[ll] = 0;
                fc_handle_line(line);
                ll = 0;
            } else if (c != '\r') {
                if (ll < sizeof line - 1) line[ll++] = c;
                else ll = 0;                            /* garbage longer than a frame: drop it */
            }
        }

        /* one command per pass, at least TC_GAP_MS apart (the FC polls its 512-byte ring every 20 ms) */
        if ((int32_t)(osKernelGetTickCount() - tc_last) >= TC_GAP_MS &&
            osMessageQueueGet(tc_q, &m, NULL, 0) == osOK) {
            const char *st = tc_filter(&m);
            tc_ack(st, &m);
            if (st[0] == 'O') {
                m.data[m.len++] = '\r';
                m.data[m.len++] = '\n';
                HAL_UART_Transmit(&huart3, (uint8_t *)m.data, m.len, 50);
                tc_last = osKernelGetTickCount() + tc_hold(m.data);
            }
        }

        if (img_ack_id >= 0 && img_min_read() >= img_ack_seq) {
            img_ack(img_ack_id, true);
            img_ack_id = -1;
        }
    }
}

/* ---------------- 2. LoRa ---------------- */
static void lora_task(void *arg)
{
    static char buf[TLM_LINE_MAX];
    LoRa_tlm t;
    uint32_t count = 0, next = osKernelGetTickCount();

    HAL_UARTEx_ReceiveToIdle_IT(&huart5, lora_rx, sizeof lora_rx);
    for (;;) {
        if (tlm_snapshot(&t, count + 1)) {
            int len = tlm_format(&t, buf, sizeof buf);
            if (len > 0) {
                int first = len > LORA_CHUNK ? LORA_CHUNK : len;
                HAL_UART_Transmit(&huart5, (uint8_t *)buf, first, 200);
                if (len > first) {
                    osDelay(LORA_CHUNK_GAP_MS);
                    HAL_UART_Transmit(&huart5, (uint8_t *)buf + first, len - first, 200);
                }
                count++;
            }
        }
        sleep_until(&next, LORA_PERIOD_MS);
    }
}

/* ---------------- 3. XBee ---------------- */
static bool xbee_write(const char *s, int len)
{
    /* XBee nCTS high = its UART buffer is nearly full: check before each XBEE_PIECE, wait a little, then send anyway */
    for (int off = 0; off < len; off += XBEE_PIECE) {
        int n = len - off < XBEE_PIECE ? len - off : XBEE_PIECE;
        for (int i = 0; i < XBEE_CTS_WAIT_MS / 5 && HAL_GPIO_ReadPin(XBEE_nCTS_GPIO_Port, XBEE_nCTS_Pin) == GPIO_PIN_SET; i++)
            osDelay(5);
        if (HAL_UART_Transmit(&huart2, (uint8_t *)s + off, n, 100) != HAL_OK) return false;
    }
    return len > 0;
}

/* ---------------- XBee AT telecommands, task side ---------------- */
static void xbee_at_clear(void)
{
    taskENTER_CRITICAL();
    xbee_at_acc_len = 0;
    xbee_at_acc[0] = 0;
    taskEXIT_CRITICAL();
}

/* Wait for one reply line from the XBee (ends in '\r'); out gets it without CR/LF. false on timeout. */
static bool xbee_at_wait(char *out, size_t n, uint32_t timeout_ms)
{
    uint32_t deadline = osKernelGetTickCount() + timeout_ms;
    for (;;) {
        bool done = false;
        taskENTER_CRITICAL();
        const char *p = xbee_at_acc;
        while (*p == '\r' || *p == '\n') p++;
        if (*p && strchr(p, '\r')) {
            size_t l = strcspn(p, "\r\n");
            if (l >= n) l = n - 1;
            memcpy(out, p, l);
            out[l] = 0;
            xbee_at_acc_len = 0;
            xbee_at_acc[0] = 0;
            done = true;
        }
        taskEXIT_CRITICAL();
        if (done) return true;
        if ((int32_t)(deadline - osKernelGetTickCount()) <= 0) return false;
        osDelay(10);
    }
}

/* "RESP|<cmd>|<status>|<detail>\r\n" to the ground; the XBee must be in transparent mode. */
static void xbee_at_resp(const char *status, const char *detail)
{
    char buf[96];
    int n = snprintf(buf, sizeof buf, "RESP|%.4s|%s|%s\r\n", xbee_at_cmd, status, detail);
    xbee_write(buf, n < (int)sizeof buf ? n : (int)sizeof buf - 1);
}

/* Run the queued AT telecommand, same sequence as the rf 4 reference but blocking in this task (telemetry
 * and image lines pause, so the XBee sees the guard silence):
 *   guard -> "+++" (OK) -> "<cmd>\r" -> "ATWR\r" only when a value was set (never for ATWR/ATCN/ATFR/ATRE
 *   or plain queries) -> "ATCN\r" -> settle -> RESP|<cmd>|OK/ERROR|<detail>
 * RF-link changes (ATCH/ATID/ATDH/ATDL/ATZS/ATEE/ATEO/ATKY) are answered "OK|ACCEPTED" before they run,
 * because afterwards the link may no longer work; ATFR gets no reply (module reboots). */
static void xbee_at_run(void)
{
    static const char *const rf_change[] = { "ATCH", "ATID", "ATDH", "ATDL", "ATZS", "ATEE", "ATEO", "ATKY", NULL };
    static const char *const no_wr[]     = { "ATWR", "ATCN", "ATFR", "ATRE", NULL };
    const char *cmd = xbee_at_cmd, *status = "OK", *detail = "-";
    char line[XBEE_AT_MAX + 2], resp[32], result[32];
    bool early = false;
    int k;

    for (k = 0; rf_change[k]; k++) if (strncmp(cmd, rf_change[k], 4) == 0) early = true;
    if (early) xbee_at_resp("OK", "ACCEPTED");

    xbee_at_state = XAT_RUNNING;                    /* USART2 RX now feeds xbee_at_acc */
    osDelay(XBEE_AT_GUARD_MS);
    xbee_at_clear();                                /* discard anything received so far; keep only the reply */
    if (!xbee_write("+++", 3)) { status = "ERROR"; detail = "UART"; }
    else if (!xbee_at_wait(resp, sizeof resp, XBEE_AT_RESP_MS) || !strstr(resp, "OK")) {
        status = "ERROR"; detail = "NO_CMD_MODE";   /* reference aborts here without ATCN */
    } else {
        int n = snprintf(line, sizeof line, "%s\r", cmd);
        if (!xbee_write(line, n)) { status = "ERROR"; detail = "UART"; }
        else if (!xbee_at_wait(result, sizeof result, XBEE_AT_RESP_MS)) { status = "ERROR"; detail = "TIMEOUT"; }
        else {
            detail = result;                        /* first reply line, e.g. "OK" or "3001" */
            if (strstr(result, "ERR")) status = "ERROR";
            else {
                bool wr = strlen(cmd) > 4;          /* a value was set: save it to flash */
                for (k = 0; no_wr[k]; k++) if (strcmp(cmd, no_wr[k]) == 0) wr = false;
                if (wr && (!xbee_write("ATWR\r", 5) || !xbee_at_wait(resp, sizeof resp, XBEE_AT_RESP_MS) ||
                           !strstr(resp, "OK"))) {
                    status = "ERROR"; detail = "ATWR_FAIL";
                }
            }
        }
        if (strcmp(cmd, "ATCN") != 0) xbee_write("ATCN\r", 5);
        osDelay(XBEE_AT_SETTLE_MS);                 /* the OK to ATCN arrives meanwhile and is dropped */
    }
    xbee_at_clear();
    xbee_at_state = XAT_IDLE;
    if (!early && strcmp(cmd, "ATFR") != 0) xbee_at_resp(status, detail);
}

static void xbee_task(void *arg)
{
    static char buf[TLM_LINE_MAX];
    Xbee_tlm t;
    const img_line_t *l;
    uint32_t count = 0, next = osKernelGetTickCount();

    HAL_GPIO_WritePin(XBEE_nRTS_GPIO_Port, XBEE_nRTS_Pin, GPIO_PIN_RESET);   /* we are always ready to receive */
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, xbee_rx, sizeof xbee_rx);
    for (;;) {
        if (xbee_at_state == XAT_PENDING) xbee_at_run();   /* ~1.5-3 s with no telemetry on XBee */
        if (tlm_snapshot(&t, count + 1) && xbee_write(buf, tlm_format(&t, buf, sizeof buf))) count++;

        /* image lines fill the rest of the 200 ms slot: ~125 ms per 1.4 KB line at 115200 (len/10 ms), so one per slot */
        while ((l = img_peek(&img_r_xbee)) &&
               (int32_t)(next + XBEE_PERIOD_MS - osKernelGetTickCount()) > XBEE_IMG_MARGIN_MS + l->len / 10) {
            xbee_write(l->data, l->len);
            img_r_xbee++;
        }
        sleep_until(&next, XBEE_PERIOD_MS);
    }
}

/* ---------------- 4. GSM (EC200U) ---------------- */
static void gsm_send(const char *s, size_t n)
{
    HAL_UART_Transmit(&huart4, (const uint8_t *)s, n, 200);
}

static void gsm_clear(void)
{
    taskENTER_CRITICAL();
    gsm_acc_len = 0;
    gsm_acc[0] = 0;
    taskEXIT_CRITICAL();
}

static bool gsm_has(const char *s)
{
    bool hit;
    taskENTER_CRITICAL();
    hit = strstr(gsm_acc, s) != NULL;
    taskEXIT_CRITICAL();
    return hit;
}

/* Pull "+QMTRECV: 0,<id>,"<topic>","<payload>"" out of the accumulator and queue the payload
 * for the FC. Same for an SMS delivered as "+CMT: "<sender>",,"<time>"\r\n<text>\r\n" (AT+CNMI=2,2),
 * if the sender is SMS_CMD_FROM. Also reports a dropped MQTT link (+QMTSTAT). */
static bool gsm_handle_urc(void)
{
    char cmd[TC_MAX];
    for (;;) {
        size_t n = 0;
        char *p, *hdr, *body;
        bool from_ok = false;
        taskENTER_CRITICAL();
        p = strstr(gsm_acc, "+CMT:");
        if (p && (hdr = strstr(p, "\r\n")) && (body = strstr(hdr + 2, "\r\n"))) {   /* header and text complete */
            *hdr = 0;
            from_ok = !SMS_CMD_FROM[0] || strstr(p, SMS_CMD_FROM) != NULL;
            n = (size_t)(body - hdr - 2);
            if (n > sizeof cmd) n = sizeof cmd;
            memcpy(cmd, hdr + 2, n);
            memmove(p, body + 2, strlen(body + 2) + 1);
            gsm_acc_len = strlen(gsm_acc);
        }
        taskEXIT_CRITICAL();
        if (!p || !n) break;                                /* none, one still arriving, or empty text */
        if (from_ok) tc_push(cmd, n);                       /* ponytail: a multi-line SMS forwards its first line only */
    }
    for (;;) {
        size_t n = 0;
        char *p, *q[4] = {0};
        taskENTER_CRITICAL();
        p = strstr(gsm_acc, "+QMTRECV:");
        if (p) {
            char *s = p;
            for (int i = 0; i < 4 && s; i++) { s = strchr(s, '"'); q[i] = s; if (s) s++; }
            if (q[3]) {                                     /* complete URC: copy payload, consume it */
                n = (size_t)(q[3] - q[2] - 1);
                if (n > sizeof cmd) n = sizeof cmd;
                memcpy(cmd, q[2] + 1, n);
                memmove(p, q[3] + 1, strlen(q[3] + 1) + 1);
                gsm_acc_len = strlen(gsm_acc);
            }
        }
        taskEXIT_CRITICAL();
        if (!n) break;                                      /* none left, or one still arriving */
        tc_push(cmd, n);
    }
    return gsm_has("+QMTSTAT:");
}

/* Wait until `expect` shows up in the response. "ERROR" or a timeout returns false. */
static bool gsm_wait(const char *expect, uint32_t timeout_ms)
{
    uint32_t deadline = osKernelGetTickCount() + timeout_ms;
    for (;;) {
        if (gsm_has(expect)) return true;
        if (gsm_has("ERROR")) return false;
        int32_t left = (int32_t)(deadline - osKernelGetTickCount());
        if (left <= 0) return false;
        osThreadFlagsWait(FLAG_RX, osFlagsWaitAny, left);
    }
}

static bool gsm_cmd(const char *cmd, const char *expect, uint32_t timeout_ms)
{
    gsm_handle_urc();
    gsm_clear();
    gsm_send(cmd, strlen(cmd));
    return gsm_wait(expect, timeout_ms);
}

/* "+QGPSLOC: hhmmss.sss,lat,lon,hdop,alt,fix,cog,spkm,spkn,ddmmyy,nsat" (AT+QGPSLOC=2) -> GNSS fields of *g.
 * Velocity east|north from speed over ground and course. false if the line is missing or malformed. */
static bool gnss_parse(const char *s, FC_tlm *g)
{
    double v[11];
    char *end;
    if (!(s = strstr(s, "+QGPSLOC:"))) return false;
    s += 9;
    for (int i = 0; i < 11; i++) {
        v[i] = strtod(s, &end);
        if (i < 10 && *end != ',') return false;
        s = end + 1;
    }
    int hh = (int)(v[0] / 10000), mm = (int)(v[0] / 100) % 100;
    g->gnss_time = (float)(hh * 3600 + mm * 60) + (float)(v[0] - hh * 10000 - mm * 100);
    g->lat = v[1];
    g->lon = v[2];
    g->gnss_alt = (float)v[4];
    g->sats = (int)v[10];
    float spd = (float)v[7] / 3.6f, cog = (float)v[6] * 3.14159265f / 180.0f;
    g->gnss_vel[0] = spd * sinf(cog);
    g->gnss_vel[1] = spd * cosf(cog);
    return true;
}

/* Every GNSS_PERIOD_MS ask the modem for its fix; no fix (+CME ERROR: 516) clears gsm_gnss. */
static void gsm_gnss_update(uint32_t *last)
{
    char line[128];
    FC_tlm g;
    if (osKernelGetTickCount() - *last < GNSS_PERIOD_MS) return;
    *last = osKernelGetTickCount();
    bool ok = gsm_cmd("AT+QGPSLOC=2\r\n", "OK", 1000);
    if (ok) {
        taskENTER_CRITICAL();
        strncpy(line, gsm_acc, sizeof line - 1);
        line[sizeof line - 1] = 0;
        taskEXIT_CRITICAL();
        ok = gnss_parse(line, &g);
    }
    osMutexAcquire(fc_mutex, osWaitForever);
    if (ok) gsm_gnss = g; else gsm_gnss.sats = 0;
    osMutexRelease(fc_mutex);
}

static bool gsm_connect(void)
{
    char cmd[128];
    int i;

    for (i = 0; i < 10 && !gsm_cmd("AT\r\n", "OK", 500); i++) osDelay(500);
    if (i == 10) return false;
    gsm_cmd("ATE0\r\n", "OK", 500);
    gsm_cmd("AT+QGPS=1\r\n", "OK", 2000);                       /* GNSS on; ERROR if already running: fine */
    if (!gsm_cmd("AT+CPIN?\r\n", "READY", 2000)) return false;
    for (i = 0; i < 60; i++) {
        gsm_cmd("AT+CREG?\r\n", "OK", 1000);
        if (gsm_has("+CREG: 0,1") || gsm_has("+CREG: 0,5")) break;
        osDelay(1000);
    }
    if (i == 60) return false;
    gsm_cmd("AT+CMGF=1\r\n", "OK", 1000);                       /* SMS text mode */
    gsm_cmd("AT+CNMI=2,2,0,0,0\r\n", "OK", 1000);               /* incoming SMS straight to the UART as +CMT, not stored */
    snprintf(cmd, sizeof cmd, "AT+QICSGP=1,1,\"%s\",\"\",\"\",1\r\n", GSM_APN);
    if (!gsm_cmd(cmd, "OK", 2000)) return false;
    gsm_cmd("AT+QIACT=1\r\n", "OK", 30000);                     /* ERROR if already active: fine */
    return true;
}

/* Broker stage only; retried on its own by gsm_task until it succeeds.
 * The modem answers a failed open/connect quickly with a non-zero result, so wait for the
 * result line itself, not for the success value, and do not sit out the full timeout. */
static bool mqtt_connect(void)
{
    char cmd[128];
    gsm_cmd("AT+QMTCLOSE=0\r\n", "OK", 2000);                   /* drop a stale session */
    gsm_cmd("AT+QMTCFG=\"version\",0,4\r\n", "OK", 1000);       /* MQTT 3.1.1 */
    snprintf(cmd, sizeof cmd, "AT+QMTOPEN=0,\"%s\",%d\r\n", MQTT_BROKER, MQTT_PORT);
    if (!gsm_cmd(cmd, "+QMTOPEN: 0,", 30000) || !gsm_has("+QMTOPEN: 0,0")) return false;
    snprintf(cmd, sizeof cmd, "AT+QMTCONN=0,\"%s\"\r\n", MQTT_CLIENT_ID);
    if (!gsm_cmd(cmd, "+QMTCONN: 0,", 15000) || !gsm_has("+QMTCONN: 0,0,0")) return false;
    snprintf(cmd, sizeof cmd, "AT+QMTSUB=0,1,\"%s\",0\r\n", MQTT_TOPIC_CMD);
    return gsm_cmd(cmd, "+QMTSUB: 0,1,", 15000) && gsm_has("+QMTSUB: 0,1,0");
}

/* false only when the modem refuses the publish (no ">" prompt, ERROR, or an explicit
 * +QMTPUBEX failure); a slow ack is tolerated so a busy network does not trigger reconnects. */
static bool mqtt_publish(const char *topic, const char *msg, int len)
{
    char cmd[96];
    snprintf(cmd, sizeof cmd, "AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n", topic, len);
    if (!gsm_cmd(cmd, ">", 300)) return false;
    gsm_clear();
    gsm_send(msg, len);
    if (gsm_wait("+QMTPUBEX: 0,0,", GSM_PUB_TIMEOUT_MS)) return gsm_has("+QMTPUBEX: 0,0,0");
    return !gsm_has("ERROR");
}

static void send_sms(const gsm_tlm *t)
{
    char body[160];
    snprintf(body, sizeof body, "T+%lus GNSS %.0f %.6f,%.6f alt %.1fm sats %d",
             (unsigned long)t->mission_time, t->gnss_time, t->lat, t->lon, t->gnss_alt, t->sats);
    if (!gsm_cmd("AT+CMGS=\"" SMS_NUMBER "\"\r\n", ">", 3000)) { gsm_send("\x1b", 1); return; }
    gsm_clear();
    gsm_send(body, strlen(body));
    gsm_send("\x1a", 1);
    gsm_wait("+CMGS", SMS_TIMEOUT_MS);
}

static void gsm_task(void *arg)
{
    static char buf[TLM_LINE_MAX];
    gsm_tlm t;
    tc_msg_t a;
    const img_line_t *l;
    uint32_t count = 0, next = 0, last_sms = 0, last_gnss = 0;
    int fails = 0;
    bool online = false, net_up = false;

    gsm_task_h = osThreadGetId();
    HAL_UARTEx_ReceiveToIdle_IT(&huart4, gsm_rx, sizeof gsm_rx);
    osDelay(GSM_BOOT_MS);

    for (;;) {
        if (!online) {
            img_r_gsm = img_w;                 /* offline: do not hold the image ring (and XBee) back */
            osMessageQueueReset(ack_q);        /* stale acks are useless */
            if (!net_up) net_up = gsm_connect();
            online = net_up && mqtt_connect();
            if (!online) {                     /* keep trying the broker; every Nth miss redo the modem bring-up too */
                if (++fails % MQTT_RETRIES_PER_NET == 0) net_up = false;
                if (net_up) gsm_gnss_update(&last_gnss);   /* fix is useful to LoRa/XBee even with no broker */
                osDelay(MQTT_RETRY_MS);
                continue;
            }
            fails = 0;
            next = last_sms = osKernelGetTickCount();
        }

        /* ponytail: 15 Hz through AT commands is at the edge of what a 115200 modem link does
         * (~30 ms payload + modem latency per publish); lower GSM_PERIOD_MS to 100 if packets drop. */
        if (tlm_snapshot(&t, count + 1)) {
            int len = tlm_format(&t, buf, sizeof buf);
            if (len > 0 && mqtt_publish(MQTT_TOPIC_TLM, buf, len)) { count++; fails = 0; }
            else if (++fails >= GSM_MAX_FAILS) online = false;
        }

        /* ponytail: GSM is the slowest relay (~15 image lines/s), so it paces the IMGACK to the FC */
        for (int i = 0; i < GSM_IMG_PER_PERIOD && online && (l = img_peek(&img_r_gsm)); i++) {
            if (mqtt_publish(MQTT_TOPIC_IMG, l->data, l->len)) fails = 0;
            else if (++fails >= GSM_MAX_FAILS) online = false;
            img_r_gsm++;                       /* a failed line is dropped; the image is incomplete over MQTT */
        }

        while (online && osMessageQueueGet(ack_q, &a, NULL, 0) == osOK)
            if (!mqtt_publish(MQTT_TOPIC_ACK, a.data, a.len) && ++fails >= GSM_MAX_FAILS) online = false;

        if (fc_valid && osKernelGetTickCount() - last_sms >= SMS_PERIOD_MS) {
            last_sms = osKernelGetTickCount();
            send_sms(&t);
        }

        gsm_gnss_update(&last_gnss);
        if (gsm_handle_urc()) online = false;
        sleep_until(&next, GSM_PERIOD_MS);
    }
}

/* ---------------- init ---------------- */
void RF_Tasks_Init(void)
{
    static const osThreadAttr_t fc_attr   = { .name = "fc_comms", .stack_size = 1024 * 4, .priority = osPriorityAboveNormal };
    static const osThreadAttr_t lora_attr = { .name = "lora",     .stack_size = 1024 * 4, .priority = osPriorityNormal };
    static const osThreadAttr_t xbee_attr = { .name = "xbee",     .stack_size = 1024 * 4, .priority = osPriorityNormal };
    static const osThreadAttr_t gsm_attr  = { .name = "gsm",      .stack_size = 1536 * 4, .priority = osPriorityBelowNormal };

    fc_mutex = osMutexNew(NULL);
    tc_q     = osMessageQueueNew(8, sizeof(tc_msg_t), NULL);
    ack_q    = osMessageQueueNew(4, sizeof(tc_msg_t), NULL);

    osThreadNew(fc_comms_task, NULL, &fc_attr);
    osThreadNew(lora_task,     NULL, &lora_attr);
    osThreadNew(xbee_task,     NULL, &xbee_attr);
    osThreadNew(gsm_task,      NULL, &gsm_attr);
}
/* USER CODE END Application */


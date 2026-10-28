/* Host self-check for the $IMG relay ring and IMGACK logic in freertos.c.
 *   gcc -Istubs -I../Core/Inc test_img_ring.c ../Core/Src/telemetry.c -o t && ./t
 * Not part of the STM32 build. */
#include "../Core/Src/freertos.c"
#include <assert.h>

UART_HandleTypeDef huart2, huart3, huart4, huart5;
DMA_HandleTypeDef hdma_usart3_rx;
static char fc_out[64];

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *p, uint16_t n, uint32_t t)
{
    if (h == &huart3) { memcpy(fc_out, p, n); fc_out[n] = 0; }
    return HAL_OK;
}

static void push(const char *fmt, int a, int b)
{
    char line[IMG_LINE_MAX];
    snprintf(line, sizeof line, fmt, a, b);
    fc_handle_line(line);
}

/* what fc_comms_task does after draining the FC bytes */
static void ack_poll(void)
{
    if (img_ack_id >= 0 && img_min_read() >= img_ack_seq) { img_ack(img_ack_id, true); img_ack_id = -1; }
}

static void drain(volatile uint32_t *r, int n)
{
    const img_line_t *l;
    while (n-- && (l = img_peek(r))) { assert(l->len > 2 && l->data[0] == '$'); (*r)++; }
}

int main(void)
{
    /* 1. one small image: ack only after BOTH readers passed $IMGEND */
    push("$IMGSTART,%d,270,%d*00", 7, 2);
    push("$IMG,7,%d,%d,135,QUJD*11", 0, 2);
    push("$IMG,7,%d,%d,135,REVG*22", 1, 2);
    push("$IMGEND,%d,%d,270*33", 7, 2);
    assert(img_w == 4 && img_ack_id == 7 && fc_out[0] == 0);
    drain(&img_r_xbee, 4); ack_poll();
    assert(fc_out[0] == 0);                                   /* gsm still behind */
    drain(&img_r_gsm, 4); ack_poll();
    assert(strcmp(fc_out, "IMGACK,7,OK\r\n") == 0 && img_ack_id == -1);
    assert(img_peek(&img_r_xbee) == NULL && img_peek(&img_r_gsm) == NULL);

    /* 2. telemetry is untouched by the image path */
    fc_handle_line("$,011,1,1,0|0|0,0|0|0,0|0|0,0|0|0,0,0,0,0|0|0,0,0,0,0,0,0|0,0,0,0,0,0,0,0,#");
    assert(fc_valid && fc_tlm.packet_count == 1 && img_w == 4);

    /* 3. readers stall: ring fills, image gets FAIL at once, nothing lost from the ring */
    fc_out[0] = 0;
    push("$IMGSTART,%d,99999,%d*00", 8, IMG_RING + 5);
    for (int i = 0; i < IMG_RING + 5; i++) push("$IMG,8,%d,%d,135,QUJD*11", i, IMG_RING + 5);
    assert(img_drop && img_w == 4 + IMG_RING);
    push("$IMGEND,%d,%d,99999*33", 8, IMG_RING + 5);
    assert(strcmp(fc_out, "IMGACK,8,FAIL\r\n") == 0 && img_ack_id == -1);

    /* 4. gsm offline discards its backlog; xbee alone still drains and the next image acks */
    drain(&img_r_xbee, IMG_RING);
    img_r_gsm = img_w;
    fc_out[0] = 0;
    push("$IMGSTART,%d,135,%d*00", 9, 1);
    push("$IMG,9,%d,%d,135,QUJD*11", 0, 1);
    push("$IMGEND,%d,%d,135*33", 9, 1);
    assert(!img_drop && img_ack_id == 9);
    drain(&img_r_xbee, 3); drain(&img_r_gsm, 3); ack_poll();
    assert(strcmp(fc_out, "IMGACK,9,OK\r\n") == 0);

    /* 5. a full-size line (1024 raw bytes -> 1368 base64, ~1400 chars) fits; one over IMG_LINE_MAX is dropped */
    {
        static char big[IMG_LINE_MAX + 8];
        int h = snprintf(big, sizeof big, "$IMG,65535,9999,9999,1024,");
        memset(big + h, 'A', 1368); strcpy(big + h + 1368, "*7F");
        assert(strlen(big) + 2 <= IMG_LINE_MAX);
        img_r_xbee = img_r_gsm = img_w;
        push("$IMGSTART,%d,1024,%d*00", 10, 1);
        fc_handle_line(big);
        assert(!img_drop && img_ring[(img_w - 1) % IMG_RING].len == strlen(big) + 2);
        memset(big + h, 'A', IMG_LINE_MAX - 1); big[IMG_LINE_MAX - 1] = 0;     /* IMG_LINE_MAX - 1 chars + CRLF > max */
        fc_handle_line(big);
        assert(img_drop);
    }
    puts("img ring ok");
    return 0;
}

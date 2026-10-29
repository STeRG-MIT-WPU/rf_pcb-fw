/* Host stub of the HAL/CMSIS surface freertos.c touches (Tests/test_img_ring.c only). */
#ifndef STUB_MAIN_H
#define STUB_MAIN_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef struct { int id; } UART_HandleTypeDef, DMA_HandleTypeDef;
typedef enum { HAL_OK = 0 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef void *osThreadId_t, *osMutexId_t, *osMessageQueueId_t;
typedef struct { const char *name; uint32_t stack_size; int priority; } osThreadAttr_t;
enum { osOK = 0, osFlagsWaitAny = 0, osWaitForever = ~0u, osFlagsError = 0x80000000u,
       osPriorityBelowNormal, osPriorityNormal, osPriorityAboveNormal };
#define XBEE_nRTS_GPIO_Port 0
#define XBEE_nRTS_Pin 0
#define XBEE_nCTS_GPIO_Port 0
#define XBEE_nCTS_Pin 0
static uint32_t stub_tick;
#define osKernelGetTickCount() stub_tick
#define __DMB()
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()
#define __HAL_DMA_GET_COUNTER(h) 0
#define __HAL_UART_CLEAR_OREFLAG(h)
#define HAL_UARTEx_ReceiveToIdle_DMA(h, b, n) HAL_OK
#define HAL_UARTEx_ReceiveToIdle_IT(h, b, n) HAL_OK
#define HAL_GPIO_WritePin(p, pin, s)
#define HAL_GPIO_ReadPin(p, pin) GPIO_PIN_RESET
#define osThreadGetId() NULL
#define osThreadFlagsSet(t, f) 0
#define osThreadFlagsWait(f, o, t) 0
#define osMutexAcquire(m, t) 0
#define osMutexRelease(m) 0
#define osMutexNew(a) NULL
#define osMessageQueueNew(n, s, a) NULL
static char stub_q[256];                       /* last message put on any queue */
static int stub_qput(const void *m, size_t n) { memcpy(stub_q, m, n < sizeof stub_q ? n : sizeof stub_q); return 0; }
#define osMessageQueuePut(q, m, p, t) stub_qput(m, sizeof *(m))
#define osMessageQueueGet(q, m, p, t) 1
#define osMessageQueueReset(q) 0
#define osThreadNew(f, a, at) NULL
#define osDelay(ms) (stub_tick += (ms))
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *p, uint16_t n, uint32_t t);
#endif

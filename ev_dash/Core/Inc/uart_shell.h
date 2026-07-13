/* Core/Inc/uart_shell.h */
#ifndef UART_SHELL_H
#define UART_SHELL_H

#include "common.h"
#include "ev_control.h"
#include "adas.h"
#include "fault.h"

#define SHELL_BUF_SIZE   128   /* ring buffer size       */
#define SHELL_CMD_SIZE    64   /* max command length     */

/* ── Ring buffer ─────────────────────────────────────────────── */
typedef struct {
    uint8_t  buf[SHELL_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} RingBuf_t;

/* ── API ─────────────────────────────────────────────────────── */
void Shell_Init    (UART_HandleTypeDef  *huart,
                    EV_HandleTypeDef    *ev,
                    ADAS_HandleTypeDef  *adas,
                    Fault_HandleTypeDef *flt);
void Shell_PushByte(uint8_t byte);   /* call from UART RX callback */
void Shell_Process (void);           /* call in main loop          */
void print_status (void);

#endif

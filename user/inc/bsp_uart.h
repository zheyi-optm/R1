#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include <stdint.h>
#include "usart.h"
#include "dma.h"
#include "remote_control.h"

typedef struct
{
    uint8_t  rx_ready;
    uint16_t rx_size;
    uint32_t start_rx_ret;
    uint32_t start_rx_cnt;
    uint32_t start_rx_busy_cnt;
    uint32_t rx_event_cnt;
    uint32_t uart2_gstate_dbg;
    uint32_t uart2_rxstate_dbg;
    uint32_t uart2_isr_dbg;
    uint32_t uart2_err_dbg;
    uint8_t  rx_buf[53];
} imu_uart_ctx_t;

void BSP_USART_Init(void);
static void REMOTE_ExtractFrame(const uint8_t *buf, uint16_t size,
                                uint16_t start, uint16_t end);
extern void REMOTE_IdleHandler(void);


#endif

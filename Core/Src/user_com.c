/*
 * user_com.c
 *
 *  Created on: Jan 28, 2026
 *      Author: trion
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "main.h"
#include "ring_buffer.h"
#include "user_com.h"
#include "user_def.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

// UART2 queues (XBee communication)
Queue rx_UART2_queue;
Queue rx_UART2_line_queue;
Queue tx_UART2_queue;

// UART status
struct uart_Stat_ST uart2_stat_ST;

// ============================================================================
// ASCII Conversion Functions
// ============================================================================
uint8_t atoh(char in_ascii)
{
    uint8_t rtn_val;

    if ((in_ascii >= '0') && (in_ascii <= '9'))
    {
        rtn_val = in_ascii - '0';
    }
    else if ((in_ascii >= 'a') && (in_ascii <= 'f'))
    {
        rtn_val = in_ascii - 'a' + 0x0A;
    }
    else if ((in_ascii >= 'A') && (in_ascii <= 'F'))
    {
        rtn_val = in_ascii - 'A' + 0x0A;
    }
    else
    {
        rtn_val = 0xFF;
    }

    return rtn_val;
}

uint8_t atod(char in_ascii)
{
    uint8_t rtn_val;

    if ((in_ascii >= '0') && (in_ascii <= '9'))
    {
        rtn_val = in_ascii - '0';
    }
    else
    {
        rtn_val = 0xFF;
    }

    return rtn_val;
}

uint32_t atoh_str(char *in_asc_str, uint8_t len)
{
    uint32_t rtn_val, i, j;

    rtn_val = 0;

    for (i = 0, j = 1; i < len; i++, j *= 0x10)
    {
        rtn_val += (uint32_t)atoh(in_asc_str[len - 1 - i]) * j;
    }

    return rtn_val;
}

uint32_t atod_str(char *in_asc_str, uint8_t len)
{
    uint32_t rtn_val, i, j;

    rtn_val = 0;

    for (i = 0, j = 1; i < len; i++, j *= 10)
    {
        rtn_val += (uint32_t)atod(in_asc_str[len - 1 - i]) * j;
    }

    return rtn_val;
}

// ============================================================================
// UART Initialization
// ============================================================================
void UART_baudrate_set(UART_HandleTypeDef *tm_hart, uint32_t baud_rate)
{
    tm_hart->Init.BaudRate = baud_rate;
    if (HAL_UART_Init(tm_hart) != HAL_OK)
    {
        Error_Handler();
    }
}

void init_UART_COM(void)
{
    // Initialize UART2 queues
    InitQueue(&rx_UART2_line_queue, 128);
    InitQueue(&rx_UART2_queue, 256);
    InitQueue(&tx_UART2_queue, 256);

    // Clear status
    memset(&uart2_stat_ST, 0, sizeof(uart2_stat_ST));

#if 0  // DMA mode disabled for testing - use polling instead
    // Enable IDLE interrupt for line detection
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    // Start DMA reception
    HAL_UART_Receive_DMA(&huart2, uart2_stat_ST.uart_rx_DMA_buf, DMA_RX_BUFFER_SIZE);
#endif
}

// ============================================================================
// UART2 TX Functions
// ============================================================================
void UART2_SendString(const char *str)
{
    uint16_t len = strlen(str);
    Enqueue_bytes(&tx_UART2_queue, (uint8_t *)str, len);
}

void UART2_SendData(uint8_t *data, uint16_t len)
{
    Enqueue_bytes(&tx_UART2_queue, data, len);
}

void printf_UART2(const char *str_buf, ...)
{
    char tx_bb[256];
    uint32_t len = 0;

    va_list ap;
    va_start(ap, str_buf);
    vsprintf(tx_bb, str_buf, ap);
    va_end(ap);

    len = strlen(tx_bb);
    Enqueue_bytes(&tx_UART2_queue, (uint8_t *)tx_bb, len);
}

void printf_UARTC(UART_HandleTypeDef *h_tmUART, uint8_t color, const char *str_buf, ...)
{
    char tx_bb[256];
    uint32_t len = 0;

    if (color != PR_NOP)
    {
        // Color set
        sprintf(tx_bb, "\033[%dm", color);
        len = strlen(tx_bb);
    }

    va_list ap;
    va_start(ap, str_buf);
    vsprintf(&tx_bb[len], str_buf, ap);
    va_end(ap);

    len = strlen(tx_bb);
    tx_bb[len] = 0;

    if (h_tmUART->Instance == USART2)
    {
        Enqueue_bytes(&tx_UART2_queue, (uint8_t *)tx_bb, len);
    }
}

// TX process - call this in main loop (blocking mode to avoid DMA conflict)
void UART2_TX_proc(void)
{
    uint16_t tx_len;

    tx_len = Len_queue(&tx_UART2_queue);
    if (tx_len > 0)
    {
        if (tx_len > DMA_TX_BUFFER_SIZE)
        {
            tx_len = DMA_TX_BUFFER_SIZE;
        }
        Dequeue_bytes(&tx_UART2_queue, uart2_stat_ST.uart_tx_usr_buf, tx_len);
        // Use blocking TX to avoid conflict with RX DMA
        HAL_UART_Transmit(&huart2, uart2_stat_ST.uart_tx_usr_buf, tx_len, 100);
    }
}

// ============================================================================
// UART2 RX Functions (DMA + IDLE)
// ============================================================================

// Call this from USART2_IRQHandler when IDLE is detected
void UART2_RX_DMA_proc(void)
{
    uint32_t rx_len;

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        printf("[IDLE IRQ]\r\n");  // Debug: IDLE interrupt triggered
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        // Calculate received length
        rx_len = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

        if (rx_len > 0)
        {
            // Copy to RX queue
            Enqueue_bytes(&rx_UART2_queue, uart2_stat_ST.uart_rx_DMA_buf, rx_len);
            uart2_stat_ST.f_uart_rcvd = 1;
            printf("[RX DMA] %lu bytes\r\n", rx_len);  // Debug
        }

        // Restart RX DMA only (don't stop TX)
        HAL_UART_AbortReceive(&huart2);
        HAL_UART_Receive_DMA(&huart2, uart2_stat_ST.uart_rx_DMA_buf, DMA_RX_BUFFER_SIZE);
    }
}

// Get a complete line from UART2 RX queue
COM_Idy_Typ UART2_GetLine(uint8_t *line_buf)
{
    uint16_t q_len;
    uint8_t rx_dat;
    COM_Idy_Typ rtn_val = NOT_LINE;

    q_len = Len_queue(&rx_UART2_queue);
    if (q_len)
    {
        for (uint16_t i = 0; i < q_len; i++)
        {
            rx_dat = Dequeue(&rx_UART2_queue);
            Enqueue(&rx_UART2_line_queue, rx_dat);

#ifdef UART2_ECHO
            printf("%c", rx_dat);
#endif

            if (rx_dat == UART2_ETX)
            {
#ifdef UART2_ECHO
                printf("\n");
#endif
                flush_queue(&rx_UART2_queue);
                q_len = Len_queue(&rx_UART2_line_queue);
                Dequeue_bytes(&rx_UART2_line_queue, line_buf, q_len);
                line_buf[q_len - 1] = 0;  // Null terminate

                printf("[RX] %s\r\n", line_buf);

                // Check for valid responses
                if (
                    (strncmp((char *)line_buf, "on", 2) == 0) ||
                    (strncmp((char *)line_buf, "off", 3) == 0) ||
                    (strncmp((char *)line_buf, "ok", 2) == 0) ||
                    (strncmp((char *)line_buf, "err", 3) == 0)
                )
                {
                    rtn_val = RCV_LINE;
                }
                else
                {
                    rtn_val = WNG_LINE;
                }
                break;  // Exit for loop
            }
        }
    }

    return rtn_val;
}

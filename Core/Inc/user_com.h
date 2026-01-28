/*
 * user_com.h
 *
 *  Created on: Jan 28, 2026
 *      Author: trion
 */

#ifndef INC_USER_COM_H_
#define INC_USER_COM_H_

#include "main.h"
#include "ring_buffer.h"

// Terminal color codes
#define PR_NOP    99        // No style
#define PR_INI    0         // initial value
#define PR_BLK    30        // Black
#define PR_RED    31        // Red
#define PR_GRN    32        // Green
#define PR_YEL    33        // Yellow
#define PR_BLE    34        // Blue
#define PR_MAG    35        // Magenta
#define PR_CYN    36        // Cyan
#define PR_WHT    37        // White

// DMA buffer sizes
#define DMA_RX_BUFFER_SIZE    128
#define DMA_TX_BUFFER_SIZE    256

// UART status structure
struct uart_Stat_ST {
    uint8_t f_uart_rcvd;                        // UART DMA RX received complete flag
    uint8_t cnt_rx;
    uint8_t uart_rx_DMA_buf[DMA_RX_BUFFER_SIZE + 2];
    uint8_t uart_rx_line_buf[DMA_RX_BUFFER_SIZE + 2];
    uint8_t uart_tx_usr_buf[DMA_TX_BUFFER_SIZE + 2];
    uint32_t dma_rx_len;
};

// Communication line status
typedef enum
{
    NOT_LINE = 0,   // No complete line received
    RCV_LINE = 1,   // Valid line received
    WNG_LINE = 2,   // Wrong/unknown command
    PAS_LINE = 3,   // Pass through line
} COM_Idy_Typ;

// UART2 settings (XBee communication)
#define UART2_ECHO
#define UART2_ETX    '\r'

// External queue declarations
extern Queue rx_UART2_queue;
extern Queue rx_UART2_line_queue;
extern Queue tx_UART2_queue;

extern struct uart_Stat_ST uart2_stat_ST;

// ASCII conversion functions
uint8_t atoh(char in_ascii);
uint8_t atod(char in_ascii);
uint32_t atoh_str(char *in_asc_str, uint8_t len);
uint32_t atod_str(char *in_asc_str, uint8_t len);

// UART functions
void UART_baudrate_set(UART_HandleTypeDef *tm_hart, uint32_t baud_rate);
void init_UART_COM(void);
void printf_UART2(const char *str_buf, ...);
void printf_UARTC(UART_HandleTypeDef *h_tmUART, uint8_t color, const char *str_buf, ...);

// UART2 TX/RX functions
void UART2_TX_proc(void);
void UART2_RX_DMA_proc(void);
COM_Idy_Typ UART2_GetLine(uint8_t *line_buf);
void UART2_SendString(const char *str);
void UART2_SendData(uint8_t *data, uint16_t len);

#endif /* INC_USER_COM_H_ */

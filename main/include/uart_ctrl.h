#ifndef __UART_CTRL_H
#define __UART_CTRL_H
#include "floorlamp.h"

#define ASR_TX     CONFIG_ASR_TXD
#define ASR_RX     CONFIG_ASR_RXD
#define ASR_RTS    UART_PIN_NO_CHANGE
#define ASR_CTS    UART_PIN_NO_CHANGE

#define ASR_UART_NUM UART_NUM_2

#define ASR_BUF_SIZE (1024)
#define ASR_RD_BUF_SIZE (ASR_BUF_SIZE)

/*
    @brief: init UART with assigned baud rate
*/
esp_err_t serial_init(int baudrate);

/*
    @brief:
*/
void uart_event_task(void *pvParameters);


#endif

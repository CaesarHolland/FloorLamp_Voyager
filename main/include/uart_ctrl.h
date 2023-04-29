#ifndef __UART_CTRL_H
#define __UART_CTRL_H
#include "floorlamp.h"

#define UART_TX     17
#define UART_RX     16
#define UART_RTS    UART_PIN_NO_CHANGE
#define UART_CTS    UART_PIN_NO_CHANGE

#define EX_UART_NUM UART_NUM_2

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static QueueHandle_t uart2_queue;


/*
    @brief: init UART with assigned baud rate
*/
esp_err_t serial_init(int baudrate);

/*
    @brief:
*/
void uart_event_task(void *pvParameters);

/*
    @brief:
*/
void judge_and_post(int typeId);


#endif

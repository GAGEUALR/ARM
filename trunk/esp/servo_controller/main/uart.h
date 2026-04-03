#ifndef UART_H
#define UART_H

#include "main.h"


#define USB_UART_NUM UART_NUM_0
#define USB_UART_BAUD 115200
#define UART_RX_BUF_SIZE 1024

typedef struct {
    bool uart_ready;
} uart_state_t;


#endif
#ifndef UART_H
#define UART_H

#include "main.h"


#define USB_UART_NUM UART_NUM_0
#define USB_UART_BAUD 115200
#define UART_RX_BUF_SIZE 1024

#define UART_PACKET_START_BYTE 0xAA

#define UART_PACKET_SIZE 15
#define UART_CRC_INDEX   14
#define CRC8_POLYNOMIAL  0x07

typedef struct {
    bool uart_ready;
} uart_state_t;


#endif
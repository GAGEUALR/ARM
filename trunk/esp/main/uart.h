#ifndef UART_H
#define UART_H

#include "main.h"


#define USB_UART_NUM UART_NUM_0
#define USB_UART_BAUD 115200
#define UART_RX_BUF_SIZE 1024

#define UART_PACKET_START_BYTE 0xAA

#define UART_PACKET_SIZE 9
#define UART_CRC_INDEX   8

#define UART_RESPONSE_PACKET_SIZE 15
#define UART_RESPONSE_CRC_INDEX 14

#define UART_ADC_VALID_FLAGS_INDEX 8
#define UART_ADC_LEVEL_START_INDEX 9

#define UART_PACKET_TYPE_FEEDBACK 'F'

#define UART_MESSAGE_ID_INDEX 1
#define UART_PACKET_TYPE_INDEX 2
#define UART_SERVO_STATE_START_INDEX 3

#define UART_PACKET_TYPE_STATUS 'S'

#define SERVO_STATE_INACTIVE 'I'
#define SERVO_STATE_POSITIVE 'P'
#define SERVO_STATE_NEGATIVE 'N'

#define UART_PACKET_TYPE_FEEDBACK 'F'


#define CRC8_POLYNOMIAL  0x07

typedef struct {
    bool uart_ready;
} uart_state_t;

void uart_tx_task(void *arg);
void uart_rx_task(void *arg);
bool calculate_crc(const uint8_t *packet);

#endif
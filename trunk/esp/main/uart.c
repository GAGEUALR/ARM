#include "uart.h"
#include <string.h>
#include <stdint.h>

static bool parse_packet(uint8_t *packet, requested_state_t *state); // eventually should return error codes
static uint8_t calculate_crc8(const uint8_t *data, int length);
bool calculate_crc(const uint8_t *packet);

void uart_rx_task(void *arg)
{
    (void)arg;

    uint8_t received_bytes[UART_PACKET_SIZE];
    requested_state_t requested_state;

    while (1) {
        int bytes_read = uart_read_bytes(
            USB_UART_NUM,
            received_bytes,
            UART_PACKET_SIZE,
            pdMS_TO_TICKS(50));

        if (bytes_read == UART_PACKET_SIZE) {
            if (parse_packet(received_bytes, &requested_state)) {
                xQueueOverwrite(servo_command_q, &requested_state);
            }
        }
    }
}

void usb_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = USB_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
        USB_UART_NUM,
        UART_RX_BUF_SIZE,
        0,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(
        USB_UART_NUM,
        &uart_config
    ));
}

static bool parse_packet(uint8_t *packet, requested_state_t *state)
{
    if (packet[0] != 0xAA) {
        return false;
    }

    if (!calculate_crc(packet)) {
        return false;
    }

    state->request_version = packet[1];
    state->message_id = packet[2];
    state->request_type = packet[3];

    int byte_num = 4;

    for (int i = 0; i < SERVO_COUNT; i++) {
        uint8_t packed_high = packet[byte_num];
        uint8_t packed_low = packet[byte_num + 1];

        state->servos[i].command_type = packed_high >> 4;

        state->servos[i].requested_pulse =
            ((packed_high & 0x0F) << 8) | packed_low;

        if (state->servos[i].command_type == SERVO_COMMAND_MOVE) {
            if ((state->servos[i].requested_pulse < 500) ||
                (state->servos[i].requested_pulse > 2500)) {
                return false;
            }
        }

        byte_num += 2;
    }

    return true;
}

bool calculate_crc(const uint8_t *packet)
{
    uint8_t calculated_crc = calculate_crc8(packet, UART_CRC_INDEX);
    uint8_t received_crc = packet[UART_CRC_INDEX];

    return calculated_crc == received_crc;
}

static uint8_t calculate_crc8(const uint8_t *data, int length)
{
    uint8_t crc = 0x00;

    for (int i = 0; i < length; i++) {
        crc ^= data[i]; //xor this byte with running crc value 

        for (int bit = 0; bit < 8; bit++) {
            //shift through each bit in the byte and combine with running
            //crc value, if msb turns out to be 1
            if (crc & 0x80) { 
                crc = (uint8_t)((crc << 1) ^ CRC8_POLYNOMIAL);
            }
            else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

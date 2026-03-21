#include "main.h"

#define UART_PACKET_START_BYTE 0xAA
#define UART_PACKET_MIN_SIZE 7
#define UART_PACKET_MAX_SIZE 12

static bool parse_packet(const uint8_t *packet, int packet_length, desired_state_t *desired_state_out);

void uart_rx_task(void *arg)
{
    (void)arg;

    uint8_t packet[UART_PACKET_MAX_SIZE];
    int packet_index = 0;
    bool receiving_packet = false;

    while (1) {
        uint8_t received_byte = 0;
        int bytes_read = uart_read_bytes(
            USB_UART_NUM,
            &received_byte,
            1,
            pdMS_TO_TICKS(100)
        );

        if (bytes_read <= 0) {
            continue;
        }

        if (!receiving_packet) {
            if (received_byte == UART_PACKET_START_BYTE) {
                packet[0] = received_byte;
                packet_index = 1;
                receiving_packet = true;
            }

            continue;
        }

        if (received_byte == UART_PACKET_START_BYTE) {
            packet[0] = received_byte;
            packet_index = 1;
            continue;
        }

        if (packet_index < UART_PACKET_MAX_SIZE) {
            packet[packet_index] = received_byte;
            packet_index++;
        }
        else {
            receiving_packet = false;
            packet_index = 0;
            uart_write_bytes(USB_UART_NUM, "BAD MESSAGE\n", 12);
            continue;
        }

        if (packet_index >= UART_PACKET_MIN_SIZE) {
            desired_state_t desired_state;

            if (parse_packet(packet, packet_index, &desired_state)) {
                xQueueOverwrite(desired_state_queue, &desired_state);
                uart_write_bytes(USB_UART_NUM, "OK\n", 3);

                receiving_packet = false;
                packet_index = 0;
                continue;
            }
        }

        if (packet_index == UART_PACKET_MAX_SIZE) {
            receiving_packet = false;
            packet_index = 0;
            uart_write_bytes(USB_UART_NUM, "BAD MESSAGE\n", 12);
        }
    }
}

void usb_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = USB_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(USB_UART_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(USB_UART_NUM, &cfg));
}

static bool parse_packet(const uint8_t *packet, int packet_length, desired_state_t *desired_state_out)
{
    static const uint8_t expected_servo_order[5] = { 'B', 'S', 'F', 'W', 'G' };

    if (packet == NULL || desired_state_out == NULL) {
        return false;
    }

    if (packet_length < UART_PACKET_MIN_SIZE || packet_length > UART_PACKET_MAX_SIZE) {
        return false;
    }

    if (packet[0] != UART_PACKET_START_BYTE) {
        return false;
    }

    {
        uint8_t checksum = 0;
        int i = 0;

        while (i < (packet_length - 1)) {
            checksum ^= packet[i];
            i++;
        }

        if (checksum != packet[packet_length - 1]) {
            return false;
        }
    }

    desired_state_t parsed_state = {
        .base = -1,
        .shoulder = -1,
        .forearm = -1,
        .wrist = -1,
        .gripper = -1
    };

    {
        int index = 1;
        int servo_index = 0;

        while (servo_index < 5) {
            if (index >= (packet_length - 1)) {
                return false;
            }

            if (packet[index] != expected_servo_order[servo_index]) {
                return false;
            }

            index++;

            if (index < (packet_length - 1)) {
                if (packet[index] == 0x00 || packet[index] == 0x01) {
                    if (servo_index == 0) {
                        parsed_state.base = (int8_t)packet[index];
                    }
                    else if (servo_index == 1) {
                        parsed_state.shoulder = (int8_t)packet[index];
                    }
                    else if (servo_index == 2) {
                        parsed_state.forearm = (int8_t)packet[index];
                    }
                    else if (servo_index == 3) {
                        parsed_state.wrist = (int8_t)packet[index];
                    }
                    else if (servo_index == 4) {
                        parsed_state.gripper = (int8_t)packet[index];
                    }

                    index++;
                }
            }

            servo_index++;
        }

        if (index != (packet_length - 1)) {
            return false;
        }
    }

    *desired_state_out = parsed_state;
    return true;
}
#include "uart.h"

#define UART_PACKET_START_BYTE 0xAA
#define UART_PACKET_SIZE 12
#define SERVO_FLAG_ACTIVE 0x01
#define SERVO_FLAG_DIRECTION 0x02

static bool parse_packet(const uint8_t *packet, int packet_length, requested_state_t *requested_state_out);
static uint8_t calculate_checksum(const uint8_t *packet, int packet_length);

void uart_rx_task(void *arg)
{
    (void)arg;

    uint8_t packet[UART_PACKET_SIZE];
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

        if (packet_index < UART_PACKET_SIZE) {
            packet[packet_index] = received_byte;
            packet_index++;
        }
        else {
            receiving_packet = false;
            packet_index = 0;
            uart_write_bytes(USB_UART_NUM, "BAD MESSAGE\n", 12);
            continue;
        }

        if (packet_index == UART_PACKET_SIZE) {
            requested_state_t requested_state;

            if (parse_packet(packet, packet_index, &requested_state)) {
                xQueueOverwrite(servo_command_q, &requested_state);
            }
            else {
                uart_write_bytes(USB_UART_NUM, "BAD MESSAGE\n", 12);
            }

            receiving_packet = false;
            packet_index = 0;
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

static bool parse_packet(const uint8_t *packet, int packet_length, requested_state_t *requested_state_out)
{
    static const uint8_t expected_servo_order[SERVO_COUNT] = { 'B', 'S', 'F', 'W', 'G' };
    requested_state_t parsed_state = {0};
    int packet_index = 1;
    int servo_index = 0;

    if (packet == NULL || requested_state_out == NULL) {
        return false;
    }

    if (packet_length != UART_PACKET_SIZE) {
        return false;
    }

    if (packet[0] != UART_PACKET_START_BYTE) {
        return false;
    }

    if (calculate_checksum(packet, packet_length) != packet[packet_length - 1]) {
        return false;
    }

    for (servo_index = 0; servo_index < SERVO_COUNT; servo_index++) {
        uint8_t servo_id = packet[packet_index];
        uint8_t servo_flags = packet[packet_index + 1];

        if (servo_id != expected_servo_order[servo_index]) {
            return false;
        }

        parsed_state.servos[servo_index].active =
            ((servo_flags & SERVO_FLAG_ACTIVE) != 0);

        parsed_state.servos[servo_index].direction =
            ((servo_flags & SERVO_FLAG_DIRECTION) != 0);

        packet_index += 2;
    }

    *requested_state_out = parsed_state;
    return true;
}

static uint8_t calculate_checksum(const uint8_t *packet, int packet_length)
{
    uint8_t checksum = 0;
    int i = 0;

    while (i < (packet_length - 1)) {
        checksum ^= packet[i];
        i++;
    }

    return checksum;
}
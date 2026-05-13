#include "uart.h"
#include "control.h"

static bool parse_packet(uint8_t *packet, requested_state_t *state); // eventually should return error codes
static uint8_t calculate_crc8(const uint8_t *data, int length);
static void debug_toggle_packet(void);

#if DEBUG_GPIO_ENABLE
static bool debugPacketState = false;
#endif

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
                debug_toggle_packet();
                xQueueOverwrite(servo_command_q, &requested_state);
            }
        }
    }
}

void uart_tx_task(void *arg)
{
    (void)arg;

    uint8_t response_bytes[UART_RESPONSE_PACKET_SIZE];
    control_response_t response;

    while (1) {
        if (xQueueReceive(control_response_q, &response, portMAX_DELAY) == pdTRUE) {
            memset(response_bytes, 0, sizeof(response_bytes));

            response_bytes[0] = UART_PACKET_START_BYTE;
            response_bytes[UART_MESSAGE_ID_INDEX] = response.message_id;
            response_bytes[UART_PACKET_TYPE_INDEX] = UART_PACKET_TYPE_FEEDBACK;

            for (int i = 0; i < SERVO_COUNT; i++) {
                response_bytes[UART_SERVO_STATE_START_INDEX + i] =
                    response.servo_states[i];
            }

            response_bytes[UART_ADC_VALID_FLAGS_INDEX] =
                response.adc_valid_flags;

            for (int i = 0; i < SERVO_COUNT; i++) {
                response_bytes[UART_ADC_LEVEL_START_INDEX + i] =
                    response.adc_levels[i] & 0x0F;
            }

            int pwm_index = UART_PWM_START_INDEX;

            for (int i = 0; i < SERVO_COUNT; i++) {
                response_bytes[pwm_index] = response.pwm_values[i] & 0xFF;
                response_bytes[pwm_index + 1] = (response.pwm_values[i] >> 8) & 0xFF;

                pwm_index += 2;
            }


            response_bytes[UART_RESPONSE_CRC_INDEX] =
                calculate_crc8(response_bytes, UART_RESPONSE_CRC_INDEX);

            uart_write_bytes(
                USB_UART_NUM,
                (const char *)response_bytes,
                UART_RESPONSE_PACKET_SIZE
            );
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

static void debug_toggle_packet(void)
{
#if DEBUG_GPIO_ENABLE
    debugPacketState = !debugPacketState;
    gpio_set_level(DEBUG_PACKET_GPIO, debugPacketState);
#endif
}

static bool parse_packet(uint8_t *packet, requested_state_t *state)
{
    if (packet[0] != UART_PACKET_START_BYTE) {
        return false;
    }

    if (!calculate_crc(packet)) {
        return false;
    }

    if (packet[UART_PACKET_TYPE_INDEX] != UART_PACKET_TYPE_STATUS) {
        return false;
    }

    memset(state, 0, sizeof(*state));

    state->message_id = packet[UART_MESSAGE_ID_INDEX];
    state->request_type = packet[UART_PACKET_TYPE_INDEX];

    for (int i = 0; i < SERVO_COUNT; i++) {
        uint8_t servo_state = packet[UART_SERVO_STATE_START_INDEX + i];

        if (servo_state == SERVO_STATE_INACTIVE) {
            state->servos[i].active = false;
            state->servos[i].direction = false;
        }
        else if (servo_state == SERVO_STATE_POSITIVE) {
            state->servos[i].active = true;
            state->servos[i].direction = true;
        }
        else if (servo_state == SERVO_STATE_NEGATIVE) {
            state->servos[i].active = true;
            state->servos[i].direction = false;
        }
        else {
            return false;
        }
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
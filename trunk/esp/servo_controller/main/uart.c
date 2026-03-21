#include "main.h"


void uart_rx_task(void *arg)
{
    (void)arg;

    uint8_t ch;
    char line[64];
    int idx = 0;

    while (1) {
        int n = uart_read_bytes(USB_UART_NUM, &ch, 1, pdMS_TO_TICKS(100));

        if (n <= 0) continue;
        if (ch == '\r') continue;

        if (ch == '\n') {
            line[idx] = '\0';

            if (idx > 0) {


                if (strcmp(line, "SHUTDOWN") == 0) {
                    system_state.shutdown_requested = true;
                    system_state.rx_valid = true;
                    system_state.send_ack = true;
                }
                else {
                    int i = 0;
                    while (i < (COMMAND_BUFFER_SIZE - 1) && line[i] != '\0') {
                        system_state.command[i] = line[i];
                        i++;
                    }
                    system_state.command[i] = '\0';

                    system_state.rx_valid = true;
                    system_state.send_ack = true;
                }
            }

            idx = 0;
            continue;
        }

        if (idx < (int)sizeof(line) - 1) {
            line[idx++] = (char)ch;
        } else {
            idx = 0;
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




#include "main.h"


control_state_t system_state = {
    .shutdown_requested = false,
    .rx_valid = false,
    .send_ack = false,
};

void app_main(void)
{
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 9, NULL);
}
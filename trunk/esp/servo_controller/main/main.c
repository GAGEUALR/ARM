#include "main.h"

QueueHandle_t servo_command_q;
system_t system_state = {0};

void app_main(void)
{
    servo_command_q = xQueueCreate(STATE_QUEUE_LENGTH, sizeof(requested_state_t));

    usb_uart_init();
    servo_init();

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 9, NULL);
}
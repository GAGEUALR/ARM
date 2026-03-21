#include "main.h"

control_state_t system_state = {
    .shutdown_requested = false,
};

QueueHandle_t servo_command_q = NULL;
QueueHandle_t control_ack_q = NULL;

void app_main(void)
{
    servo_command_q = xQueueCreate(STATE_QUEUE_LENGTH, sizeof(requested_state_t));
    control_ack_q = xQueueCreate(CONTROL_ACK_QUEUE_LENGTH, sizeof(control_ack_t));

    configASSERT(servo_command_q != NULL);
    configASSERT(control_ack_q != NULL);

    usb_uart_init();
    servo_init();

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 9, NULL);
}
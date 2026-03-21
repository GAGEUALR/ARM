#include "main.h"


control_state_t system_state = {
    .shutdown_requested = false,
    .rx_valid = false,
    .send_ack = false,
};

void app_main(void)
{

    QueueHandle_t servo_command_q;
    QueueHandle_t control_ack_q;
    servo_command_q = xQueueCreate(STATE_QUEUE_LENGTH, sizeof(desired_state_t));
    control_ack_q = xQueueCreate(CONTROL_ACK_QUEUE_LENGTH, sizeof(desired_state_t));
    
    usb_uart_init();
    servo_init();

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 9, NULL);
}
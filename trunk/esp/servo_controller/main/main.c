#include "main.h"



QueueHandle_t servo_command_q;

void app_main(void)
{

    //create queue for sending servo commands from uart task to control task.
    servo_command_q = xQueueCreate(STATE_QUEUE_LENGTH, sizeof(control_state_t));

    //startup state
    usb_uart_init();
    servo_init();

    //create tasks (operational state)
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 9, NULL);

    //create shutdown task that relays to uart.c when the system is ready for power off.
}
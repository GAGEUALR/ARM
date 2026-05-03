#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#include "esp_err.h"
#include "esp_timer.h"

#define SERVO_COUNT 5
#define STATE_QUEUE_LENGTH 1
#define RESPONSE_QUEUE_LENGTH 1

typedef struct {
    int requested_pulse;
    int command_type;
    int dac_value;
} servo_request_t;

typedef struct {
    servo_request_t servos[SERVO_COUNT];
    int request_version;
    int message_id;
    int request_type;
} requested_state_t;

typedef struct {
    volatile bool shutdown_requested;
} system_t;



void usb_uart_init(void);
void servo_init(void);

void servo_control_task(void *arg);
void uart_rx_task(void *arg);



extern QueueHandle_t servo_command_q;
extern system_t system_state;

#endif
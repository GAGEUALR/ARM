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

#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define BASE_GPIO GPIO_NUM_4
#define SHOULDER_GPIO GPIO_NUM_17
#define FOREARM_GPIO GPIO_NUM_16
#define WRIST_GPIO GPIO_NUM_18
#define GRIPPER_GPIO GPIO_NUM_19

#define USB_UART_NUM UART_NUM_0
#define USB_UART_BAUD 115200
#define UART_RX_BUF_SIZE 1024

#define STATE_QUEUE_LENGTH 1

typedef enum {
    BASE = 0,
    SHOULDER,
    FOREARM,
    WRIST,
    GRIPPER,
    SERVO_COUNT
} servo_id_t;

typedef enum {
    accellerating,
    decellerating,
    at_max_speed,
    at_target
} control_status_t;

typedef struct {
    bool active;
    bool direction;
} servo_request_t;

typedef struct {
    servo_request_t servos[SERVO_COUNT];
} requested_state_t;

typedef struct {
    bool active;
    bool direction;
    control_status_t status;
    uint32_t current_pulse_us;
} servo_state_t;

typedef struct {
    servo_state_t servos[SERVO_COUNT];
} control_state_t;

typedef struct {
    bool uart_ready;
} uart_state_t;

typedef struct {
    volatile bool shutdown_requested;
    control_state_t control_state;
    uart_state_t uart_state;
} system_t;

void usb_uart_init(void);
void servo_init(void);

void servo_control_task(void *arg);
void uart_rx_task(void *arg);

extern QueueHandle_t servo_command_q;
extern system_t system_state;

#endif
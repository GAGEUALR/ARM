#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#define BASE_GPIO GPIO_NUM_4
#define FOREARM_GPIO GPIO_NUM_16
#define SHOULDER_GPIO GPIO_NUM_17
#define WRIST_GPIO GPIO_NUM_18
#define GRIPPER_GPIO GPIO_NUM_19

#define LEDC_TIMER_ID LEDC_TIMER_0
#define LEDC_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_16_BIT

#define BASE_CHANNEL LEDC_CHANNEL_0
#define SHOULDER_CHANNEL LEDC_CHANNEL_1
#define FOREARM_CHANNEL LEDC_CHANNEL_2
#define WRIST_CHANNEL LEDC_CHANNEL_3
#define GRIPPER_CHANNEL LEDC_CHANNEL_4

#define SERVO_FREQ_HZ 50
#define SERVO_PERIOD_US 20000

#define SERVO_US_MIN_SAFE 500
#define SERVO_US_MAX_SAFE 2500
#define SERVO_US_CENTER 1500

#define BASE_MAX_STEP_US_PER_TICK 7
#define SHOULDER_MAX_STEP_US_PER_TICK 5
#define FOREARM_MAX_STEP_US_PER_TICK 5
#define WRIST_MAX_STEP_US_PER_TICK 14
#define GRIPPER_MAX_STEP_US_PER_TICK 18

#define SERVO_ACCEL_STEP_US_PER_TICK 1
#define SERVO_DECEL_STEP_US_PER_TICK 2
#define SERVO_TARGET_TOLERANCE_US 2

#define USB_UART_NUM UART_NUM_0
#define USB_UART_BAUD 115200
#define UART_RX_BUF_SIZE 1024

#define COMMAND_BUFFER_SIZE 64

typedef struct {
    volatile bool shutdown_requested;
    volatile char command[COMMAND_BUFFER_SIZE];
    volatile bool rx_valid;
    volatile bool send_ack;
} control_state_t;

static control_state_t system_state = {
    .shutdown_requested = false,
    .rx_valid = false,
    .send_ack = false,
};

#endif
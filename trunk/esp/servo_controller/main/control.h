#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

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

#define CONTROL_LOOP_HZ 100 //update servo pulses every 10ms. 
                            //max servo pulse: 2.5ms, min is 0.5ms
                            //control loop has ~7.5ms to update and handle other tasks.


static void servo_write_us(ledc_channel_t channel, uint32_t pulse_us);
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);
static inline uint32_t servo_us_to_duty(uint32_t pulse_us);
static void configure_servo_channel(gpio_num_t gpio, ledc_channel_t channel);
static void control_startup(void);

system_t system_state = {
    .shutdown_requested = false,
    .control_state = {
        .base.state = { .active = false, .direction = false },
        .shoulder.state = { .active = false, .direction = false },
        .forearm.state = { .active = false, .direction = false },
        .wrist.state = { .active = false, .direction = false },
        .gripper.state = { .active = false, .direction = false }
    },
    .uart_state = { .uart_ready = false }

};

#endif
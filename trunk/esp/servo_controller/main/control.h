#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

#define BASE_GPIO GPIO_NUM_4
#define SHOULDER_GPIO GPIO_NUM_17
#define FOREARM_GPIO GPIO_NUM_16 
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

#define BASE_MAX_STEP_US_PER_TICK 3
#define SHOULDER_MAX_STEP_US_PER_TICK 2
#define FOREARM_MAX_STEP_US_PER_TICK 5
#define WRIST_MAX_STEP_US_PER_TICK 14
#define GRIPPER_MAX_STEP_US_PER_TICK 18

#define SERVO_ACCEL_STEP_US_PER_TICK 1
#define SERVO_DECEL_STEP_US_PER_TICK 2

#define CONTROL_LOOP_HZ 100
#define CONTROL_TIMEOUT_MS 250

typedef enum {
    BASE = 0,
    SHOULDER,
    FOREARM,
    WRIST,
    GRIPPER,
} servo_id_t;

typedef enum {
    accelerating,
    decelerating,
    at_max_speed,
    at_target
} control_status_t;

typedef struct {
    bool active;
    bool direction;
    control_status_t status;
    uint32_t current_pulse_us;
    uint32_t last_written_pulse_us;
    int32_t servo_step_us;
} servo_state_t;

typedef struct {
    servo_state_t servos[SERVO_COUNT];
} control_state_t;

#endif
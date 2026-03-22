#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"


#define CONTROL_LOOP_HZ 100 //update servo pulses every 10ms. 
                            //max servo pulse: 2.5ms, min is 0.5ms
                            //control loop has ~7.5ms to update and handle other tasks.


static void servo_write_us(ledc_channel_t channel, uint32_t pulse_us);
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);
static inline uint32_t servo_us_to_duty(uint32_t pulse_us);
static void configure_servo_channel(gpio_num_t gpio, ledc_channel_t channel);
static void control_startup(void);

typedef enum {

    BASE,
    SHOULDER,
    FOREARM,
    WRIST,
    GRIPPER
} servo_id_t;

typedef struct {
    servo_id_t id;
    uint32_t current_pulse_us;
    servo_state_t state;
    control_status status;
} servo;

#endif
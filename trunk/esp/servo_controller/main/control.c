#include "control.h"

static void servo_write_us(ledc_channel_t channel, uint32_t pulse_us);
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);
static inline uint32_t servo_us_to_duty(uint32_t pulse_us);
static void configure_servo_channel(gpio_num_t gpio, ledc_channel_t channel);
static void control_startup(void);
static void center_all_servos(void);
static void update_control_state_from_request(
    servo_state_t *control_servo,
    const servo_request_t *requested_servo
);
static void apply_control_state(
    servo_state_t *control_servo,
    int32_t *current_step_us,
    uint32_t max_step_us,
    const servo_request_t *requested_servo
);

control_state_t control_state = {0};

static const ledc_channel_t servo_channels[SERVO_COUNT] = {
    BASE_CHANNEL,
    SHOULDER_CHANNEL,
    FOREARM_CHANNEL,
    WRIST_CHANNEL,
    GRIPPER_CHANNEL
};

static const uint32_t servo_max_step_us[SERVO_COUNT] = {
    BASE_MAX_STEP_US_PER_TICK,
    SHOULDER_MAX_STEP_US_PER_TICK,
    FOREARM_MAX_STEP_US_PER_TICK,
    WRIST_MAX_STEP_US_PER_TICK,
    GRIPPER_MAX_STEP_US_PER_TICK
};

void servo_control_task(void *arg)
{
    (void)arg;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(20);
    const TickType_t commandTimeoutTicks = pdMS_TO_TICKS(CONTROL_TIMEOUT_MS);

    requested_state_t requested_state = {0};
    requested_state_t received_state;
    requested_state_t timeout_state = {0};

    TickType_t last_command_tick = xTaskGetTickCount();

    control_startup();

    while (!system_state.shutdown_requested) {
        const requested_state_t *effective_state = &requested_state;
        int i;

        if (xQueueReceive(servo_command_q, &received_state, 0) == pdTRUE) {
            requested_state = received_state;
            last_command_tick = xTaskGetTickCount();
        }

        if ((xTaskGetTickCount() - last_command_tick) >= commandTimeoutTicks) {
            effective_state = &timeout_state;
        }

        for (i = 0; i < SERVO_COUNT; i++) {
            update_control_state_from_request(
                &control_state.servos[i],
                &effective_state->servos[i]
            );

            apply_control_state(
                &control_state.servos[i],
                &control_state.servos[i].servo_step_us,
                servo_max_step_us[i],
                &effective_state->servos[i]
            );

            if (control_state.servos[i].current_pulse_us !=
                control_state.servos[i].last_written_pulse_us) {

                servo_write_us(
                    servo_channels[i],
                    control_state.servos[i].current_pulse_us
                );

                control_state.servos[i].last_written_pulse_us =
                    control_state.servos[i].current_pulse_us;
            }
        }

        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }

    center_all_servos();
}

static void update_control_state_from_request(servo_state_t *control_servo, const servo_request_t *requested_servo)
{
    if (!requested_servo->active) {
        if (control_servo->active) {
            control_servo->status = decelerating;
        }
        return;
    }

    if (!control_servo->active) {
        control_servo->active = true;
        control_servo->direction = requested_servo->direction;
        control_servo->status = accelerating;
        return;
    }

    if (control_servo->direction != requested_servo->direction) {
        control_servo->status = decelerating;
        return;
    }

    if (control_servo->status == at_target) {
        control_servo->status = accelerating;
    }
}

static void apply_control_state(servo_state_t *control_servo, int32_t *current_step_us, uint32_t max_step_us, const servo_request_t *requested_servo)
{
    if (control_servo->status == decelerating) {
        if (*current_step_us > 0) {
            *current_step_us -= SERVO_DECEL_STEP_US_PER_TICK;
            if (*current_step_us < 0) {
                *current_step_us = 0;
            }
        }

        if (*current_step_us == 0) {
            if (!requested_servo->active) {
                control_servo->active = false;
                control_servo->status = at_target;
            }
            else if (control_servo->direction != requested_servo->direction) {
                control_servo->direction = requested_servo->direction;
                control_servo->active = true;
                control_servo->status = accelerating;
            }
            else {
                control_servo->active = false;
                control_servo->status = at_target;
            }
        }
    }
    else if (control_servo->status == accelerating) {
        control_servo->active = true;

        if (*current_step_us < (int32_t)max_step_us) {
            *current_step_us += SERVO_ACCEL_STEP_US_PER_TICK;
            if (*current_step_us > (int32_t)max_step_us) {
                *current_step_us = (int32_t)max_step_us;
            }
        }

        if (*current_step_us >= (int32_t)max_step_us) {
            control_servo->status = at_max_speed;
        }
    }
    else if (control_servo->status == at_max_speed) {
        control_servo->active = true;
        *current_step_us = (int32_t)max_step_us;
    }
    else {
        control_servo->active = false;
        *current_step_us = 0;
    }

    if (*current_step_us > 0) {
        if (control_servo->direction) {
            if (control_servo->current_pulse_us < SERVO_US_MAX_SAFE) {
                control_servo->current_pulse_us += (uint32_t)(*current_step_us);
                if (control_servo->current_pulse_us > SERVO_US_MAX_SAFE) {
                    control_servo->current_pulse_us = SERVO_US_MAX_SAFE;
                }
            }
        }
        else {
            uint32_t step_u32 = (uint32_t)(*current_step_us);

            if (control_servo->current_pulse_us > SERVO_US_MIN_SAFE + step_u32) {
                control_servo->current_pulse_us -= step_u32;
            }
            else {
                control_servo->current_pulse_us = SERVO_US_MIN_SAFE;
            }
        }
    }

    if (control_servo->current_pulse_us == SERVO_US_MIN_SAFE ||
        control_servo->current_pulse_us == SERVO_US_MAX_SAFE) {
        *current_step_us = 0;
        control_servo->active = false;
        control_servo->status = at_target;
    }
}

static void control_startup(void)
{
    int i;

    system_state.shutdown_requested = false;

    for (i = 0; i < SERVO_COUNT; i++) {
        control_state.servos[i].active = false;
        control_state.servos[i].direction = false;
        control_state.servos[i].status = at_target;
        control_state.servos[i].current_pulse_us = SERVO_US_CENTER;
        control_state.servos[i].last_written_pulse_us = SERVO_US_CENTER;
        control_state.servos[i].servo_step_us = 0;
    }

    center_all_servos();
}

static void center_all_servos(void)
{
    int i;

    for (i = 0; i < SERVO_COUNT; i++) {
        control_state.servos[i].active = false;
        control_state.servos[i].direction = false;
        control_state.servos[i].status = at_target;
        control_state.servos[i].current_pulse_us = SERVO_US_CENTER;
        control_state.servos[i].last_written_pulse_us = SERVO_US_CENTER;
        control_state.servos[i].servo_step_us = 0;

        servo_write_us(servo_channels[i], SERVO_US_CENTER);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


static void servo_write_us(ledc_channel_t channel, uint32_t pulse_us)
{
    pulse_us = clamp_u32(pulse_us, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);

    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_SPEED_MODE,
        channel,
        servo_us_to_duty(pulse_us)
    ));

    ESP_ERROR_CHECK(ledc_update_duty(
        LEDC_SPEED_MODE,
        channel
    ));
}

void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_SPEED_MODE,
        .timer_num        = LEDC_TIMER_ID,
        .duty_resolution  = LEDC_DUTY_RESOLUTION,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    configure_servo_channel(BASE_GPIO, BASE_CHANNEL);
    configure_servo_channel(SHOULDER_GPIO, SHOULDER_CHANNEL);
    configure_servo_channel(FOREARM_GPIO, FOREARM_CHANNEL);
    configure_servo_channel(WRIST_GPIO, WRIST_CHANNEL);
    configure_servo_channel(GRIPPER_GPIO, GRIPPER_CHANNEL);
}

static void configure_servo_channel(gpio_num_t gpio, ledc_channel_t channel)
{
    ledc_channel_config_t ch = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = channel,
        .timer_sel  = LEDC_TIMER_ID,
        .duty       = servo_us_to_duty(SERVO_US_CENTER),
        .hpoint     = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_SPEED_MODE, channel));
}

static inline uint32_t servo_us_to_duty(uint32_t pulse_us)
{
    const uint32_t duty_max = (1u << 16) - 1u;

    pulse_us = clamp_u32(pulse_us, 0, SERVO_PERIOD_US);
    return (pulse_us * duty_max) / SERVO_PERIOD_US;
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
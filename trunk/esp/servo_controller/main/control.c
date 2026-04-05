#include "control.h"

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);
static void control_startup(void);
static void center_all_servos(void);
static void servo_write_us(servo_id_t servo, uint32_t pulse_us);
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


static mcpwm_timer_handle_t servo_timer = NULL;
control_state_t control_state = {0};
servo_output_t servo_outputs[SERVO_COUNT] = {0};


const gpio_num_t servo_gpios[SERVO_COUNT] = {
    BASE_GPIO,
    SHOULDER_GPIO,
    FOREARM_GPIO,
    WRIST_GPIO,
    GRIPPER_GPIO
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
    const TickType_t periodTicks = pdMS_TO_TICKS(10);
    const TickType_t commandTimeoutTicks = pdMS_TO_TICKS(30);

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
                    (servo_id_t)i,
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

        servo_write_us((servo_id_t)i, SERVO_US_CENTER);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void servo_init(void)
{
    int i;

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_FRAME_US,
        .flags.update_period_on_empty = false,
        .flags.update_period_on_sync = false,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &servo_timer));

    for (i = 0; i < SERVO_COUNT; i++) {
        mcpwm_operator_config_t operator_config = {
            .group_id = 0,
        };

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = servo_gpios[i],
        };

        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &servo_outputs[i].oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(servo_outputs[i].oper, servo_timer));

        ESP_ERROR_CHECK(mcpwm_new_comparator(
            servo_outputs[i].oper,
            &comparator_config,
            &servo_outputs[i].comparator
        ));

        ESP_ERROR_CHECK(mcpwm_new_generator(
            servo_outputs[i].oper,
            &generator_config,
            &servo_outputs[i].generator
        ));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            servo_outputs[i].generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH
            )
        ));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            servo_outputs[i].generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                servo_outputs[i].comparator,
                MCPWM_GEN_ACTION_LOW
            )
        ));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
            servo_outputs[i].comparator,
            SERVO_US_CENTER
        ));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(servo_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(servo_timer, MCPWM_TIMER_START_NO_STOP));
}


static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void servo_write_us(servo_id_t servo, uint32_t pulse_us)
{
    pulse_us = clamp_u32(pulse_us, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
        servo_outputs[servo].comparator,
        pulse_us
    ));
}
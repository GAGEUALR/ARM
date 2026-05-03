#include "control.h"

#ifndef SERVO_COMMAND_MOVE
#define SERVO_COMMAND_MOVE 0x01
#endif

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);
static void control_startup(void);
static void center_all_servos(void);
static void servo_write_us(servo_id_t servo, uint32_t pulse_us);
static void debug_gpio_init(void);
static void debug_toggle_loop(void);
static void debug_toggle_write(void);

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
servo_output_t servo_outputs[SERVO_COUNT] = {0};

#if DEBUG_GPIO_ENABLE
static bool debugLoopState = false;
static bool debugWriteState = false;
#endif

static mcpwm_timer_handle_t servo_timers[2] = {0};

static const int servo_group_ids[SERVO_COUNT] = {
    0,
    0,
    0,
    1,
    1
};

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
    const TickType_t commandTimeoutTicks = pdMS_TO_TICKS(200);

    requested_state_t received_state = {0};
    requested_state_t requested_state = {0};

    TickType_t last_command_tick = xTaskGetTickCount();

    control_startup();

    while (!system_state.shutdown_requested) {
        debug_toggle_loop();

        bool command_timed_out;
        int i;

        if (xQueueReceive(servo_command_q, &received_state, 0) == pdTRUE) {
            requested_state = received_state;
            last_command_tick = xTaskGetTickCount();
        }

        command_timed_out =
            ((xTaskGetTickCount() - last_command_tick) >= commandTimeoutTicks);

        for (i = 0; i < SERVO_COUNT; i++) {
            if (!command_timed_out) {
                update_control_state_from_request(
                    &control_state.servos[i],
                    &requested_state.servos[i]
                );
            }
            else {
                control_state.servos[i].current_requested_pulse =
                    control_state.servos[i].current_pulse_us;

                control_state.servos[i].active = false;
            }

            apply_control_state(
                &control_state.servos[i],
                &control_state.servos[i].servo_step_us,
                servo_max_step_us[i],
                &requested_state.servos[i]
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

static void update_control_state_from_request(
    servo_state_t *control_servo,
    const servo_request_t *requested_servo
)
{
    if (requested_servo->command_type != SERVO_COMMAND_MOVE) {
        return;
    }

    control_servo->current_requested_pulse = (int32_t)clamp_u32(
        (uint32_t)requested_servo->requested_pulse,
        SERVO_US_MIN_SAFE,
        SERVO_US_MAX_SAFE
    );

    control_servo->active =
        (control_servo->current_requested_pulse !=
         (int32_t)control_servo->current_pulse_us);
}

static void apply_control_state(
    servo_state_t *control_servo,
    int32_t *current_step_us,
    uint32_t max_step_us,
    const servo_request_t *requested_servo
)
{
    (void)requested_servo;

    uint32_t target_pulse;

    target_pulse = clamp_u32(
        (uint32_t)control_servo->current_requested_pulse,
        SERVO_US_MIN_SAFE,
        SERVO_US_MAX_SAFE
    );

    *current_step_us = (int32_t)max_step_us;

    if (target_pulse < control_servo->current_pulse_us) {
        uint32_t difference =
            control_servo->current_pulse_us - target_pulse;

        if (difference <= max_step_us) {
            control_servo->current_pulse_us = target_pulse;
        }
        else {
            control_servo->current_pulse_us -= max_step_us;
        }
    }
    else if (target_pulse > control_servo->current_pulse_us) {
        uint32_t difference =
            target_pulse - control_servo->current_pulse_us;

        if (difference <= max_step_us) {
            control_servo->current_pulse_us = target_pulse;
        }
        else {
            control_servo->current_pulse_us += max_step_us;
        }
    }

    control_servo->current_pulse_us = clamp_u32(
        control_servo->current_pulse_us,
        SERVO_US_MIN_SAFE,
        SERVO_US_MAX_SAFE
    );

    if (control_servo->current_pulse_us == target_pulse) {
        *current_step_us = 0;
        control_servo->active = false;
    }
}

static void control_startup(void)
{
    int i;

    system_state.shutdown_requested = false;

    for (i = 0; i < SERVO_COUNT; i++) {
        control_state.servos[i].active = false;
        control_state.servos[i].current_pulse_us = SERVO_US_CENTER;
        control_state.servos[i].current_requested_pulse = SERVO_US_CENTER;
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
        control_state.servos[i].current_pulse_us = SERVO_US_CENTER;
        control_state.servos[i].current_requested_pulse = SERVO_US_CENTER;
        control_state.servos[i].last_written_pulse_us = SERVO_US_CENTER;
        control_state.servos[i].servo_step_us = 0;

        servo_write_us((servo_id_t)i, SERVO_US_CENTER);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void servo_init(void)
{
    debug_gpio_init();

    int group_id;
    int i;

    for (group_id = 0; group_id < 2; group_id++) {
        mcpwm_timer_config_t timer_config = {
            .group_id = group_id,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = SERVO_FRAME_US,
            .flags.update_period_on_empty = false,
            .flags.update_period_on_sync = false,
        };

        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &servo_timers[group_id]));
    }

    for (i = 0; i < SERVO_COUNT; i++) {
        int current_group = servo_group_ids[i];

        mcpwm_operator_config_t operator_config = {
            .group_id = current_group,
        };

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = servo_gpios[i],
        };

        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &servo_outputs[i].oper));

        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(
            servo_outputs[i].oper,
            servo_timers[current_group]
        ));

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

    for (group_id = 0; group_id < 2; group_id++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(servo_timers[group_id]));

        ESP_ERROR_CHECK(mcpwm_timer_start_stop(
            servo_timers[group_id],
            MCPWM_TIMER_START_NO_STOP
        ));
    }
}

static void debug_gpio_init(void)
{
#if DEBUG_GPIO_ENABLE
    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << DEBUG_LOOP_GPIO) |
            (1ULL << DEBUG_PACKET_GPIO) |
            (1ULL << DEBUG_WRITE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    gpio_set_level(DEBUG_LOOP_GPIO, 0);
    gpio_set_level(DEBUG_PACKET_GPIO, 0);
    gpio_set_level(DEBUG_WRITE_GPIO, 0);
#endif
}

static void debug_toggle_loop(void)
{
#if DEBUG_GPIO_ENABLE
    debugLoopState = !debugLoopState;
    gpio_set_level(DEBUG_LOOP_GPIO, debugLoopState);
#endif
}

static void debug_toggle_write(void)
{
#if DEBUG_GPIO_ENABLE
    debugWriteState = !debugWriteState;
    gpio_set_level(DEBUG_WRITE_GPIO, debugWriteState);
#endif
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) {
        return lo;
    }

    if (v > hi) {
        return hi;
    }

    return v;
}

static void servo_write_us(servo_id_t servo, uint32_t pulse_us)
{
    pulse_us = clamp_u32(pulse_us, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);

    debug_toggle_write();

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
        servo_outputs[servo].comparator,
        pulse_us
    ));
}
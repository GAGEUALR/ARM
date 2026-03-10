#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"


static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int abs_i(int v)
{
    return (v < 0) ? -v : v;
}


static inline uint32_t servo_us_to_duty(uint32_t pulse_us)
{
    const uint32_t duty_max = (1u << 16) - 1u;

    pulse_us = clamp_u32(pulse_us, 0, SERVO_PERIOD_US);
    return (pulse_us * duty_max) / SERVO_PERIOD_US;
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

static void servo_init(void)
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

static void servo_write_us(ledc_channel_t channel, uint32_t pulse_us)
{
    pulse_us = clamp_u32(pulse_us, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);

    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_SPEED_MODE,
        channel,
        servo_us_to_duty(pulse_us)
    ));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_SPEED_MODE, channel));
}

static void servo_disable(ledc_channel_t channel)
{
    ESP_ERROR_CHECK(ledc_stop(LEDC_SPEED_MODE, channel, 0));
}

static void usb_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = USB_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(USB_UART_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(USB_UART_NUM, &cfg));
}

static const char *skip_spaces(const char *s)
{
    while (*s && isspace((unsigned char)*s)) {
        s++;
    }
    return s;
}

static int parse_kv_line(const char *line, char key_out[5], int *value_out)
{
    line = skip_spaces(line);

    if (!isalpha((unsigned char)line[0]) || !isalpha((unsigned char)line[1])) {
        return 0;
    }

    char key[5] = {0};
    int key_len = 0;

    while (isalpha((unsigned char)*line) && key_len < 4) {
        key[key_len++] = (char)toupper((unsigned char)*line);
        line++;
    }

    line = skip_spaces(line);

    if (*line != '=') {
        return 0;
    }
    line++;

    line = skip_spaces(line);

    char *end = NULL;
    long v = strtol(line, &end, 10);

    if (end == line) {
        return 0;
    }

    end = (char *)skip_spaces(end);

    if (*end != '\0') {
        return 0;
    }

    strcpy(key_out, key);
    *value_out = (int)v;
    return 1;
}

static void uart_rx_task(void *arg)
{
    (void)arg;

    uint8_t ch;
    char line[64];
    int idx = 0;

    while (1) {
        int n = uart_read_bytes(USB_UART_NUM, &ch, 1, pdMS_TO_TICKS(100));

        if (n <= 0) {
            continue;
        }

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            line[idx] = '\0';

            if (idx > 0) {
                if (strcmp(line, "SHUTDOWN") == 0) {
                    system.shutdown_requested = 1;
                } else {
                    char key[5];
                    int val;

                    if (parse_kv_line(line, key, &val)) {
                        if (strcmp(key, "LT") == 0) {
                            val = clamp_i(val, 0, TRIGGER_RAW_MAX);
                            if (val <= TRIGGER_DEADBAND) val = 0;
                            system.lt = val;
                        }
                        else if (strcmp(key, "RT") == 0) {
                            val = clamp_i(val, 0, TRIGGER_RAW_MAX);
                            if (val <= TRIGGER_DEADBAND) val = 0;
                            system.rt = val;
                        }
                        else if (strcmp(key, "LSX") == 0) {
                            val = clamp_i(val, -STICK_RAW_MAX_SIGNED, STICK_RAW_MAX_SIGNED);
                            if (abs_i(val) <= STICK_DEADBAND_SIGNED) val = 0;
                            system.lsx = val;
                        }
                        else if (strcmp(key, "RSX") == 0) {
                            val = clamp_i(val, -STICK_RAW_MAX_SIGNED, STICK_RAW_MAX_SIGNED);
                            if (abs_i(val) <= STICK_DEADBAND_SIGNED) val = 0;
                            system.rsx = val;
                        }
                        else if (strcmp(key, "LB") == 0) {
                            system.lb = (val != 0) ? 1 : 0;
                        }
                        else if (strcmp(key, "RB") == 0) {
                            system.rb = (val != 0) ? 1 : 0;
                        }
                        else if (strcmp(key, "DPX") == 0) {
                            system.dpx = clamp_i(val, -1, 1);
                        }
                    }
                }
            }

            idx = 0;
            continue;
        }

        if (idx < (int)sizeof(line) - 1) {
            line[idx++] = (char)ch;
        } else {
            idx = 0;
        }
    }
}

static int step_from_trigger(int trig, int max_step_per_tick)
{
    if (trig <= 0) {
        return 0;
    }

    int step = (trig * max_step_per_tick) / TRIGGER_RAW_MAX;

    if (step < 1) {
        step = 1;
    }

    return step;
}

static int step_from_stick_signed(int value, int max_step_per_tick)
{
    if (value == 0) {
        return 0;
    }

    int magnitude = abs_i(value);
    int step = (magnitude * max_step_per_tick) / STICK_RAW_MAX_SIGNED;

    if (step < 1) {
        step = 1;
    }

    return (value < 0) ? -step : step;
}

static int step_from_button_pair(int negative_pressed, int positive_pressed, int max_step_per_tick)
{
    if (positive_pressed && !negative_pressed) {
        return max_step_per_tick;
    }

    if (negative_pressed && !positive_pressed) {
        return -max_step_per_tick;
    }

    return 0;
}

static int step_from_dpad_x(int dpx, int max_step_per_tick)
{
    if (dpx > 0) {
        return max_step_per_tick;
    }

    if (dpx < 0) {
        return -max_step_per_tick;
    }

    return 0;
}

static void move_toward_target(uint32_t *pulse, uint32_t target, uint32_t step_us)
{
    if (*pulse < target) {
        uint32_t d = target - *pulse;
        *pulse += (d > step_us) ? step_us : d;
    }
    else if (*pulse > target) {
        uint32_t d = *pulse - target;
        *pulse -= (d > step_us) ? step_us : d;
    }
}

static int pulse_at_target(uint32_t a, uint32_t b, uint32_t tolerance)
{
    if (a > b) {
        return (a - b) <= tolerance;
    }
    return (b - a) <= tolerance;
}

static void consider_request(
    motion_request_t req,
    motion_request_t *best_a,
    motion_request_t *best_b
)
{
    if (req.magnitude <= 0) {
        return;
    }

    bool better_than_a = false;
    bool better_than_b = false;

    if (req.magnitude > best_a->magnitude) {
        better_than_a = true;
    } else if (req.magnitude == best_a->magnitude && req.priority < best_a->priority) {
        better_than_a = true;
    }

    if (better_than_a) {
        *best_b = *best_a;
        *best_a = req;
        return;
    }

    if (req.magnitude > best_b->magnitude) {
        better_than_b = true;
    } else if (req.magnitude == best_b->magnitude && req.priority < best_b->priority) {
        better_than_b = true;
    }

    if (better_than_b) {
        *best_b = req;
    }
}

static void servo_control_task(void *arg)
{
    uint32_t base_pulse     = SERVO_US_CENTER;
    uint32_t shoulder_pulse = SERVO_US_CENTER;
    uint32_t forearm_pulse  = SERVO_US_CENTER;
    uint32_t wrist_pulse    = SERVO_US_CENTER;
    uint32_t gripper_pulse  = SERVO_US_CENTER;

    /* Example shutdown positions */
    const uint32_t BASE_REST_US     = SERVO_US_CENTER;
    const uint32_t SHOULDER_REST_US = 1200;
    const uint32_t FOREARM_REST_US  = 1400;
    const uint32_t WRIST_REST_US    = SERVO_US_CENTER;
    const uint32_t GRIPPER_REST_US  = SERVO_US_CENTER;

    const uint32_t SHUTDOWN_STEP_US = 12;
    const uint32_t SHUTDOWN_TOL_US  = 8;

    int shutdown_complete = 0;

    servo_write_us(BASE_CHANNEL, base_pulse);
    servo_write_us(SHOULDER_CHANNEL, shoulder_pulse);
    servo_write_us(FOREARM_CHANNEL, forearm_pulse);
    servo_write_us(WRIST_CHANNEL, wrist_pulse);
    servo_write_us(GRIPPER_CHANNEL, gripper_pulse);

    while (1) {
        if (system.shutdown_requested && !shutdown_complete) {

            servo_write_us(BASE_CHANNEL, SERVO_US_CENTER);
            vTaskDelay(pdMS_TO_TICKS(100));

            servo_write_us(SHOULDER_CHANNEL, SERVO_US_CENTER);
            vTaskDelay(pdMS_TO_TICKS(100));

            servo_write_us(FOREARM_CHANNEL, SERVO_US_CENTER);
            vTaskDelay(pdMS_TO_TICKS(100));

            servo_write_us(WRIST_CHANNEL, SERVO_US_CENTER);
            vTaskDelay(pdMS_TO_TICKS(100));

            servo_write_us(GRIPPER_CHANNEL, SERVO_US_CENTER);
            vTaskDelay(pdMS_TO_TICKS(100));

            continue;
        }

        if (shutdown_complete) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* ----- Read latest controls ----- */
        int lt  = system.lt;
        int rt  = system.rt;
        int lsx = system.lsx;
        int rsx = system.rsx;
        int lb  = system.lb;
        int rb  = system.rb;
        int dpx = system.dpx;

        /* ----- Convert controls to requested step sizes ----- */
        int base_step = step_from_stick_signed(lsx, BASE_MAX_STEP_US_PER_TICK);
        int shoulder_step = step_from_stick_signed(rsx, SHOULDER_MAX_STEP_US_PER_TICK);
        int forearm_step = step_from_button_pair(lb, rb, FOREARM_MAX_STEP_US_PER_TICK);
        int wrist_step = step_from_dpad_x(dpx, WRIST_MAX_STEP_US_PER_TICK);

        int gripper_step = 0;
        int grip_open_step = step_from_trigger(rt, GRIPPER_MAX_STEP_US_PER_TICK);
        int grip_close_step = step_from_trigger(lt, GRIPPER_MAX_STEP_US_PER_TICK);

        if (grip_open_step > 0 && grip_close_step == 0) {
            gripper_step = grip_open_step;
        }
        else if (grip_close_step > 0 && grip_open_step == 0) {
            gripper_step = -grip_close_step;
        }

        /* ----- Choose only the two strongest requests ----- */
        motion_request_t best_a = { .servo_index = -1, .step = 0, .magnitude = 0, .priority = 999 };
        motion_request_t best_b = { .servo_index = -1, .step = 0, .magnitude = 0, .priority = 999 };

        consider_request((motion_request_t){ 0, base_step,     abs_i(base_step),     2 }, &best_a, &best_b);
        consider_request((motion_request_t){ 1, shoulder_step, abs_i(shoulder_step), 1 }, &best_a, &best_b);
        consider_request((motion_request_t){ 2, forearm_step,  abs_i(forearm_step),  3 }, &best_a, &best_b);
        consider_request((motion_request_t){ 3, wrist_step,    abs_i(wrist_step),    4 }, &best_a, &best_b);
        consider_request((motion_request_t){ 4, gripper_step,  abs_i(gripper_step),  0 }, &best_a, &best_b);

        int selected_step[5] = {0, 0, 0, 0, 0};

        if (best_a.servo_index >= 0) {
            selected_step[best_a.servo_index] = best_a.step;
        }

        if (best_b.servo_index >= 0) {
            selected_step[best_b.servo_index] = best_b.step;
        }

        /* ----- Apply only selected motions ----- */
        if (selected_step[0] > 0) {
            base_pulse += (uint32_t)selected_step[0];
        }
        else if (selected_step[0] < 0) {
            uint32_t amount = (uint32_t)(-selected_step[0]);
            if (base_pulse > amount) {
                base_pulse -= amount;
            } else {
                base_pulse = 0;
            }
        }

        if (selected_step[1] > 0) {
            shoulder_pulse += (uint32_t)selected_step[1];
        }
        else if (selected_step[1] < 0) {
            uint32_t amount = (uint32_t)(-selected_step[1]);
            if (shoulder_pulse > amount) {
                shoulder_pulse -= amount;
            } else {
                shoulder_pulse = 0;
            }
        }

        if (selected_step[2] > 0) {
            forearm_pulse += (uint32_t)selected_step[2];
        }
        else if (selected_step[2] < 0) {
            uint32_t amount = (uint32_t)(-selected_step[2]);
            if (forearm_pulse > amount) {
                forearm_pulse -= amount;
            } else {
                forearm_pulse = 0;
            }
        }

        if (selected_step[3] > 0) {
            wrist_pulse += (uint32_t)selected_step[3];
        }
        else if (selected_step[3] < 0) {
            uint32_t amount = (uint32_t)(-selected_step[3]);
            if (wrist_pulse > amount) {
                wrist_pulse -= amount;
            } else {
                wrist_pulse = 0;
            }
        }

        if (selected_step[4] > 0) {
            gripper_pulse += (uint32_t)selected_step[4];
        }
        else if (selected_step[4] < 0) {
            uint32_t amount = (uint32_t)(-selected_step[4]);
            if (gripper_pulse > amount) {
                gripper_pulse -= amount;
            } else {
                gripper_pulse = 0;
            }
        }

        /* ----- Clamp ----- */
        base_pulse     = clamp_u32(base_pulse,     SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);
        shoulder_pulse = clamp_u32(shoulder_pulse, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);
        forearm_pulse  = clamp_u32(forearm_pulse,  SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);
        wrist_pulse    = clamp_u32(wrist_pulse,    SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);
        gripper_pulse  = clamp_u32(gripper_pulse,  SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);

        /* ----- Output ----- */
        servo_write_us(BASE_CHANNEL, base_pulse);
        servo_write_us(SHOULDER_CHANNEL, shoulder_pulse);
        servo_write_us(FOREARM_CHANNEL, forearm_pulse);
        servo_write_us(WRIST_CHANNEL, wrist_pulse);
        servo_write_us(GRIPPER_CHANNEL, gripper_pulse);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* =========================================================
   app_main
   ========================================================= */
void app_main(void)
{
    servo_init();
    usb_uart_init();

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 9, NULL);
}
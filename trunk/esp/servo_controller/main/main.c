#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"


#define BASE_GPIO                GPIO_NUM_4
#define SHOULDER_GPIO            GPIO_NUM_17

#define LEDC_TIMER               LEDC_TIMER_0
#define LEDC_MODE                LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES            LEDC_TIMER_16_BIT

#define BASE_CHANNEL             LEDC_CHANNEL_0
#define SHOULDER_CHANNEL         LEDC_CHANNEL_1

#define SERVO_FREQ_HZ            50
#define SERVO_PERIOD_US          20000

#define SERVO_US_MIN_SAFE        500
#define SERVO_US_MAX_SAFE        2500
#define SERVO_US_CENTER          1500

#define BASE_MAX_STEP_US_PER_TICK       14
#define SHOULDER_MAX_STEP_US_PER_TICK   14

#define TRIGGER_RAW_MAX          1023
#define STICK_RAW_MAX_SIGNED     1023

#define TRIGGER_DEADBAND         60
#define STICK_DEADBAND_SIGNED    200

#define USB_UART_NUM             UART_NUM_0
#define USB_UART_BAUD            115200
#define UART_RX_BUF_SIZE         1024

typedef struct {
    volatile int rt;
    volatile int lt;
    volatile int rsx;
} control_state_t;

static control_state_t g = {
    .rt = 0,
    .lt = 0,
    .rsx = 0
};

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

static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t base = {
        .gpio_num   = BASE_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = BASE_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = servo_us_to_duty(SERVO_US_CENTER),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&base));

    ledc_channel_config_t shoulder = {
        .gpio_num   = SHOULDER_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = SHOULDER_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = servo_us_to_duty(SERVO_US_CENTER),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&shoulder));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, BASE_CHANNEL));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SHOULDER_CHANNEL));
}

static void servo_write_us(ledc_channel_t ch, uint32_t pulse_us)
{
    pulse_us = clamp_u32(pulse_us, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ch, servo_us_to_duty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ch));
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
    while (*s && isspace((unsigned char)*s)) s++;
    return s;
}

static int parse_kv_line(const char *line, char key_out[4], int *value_out)
{
    line = skip_spaces(line);

    if (!isalpha((unsigned char)line[0]) || !isalpha((unsigned char)line[1])) {
        return 0;
    }

    char k0 = (char)toupper((unsigned char)line[0]);
    char k1 = (char)toupper((unsigned char)line[1]);
    char k2 = '\0';

    line += 2;

    if (isalpha((unsigned char)line[0])) {
        k2 = (char)toupper((unsigned char)line[0]);
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

    key_out[0] = k0;
    key_out[1] = k1;
    if (k2) {
        key_out[2] = k2;
        key_out[3] = '\0';
    } else {
        key_out[2] = '\0';
        key_out[3] = '\0';
    }

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
        if (n <= 0) continue;

        if (ch == '\r') continue;

        if (ch == '\n') {
            line[idx] = '\0';

            if (idx > 0) {
                char key[4];
                int val;

                if (parse_kv_line(line, key, &val)) {
                    if (strcmp(key, "RT") == 0) {
                        val = clamp_i(val, 0, TRIGGER_RAW_MAX);
                        if (val <= TRIGGER_DEADBAND) val = 0;
                        g.rt = val;
                    }
                    else if (strcmp(key, "LT") == 0) {
                        val = clamp_i(val, 0, TRIGGER_RAW_MAX);
                        if (val <= TRIGGER_DEADBAND) val = 0;
                        g.lt = val;
                    }
                    else if (strcmp(key, "RSX") == 0) {
                        val = clamp_i(val, -STICK_RAW_MAX_SIGNED, STICK_RAW_MAX_SIGNED);
                        if (abs_i(val) <= STICK_DEADBAND_SIGNED) val = 0;
                        g.rsx = val;
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

static int step_from_trigger(int trig)
{
    if (trig <= 0) return 0;

    int step = (trig * BASE_MAX_STEP_US_PER_TICK) / TRIGGER_RAW_MAX;
    if (step < 1) step = 1;
    return step;
}

static int step_from_stick_signed(int value)
{
    if (value == 0) return 0;

    int magnitude = abs_i(value);
    int step = (magnitude * SHOULDER_MAX_STEP_US_PER_TICK) / STICK_RAW_MAX_SIGNED;
    if (step < 1) step = 1;

    if (value < 0) {
        return -step;
    }

    return step;
}

static void servo_control_task(void *arg)
{
    (void)arg;

    uint32_t base_pulse = SERVO_US_CENTER;
    uint32_t shoulder_pulse = SERVO_US_CENTER;

    servo_write_us(BASE_CHANNEL, base_pulse);
    servo_write_us(SHOULDER_CHANNEL, shoulder_pulse);

    while (1) {
        int rt = g.rt;
        int lt = g.lt;
        int rsx = g.rsx;

        int rt_step = step_from_trigger(rt);
        int lt_step = step_from_trigger(lt);
        int shoulder_step = step_from_stick_signed(rsx);

        if (rt_step > 0 && lt_step == 0) {
            base_pulse += (uint32_t)rt_step;
        }
        else if (lt_step > 0 && rt_step == 0) {
            if (base_pulse > (uint32_t)lt_step) {
                base_pulse -= (uint32_t)lt_step;
            } else {
                base_pulse = 0;
            }
        }

        if (shoulder_step > 0) {
            shoulder_pulse += (uint32_t)shoulder_step;
        }
        else if (shoulder_step < 0) {
            uint32_t amount = (uint32_t)(-shoulder_step);
            if (shoulder_pulse > amount) {
                shoulder_pulse -= amount;
            } else {
                shoulder_pulse = 0;
            }
        }

        base_pulse = clamp_u32(base_pulse, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);
        shoulder_pulse = clamp_u32(shoulder_pulse, SERVO_US_MIN_SAFE, SERVO_US_MAX_SAFE);

        servo_write_us(BASE_CHANNEL, base_pulse);
        servo_write_us(SHOULDER_CHANNEL, shoulder_pulse);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    servo_init();
    usb_uart_init();

    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 9, NULL);
}
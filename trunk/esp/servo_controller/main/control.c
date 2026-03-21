#include "main.h"

static void servo_write_us(ledc_channel_t channel, uint32_t pulse_us);
static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi);
static inline uint32_t servo_us_to_duty(uint32_t pulse_us);
static void servo_init(void);
static void configure_servo_channel(gpio_num_t gpio, ledc_channel_t channel);
static bool decode_command(char *cmd);
static const char *skip_spaces(const char *s);
static int parse_command_line(const char *line,
    char key1_out[5], int *value1_out,
    char key2_out[5], int *value2_out);



void servo_control_task(void *arg)
{

    servo_init();

    (void)arg;

    bool startup_active = true;

    servo_write_us(BASE_CHANNEL, SERVO_US_CENTER);
    servo_write_us(SHOULDER_CHANNEL, SERVO_US_CENTER);
    servo_write_us(FOREARM_CHANNEL, SERVO_US_CENTER);
    servo_write_us(WRIST_CHANNEL, SERVO_US_CENTER);
    servo_write_us(GRIPPER_CHANNEL, SERVO_US_CENTER);

    while (1) {

        if (system_state.shutdown_requested) {
            servo_write_us(BASE_CHANNEL, SERVO_US_CENTER);
            servo_write_us(SHOULDER_CHANNEL, SERVO_US_CENTER);
            servo_write_us(FOREARM_CHANNEL, SERVO_US_CENTER);
            servo_write_us(WRIST_CHANNEL, SERVO_US_CENTER);
            servo_write_us(GRIPPER_CHANNEL, SERVO_US_CENTER);

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (startup_active) {

            servo_write_us(BASE_CHANNEL, SERVO_US_CENTER);
            servo_write_us(SHOULDER_CHANNEL, SERVO_US_CENTER);
            servo_write_us(FOREARM_CHANNEL, SERVO_US_CENTER);
            servo_write_us(WRIST_CHANNEL, SERVO_US_CENTER);
            servo_write_us(GRIPPER_CHANNEL, SERVO_US_CENTER);

            if (system_state.rx_valid) {
                char local_cmd[COMMAND_BUFFER_SIZE];
                int i = 0;
                bool valid = false;

                while (i < (COMMAND_BUFFER_SIZE - 1) && system_state.command[i] != '\0') {
                    local_cmd[i] = (char)system_state.command[i];
                    i++;
                }
                local_cmd[i] = '\0';
                system_state.rx_valid = false;

                valid = decode_command(local_cmd);

                if (valid) {
                    startup_active = false;
                }

                if (system_state.send_ack) {
                    uart_write_bytes(USB_UART_NUM, "OK\n", 3);
                    system_state.send_ack = false;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (system_state.rx_valid) {

            char local_cmd[COMMAND_BUFFER_SIZE];
            int i = 0;


            while (i < (COMMAND_BUFFER_SIZE - 1) && system_state.command[i] != '\0') {
                local_cmd[i] = (char)system_state.command[i];
                i++;
            }
            local_cmd[i] = '\0';
            system_state.rx_valid = false;

            decode_command(local_cmd);

            if (system_state.send_ack) {
                uart_write_bytes(USB_UART_NUM, "OK\n", 3);
                system_state.send_ack = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
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

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool decode_command(char *cmd)
{
    char key1[5] = {0};
    char key2[5] = {0};
    int val1 = 0;
    int val2 = 0;

    if (!parse_command_line(cmd, key1, &val1, key2, &val2)) {
        return false;
    }

    if (strcmp(key1, "B") == 0) {
        if (val1 == 0) {
            // BASE LEFT
        } else if (val1 == 1) {
            // BASE RIGHT
        }
    }
    else if (strcmp(key1, "S") == 0) {
        if (val1 == 0) {
            // SHOULDER LEFT
        } else if (val1 == 1) {
            // SHOULDER RIGHT
        }
    }
    else if (strcmp(key1, "F") == 0) {
        if (val1 == 0) {
            // FOREARM LEFT
        } else if (val1 == 1) {
            // FOREARM RIGHT
        }
    }
    else if (strcmp(key1, "W") == 0) {
        if (val1 == 0) {
            // WRIST LEFT
        } else if (val1 == 1) {
            // WRIST RIGHT
        }
    }
    else if (strcmp(key1, "G") == 0) {
        if (val1 == 0) {
            // GRIPPER LEFT
        } else if (val1 == 1) {
            // GRIPPER RIGHT
        }
    }

    if (strcmp(key2, "B") == 0) {
        if (val2 == 0) {
            // BASE LEFT
        } else if (val2 == 1) {
            // BASE RIGHT
        }
    }
    else if (strcmp(key2, "S") == 0) {
        if (val2 == 0) {
            // SHOULDER LEFT
        } else if (val2 == 1) {
            // SHOULDER RIGHT
        }
    }
    else if (strcmp(key2, "F") == 0) {
        if (val2 == 0) {
            // FOREARM LEFT
        } else if (val2 == 1) {
            // FOREARM RIGHT
        }
    }
    else if (strcmp(key2, "W") == 0) {
        if (val2 == 0) {
            // WRIST LEFT
        } else if (val2 == 1) {
            // WRIST RIGHT
        }
    }
    else if (strcmp(key2, "G") == 0) {
        if (val2 == 0) {
            // GRIPPER LEFT
        } else if (val2 == 1) {
            // GRIPPER RIGHT
        }
    }

    return true;
}

static int parse_command_line(const char *line,
    char key1_out[5], int *value1_out,
    char key2_out[5], int *value2_out)
{
    line = skip_spaces(line);

    const char *equals1 = strchr(line, '=');
    const char *comma   = strchr(line, ',');
    const char *equals2 = NULL;

    if (equals1 == NULL || comma == NULL || equals1 > comma) {
        return 0;
    }

    equals2 = strchr(comma + 1, '=');
    if (equals2 == NULL) {
        return 0;
    }

    int key1_len = (int)(equals1 - line);
    while (key1_len > 0 && isspace((unsigned char)line[key1_len - 1])) {
        key1_len--;
    }

    if (key1_len < 1 || key1_len > 4) {
        return 0;
    }

    for (int i = 0; i < key1_len; i++) {
        key1_out[i] = (char)toupper((unsigned char)line[i]);
    }
    key1_out[key1_len] = '\0';

    const char *value1_start = skip_spaces(equals1 + 1);
    char *end1 = NULL;
    long value1 = strtol(value1_start, &end1, 10);

    if (end1 == value1_start) {
        return 0;
    }

    while (*end1 && isspace((unsigned char)*end1)) {
        end1++;
    }

    if (end1 != comma) {
        return 0;
    }

    const char *key2_start = skip_spaces(comma + 1);
    int key2_len = (int)(equals2 - key2_start);

    while (key2_len > 0 && isspace((unsigned char)key2_start[key2_len - 1])) {
        key2_len--;
    }

    if (key2_len < 1 || key2_len > 4) {
        return 0;
    }

    for (int i = 0; i < key2_len; i++) {
        key2_out[i] = (char)toupper((unsigned char)key2_start[i]);
    }
    key2_out[key2_len] = '\0';

    const char *value2_start = skip_spaces(equals2 + 1);
    char *end2 = NULL;
    long value2 = strtol(value2_start, &end2, 10);

    if (end2 == value2_start) {
        return 0;
    }

    end2 = (char *)skip_spaces(end2);
    if (*end2 != '\0') {
        return 0;
    }

    *value1_out = (int)value1;
    *value2_out = (int)value2;

    return 1;
}

static const char *skip_spaces(const char *s)
{
    while (*s && isspace((unsigned char)*s)) {
        s++;
    }
    return s;
}
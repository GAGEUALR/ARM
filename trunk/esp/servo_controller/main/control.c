#include "main.h"
#include "control.h"




void servo_control_task(void *arg)
{
    (void)arg;

    //build system state 
    
    control_startup();

    while(1){

    control_state_t received_state;

    if (xQueueReceive(servo_command_q, &received_state, 0) == pdTRUE){

        //we can only receive two commands at a time from the pi, so we don't need
        //to worry about writing too many servos at once.
        

        //this is a start..it's a bit long
        if (system_state.control_state.base.state.active != received_state.base.state.active){
            ~system_state.control_state.base.state.active;}

        //send pulse on clk

        //I need a 10ms clock source to update the servo pwm. 
    }
    }
}

static void control_startup(void)
{

    //this is a good place for closed-loop control instead of delays
    servo_write_us(BASE_CHANNEL, SERVO_US_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));

    servo_write_us(SHOULDER_CHANNEL, SERVO_US_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));

    servo_write_us(FOREARM_CHANNEL, SERVO_US_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));

    servo_write_us(WRIST_CHANNEL, SERVO_US_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));

    servo_write_us(GRIPPER_CHANNEL, SERVO_US_CENTER);
    vTaskDelay(pdMS_TO_TICKS(500));

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

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


#include <stdio.h>


/* =========================================================
   GPIO assignment
   Edit these to match your wiring.
   ========================================================= */
#define BASE_GPIO               GPIO_NUM_4
#define FOREARM_GPIO            GPIO_NUM_16
#define SHOULDER_GPIO           GPIO_NUM_17
#define WRIST_GPIO              GPIO_NUM_18
#define GRIPPER_GPIO            GPIO_NUM_19

#define LEDC_TIMER_ID           LEDC_TIMER_0
#define LEDC_SPEED_MODE         LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RESOLUTION    LEDC_TIMER_16_BIT

#define BASE_CHANNEL            LEDC_CHANNEL_0
#define SHOULDER_CHANNEL        LEDC_CHANNEL_1
#define FOREARM_CHANNEL         LEDC_CHANNEL_2
#define WRIST_CHANNEL           LEDC_CHANNEL_3
#define GRIPPER_CHANNEL         LEDC_CHANNEL_4

#define SERVO_FREQ_HZ           50
#define SERVO_PERIOD_US         20000

#define SERVO_US_MIN_SAFE       500
#define SERVO_US_MAX_SAFE       2500
#define SERVO_US_CENTER         1500

/* Maximum movement per 20 ms tick */
#define BASE_MAX_STEP_US_PER_TICK       7
#define SHOULDER_MAX_STEP_US_PER_TICK   5
#define FOREARM_MAX_STEP_US_PER_TICK    5
#define WRIST_MAX_STEP_US_PER_TICK      14
#define GRIPPER_MAX_STEP_US_PER_TICK    18

/* Input scaling */
#define TRIGGER_RAW_MAX         1023
#define STICK_RAW_MAX_SIGNED    1023

#define TRIGGER_DEADBAND        60
#define STICK_DEADBAND_SIGNED   200

/* USB serial */
#define USB_UART_NUM            UART_NUM_0
#define USB_UART_BAUD           115200
#define UART_RX_BUF_SIZE        1024

/* Two-servo movement limit */
#define MAX_ACTIVE_SERVOS_PER_TICK      2

/* =========================================================
   Shared state from Pi
   ========================================================= */
typedef struct {
    volatile int lt;       /* 0..1023 */
    volatile int rt;       /* 0..1023 */
    volatile int lsx;      /* -1023..1023 */
    volatile int rsx;      /* -1023..1023 */
    volatile int lb;       /* 0 or 1 */
    volatile int rb;       /* 0 or 1 */
    volatile int dpx;      /* -1, 0, 1 */
    volatile int shutdown_requested;
} control_state_t;

static control_state_t g = {
    .lt = 0,
    .rt = 0,
    .lsx = 0,
    .rsx = 0,
    .lb = 0,
    .rb = 0,
    .dpx = 0,
    .shutdown_requested = 0
};
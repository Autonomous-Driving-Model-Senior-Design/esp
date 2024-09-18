#include "driver/ledc.h" 
#include "esp_err.h"
#include "pwm.h"

void initPWM() {
    // Configure PWM timer
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    // Configure PWM channel
    ledc_channel_config_t pwm_channel = {
        .gpio_num = PWM_PIN,
        .speed_mode = LEDC_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);

    // Inital values
    ledc_set_duty(LEDC_MODE, PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, PWM_CHANNEL);
}
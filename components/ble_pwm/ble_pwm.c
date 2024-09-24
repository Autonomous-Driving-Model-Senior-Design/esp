#include <stdio.h>
#include "ble_pwm.h"
#include "driver/gpio.h"
#include "driver/ledc.h" 
#include "hal/ledc_types.h"

void setServoAngle(int angle)
{
	if (angle < 0) {
		angle = 0;
	}
	else if (angle > 180) {
		angle = 180;
	}

	int pulseWidth = MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle) / 180;

	uint32_t duty_max = (1 << PWM_RESOLUTION) - 1;
	uint32_t period_us = 1000000 / PWM_FREQUENCY;
	uint32_t duty = (pulseWidth * duty_max) / period_us;

	ledc_set_duty(LEDC_MODE, PWM_CHANNEL1, duty);
	ledc_update_duty(LEDC_MODE, PWM_CHANNEL1);
}

void initPWM1() {
	ledc_timer_config_t pwm_timer = {
		.speed_mode = LEDC_MODE,
		.duty_resolution = PWM_RESOLUTION,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = PWM_FREQUENCY,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwm_timer);

	ledc_channel_config_t pwm_channel = {
		.gpio_num = LED_PIN1,               
		.speed_mode = LEDC_MODE,
		.channel = PWM_CHANNEL1,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0
	};
	ledc_channel_config(&pwm_channel);
	ledc_set_duty(LEDC_MODE, PWM_CHANNEL1, 0);
	ledc_update_duty(LEDC_MODE, PWM_CHANNEL1);
}

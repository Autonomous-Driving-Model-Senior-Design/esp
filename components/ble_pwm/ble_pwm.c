#include <stdio.h>
#include "ble_pwm.h"
#include "driver/gpio.h"
#include "driver/ledc.h" 
#include "hal/ledc_types.h"
#include "esp_log.h"
#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"

void setServoAngle(long angle)
{
	if (angle >= 40 && angle <= 120) {
		ESP_LOGI("UART TEST", "Angle: %ld", angle);
		
		int pulseWidth = MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle) / 180;

		uint32_t duty_max = (1 << PWM_RESOLUTION) - 1;
		uint32_t period_us = 1000000 / PWM_FREQUENCY;
		uint32_t duty = (pulseWidth * duty_max) / period_us;
		ESP_LOGI("UART TEST", "Duty: %d", duty);

		ledc_set_duty(LEDC_MODE, PWM_CHANNEL1, duty);
		ledc_update_duty(LEDC_MODE, PWM_CHANNEL1);
	}
}
void setHBridgePWM(long dutyCycle, long angle) {
	//  gpio_set_level(GPIO_NUM_20, 1);
	if(angle >= 40 && angle <= 120) {
		bool neg = false;
		if(dutyCycle > 100) {
			neg = true;
			dutyCycle -= 100;
		}
		int cycle = 8191 * dutyCycle / 100;
		ESP_LOGI("Motor", "cycle: %d", cycle);
		if(neg) {
			ledc_set_duty(LEDC_MODE, PWM_CHANNEL2, 0);
			ledc_set_duty(LEDC_MODE, PWM_CHANNEL3, cycle);
		}
		else {
			if(cycle < 60){
				cycle += 60;
			}
			ledc_set_duty(LEDC_MODE, PWM_CHANNEL2, cycle - 60);
			ledc_set_duty(LEDC_MODE, PWM_CHANNEL3, 0);
		}
		ledc_update_duty(LEDC_MODE, PWM_CHANNEL2);
		ledc_update_duty(LEDC_MODE, PWM_CHANNEL3);
	}
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

void initPWM2and3() {
	ledc_timer_config_t pwm_timer = {
		.speed_mode = LEDC_MODE,
		.duty_resolution = PWM_RESOLUTION,
		.timer_num = LEDC_TIMER_1,
		.freq_hz = HBRIDGE_PWM_FREQUENCY,
		.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwm_timer);

	ledc_channel_config_t pwm_channel1 = {
		.gpio_num = LED_PIN2,
		.speed_mode = LEDC_MODE,
		.channel = PWM_CHANNEL2,
		.timer_sel = LEDC_TIMER_1,
		.duty = 0,
		.hpoint = 0
	};

	ledc_channel_config_t pwm_channel2 = {
		.gpio_num = LED_PIN3,
		.speed_mode = LEDC_MODE,
		.channel = PWM_CHANNEL3,
		.timer_sel = LEDC_TIMER_1,
		.duty = 0,
		.hpoint = 0
	};
	ledc_channel_config(&pwm_channel1);
	ledc_set_duty(LEDC_MODE, PWM_CHANNEL2, 0);
	ledc_update_duty(LEDC_MODE, PWM_CHANNEL2);

	ledc_channel_config(&pwm_channel2);
	ledc_set_duty(LEDC_MODE, PWM_CHANNEL3, 0);
	ledc_update_duty(LEDC_MODE, PWM_CHANNEL3);
}
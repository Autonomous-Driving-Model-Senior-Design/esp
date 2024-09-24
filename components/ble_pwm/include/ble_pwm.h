#ifndef BLE_PWM_H_
#define BLE_PWM_H_

#include "driver/gpio.h"

#define LED_PIN1 GPIO_NUM_6

#define PWM_CHANNEL1 LEDC_CHANNEL_2
#define PWM_FREQUENCY 50 

#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500

void setServoAngle(int);
void initPWM1(void);

#endif

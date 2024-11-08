#ifndef BLE_PWM_H_
#define BLE_PWM_H_

#include "driver/gpio.h"

#define LED_PIN1 GPIO_NUM_13
#define LED_PIN2 GPIO_NUM_10
#define LED_PIN3 GPIO_NUM_9

#define PWM_CHANNEL1 LEDC_CHANNEL_2
#define PWM_CHANNEL2 LEDC_CHANNEL_1
#define PWM_CHANNEL3 LEDC_CHANNEL_0
#define PWM_FREQUENCY 50 
#define HBRIDGE_PWM_FREQUENCY 100 

#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500

void setServoAngle(long);
void setHBridgePWM(long, long);
void initPWM1(void);
void initPWM2and3(void);

#endif

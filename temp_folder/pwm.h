#ifndef PWM_H
#define PWM_H

#define PWM_PIN GPIO_NUM_6 // Pin 6
#define PWM_CHANNEL LEDC_CHANNEL_1
#define PWM_RESOLUTION LEDC_TIMER_8_BIT // 0 - 255
#define PWM_FREQUENCY 1000 // Set PWM frequency to 5kHz
#define LEDC_MODE LEDC_LOW_SPEED_MODE // S3 only has low speed

void initPWM();

#endif
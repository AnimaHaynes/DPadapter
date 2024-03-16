#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>
#include <stm32f7xx.h>

#define PWM_SWITCHING_FREQUENCY 16000

int pwm_init();

int pwm_set_frequency(int freq);
int pwm_set_duty(float duty1, float duty2, float duty3);
int pwm_start();
int pwm_stop();

#endif /* PWM_DRIVER_H */

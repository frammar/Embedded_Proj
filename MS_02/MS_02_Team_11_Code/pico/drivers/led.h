#ifndef LED_H
#define LED_H

#include "pico/stdlib.h"

// Plain on/off
void led_set(uint pin, bool on);

// Brightness 0..100 percent (PWM)
void led_pwm(uint pin, uint8_t pct);

#endif
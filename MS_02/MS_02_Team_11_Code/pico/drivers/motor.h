#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"

#define MOTOR_ENA_PIN 6
#define MOTOR_IN1_PIN 7
#define MOTOR_IN2_PIN 8
#define MOTOR_MAX_PCT 85   // 9 V supply, 6 V motor: never go above this

void motor_init(void);
void motor_forward(uint8_t pct);    // 0..100, clamped to MOTOR_MAX_PCT
void motor_backward(uint8_t pct);
void motor_stop(void);

#endif
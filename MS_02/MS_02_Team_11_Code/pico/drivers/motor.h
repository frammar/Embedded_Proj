#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "pins.h"

#define MOTOR_MAX_PCT 85

void motor_init(void);
void motor_forward(uint8_t pct);
void motor_backward(uint8_t pct);
void motor_stop(void);    // coast
void motor_brake(void);   // sharp stop (both inputs high)

#endif
#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"
#include "pins.h"

void servo_init(uint pin);                          // 50 Hz, no pulse yet (servo stays limp)
void servo_set_pulse_us(uint pin, uint16_t us);     // 400..2600
void servo_set_angle(uint pin, uint8_t deg);        // 0..180
void servo_off(uint pin);                           // stop sending pulses

#endif  
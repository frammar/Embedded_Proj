#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"

#define US_TRIG_PIN 2
#define US_ECHO_PIN 3

void  us_init(void);
float us_read_cm(void);   // median of 5 readings; returns -1.0 if the sensor gives no valid echo

#endif
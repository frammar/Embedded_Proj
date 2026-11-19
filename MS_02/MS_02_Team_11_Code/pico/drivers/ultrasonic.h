#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include "pins.h"

void  us_init(void);
float us_read_cm(void);   // median of 5; -1.0 if no valid echo

#endif
#ifndef BUZZER_H
#define BUZZER_H

#include "pico/stdlib.h"
#include "pins.h"

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_pattern_win(void);     // 3 short beeps + 1 long, never blocks
void buzzer_pattern_error(void);   // 1 long beep, never blocks

#endif
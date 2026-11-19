#ifndef GATE_H
#define GATE_H

#include "pico/stdlib.h"
#include "pins.h"

// ---- EDIT THESE with the angles you measured ----
#define GATE_L_CLOSED_DEG   90   // drop latch (GP14), closed
#define GATE_L_OPEN_DEG      0   // drop latch (GP14), open
#define LATCH_LOCKED_DEG    90   // reset servo (GP18), normal state
#define LATCH_RELEASED_DEG  45   // reset servo (GP18), open
#define DROP_OPEN_MS      5000
// --------------------------------------------------

void gate_init(void);      // all servos ready, gate closed, latch locked
void gate_open(void);      // both flaps (R starts 50 ms after L)
void gate_close(void);
void latch_release(void);  // floor can drop
void latch_lock(void);

#endif
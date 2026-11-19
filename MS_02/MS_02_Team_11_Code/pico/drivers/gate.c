#include "gate.h"
#include "servo.h"

void gate_open(void)  { servo_set_angle(SERVO_DROP_L_PIN, GATE_L_OPEN_DEG); }
void gate_close(void) { servo_set_angle(SERVO_DROP_L_PIN, GATE_L_CLOSED_DEG); }

void latch_release(void) { servo_set_angle(SERVO_RESET_PIN, LATCH_RELEASED_DEG); }
void latch_lock(void)    { servo_set_angle(SERVO_RESET_PIN, LATCH_LOCKED_DEG); }

void gate_init(void) {
    servo_init(SERVO_DROP_L_PIN);
    servo_init(SERVO_RESET_PIN);
    gate_close();
    sleep_ms(300);          // never move the latch at the same moment as the gate
    latch_lock();
}
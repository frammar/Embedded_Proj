#include "pico/stdlib.h"
#include "motor.h"

int main() {
    motor_init();
    sleep_ms(3000);                     // time to get your hands ready

    motor_forward(40);  sleep_ms(2000); // slow
    motor_forward(85);  sleep_ms(2000); // fast
    motor_stop();       sleep_ms(1000); // stop
    motor_backward(40); sleep_ms(2000); // reverse, slow
    motor_stop();

    while (true) { sleep_ms(1000); }    // done: do nothing
}
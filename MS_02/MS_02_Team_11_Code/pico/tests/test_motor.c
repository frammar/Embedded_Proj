#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"

int main() {
    stdio_init_all();
    motor_init();
    for (int i = 0; i < 100 && !stdio_usb_connected(); i++) sleep_ms(100);
    sleep_ms(2000);

    // Phase 1: ramp up. Note the FIRST percentage where the motor really spins.
    for (int pct = 10; pct <= 85; pct += 5) {
        printf("forward %d%%\n", pct);
        motor_forward(pct);
        sleep_ms(1500);
    }
    motor_stop(); sleep_ms(2000);

    // Phase 2: the Milestone 2 sequence
    printf("slow\n");   motor_forward(40);  sleep_ms(2000);
    printf("fast\n");   motor_forward(85);  sleep_ms(2000);
    printf("stop\n");   motor_stop();       sleep_ms(2000);
    printf("fast, then brake\n");
    motor_forward(85); sleep_ms(1000);
    motor_brake();     sleep_ms(1500);
    motor_stop();
    printf("reverse\n"); motor_backward(40); sleep_ms(2000);
    motor_stop();
    printf("done\n");

    while (true) sleep_ms(1000);
}
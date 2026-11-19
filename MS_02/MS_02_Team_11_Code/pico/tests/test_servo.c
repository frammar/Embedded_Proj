#include <stdio.h>
#include "pico/stdlib.h"
#include "servo.h"

static bool valid_pin(unsigned p) {
    return p == SERVO_DROP_L_PIN || p == SERVO_DROP_R_PIN || p == SERVO_RESET_PIN;
}

int main() {
    stdio_init_all();
    servo_init(SERVO_DROP_L_PIN);
    servo_init(SERVO_DROP_R_PIN);
    servo_init(SERVO_RESET_PIN);

    while (!stdio_usb_connected()) sleep_ms(100);
    printf("\nType:  <pin> <angle>     e.g.  14 90   (angle 0..180)\n");
    printf("   or  <pin> <pulse_us>  e.g.  14 1500 (value above 180 = microseconds)\n");
    printf("Pins: 14 = drop L, 15 = drop R, 18 = reset. Move ONE servo at a time.\n");

    char line[32];
    while (true) {
        printf("> ");
        if (!fgets(line, sizeof line, stdin)) continue;

        unsigned pin, val;
        if (sscanf(line, "%u %u", &pin, &val) != 2 || !valid_pin(pin)) {
            printf("bad input\n");
            continue;
        }
        if (val <= 180) { servo_set_angle(pin, val);      printf("pin %u -> %u deg\n", pin, val); }
        else            { servo_set_pulse_us(pin, val);   printf("pin %u -> %u us\n",  pin, val); }
    }
}
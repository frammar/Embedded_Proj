#include "pico/stdlib.h"
#include "led.h"

#define TEST_PIN 25   // on-board LED

int main() {
    while (true) {
        // fade up, then fade down
        for (int p = 0; p <= 100; p += 2) { led_pwm(TEST_PIN, p); sleep_ms(20); }
        for (int p = 100; p >= 0; p -= 2) { led_pwm(TEST_PIN, p); sleep_ms(20); }

        // then blink 3 times with led_set
        for (int i = 0; i < 3; i++) {
            led_set(TEST_PIN, true);  sleep_ms(200);
            led_set(TEST_PIN, false); sleep_ms(200);
        }
    }
}
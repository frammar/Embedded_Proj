#include "pico/stdlib.h"
#include "buzzer.h"
#include "led.h"

int main() {
    buzzer_init();
    sleep_ms(2000);

    while (true) {
        buzzer_pattern_win();
        for (int i = 0; i < 12; i++) { led_set(LED_ONBOARD_PIN, i % 2 == 0); sleep_ms(250); }

        buzzer_pattern_error();
        for (int i = 0; i < 12; i++) { led_set(LED_ONBOARD_PIN, i % 2 == 0); sleep_ms(250); }
    }
}
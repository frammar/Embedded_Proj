#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#define PIN 14   // physical pin 19

int main(void) {
    stdio_init_all();
    gpio_set_function(PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN);
    pwm_set_clkdiv(slice, 125.0f);   // 125 MHz / 125 = 1 MHz, so 1 tick = 1 us
    pwm_set_wrap(slice, 19999);      // 20 ms period = 50 Hz
    pwm_set_enabled(slice, true);

    while (true) {
        printf("1000 us\n"); pwm_set_gpio_level(PIN, 1000); sleep_ms(1500);
        printf("1500 us\n"); pwm_set_gpio_level(PIN, 1500); sleep_ms(1500);
        printf("2000 us\n"); pwm_set_gpio_level(PIN, 2000); sleep_ms(1500);
    }
}
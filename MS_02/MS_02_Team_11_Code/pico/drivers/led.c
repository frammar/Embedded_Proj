#include "led.h"
#include "hardware/pwm.h"

void led_set(uint pin, bool on) {
    gpio_set_function(pin, GPIO_FUNC_SIO);  // back to plain on/off mode
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, on);
}

void led_pwm(uint pin, uint8_t pct) {
    if (pct > 100) pct = 100;

    gpio_set_function(pin, GPIO_FUNC_PWM);   // let the PWM hardware drive the pin
    uint slice = pwm_gpio_to_slice_num(pin);
    uint chan  = pwm_gpio_to_channel(pin);

    pwm_set_clkdiv(slice, 125.0f);           // 125 MHz / 125 = 1 MHz tick
    pwm_set_wrap(slice, 99);                 // 100 ticks per cycle = 10 kHz
    pwm_set_chan_level(slice, chan, pct);    // "on" for pct ticks out of 100
    pwm_set_enabled(slice, true);
}
#include "servo.h"
#include "hardware/pwm.h"

#define SERVO_MIN_US 500     // 0 deg   (we will narrow these after calibration)
#define SERVO_MAX_US 2500    // 180 deg

static void set_level(uint pin, uint16_t us) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(pin), pwm_gpio_to_channel(pin), us);
}

void servo_init(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, 125.0f);   // 1 tick = 1 us
    pwm_set_wrap(slice, 19999);      // 20 000 us = 50 Hz
    set_level(pin, 0);
    pwm_set_enabled(slice, true);
}

void servo_set_pulse_us(uint pin, uint16_t us) {
    if (us < 400)  us = 400;
    if (us > 2600) us = 2600;
    set_level(pin, us);
}

void servo_set_angle(uint pin, uint8_t deg) {
    if (deg > 180) deg = 180;
    servo_set_pulse_us(pin, SERVO_MIN_US + (uint32_t)deg * (SERVO_MAX_US - SERVO_MIN_US) / 180);
}

void servo_off(uint pin) { set_level(pin, 0); }
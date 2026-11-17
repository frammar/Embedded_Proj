#include "motor.h"
#include "hardware/pwm.h"

static void set_speed(uint8_t pct) {
    if (pct > MOTOR_MAX_PCT) pct = MOTOR_MAX_PCT;      // safety clamp
    uint slice = pwm_gpio_to_slice_num(MOTOR_ENA_PIN);
    uint chan  = pwm_gpio_to_channel(MOTOR_ENA_PIN);
    pwm_set_chan_level(slice, chan, pct * 10);         // wrap is 999, so 1 percent = 10 ticks
}

void motor_init(void) {
    gpio_init(MOTOR_IN1_PIN);
    gpio_set_dir(MOTOR_IN1_PIN, GPIO_OUT);
    gpio_put(MOTOR_IN1_PIN, 0);

    gpio_init(MOTOR_IN2_PIN);
    gpio_set_dir(MOTOR_IN2_PIN, GPIO_OUT);
    gpio_put(MOTOR_IN2_PIN, 0);

    gpio_set_function(MOTOR_ENA_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_ENA_PIN);
    pwm_set_clkdiv(slice, 125.0f);     // 1 MHz tick
    pwm_set_wrap(slice, 999);          // 1000 ticks = 1 kHz, good for a DC motor
    pwm_set_enabled(slice, true);
    set_speed(0);
}

void motor_forward(uint8_t pct) {
    gpio_put(MOTOR_IN1_PIN, 1);
    gpio_put(MOTOR_IN2_PIN, 0);
    set_speed(pct);
}

void motor_backward(uint8_t pct) {
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 1);
    set_speed(pct);
}

void motor_stop(void) {
    set_speed(0);
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 0);
}
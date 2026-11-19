#include "motor.h"
#include "hardware/pwm.h"

static void set_level(uint16_t level) {            // raw 0..1000 (wrap = 999)
    pwm_set_chan_level(pwm_gpio_to_slice_num(MOTOR_ENA_PIN),
                       pwm_gpio_to_channel(MOTOR_ENA_PIN), level);
}

static void set_speed(uint8_t pct) {
    if (pct > MOTOR_MAX_PCT) pct = MOTOR_MAX_PCT;  // safety clamp
    set_level(pct * 10);
}

void motor_init(void) {
    gpio_init(MOTOR_IN1_PIN); gpio_set_dir(MOTOR_IN1_PIN, GPIO_OUT); gpio_put(MOTOR_IN1_PIN, 0);
    gpio_init(MOTOR_IN2_PIN); gpio_set_dir(MOTOR_IN2_PIN, GPIO_OUT); gpio_put(MOTOR_IN2_PIN, 0);

    gpio_set_function(MOTOR_ENA_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_ENA_PIN);
    pwm_set_clkdiv(slice, 125.0f);   // 1 MHz tick
    pwm_set_wrap(slice, 999);        // 1 kHz
    pwm_set_enabled(slice, true);
    set_level(0);
}

void motor_forward(uint8_t pct)  { gpio_put(MOTOR_IN1_PIN, 1); gpio_put(MOTOR_IN2_PIN, 0); set_speed(pct); }
void motor_backward(uint8_t pct) { gpio_put(MOTOR_IN1_PIN, 0); gpio_put(MOTOR_IN2_PIN, 1); set_speed(pct); }

void motor_stop(void) {
    set_level(0);
    gpio_put(MOTOR_IN1_PIN, 0);
    gpio_put(MOTOR_IN2_PIN, 0);
}

void motor_brake(void) {            // outputs enabled and equal: motor terminals shorted
    gpio_put(MOTOR_IN1_PIN, 1);
    gpio_put(MOTOR_IN2_PIN, 1);
    set_level(1000);
}
#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "led.h"

#define NEAR_CM   5.0f
#define FAR_CM   40.0f

int main() {
    stdio_init_all();
    us_init();

    while (true) {
        float d = us_read_cm();
        if (d < 0) {
            printf("no echo\n");
            led_set(LED_DIST_PIN, false);
            led_set(LED_STATUS_PIN, false);
        } else {
            float t = (FAR_CM - d) / (FAR_CM - NEAR_CM);    // 1 = near, 0 = far
            if (t < 0) t = 0;
            if (t > 1) t = 1;
            led_pwm(LED_DIST_PIN, (uint8_t)(t * 100));      // closer = brighter
            led_set(LED_STATUS_PIN, true);                  // lit = valid echo
            printf("%.1f cm\n", d);
        }
        sleep_ms(50);
    }
}
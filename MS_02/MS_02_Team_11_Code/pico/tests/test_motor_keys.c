#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "ultrasonic.h"

// ---------------- settings ----------------
#define USE_SAFETY            1
#define NEAR_LIMIT_CM         3.7
#define FAR_LIMIT_CM          9.7
#define W_INCREASES_DISTANCE  1  // never go farther from the sensor than this
#define MOVE_MS              80    // how long one key press moves
#define KICK_MS              40      // short full-power push to start moving (0 = off)
#define SPEED_MIN            10
// ------------------------------------------

static int speed = 50;   // percent

static void burst(bool forward) {
#if USE_SAFETY
    float d = us_read_cm();
    if (d < 0) { printf("REFUSED: no distance reading\n"); return; }

    bool away = (forward == (W_INCREASES_DISTANCE != 0));
    if (away  && d >= FAR_LIMIT_CM)  { printf("REFUSED: at far limit (%.1f cm)\n", d);  return; }
    if (!away && d <= NEAR_LIMIT_CM) { printf("REFUSED: at near limit (%.1f cm)\n", d); return; }
    printf("before: %.1f cm\n", d);
#endif

    if (KICK_MS > 0) {                       // short strong push to overcome friction
        if (forward) motor_forward(MOTOR_MAX_PCT); else motor_backward(MOTOR_MAX_PCT);
        sleep_ms(KICK_MS);
    }
    if (forward) motor_forward(speed); else motor_backward(speed);
    sleep_ms(MOVE_MS - KICK_MS);

    motor_brake();                           // sharp stop
    sleep_ms(100);
    motor_stop();

#if USE_SAFETY
    printf("after:  %.1f cm\n", us_read_cm());
#endif
}

int main() {
    stdio_init_all();
    motor_init();
    us_init();
    while (!stdio_usb_connected()) sleep_ms(100);

    printf("\nw = forward   s = backward   (each press moves %d ms)\n", MOVE_MS);
    printf("+ / - = speed   b = brake   p = print distance\n");
    printf("speed = %d%%\n", speed);

    while (true) {
        int ch = getchar();
        switch (ch) {
            case 'w': burst(true);  break;
            case 's': burst(false); break;
            case '+': if (speed + 5 <= MOTOR_MAX_PCT) speed += 5; printf("speed = %d%%\n", speed); break;
            case '-': if (speed - 5 >= SPEED_MIN)     speed -= 5; printf("speed = %d%%\n", speed); break;
            case 'b': motor_brake(); sleep_ms(300); motor_stop(); printf("braked\n"); break;
            case 'p': { float d = us_read_cm(); printf("%.1f cm\n", d); break; }
            default: break;   // ignore Enter and other keys
        }
    }
}
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "ultrasonic.h"
#include "gate.h"

// ---------- TUNE THESE ----------
#define SPEED_PCT             30      // cruise duty (your 50 %)
#define KICK_PCT              50      // short start push so the motor overcomes friction
#define KICK_MS               40
#define STEP_FINE_MS          60      // fine pulse (about 0.2-0.3 cm, tune from your test)
#define STEP_COARSE_MS        200     // coarse pulse, used far from the target
#define SETTLE_MS             120     // wait after each pulse before measuring
#define TOL_CM                0.3f    // "arrived" tolerance; must be >= half a fine step
#define COARSE_ABOVE_CM       1.5f    // use coarse pulses when farther than this
#define NEAR_LIMIT_CM         2.1f
#define FAR_LIMIT_CM          9.7f
#define W_INCREASES_DISTANCE  1       // w (forward, moves right) raises the distance
#define AGREE_CM              0.4f    // readings must agree within this
#define AGREE_COUNT           3       // ... this many times in a row
#define GOTO_TIMEOUT_MS       10000
#define NUM_SLOTS             3
// --------------------------------

typedef enum { DIR_FORWARD, DIR_BACKWARD } dir_t;   // forward = key w
#if W_INCREASES_DISTANCE
  #define DIR_INC DIR_FORWARD
  #define DIR_DEC DIR_BACKWARD
#else
  #define DIR_INC DIR_BACKWARD
  #define DIR_DEC DIR_FORWARD
#endif

static float slot_cm[NUM_SLOTS];
static bool  slot_set[NUM_SLOTS];

// Servo state (gate_init() closes the gate and locks the latch at start-up)
static bool gate_is_open  = false;
static bool latch_is_open = false;

// One short move: kick, cruise, brake, wait for the trolley to settle.
static void pulse(dir_t d, uint ms) {
    if (d == DIR_FORWARD) motor_forward(KICK_PCT); else motor_backward(KICK_PCT);
    sleep_ms(KICK_MS);
    if (d == DIR_FORWARD) motor_forward(SPEED_PCT); else motor_backward(SPEED_PCT);
    if (ms > KICK_MS) sleep_ms(ms - KICK_MS);
    motor_brake();
    sleep_ms(SETTLE_MS);
    motor_stop();
}

// Returns the average of AGREE_COUNT consecutive agreeing readings, or -1.
static float stable_cm(void) {
    float sum = 0, last = 0;
    int cnt = 0;
    for (int tries = 0; tries < 12; tries++) {
        float r = us_read_cm();
        if (r < 0) { cnt = 0; sum = 0; continue; }           // invalid, restart
        if (cnt > 0 && fabsf(r - last) > AGREE_CM) { cnt = 0; sum = 0; }  // outlier, restart
        sum += r; cnt++; last = r;
        if (cnt >= AGREE_COUNT) return sum / cnt;
    }
    return -1.0f;
}

// Jog one pulse, refusing to go past the limits or when blind.
static void jog(dir_t dir, uint ms) {
    float d = stable_cm();
    if (d < 0) { printf("refused: no stable reading\n"); return; }
    if (dir == DIR_INC && d >= FAR_LIMIT_CM)  { printf("refused: far limit (%.2f)\n", d); return; }
    if (dir == DIR_DEC && d <= NEAR_LIMIT_CM) { printf("refused: near limit (%.2f)\n", d); return; }
    pulse(dir, ms);
    float after = stable_cm();
    if (after < 0) printf("moved, no stable reading\n");
    else           printf("distance %.2f cm\n", after);
}

// Drive to target_cm using the sensor as feedback.
static bool goto_cm(float target) {
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)GOTO_TIMEOUT_MS * 1000) {
        float d = stable_cm();
        if (d < 0) { printf("ABORT: no stable reading\n"); return false; }
        float err = target - d;
        if (fabsf(err) <= TOL_CM) { printf("arrived at %.2f cm (target %.2f)\n", d, target); return true; }
        dir_t dir = (err > 0) ? DIR_INC : DIR_DEC;
        if (dir == DIR_INC && d >= FAR_LIMIT_CM)  { printf("ABORT: far limit\n");  return false; }
        if (dir == DIR_DEC && d <= NEAR_LIMIT_CM) { printf("ABORT: near limit\n"); return false; }
        pulse(dir, fabsf(err) > COARSE_ABOVE_CM ? STEP_COARSE_MS : STEP_FINE_MS);
    }
    printf("ABORT: timeout\n");
    return false;
}

static bool all_saved(void) {
    for (int i = 0; i < NUM_SLOTS; i++) if (!slot_set[i]) return false;
    return true;
}

// ---------- servo controls ----------
static void gate_toggle(void) {
    if (!gate_is_open) { gate_open();  gate_is_open = true;  printf("gate OPEN\n"); }
    else               { gate_close(); gate_is_open = false; printf("gate CLOSED\n"); }
}

static void latch_toggle(void) {
    if (!latch_is_open) { latch_release(); latch_is_open = true;  printf("latch RELEASED (floor drops)\n"); }
    else                { latch_lock();    latch_is_open = false; printf("latch LOCKED\n"); }
}

// Official drop: gate open for DROP_OPEN_MS (5 s), then closes by itself.
static void drop_now(void) {
    if (gate_is_open) { printf("refused: gate is already open\n"); return; }
    printf("DROP: gate open for %d ms\n", DROP_OPEN_MS);
    gate_open();  gate_is_open = true;
    sleep_ms(DROP_OPEN_MS);
    gate_close(); gate_is_open = false;
    printf("gate closed\n");
}

static void print_help(bool calib) {
    if (calib) {
        printf("\n=== CALIBRATION ===\n");
        printf("w/s = fine jog right/left   W/S = coarse jog\n");
        printf("1/2/3 = save current distance as slot 1/2/3\n");
        printf("p = show distance   x = finish calibration (needs all 3)\n");
    } else {
        printf("\n=== RUN ===\n");
        printf("1/2/3 = go to slot   p = show distance   c = recalibrate\n");
    }
    printf("g = gate toggle   d = timed drop   r = reset latch toggle\n");
}

int main(void) {
    stdio_init_all();
    motor_init();
    us_init();
    gate_init();     // closes the gate, then locks the latch
    while (!stdio_usb_connected()) sleep_ms(100);

    bool calib = true;
    print_help(calib);

    while (true) {
        int ch = getchar();
        if (ch == 'p') {
            float d = stable_cm();
            if (d < 0) printf("no stable reading\n"); else printf("distance %.2f cm\n", d);
            continue;
        }
        // servo keys work in both modes
        if (ch == 'g') { gate_toggle();  continue; }
        if (ch == 'r') { latch_toggle(); continue; }
        if (ch == 'd') { drop_now();     continue; }

        if (calib) {
            switch (ch) {
                case 'w': jog(DIR_FORWARD,  STEP_FINE_MS);   break;
                case 's': jog(DIR_BACKWARD, STEP_FINE_MS);   break;
                case 'W': jog(DIR_FORWARD,  STEP_COARSE_MS); break;
                case 'S': jog(DIR_BACKWARD, STEP_COARSE_MS); break;
                case '1': case '2': case '3': {
                    int i = ch - '1';
                    float d = stable_cm();
                    if (d < 0) { printf("not saved: no stable reading\n"); break; }
                    slot_cm[i] = d; slot_set[i] = true;
                    printf("slot %d saved: %.2f cm\n", i + 1, d);
                    break;
                }
                case 'x':
                    if (all_saved()) { calib = false; print_help(calib);
                        printf("slots: %.2f / %.2f / %.2f cm\n", slot_cm[0], slot_cm[1], slot_cm[2]); }
                    else printf("save all 3 slots first\n");
                    break;
                default: break;   // ignore Enter and other keys
            }
        } else {
            switch (ch) {
                case '1': case '2': case '3': {
                    int i = ch - '1';
                    printf("going to slot %d (%.2f cm)\n", i + 1, slot_cm[i]);
                    goto_cm(slot_cm[i]);
                    break;
                }
                case 'c': calib = true; print_help(calib); break;
                default: break;
            }
        }
    }
}
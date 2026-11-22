// Factory 4 - Project 4: Surgical Image Reconstruction. Pico firmware (one UF2).
//
// The Pico owns the machine: DC motor, ultrasonic positioning, drop gate, reset latch, buzzer,
// the three column counters and the state machine. The ESP32-CAM hosts the web page and the
// computer-vision check and talks to the Pico over a text link (see link.c).
//
// Lines sent by the ESP:   PING | START | SEL n | DROP | CONFIRM | OVER | STOP | CAL | CALDONE |
//                          JOG f|b|F|B | SAVE n | GATE | LATCH | SLOTS a b c | WIN | LOSE ... | ERR ...
// Lines sent by this code: BOOT | PONG | OK | ERR code | ST ... (status, about 4 per second) | CHECK

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "ultrasonic.h"
#include "gate.h"
#include "link.h"

// ------------------------------------------------------------------ TUNE (copy your tuned numbers here)
#define SPEED_PCT             30
#define KICK_PCT              50
#define KICK_MS               40
#define STEP_FINE_MS          60
#define STEP_COARSE_MS        200
#define SETTLE_MS             120
#define TOL_CM                0.3f
#define COARSE_ABOVE_CM       1.5f
#define W_INCREASES_DISTANCE  1       // forward (moves right) raises the distance
#define AGREE_CM              0.4f
#define AGREE_COUNT           2
#define GOTO_TIMEOUT_MS       20000
#define NUM_SLOTS             3
#define LIMIT_MARGIN_CM       1.0f
#define ABS_MIN_CM            2.5f
#define ABS_MAX_CM            60.0f
#define NO_MOVE_WARN_CM       0.1f
#define ALLOW_STILL_CM        0.6f
#define ALLOW_AFTER_FINE_CM   1.5f
#define ALLOW_AFTER_COARSE_CM 5.0f
#define JUMP_RETRIES          3

#define BUZZER_PIN            16      // active buzzer through a transistor, high = beep
#define RESET_OPEN_MS         3000    // how long the reset latch stays open so all 9 pieces fall out
#define STATUS_PERIOD_MS      250
#define CV_TIMEOUT_MS         9000    // how long to wait for the ESP's image check
#define PIECES_PER_COLUMN     3
// ------------------------------------------------------------------------------------------------

typedef enum { DIR_FORWARD, DIR_BACKWARD } dir_t;
#if W_INCREASES_DISTANCE
  #define DIR_INC DIR_FORWARD
  #define DIR_DEC DIR_BACKWARD
#else
  #define DIR_INC DIR_BACKWARD
  #define DIR_DEC DIR_FORWARD
#endif

typedef enum {
    ST_IDLE, ST_CALIB, ST_READY, ST_MOVING, ST_ALIGNED,
    ST_DROPPING, ST_FULL, ST_CHECKING, ST_WIN, ST_RESETTING
} state_t;

static const char *const STATE_NAME[] = {
    "IDLE", "CALIB", "READY", "MOVING", "ALIGNED",
    "DROPPING", "FULL", "CHECKING", "WIN", "RESETTING"
};

// ------------------------------------------------------------------ machine state
static state_t st = ST_IDLE;
static int   cnt[NUM_SLOTS];                 // column counters C1..C3
static float slot_cm[NUM_SLOTS] = {-1, -1, -1};
static int   sel = 0;                        // selected column 1..3, 0 = none
static float target = -1.0f;                 // target distance of the selected column
static float g_dist = -1.0f;                 // latest distance reading
static char  err[16] = "-";                  // last error code shown to the user
static char  cv[24]  = "-";                  // last image-check result (spaces replaced by _)
static bool  g_abort = false;
static bool  pending_over = false;          // Start Over pressed while moving/dropping: done when that action ends
static bool  gate_is_open = false, latch_is_open = false;

// jump filter
static float last_good = -1.0f;
static float allow_cm  = ALLOW_STILL_CM;

static bool all_saved(void) {
    for (int i = 0; i < NUM_SLOTS; i++) if (slot_cm[i] < 0) return false;
    return true;
}

static int total_drops(void) { return cnt[0] + cnt[1] + cnt[2]; }

// ------------------------------------------------------------------ link: status + command handling
static void send_status(void) {
    link_send("ST %s %d %d %d %.2f %.2f %d %.2f %.2f %.2f %s %s",
              STATE_NAME[st], cnt[0], cnt[1], cnt[2], g_dist, target, sel,
              slot_cm[0], slot_cm[1], slot_cm[2], err, cv);
}

static void reply_ok(void)             { link_send("OK"); }
static void reply_err(const char *c)   { link_send("ERR %s", c); snprintf(err, sizeof err, "%s", c); }

static void handle_cmd(const char *line);

// Called from every wait loop. busy = a long action is running, so commands are not started now.
static void app_poll(bool busy) {
    static absolute_time_t next_status;
    char line[96];
    while (link_read_line(line, sizeof line)) {
        if (!strncmp(line, "WIN", 3) || !strncmp(line, "LOSE", 4) || !strncmp(line, "ERR", 3)) continue; // late check reply
        if (!busy) { handle_cmd(line); continue; }
        if (!strcmp(line, "PING")) link_send("PONG");
        else if (!strcmp(line, "STOP")) { g_abort = true; link_send("OK"); }
        else if (!strcmp(line, "OVER") && (st == ST_MOVING || st == ST_DROPPING)) {
            pending_over = true; g_abort = true; link_send("OK");     // stop moving, then reset
        }
        else link_send("ERR BUSY");
    }
    if (time_reached(next_status)) {
        send_status();
        next_status = make_timeout_time_ms(STATUS_PERIOD_MS);
    }
}

static void wait_ms(uint ms) {
    absolute_time_t end = make_timeout_time_ms(ms);
    while (!time_reached(end)) { sleep_ms(10); app_poll(true); }
}

// ------------------------------------------------------------------ buzzer
static void buzz(bool on) { gpio_put(BUZZER_PIN, on); }
static void beep(uint on_ms, uint off_ms) { buzz(true); wait_ms(on_ms); buzz(false); if (off_ms) wait_ms(off_ms); }

static void success_jingle(void) {
    beep(120, 60); beep(120, 60); beep(120, 60);
    beep(380, 100); beep(120, 60); beep(600, 0);
}
static void failure_beeps(void) { beep(400, 150); beep(400, 150); beep(700, 0); }

// ------------------------------------------------------------------ positioning (from the tested test_positioning)
static void get_limits(float *lo, float *hi) {
    if (st == ST_CALIB || !all_saved()) { *lo = ABS_MIN_CM; *hi = ABS_MAX_CM; return; }
    float mn = slot_cm[0], mx = slot_cm[0];
    for (int i = 1; i < NUM_SLOTS; i++) {
        if (slot_cm[i] < mn) mn = slot_cm[i];
        if (slot_cm[i] > mx) mx = slot_cm[i];
    }
    *lo = mn - LIMIT_MARGIN_CM;
    *hi = mx + LIMIT_MARGIN_CM;
}

static void pulse(dir_t d, uint ms) {
    allow_cm = (ms >= STEP_COARSE_MS) ? ALLOW_AFTER_COARSE_CM : ALLOW_AFTER_FINE_CM;
    if (d == DIR_FORWARD) motor_forward(KICK_PCT); else motor_backward(KICK_PCT);
    sleep_ms(KICK_MS);
    if (d == DIR_FORWARD) motor_forward(SPEED_PCT); else motor_backward(SPEED_PCT);
    if (ms > KICK_MS) sleep_ms(ms - KICK_MS);
    motor_brake();
    sleep_ms(SETTLE_MS);
    motor_stop();
}

static float stable_raw(void) {
    float sum = 0, last = 0;
    int n = 0;
    for (int tries = 0; tries < 12; tries++) {
        float r = us_read_cm();
        if (r < 0) { n = 0; sum = 0; continue; }
        if (n > 0 && fabsf(r - last) > AGREE_CM) { n = 0; sum = 0; }
        sum += r; n++; last = r;
        if (n >= AGREE_COUNT) return sum / n;
    }
    return -1.0f;
}

static float stable_cm(void) {
    float cand = -1.0f;
    for (int a = 0; a < JUMP_RETRIES; a++) {
        cand = stable_raw();
        if (cand < 0) { g_dist = -1.0f; return -1.0f; }
        if (last_good < 0 || fabsf(cand - last_good) <= allow_cm) break;
        sleep_ms(100);
    }
    last_good = cand;
    allow_cm  = ALLOW_STILL_CM;
    g_dist = cand;
    return cand;
}

// One calibration jog. Returns false (and sets err) if refused.
static bool jog(dir_t dir, uint ms) {
    float lo, hi;
    get_limits(&lo, &hi);
    last_good = -1.0f;
    float d = stable_cm();
    if (d < 0)                          { reply_err("NOREAD");   return false; }
    if (dir == DIR_INC && d >= hi)      { reply_err("LIMIT_HI"); return false; }
    if (dir == DIR_DEC && d <= lo)      { reply_err("LIMIT_LO"); return false; }
    pulse(dir, ms);
    float after = stable_cm();
    if (after < 0)                      { reply_err("NOREAD");   return false; }
    if (fabsf(after - d) < NO_MOVE_WARN_CM) { reply_err("NOMOVE"); return false; }
    return true;
}

// Drive to target_cm with the sensor as feedback. The web page watches the distance meanwhile.
static bool goto_cm(float tgt) {
    float lo, hi;
    get_limits(&lo, &hi);
    last_good = -1.0f;
    g_abort = false;
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)GOTO_TIMEOUT_MS * 1000) {
        app_poll(true);
        if (g_abort) { motor_stop(); snprintf(err, sizeof err, "STOPPED"); return false; }
        float d = stable_cm();
        if (d < 0)  { motor_stop(); snprintf(err, sizeof err, "NOREAD");   return false; }
        float e = tgt - d;
        if (fabsf(e) <= TOL_CM) return true;
        dir_t dir = (e > 0) ? DIR_INC : DIR_DEC;
        if (dir == DIR_INC && d >= hi) { snprintf(err, sizeof err, "LIMIT_HI"); return false; }
        if (dir == DIR_DEC && d <= lo) { snprintf(err, sizeof err, "LIMIT_LO"); return false; }
        pulse(dir, fabsf(e) > COARSE_ABOVE_CM ? STEP_COARSE_MS : STEP_FINE_MS);
    }
    motor_stop();
    snprintf(err, sizeof err, "TIMEOUT");
    return false;
}

// ------------------------------------------------------------------ actions
static void do_reset_release(void) {          // opens the reset latch so all pieces fall back to the user
    st = ST_RESETTING;
    send_status();
    latch_release(); latch_is_open = true;
    wait_ms(RESET_OPEN_MS);
    latch_lock();    latch_is_open = false;
    wait_ms(300);
    cnt[0] = cnt[1] = cnt[2] = 0;
    sel = 0; target = -1.0f;
}

static void cmd_start(void) {
    if (st != ST_IDLE && st != ST_WIN)  { reply_err("STATE");  return; }
    if (!all_saved())                   { reply_err("NOCAL");  return; }
    cnt[0] = cnt[1] = cnt[2] = 0;
    sel = 0; target = -1.0f;
    snprintf(err, sizeof err, "-"); snprintf(cv, sizeof cv, "-");
    st = ST_READY;
    reply_ok();
}

static void cmd_sel(int n) {
    if (n < 1 || n > NUM_SLOTS)                 { reply_err("ARG");     return; }
    if (st != ST_READY && st != ST_ALIGNED)     { reply_err("STATE");   return; }
    if (cnt[n - 1] >= PIECES_PER_COLUMN)        { reply_err("COLFULL"); return; }
    if (!all_saved())                           { reply_err("NOCAL");   return; }
    snprintf(err, sizeof err, "-");
    sel = n; target = slot_cm[n - 1];
    st = ST_MOVING;
    send_status();
    if (goto_cm(target)) { st = ST_ALIGNED; reply_ok(); }
    else                 { st = ST_READY; sel = 0; target = -1.0f; link_send("ERR %s", err); }
}

static void cmd_drop(void) {
    if (st != ST_ALIGNED) { reply_err("STATE"); return; }
    snprintf(err, sizeof err, "-");
    st = ST_DROPPING;
    send_status();
    gate_open(); gate_is_open = true;
    wait_ms(DROP_OPEN_MS);
    gate_close(); gate_is_open = false;
    cnt[sel - 1]++;
    sel = 0; target = -1.0f;
    st = (total_drops() >= NUM_SLOTS * PIECES_PER_COLUMN) ? ST_FULL : ST_READY;
    reply_ok();
}

static bool wait_check_result(char *res, size_t n) {
    absolute_time_t end = make_timeout_time_ms(CV_TIMEOUT_MS);
    while (!time_reached(end)) {
        char line[96];
        if (link_read_line(line, sizeof line)) {
            if (!strncmp(line, "WIN", 3) || !strncmp(line, "LOSE", 4) || !strncmp(line, "ERR", 3)) {
                snprintf(res, n, "%s", line);
                return true;
            }
            if (!strcmp(line, "PING")) link_send("PONG"); else link_send("ERR BUSY");
        }
        sleep_ms(5);
    }
    return false;
}

static void set_cv(const char *s) {
    snprintf(cv, sizeof cv, "%s", s);
    for (char *p = cv; *p; p++) if (*p == ' ') *p = '_';
}

static void cmd_confirm(void) {
    if (st != ST_FULL) { reply_err("STATE"); return; }
    snprintf(err, sizeof err, "-");
    st = ST_CHECKING;
    send_status();
    char junk[96];
    while (link_read_line(junk, sizeof junk)) { }       // drop anything stale
    link_send("CHECK");
    char res[96];
    if (!wait_check_result(res, sizeof res)) {          // no answer: keep the pieces, let the user retry
        set_cv("ERR_timeout"); st = ST_FULL; reply_err("CVTIMEOUT"); return;
    }
    set_cv(res);
    if (!strncmp(res, "WIN", 3)) {                      // success sequence
        st = ST_WIN;
        reply_ok();
        success_jingle();
    } else if (!strncmp(res, "LOSE", 4)) {              // wrong image: keep the pieces, the user may fix them,
        reply_ok();                                     // confirm again, or press Start Over
        failure_beeps();
        st = ST_FULL;
    } else {                                            // camera fault: never a win and never a reset
        st = ST_FULL;
        reply_err("CVERR");
    }
}

static void do_over(void) {
    snprintf(err, sizeof err, "-"); snprintf(cv, sizeof cv, "-");
    do_reset_release();
    st = ST_READY;
}

static void cmd_over(void) {                            // allowed at any time except calibration, checking, resetting
    if (st != ST_FULL && st != ST_WIN && st != ST_READY && st != ST_ALIGNED) { reply_err("STATE"); return; }
    do_over();
    reply_ok();
}

static void cmd_cal(void) {
    if (st == ST_MOVING || st == ST_DROPPING || st == ST_CHECKING || st == ST_RESETTING) { reply_err("STATE"); return; }
    snprintf(err, sizeof err, "-");
    st = ST_CALIB;
    sel = 0; target = -1.0f;
    reply_ok();
}

static void cmd_caldone(void) {
    if (st != ST_CALIB) { reply_err("STATE"); return; }
    if (!all_saved())   { reply_err("NOCAL");  return; }
    st = ST_IDLE;
    reply_ok();
}

static void cmd_jog(const char *a) {
    if (st != ST_CALIB) { reply_err("STATE"); return; }
    snprintf(err, sizeof err, "-");
    bool ok = false;
    switch (a[0]) {
        case 'f': ok = jog(DIR_FORWARD,  STEP_FINE_MS);   break;
        case 'b': ok = jog(DIR_BACKWARD, STEP_FINE_MS);   break;
        case 'F': ok = jog(DIR_FORWARD,  STEP_COARSE_MS); break;
        case 'B': ok = jog(DIR_BACKWARD, STEP_COARSE_MS); break;
        default: reply_err("ARG"); return;
    }
    if (ok) reply_ok();
}

static void cmd_save(int n) {
    if (st != ST_CALIB)                 { reply_err("STATE"); return; }
    if (n < 1 || n > NUM_SLOTS)         { reply_err("ARG");   return; }
    last_good = -1.0f;
    float d = stable_cm();
    if (d < 0) { reply_err("NOREAD"); return; }
    slot_cm[n - 1] = d;
    snprintf(err, sizeof err, "-");
    reply_ok();
}

static void cmd_gate(void) {
    if (st != ST_CALIB) { reply_err("STATE"); return; }
    if (!gate_is_open) { gate_open();  gate_is_open = true;  }
    else               { gate_close(); gate_is_open = false; }
    reply_ok();
}

static void cmd_latch(void) {
    if (st != ST_CALIB) { reply_err("STATE"); return; }
    if (!latch_is_open) { latch_release(); latch_is_open = true;  }
    else                { latch_lock();    latch_is_open = false; }
    reply_ok();
}

static void cmd_slots(const char *a) {                  // stored calibration pushed by the ESP after a restart
    if (st != ST_IDLE && st != ST_CALIB) { reply_err("STATE"); return; }
    float v[NUM_SLOTS];
    if (sscanf(a, "%f %f %f", &v[0], &v[1], &v[2]) != 3) { reply_err("ARG"); return; }
    for (int i = 0; i < NUM_SLOTS; i++) if (v[i] > 0) slot_cm[i] = v[i];
    reply_ok();
}

static void handle_cmd(const char *line) {
    char c[16] = "", a[64] = "";
    sscanf(line, "%15s %63[^\n]", c, a);
    if      (!strcmp(c, "PING"))    link_send("PONG");
    else if (!strcmp(c, "START"))   cmd_start();
    else if (!strcmp(c, "SEL"))     cmd_sel(atoi(a));
    else if (!strcmp(c, "DROP"))    cmd_drop();
    else if (!strcmp(c, "CONFIRM")) cmd_confirm();
    else if (!strcmp(c, "OVER"))    cmd_over();
    else if (!strcmp(c, "STOP"))    { g_abort = true; motor_stop(); reply_ok(); }
    else if (!strcmp(c, "CAL"))     cmd_cal();
    else if (!strcmp(c, "CALDONE")) cmd_caldone();
    else if (!strcmp(c, "JOG"))     cmd_jog(a);
    else if (!strcmp(c, "SAVE"))    cmd_save(atoi(a));
    else if (!strcmp(c, "GATE"))    cmd_gate();
    else if (!strcmp(c, "LATCH"))   cmd_latch();
    else if (!strcmp(c, "SLOTS"))   cmd_slots(a);
    else                            reply_err("UNKNOWN");
    if (pending_over) { pending_over = false; do_over(); }
}

// ------------------------------------------------------------------ main
int main(void) {
    stdio_init_all();                 // USB serial for debugging; the program does NOT wait for a PC
    gpio_init(BUZZER_PIN); gpio_set_dir(BUZZER_PIN, GPIO_OUT); gpio_put(BUZZER_PIN, 0);
    motor_init();
    us_init();
    gate_init();                      // closes the gate, then locks the latch
    link_init();

    sleep_ms(500);
    link_send("BOOT");

    while (true) {
        app_poll(false);
        // keep the distance on the web page live while idle
        if (st != ST_MOVING) {
            float d = us_read_cm();
            g_dist = d;
        }
    }
}
h
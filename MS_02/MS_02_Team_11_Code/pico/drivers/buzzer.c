#include "buzzer.h"

// Pattern = alternating durations in ms: ON, OFF, ON, OFF, ... (odd length ends with ON)
static const uint16_t WIN_PATTERN[]   = {120, 100, 120, 100, 120, 100, 400};
static const uint16_t ERROR_PATTERN[] = {800};

static const uint16_t *pattern;
static int pattern_len, pattern_idx;
static alarm_id_t pattern_alarm = -1;

static int64_t step_cb(alarm_id_t id, void *user_data) {
    if (pattern_idx >= pattern_len) {            // finished
        gpio_put(BUZZER_PIN, 0);
        pattern_alarm = -1;
        return 0;
    }
    gpio_put(BUZZER_PIN, (pattern_idx % 2) == 0);     // even step = beep, odd = silence
    return -(int64_t)pattern[pattern_idx++] * 1000;   // call me again after this many us
}

static void stop_pattern(void) {
    if (pattern_alarm >= 0) cancel_alarm(pattern_alarm);
    pattern_alarm = -1;
}

static void play(const uint16_t *p, int n) {
    stop_pattern();
    pattern = p; pattern_len = n; pattern_idx = 0;
    pattern_alarm = add_alarm_in_ms(1, step_cb, NULL, true);
}

void buzzer_init(void) {
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);
}

void buzzer_on(void)  { stop_pattern(); gpio_put(BUZZER_PIN, 1); }
void buzzer_off(void) { stop_pattern(); gpio_put(BUZZER_PIN, 0); }

void buzzer_pattern_win(void)   { play(WIN_PATTERN,   sizeof(WIN_PATTERN)   / sizeof(WIN_PATTERN[0])); }
void buzzer_pattern_error(void) { play(ERROR_PATTERN, sizeof(ERROR_PATTERN) / sizeof(ERROR_PATTERN[0])); }
#include "ultrasonic.h"
#include <stdio.h>
#include <math.h>
#include "hardware/sync.h"

// ---------- TUNE THESE ----------
#define US_SAMPLES      5        // readings per us_read_cm()
#define US_PERIOD_US    60000    // minimum time between two pings (HC-SR04 needs >= 60 ms)
#define US_TRIG_US      20       // trigger pulse length (datasheet minimum is 10 us)
#define US_AGREE_CM     0.5f     // readings must be this close to the median to count
#define US_IRQ_GUARD    1        // 1 = block interrupts while timing the echo, 0 = off
#define US_DEBUG        0        // 1 = print every raw reading (set to 0 when it works)
// --------------------------------

#if US_IRQ_GUARD
  #define US_RISE_TIMEOUT_US  4000    // echo line must rise within 4 ms of the trigger
  #define US_MAX_ECHO_US      3500    // longest echo timed (about 60 cm); interrupts are off meanwhile
#else
  #define US_RISE_TIMEOUT_US  30000
  #define US_MAX_ECHO_US      30000   // about 5 m
#endif

#define US_WAIT_LOW_US  30000    // how long we wait for the echo line to be idle before pinging

// error codes returned by read_once() (all negative)
#define US_ERR_STUCK_HIGH  (-1.0f)   // echo line already high before the ping
#define US_ERR_NO_RISE     (-2.0f)   // echo line never went high (sensor not triggered or not powered)
#define US_ERR_TOO_LONG    (-3.0f)   // echo line stayed high too long (no object in range)

static absolute_time_t next_ping;   // earliest time the next ping is allowed

// Wait until the echo line is low (a late echo of the previous ping would ruin this one).
static bool wait_echo_low(void) {
    uint64_t t0 = time_us_64();
    while (gpio_get(US_ECHO_PIN)) {
        if (time_us_64() - t0 > US_WAIT_LOW_US) return false;
    }
    return true;
}

// One measurement. Returns cm, or a negative error code.
static float read_once(void) {
    sleep_until(next_ping);                      // keep the ping spacing
    if (!wait_echo_low()) {
        next_ping = delayed_by_us(get_absolute_time(), US_PERIOD_US);
        return US_ERR_STUCK_HIGH;
    }

    absolute_time_t t_trig = get_absolute_time();

#if US_IRQ_GUARD
    uint32_t irq = save_and_disable_interrupts();
#endif
    // trigger pulse (busy_wait_us_32, not sleep_us: sleep_us can hang when interrupts are off)
    gpio_put(US_TRIG_PIN, 0);
    busy_wait_us_32(2);
    gpio_put(US_TRIG_PIN, 1);
    busy_wait_us_32(US_TRIG_US);
    gpio_put(US_TRIG_PIN, 0);

    // wait for the echo line to go high
    float result = 0.0f;
    bool  ok = true;
    uint64_t t0 = time_us_64();
    while (!gpio_get(US_ECHO_PIN)) {
        if (time_us_64() - t0 > US_RISE_TIMEOUT_US) { ok = false; result = US_ERR_NO_RISE; break; }
    }

    // time how long it stays high
    uint64_t width = 0;
    if (ok) {
        uint64_t start = time_us_64();
        while (gpio_get(US_ECHO_PIN)) {
            if (time_us_64() - start > US_MAX_ECHO_US) { ok = false; result = US_ERR_TOO_LONG; break; }
        }
        width = time_us_64() - start;
    }
#if US_IRQ_GUARD
    restore_interrupts(irq);
#endif

    next_ping = delayed_by_us(t_trig, US_PERIOD_US);
    return ok ? (float)width / 58.0f : result;
}

void us_init(void) {
    gpio_init(US_TRIG_PIN);
    gpio_set_dir(US_TRIG_PIN, GPIO_OUT);
    gpio_put(US_TRIG_PIN, 0);

    gpio_init(US_ECHO_PIN);
    gpio_set_dir(US_ECHO_PIN, GPIO_IN);

    next_ping = get_absolute_time();
}

float us_read_cm(void) {
    float v[US_SAMPLES];
    int n = 0;

    for (int i = 0; i < US_SAMPLES; i++) {
        float d = read_once();
#if US_DEBUG
        if (d == US_ERR_STUCK_HIGH)      printf("  us[%d] ERR echo stuck high\n", i);
        else if (d == US_ERR_NO_RISE)    printf("  us[%d] ERR no echo rise (sensor not triggered / no power)\n", i);
        else if (d == US_ERR_TOO_LONG)   printf("  us[%d] ERR echo too long\n", i);
        else                             printf("  us[%d] %.2f cm\n", i, d);
#endif
        if (d >= 2.0f && d <= 400.0f) v[n++] = d;   // the sensor's valid range
    }
    if (n < 3) return -1.0f;                        // too few good readings

    // sort the good readings (small insertion sort)
    for (int i = 1; i < n; i++) {
        float key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
        v[j + 1] = key;
    }

    // median (average of the two middle values when n is even)
    float med = (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);

    // consensus: most readings must sit close to the median, otherwise we don't trust the result
    int   agree = 0;
    float sum   = 0.0f;
    for (int i = 0; i < n; i++) {
        if (fabsf(v[i] - med) <= US_AGREE_CM) { agree++; sum += v[i]; }
    }
    int need = (n > 3) ? (n - 1) : 3;               // allow one stray reading when we have 4 or 5
    if (agree < need) return -1.0f;

    return sum / agree;                             // average of the agreeing readings
}

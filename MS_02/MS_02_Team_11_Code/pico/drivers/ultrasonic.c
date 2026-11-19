#include "ultrasonic.h"

#define US_TIMEOUT_US 30000   // give up after 30 ms
#define US_SAMPLES    5

// One measurement. Returns cm, or -1.0 on timeout.
static float read_once(void) {
    // 10 us shout
    gpio_put(US_TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(US_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(US_TRIG_PIN, 0);

    // wait for the echo line to go high
    uint64_t t0 = time_us_64();
    while (!gpio_get(US_ECHO_PIN)) {
        if (time_us_64() - t0 > US_TIMEOUT_US) return -1.0f;
    }

    // time how long it stays high
    uint64_t start = time_us_64();
    while (gpio_get(US_ECHO_PIN)) {
        if (time_us_64() - start > US_TIMEOUT_US) return -1.0f;
    }
    uint64_t width = time_us_64() - start;

    return (float)width / 58.0f;
}

void us_init(void) {
    gpio_init(US_TRIG_PIN);
    gpio_set_dir(US_TRIG_PIN, GPIO_OUT);
    gpio_put(US_TRIG_PIN, 0);

    gpio_init(US_ECHO_PIN);
    gpio_set_dir(US_ECHO_PIN, GPIO_IN);
}

float us_read_cm(void) {
    float v[US_SAMPLES];
    int n = 0;

    for (int i = 0; i < US_SAMPLES; i++) {
        float d = read_once();
        if (d >= 2.0f && d <= 400.0f) v[n++] = d;   // the sensor's valid range
        sleep_ms(30);                               // let old echoes die out
    }
    if (n < 3) return -1.0f;                        // too few good readings

    // sort the good readings (small insertion sort)
    for (int i = 1; i < n; i++) {
        float key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
        v[j + 1] = key;
    }
    return v[n / 2];                                // the median ignores one-off glitches
}
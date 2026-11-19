#include <stdio.h>
#include "pico/stdlib.h"
#include "gate.h"

static bool latch_is_open = false;

static void drop_for(uint ms) {
    printf("gate open for %u ms\n", ms);
    gate_open();
    sleep_ms(ms);
    gate_close();
    printf("closed\n");
}

static void reset_toggle(void) {
    if (!latch_is_open) {
        latch_release();
        latch_is_open = true;
        printf("latch RELEASED: floor drops (press r again to lock)\n");
    } else {
        latch_lock();
        latch_is_open = false;
        printf("latch LOCKED\n");
    }
}

int main() {
    stdio_init_all();
    gate_init();
    while (!stdio_usb_connected()) sleep_ms(100);

    printf("\nd = drop (gate open 5 s, then close)\n");
    printf("r = reset latch: 1st press opens, 2nd press closes\n");
    printf("o = open gate   c = close gate\n");

    while (true) {
        int ch = getchar();
        switch (ch) {
            case '1': drop_for(75);   break;
            case 'd': drop_for(DROP_OPEN_MS);  break;   // the official 5 s drop
            case 'r': reset_toggle();   break;
            default: break;   // ignore Enter and other keys
        }
    }
}
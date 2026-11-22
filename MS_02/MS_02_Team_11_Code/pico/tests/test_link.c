    // Talks to the ESP32-CAM over the UART link. Open the Pico's USB serial monitor (115200).
// Keys: p = ping, c = check, l = learn, h = help
#include <stdio.h>
#include "pico/stdlib.h"
#include "esp_link.h"

static void help(void) {
    printf("\n[link test] p = ping ESP, c = CHECK, l = LEARN, h = help\n");
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(100);
    sleep_ms(300);
    esp_link_init();
    printf("ESP link on UART0 (GP0 TX, GP1 RX)\n");
    help();

    for (;;) {
        int ch = getchar_timeout_us(100000);
        if (ch == PICO_ERROR_TIMEOUT) continue;
        if (ch == 'p') {
            printf("ping: %s\n", esp_link_ping() ? "PONG (link ok)" : "no answer");
        } else if (ch == 'l') {
            printf("learning...\n");
            printf("learn: %s\n", esp_link_learn() ? "LEARNED" : "failed / no answer");
        } else if (ch == 'c') {
            uint16_t bad; char raw[64];
            absolute_time_t t0 = get_absolute_time();
            esp_result_t r = esp_link_check(&bad, raw, sizeof raw);
            int ms = (int)(absolute_time_diff_us(t0, get_absolute_time()) / 1000);
            if (r == ESP_RES_WIN)          printf("WIN (%d ms)\n", ms);
            else if (r == ESP_RES_LOSE)    printf("LOSE, wrong mask 0x%03x  raw='%s' (%d ms)\n", bad, raw, ms);
            else if (r == ESP_RES_ERR)     printf("ERR  raw='%s' (%d ms)\n", raw, ms);
            else                           printf("TIMEOUT - no reply from the ESP\n");
        } else if (ch == 'h') {
            help();
        }
    }
}

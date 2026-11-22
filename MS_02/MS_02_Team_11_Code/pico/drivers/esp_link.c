#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "esp_link.h"

#define LINK_UART      uart0
#define LINK_TX_PIN    0
#define LINK_RX_PIN    1
#define LINK_BAUD      115200
#define CHECK_TIMEOUT_MS 8000   // a full check takes about 2 to 3 seconds on the ESP

void esp_link_init(void) {
    uart_init(LINK_UART, LINK_BAUD);
    gpio_set_function(LINK_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LINK_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(LINK_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LINK_UART, true);
}

static void drain_rx(void) {
    while (uart_is_readable(LINK_UART)) (void)uart_getc(LINK_UART);
}

int esp_link_cmd(const char *cmd, char *reply, size_t reply_size, uint32_t timeout_ms) {
    drain_rx();
    uart_puts(LINK_UART, cmd);
    uart_putc_raw(LINK_UART, '\n');

    absolute_time_t end = make_timeout_time_ms(timeout_ms);
    size_t n = 0;
    while (absolute_time_diff_us(get_absolute_time(), end) > 0) {
        if (!uart_is_readable_within_us(LINK_UART, 2000)) continue;
        char c = (char)uart_getc(LINK_UART);
        if (c == '\r') continue;
        if (c == '\n') {
            if (n == 0) continue;           // skip empty lines
            reply[n] = 0;
            return (int)n;
        }
        if (n + 1 < reply_size) reply[n++] = c;
    }
    reply[n] = 0;
    return -1;
}

int esp_link_ping(void) {
    char r[32];
    return esp_link_cmd("PING", r, sizeof r, 500) > 0 && strcmp(r, "PONG") == 0;
}

int esp_link_learn(void) {
    char r[48];
    return esp_link_cmd("LEARN", r, sizeof r, CHECK_TIMEOUT_MS) > 0 && strcmp(r, "LEARNED") == 0;
}

esp_result_t esp_link_check(uint16_t *bad_mask, char *reply, size_t reply_size) {
    char local[64];
    char *r = reply ? reply : local;
    size_t rs = reply ? reply_size : sizeof local;
    if (bad_mask) *bad_mask = 0;

    if (esp_link_cmd("CHECK", r, rs, CHECK_TIMEOUT_MS) < 0) return ESP_RES_TIMEOUT;
    if (strcmp(r, "WIN") == 0) return ESP_RES_WIN;
    if (strncmp(r, "LOSE", 4) == 0) {
        if (bad_mask) {
            for (const char *p = r + 4; *p; p++)
                if (*p >= '1' && *p <= '9') *bad_mask |= (uint16_t)(1u << (*p - '1'));
        }
        return ESP_RES_LOSE;
    }
    return ESP_RES_ERR;     // "ERR <reason>" or anything unexpected: never treated as a win
}

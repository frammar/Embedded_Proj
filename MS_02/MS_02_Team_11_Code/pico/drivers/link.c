#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "link.h"

#define LINK_UART    uart0
#define LINK_IRQ     UART0_IRQ
#define LINK_TX_PIN  0
#define LINK_RX_PIN  1
#define LINK_BAUD    115200
#define RING_SIZE    512            // power of two not required, but keep it larger than the longest burst

static volatile uint8_t ring[RING_SIZE];
static volatile uint16_t head = 0, tail = 0;

static void on_rx(void) {
    while (uart_is_readable(LINK_UART)) {
        uint8_t c = (uint8_t)uart_getc(LINK_UART);
        uint16_t next = (uint16_t)((head + 1) % RING_SIZE);
        if (next != tail) { ring[head] = c; head = next; }    // drop the byte if the ring is full
    }
}

void link_init(void) {
    uart_init(LINK_UART, LINK_BAUD);
    gpio_set_function(LINK_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LINK_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(LINK_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LINK_UART, true);
    irq_set_exclusive_handler(LINK_IRQ, on_rx);
    irq_set_enabled(LINK_IRQ, true);
    uart_set_irq_enables(LINK_UART, true, false);
}

bool link_read_line(char *buf, size_t n) {
    static char line[96];
    static size_t len = 0;
    while (tail != head) {
        char c = (char)ring[tail];
        tail = (uint16_t)((tail + 1) % RING_SIZE);
        if (c == '\r') continue;
        if (c == '\n') {
            if (len == 0) continue;               // skip empty lines
            line[len] = 0;
            size_t k = len < n - 1 ? len : n - 1;
            memcpy(buf, line, k);
            buf[k] = 0;
            len = 0;
            return true;
        }
        if (len < sizeof line - 1) line[len++] = c;
    }
    return false;
}

void link_send(const char *fmt, ...) {
    char out[160];
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(out, sizeof out - 1, fmt, ap);
    va_end(ap);
    if (k < 0) return;
    if (k > (int)sizeof out - 2) k = (int)sizeof out - 2;
    out[k++] = '\n';
    out[k] = 0;
    uart_puts(LINK_UART, out);
}

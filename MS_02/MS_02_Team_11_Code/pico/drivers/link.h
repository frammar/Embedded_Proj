#pragma once
#include <stdbool.h>
#include <stddef.h>

// Non-blocking text link to the ESP32-CAM: UART0, GP0 = TX, GP1 = RX, 115200 8N1.
// Received bytes are collected by an interrupt, so nothing is lost while the main code is busy.
void link_init(void);

// Returns true and fills buf with one complete line (no newline) when one is available.
bool link_read_line(char *buf, size_t n);

// Sends one line (a newline is added).
void link_send(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

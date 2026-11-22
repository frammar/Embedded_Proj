#pragma once
#include "esp_http_server.h"

// Image check for the 3x3 container (win condition).
//   /vision          page with Learn and Check buttons
//   /learn           store the current picture as the "correct" reference (NVS)
//   /check           run one full check, JSON result
//   UART1 (IO14 TX -> Pico GP1, IO13 RX <- Pico GP0, 115200):
//       Pico sends  CHECK | LEARN | PING
//       ESP replies WIN | LOSE 2,5 | ERR <reason> | LEARNED | PONG
// Call vision_start() once, after camera_start() and httpd_start().
void vision_start(httpd_handle_t server);

// Hooks used by app.c (the web console):
typedef void (*vision_line_cb_t)(const char *line);
void vision_set_line_handler(vision_line_cb_t cb);   // gets every UART line that is not PING/LEARN/CHECK
void vision_uart_send(const char *line);             // sends one line to the Pico

#pragma once
#include "esp_http_server.h"

// Web console for the whole machine (Factory 4 - Project 4).
//   /app           the page: code login, Welcome/Start, game, trolley calibration, camera calibration
//   /api/login     ?code=1234
//   /api/state     JSON with the latest status line from the Pico
//   /api/cmd       ?c=START | DROP | CONFIRM | OVER | CAL | CALDONE | STOP | GATE | LATCH |
//                     SEL_1..3 | SAVE_1..3 | JOG_f | JOG_b | JOG_F | JOG_B
// Call app_register() once, after vision_start().
#define APP_CODE "1234"   // access code typed on the login screen

void app_register(httpd_handle_t server);

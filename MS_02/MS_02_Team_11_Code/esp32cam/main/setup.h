#pragma once
#include "esp_http_server.h"

// Registers the camera-placement tool: /setup (page), /spic (picture), /stats (numbers),
// /flash (steady flash LED), /cam (manual exposure + gain).
void setup_register(httpd_handle_t server);

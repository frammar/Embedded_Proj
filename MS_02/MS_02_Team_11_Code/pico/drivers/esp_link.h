#pragma once
#include <stddef.h>
#include <stdint.h>

// UART link to the ESP32-CAM: UART0, GP0 = TX (to ESP IO13), GP1 = RX (from ESP IO14), 115200 8N1.
typedef enum { ESP_RES_WIN, ESP_RES_LOSE, ESP_RES_ERR, ESP_RES_TIMEOUT } esp_result_t;

void esp_link_init(void);

// Sends one command line and waits for one reply line. Returns the reply length, or -1 on timeout.
int esp_link_cmd(const char *cmd, char *reply, size_t reply_size, uint32_t timeout_ms);

// "PING" -> true if the ESP answers PONG.
int esp_link_ping(void);

// "LEARN" -> true if the ESP answers LEARNED.
int esp_link_learn(void);

// "CHECK" -> WIN / LOSE / ERR / TIMEOUT. On LOSE, bad_mask has bit (n-1) set for every wrong cell n.
// reply (optional, may be NULL) receives the raw line.
esp_result_t esp_link_check(uint16_t *bad_mask, char *reply, size_t reply_size);

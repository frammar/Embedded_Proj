// Web console: forwards button presses to the Pico and shows the Pico's status.
// The Pico owns the machine state. This file only relays and remembers the trolley slots (NVS).
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "vision.h"
#include "app.h"
#include "app_page.h"

static const char *TAG = "app";

#define LINK_TIMEOUT_MS   2000    // no status for this long = Pico not connected
#define SLOT_PUSH_GAP_MS  3000    // minimum time between two SLOTS pushes

typedef struct {
    char   state[16];
    int    c[3];
    float  dist, target;
    int    sel;
    float  slots[3];
    char   err[24], cv[40];
    int64_t last_st_ms;
    unsigned seq;                 // counts OK/ERR replies
    char   reply[32];
} st_t;

static st_t s_st = { .state = "IDLE", .dist = -1, .target = -1, .slots = {-1, -1, -1}, .err = "-", .cv = "-" };
static SemaphoreHandle_t s_mx;
static float s_saved[3] = {-1, -1, -1};     // copy kept in NVS
static int64_t s_last_push_ms = 0;

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

// ------------------------------------------------------------------ NVS (trolley slots survive a restart)
static void load_slots(void) {
    nvs_handle_t h; size_t n = sizeof s_saved;
    if (nvs_open("app", NVS_READONLY, &h) != ESP_OK) return;
    float v[3]; size_t len = sizeof v;
    if (nvs_get_blob(h, "slots", v, &len) == ESP_OK && len == sizeof v) memcpy(s_saved, v, sizeof v);
    (void)n;
    nvs_close(h);
}
static void store_slots(const float v[3]) {
    nvs_handle_t h;
    if (nvs_open("app", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "slots", v, 3 * sizeof(float));
    nvs_commit(h);
    nvs_close(h);
    memcpy(s_saved, v, 3 * sizeof(float));
    ESP_LOGI(TAG, "slots stored: %.2f %.2f %.2f", v[0], v[1], v[2]);
}
static bool have_saved(void) { return s_saved[0] > 0 && s_saved[1] > 0 && s_saved[2] > 0; }

static void push_slots(void) {
    char b[400];
    snprintf(b, sizeof b, "SLOTS %.2f %.2f %.2f", s_saved[0], s_saved[1], s_saved[2]);
    vision_uart_send(b);
    s_last_push_ms = now_ms();
    ESP_LOGI(TAG, "pushed %s", b);
}

// ------------------------------------------------------------------ lines coming from the Pico
static void on_line(const char *line) {
    if (!strncmp(line, "ST ", 3)) {
        char state[16], err[24], cv[40];
        int c0, c1, c2, sel;
        float dist, tgt, s0, s1, s2;
        int n = sscanf(line + 3, "%15s %d %d %d %f %f %d %f %f %f %23s %39s",
                       state, &c0, &c1, &c2, &dist, &tgt, &sel, &s0, &s1, &s2, err, cv);
        if (n < 12) { ESP_LOGW(TAG, "bad status (%d fields): %s", n, line); return; }
        bool all = s0 > 0 && s1 > 0 && s2 > 0;
        bool need_push = false, need_store = false;
        float cur[3] = { s0, s1, s2 };
        xSemaphoreTake(s_mx, portMAX_DELAY);
        snprintf(s_st.state, sizeof s_st.state, "%s", state);
        s_st.c[0] = c0; s_st.c[1] = c1; s_st.c[2] = c2;
        s_st.dist = dist; s_st.target = tgt; s_st.sel = sel;
        s_st.slots[0] = s0; s_st.slots[1] = s1; s_st.slots[2] = s2;
        snprintf(s_st.err, sizeof s_st.err, "%s", err);
        snprintf(s_st.cv, sizeof s_st.cv, "%s", cv);
        s_st.last_st_ms = now_ms();
        if (all) {
            for (int i = 0; i < 3; i++) if (fabsf(cur[i] - s_saved[i]) > 0.005f) need_store = true;
        } else if (have_saved() && (!strcmp(state, "IDLE") || !strcmp(state, "CALIB")) &&
                   now_ms() - s_last_push_ms > SLOT_PUSH_GAP_MS) {
            need_push = true;
        }
        xSemaphoreGive(s_mx);
        if (need_store) store_slots(cur);
        if (need_push) push_slots();
    } else if (!strcmp(line, "BOOT")) {                 // Pico just restarted: give it the stored slots
        ESP_LOGI(TAG, "Pico booted");
        if (have_saved()) push_slots();
    } else if (!strcmp(line, "OK") || !strncmp(line, "ERR ", 4)) {
        xSemaphoreTake(s_mx, portMAX_DELAY);
        s_st.seq++;
        snprintf(s_st.reply, sizeof s_st.reply, "%s", line);
        xSemaphoreGive(s_mx);
    }
}

// ------------------------------------------------------------------ HTTP
static esp_err_t page_h(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, APP_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t json_send(httpd_req_t *req, const char *s) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, s, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t login_h(httpd_req_t *req) {
    char q[48], code[24] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK) httpd_query_key_value(q, "code", code, sizeof code);
    return json_send(req, strcmp(code, APP_CODE) == 0 ? "{\"ok\":true}" : "{\"ok\":false}");
}

static esp_err_t state_h(httpd_req_t *req) {
    char b[900];
    xSemaphoreTake(s_mx, portMAX_DELAY);
    bool link = s_st.last_st_ms > 0 && now_ms() - s_st.last_st_ms < LINK_TIMEOUT_MS;
    snprintf(b, sizeof b,
        "{\"link\":%s,\"state\":\"%s\",\"c\":[%d,%d,%d],\"dist\":%.2f,\"target\":%.2f,\"sel\":%d,"
        "\"slots\":[%.2f,%.2f,%.2f],\"err\":\"%s\",\"cv\":\"%s\",\"seq\":%u,\"reply\":\"%s\"}",
        link ? "true" : "false", s_st.state, s_st.c[0], s_st.c[1], s_st.c[2], s_st.dist, s_st.target, s_st.sel,
        s_st.slots[0], s_st.slots[1], s_st.slots[2], s_st.err, s_st.cv, s_st.seq, s_st.reply);
    xSemaphoreGive(s_mx);
    return json_send(req, b);
}

static const char *const ALLOWED[] = {
    "START", "DROP", "CONFIRM", "OVER", "CAL", "CALDONE", "STOP", "GATE", "LATCH",
    "SEL 1", "SEL 2", "SEL 3", "SAVE 1", "SAVE 2", "SAVE 3", "JOG f", "JOG b", "JOG F", "JOG B",
};

static esp_err_t cmd_h(httpd_req_t *req) {
    char q[48], c[16] = "";
    if (httpd_req_get_url_query_str(req, q, sizeof q) != ESP_OK || httpd_query_key_value(q, "c", c, sizeof c) != ESP_OK)
        return json_send(req, "{\"ok\":false}");
    for (char *p = c; *p; p++) if (*p == '_') *p = ' ';
    for (size_t i = 0; i < sizeof ALLOWED / sizeof *ALLOWED; i++) {
        if (strcmp(c, ALLOWED[i]) == 0) {
            vision_uart_send(c);
            return json_send(req, "{\"ok\":true}");
        }
    }
    return json_send(req, "{\"ok\":false}");
}

void app_register(httpd_handle_t server) {
    s_mx = xSemaphoreCreateMutex();
    load_slots();
    ESP_LOGI(TAG, "stored slots: %.2f %.2f %.2f", s_saved[0], s_saved[1], s_saved[2]);
    vision_set_line_handler(on_line);
    static const httpd_uri_t uris[] = {
        { .uri = "/app",        .method = HTTP_GET, .handler = page_h,  .user_ctx = NULL },
        { .uri = "/api/login",  .method = HTTP_GET, .handler = login_h, .user_ctx = NULL },
        { .uri = "/api/state",  .method = HTTP_GET, .handler = state_h, .user_ctx = NULL },
        { .uri = "/api/cmd",    .method = HTTP_GET, .handler = cmd_h,   .user_ctx = NULL },
    };
    for (size_t i = 0; i < sizeof uris / sizeof *uris; i++) {
        esp_err_t e = httpd_register_uri_handler(server, &uris[i]);
        if (e != ESP_OK) ESP_LOGE(TAG, "register %s failed: %s", uris[i].uri, esp_err_to_name(e));
    }
}

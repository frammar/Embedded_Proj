// Colour check of the 9 tiles. All numbers marked TUNE come from the setup tool and can change.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "camera.h"
#include "vision.h"

static const char *TAG = "vision";

// ------------------------------------------------------------------ TUNE
// Must match RGB_IS_BGR in setup.c (set to whatever you ended up with there).
#define RGB_IS_BGR      1

// Locked camera settings (copy the Exposure and Gain shown in the setup tool).
#define CAM_AEC         540
#define CAM_GAIN        2

// Rectangles are drawn on the /vision page and stored in flash. These are only the first-boot defaults
// (320x240 coordinates).
typedef struct { int x, y, w, h; } rect_t;
static rect_t s_white = {258, 108, 29, 51};
static rect_t s_cell[9] = {            // cell 1..9 = row by row, left to right, top to bottom
    { 67, 71, 40, 29}, {130, 71, 40, 29}, {194, 71, 40, 29},
    { 67,117, 40, 29}, {130,117, 40, 29}, {194,117, 40, 29},
    { 67,163, 40, 29}, {130,163, 40, 29}, {194,163, 40, 29},
};

#define OWN_MAX         35.0f   // a tile must be this close to its own reference
#define OWN_RATIO       1.6f    // ...and at least this many times closer to its own than to any other
#define SAME_COLOUR     12.0f   // two references closer than this count as the same colour (swap undetectable)
#define LIGHT_TOL       0.30f   // white patch may drift this fraction from the learned value before ERR
#define FRAMES_PER_ROUND 3
#define ROUNDS_FOR_WIN   3      // consecutive good rounds needed for WIN
#define LEARN_FRAMES     6
#define WHITE_TARGET     200    // white patch is scaled to this, which cancels colour cast and brightness

#define UART_PORT   UART_NUM_1
#define UART_TX_PIN 14
#define UART_RX_PIN 13
// -----------------------------------------------------------------------

typedef struct { float f[9][3]; float white_lum; } frame_feat_t;
typedef struct { uint32_t magic; frame_feat_t ref; } stored_t;
#define MAGIC 0xF4C0FFEE

typedef enum { V_WIN, V_LOSE, V_ERR } vres_t;
typedef struct {
    vres_t res;
    int bad_mask;           // bit i set = cell i+1 wrong
    char why[40];
    float d_own[9], d_other[9];
} vout_t;

static frame_feat_t s_ref;
static bool s_have_ref = false;
static SemaphoreHandle_t s_lock;

// ------------------------------------------------------------------ image helpers
static inline void px(const uint8_t *p, int *r, int *g, int *b) {
#if RGB_IS_BGR
    *b = p[0]; *g = p[1]; *r = p[2];
#else
    *r = p[0]; *g = p[1]; *b = p[2];
#endif
}

// Median of each channel inside a rectangle (robust against glare spots and edges).
static void rect_median(const uint8_t *rgb, int W, int H, int x0, int y0, int w, int h, int out[3]) {
    static uint16_t hist[3][256];      // static: too big for the small web-server stack (only used under s_lock)
    memset(hist, 0, sizeof hist);
    int n = 0;
    for (int y = y0; y < y0 + h && y < H; y++) {
        for (int x = x0; x < x0 + w && x < W; x++) {
            int r, g, b;
            px(rgb + (y * W + x) * 3, &r, &g, &b);
            hist[0][r]++; hist[1][g]++; hist[2][b]++; n++;
        }
    }
    for (int c = 0; c < 3; c++) {
        int acc = 0, v = 0;
        for (v = 0; v < 256; v++) { acc += hist[c][v]; if (acc * 2 >= n) break; }
        out[c] = v > 255 ? 255 : v;
    }
}

// One picture -> 9 white-normalised colours. Returns NULL-ish error text, or NULL if ok.
static const char *grab_features(frame_feat_t *o) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return "no frame";
    int W = fb->width, H = fb->height;
    uint8_t *rgb = heap_caps_malloc((size_t)W * H * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rgb) { esp_camera_fb_return(fb); return "no memory"; }
    bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
    esp_camera_fb_return(fb);
    if (!ok) { free(rgb); return "decode"; }

    int wm[3];
    rect_median(rgb, W, H, s_white.x, s_white.y, s_white.w, s_white.h, wm);
    if (wm[0] < 40 || wm[1] < 40 || wm[2] < 40) { free(rgb); return "white too dark"; }
    if (wm[0] >= 250 || wm[1] >= 250 || wm[2] >= 250) { free(rgb); return "white clipped"; }
    o->white_lum = (wm[0] * 30 + wm[1] * 59 + wm[2] * 11) / 100.0f;

    for (int i = 0; i < 9; i++) {
        int m[3];
        rect_median(rgb, W, H, s_cell[i].x, s_cell[i].y, s_cell[i].w, s_cell[i].h, m);
        for (int c = 0; c < 3; c++) o->f[i][c] = (float)m[c] * WHITE_TARGET / wm[c];
    }
    free(rgb);
    return NULL;
}

static const char *grab_avg(int n, frame_feat_t *avg) {
    memset(avg, 0, sizeof *avg);
    for (int k = 0; k < n; k++) {
        frame_feat_t t;
        const char *e = grab_features(&t);
        if (e) return e;
        for (int i = 0; i < 9; i++) for (int c = 0; c < 3; c++) avg->f[i][c] += t.f[i][c] / n;
        avg->white_lum += t.white_lum / n;
        vTaskDelay(pdMS_TO_TICKS(80));       // let the sensor give a fresh frame
    }
    return NULL;
}

static float dist3(const float a[3], const float b[3]) {
    float d0 = a[0] - b[0], d1 = a[1] - b[1], d2 = a[2] - b[2];
    return sqrtf(d0 * d0 + d1 * d1 + d2 * d2);
}

// ------------------------------------------------------------------ learn / check
static esp_err_t save_ref(void) {
    nvs_handle_t h;
    esp_err_t e = nvs_open("vision", NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    stored_t st = { MAGIC, s_ref };
    e = nvs_set_blob(h, "ref", &st, sizeof st);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

static void load_ref(void) {
    nvs_handle_t h;
    if (nvs_open("vision", NVS_READONLY, &h) != ESP_OK) return;
    stored_t st; size_t n = sizeof st;
    if (nvs_get_blob(h, "ref", &st, &n) == ESP_OK && n == sizeof st && st.magic == MAGIC) {
        s_ref = st.ref; s_have_ref = true;
    }
    nvs_close(h);
}

typedef struct { uint32_t magic; rect_t white; rect_t cell[9]; } rects_t;
#define RMAGIC 0xF4C0AAA1

static void save_rects(void) {
    nvs_handle_t h;
    if (nvs_open("vision", NVS_READWRITE, &h) != ESP_OK) return;
    rects_t r = { RMAGIC, s_white, {{0}} };
    memcpy(r.cell, s_cell, sizeof s_cell);
    nvs_set_blob(h, "rects", &r, sizeof r);
    nvs_commit(h);
    nvs_close(h);
}

static void load_rects(void) {
    nvs_handle_t h;
    if (nvs_open("vision", NVS_READONLY, &h) != ESP_OK) return;
    rects_t r; size_t n = sizeof r;
    if (nvs_get_blob(h, "rects", &r, &n) == ESP_OK && n == sizeof r && r.magic == RMAGIC) {
        s_white = r.white; memcpy(s_cell, r.cell, sizeof s_cell);
    }
    nvs_close(h);
}

static int qint(const char *q, const char *key, int def) {
    char v[16];
    if (httpd_query_key_value(q, key, v, sizeof v) == ESP_OK) return atoi(v);
    return def;
}

// /rects            -> JSON of the 10 rectangles (9 tiles, then white)
// /rects?i=..&x=..&y=..&w=..&h=..  -> set rectangle i (0..8 tiles, 9 white), then forget the learned reference
static esp_err_t rects_handler(httpd_req_t *req) {
    char q[100] = {0};
    httpd_req_get_url_query_str(req, q, sizeof q);
    int i = qint(q, "i", -1);
    if (i >= 0 && i <= 9) {
        rect_t r = { qint(q, "x", 0), qint(q, "y", 0), qint(q, "w", 0), qint(q, "h", 0) };
        if (r.x < 0) r.x = 0;
        if (r.y < 0) r.y = 0;
        if (r.x > 315) r.x = 315;
        if (r.y > 235) r.y = 235;
        if (r.w < 4) r.w = 4;
        if (r.h < 4) r.h = 4;
        if (r.x + r.w > 320) r.w = 320 - r.x;
        if (r.y + r.h > 240) r.h = 240 - r.y;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (i == 9) s_white = r; else s_cell[i] = r;
        s_have_ref = false;          // old reference no longer matches these boxes
        save_rects();
        xSemaphoreGive(s_lock);
    }
    char buf[420]; int k = snprintf(buf, sizeof buf, "{\"r\":[");
    for (int j = 0; j < 10; j++) {
        rect_t r = j == 9 ? s_white : s_cell[j];
        k += snprintf(buf + k, sizeof buf - k, "%s[%d,%d,%d,%d]", j ? "," : "", r.x, r.y, r.w, r.h);
    }
    snprintf(buf + k, sizeof buf - k, "]}");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

// Returns NULL if ok, otherwise an error text. Look-alike cells are written to lookalike_mask.
static const char *do_learn(int *lookalike_mask) {
    frame_feat_t a;
    const char *e = grab_avg(LEARN_FRAMES, &a);
    if (e) return e;
    s_ref = a; s_have_ref = true;
    if (save_ref() != ESP_OK) return "nvs save";
    int mask = 0;
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++)
            if (i != j && dist3(a.f[i], a.f[j]) < SAME_COLOUR) mask |= 1 << i;
    if (lookalike_mask) *lookalike_mask = mask;
    return NULL;
}

// Rule (b): tile OK if close to its own reference AND clearly closer to it than to the others.
static void judge(const frame_feat_t *cur, vout_t *o) {
    o->bad_mask = 0;
    for (int i = 0; i < 9; i++) {
        float own = dist3(cur->f[i], s_ref.f[i]);
        float other = 1e9f;
        for (int j = 0; j < 9; j++) {
            if (j == i) continue;
            if (dist3(s_ref.f[i], s_ref.f[j]) < SAME_COLOUR) continue;   // same colour: cannot tell apart
            float d = dist3(cur->f[i], s_ref.f[j]);
            if (d < other) other = d;
        }
        o->d_own[i] = own; o->d_other[i] = other;
        bool ok = own < OWN_MAX && own * OWN_RATIO < other;
        if (!ok) o->bad_mask |= 1 << i;
    }
}

static void do_check(vout_t *o) {
    memset(o, 0, sizeof *o);
    if (!s_have_ref) { o->res = V_ERR; strcpy(o->why, "not learned"); return; }
    for (int r = 0; r < ROUNDS_FOR_WIN; r++) {
        frame_feat_t cur;
        const char *e = grab_avg(FRAMES_PER_ROUND, &cur);
        if (e) { o->res = V_ERR; snprintf(o->why, sizeof o->why, "%s", e); return; }
        if (fabsf(cur.white_lum - s_ref.white_lum) > LIGHT_TOL * s_ref.white_lum) {
            o->res = V_ERR; strcpy(o->why, "light changed"); return;
        }
        judge(&cur, o);
        if (o->bad_mask) { o->res = V_LOSE; return; }   // any miss -> LOSE (shows the cells of this round)
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    o->res = V_WIN;
}

static void list_cells(int mask, char *out, size_t n) {
    out[0] = 0;
    size_t k = 0;
    for (int i = 0; i < 9; i++) {
        if (mask & (1 << i)) k += snprintf(out + k, n - k, "%s%d", k ? "," : "", i + 1);
    }
}

// ------------------------------------------------------------------ web
static void result_json(const vout_t *o, char *buf, size_t n) {
    const char *name = o->res == V_WIN ? "WIN" : o->res == V_LOSE ? "LOSE" : "ERR";
    char bad[32]; list_cells(o->bad_mask, bad, sizeof bad);
    int k = snprintf(buf, n, "{\"result\":\"%s\",\"bad\":[%s],\"why\":\"%s\",\"own\":[", name, bad, o->why);
    for (int i = 0; i < 9; i++) k += snprintf(buf + k, n - k, "%s%.0f", i ? "," : "", o->d_own[i]);
    k += snprintf(buf + k, n - k, "],\"other\":[");
    for (int i = 0; i < 9; i++) k += snprintf(buf + k, n - k, "%s%.0f", i ? "," : "", o->d_other[i] > 9999 ? -1.0f : o->d_other[i]);
    snprintf(buf + k, n - k, "]}");
}

static esp_err_t check_handler(httpd_req_t *req) {
    vout_t o;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    do_check(&o);
    xSemaphoreGive(s_lock);
    char buf[300]; result_json(&o, buf, sizeof buf);
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t learn_handler(httpd_req_t *req) {
    int look = 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const char *e = do_learn(&look);
    xSemaphoreGive(s_lock);
    char l[32]; list_cells(look, l, sizeof l);
    char buf[160];
    snprintf(buf, sizeof buf, "{\"learned\":%s,\"why\":\"%s\",\"lookalike\":[%s]}", e ? "false" : "true", e ? e : "", l);
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static const char PAGE[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=700'><title>Vision</title><style>"
"body{background:#111;color:#eee;font-family:sans-serif;margin:10px}button{font-size:16px;margin:3px;padding:6px 10px}"
"#box{position:relative;width:640px;height:480px}#p,#ov{position:absolute;left:0;top:0;width:640px;height:480px}#ov{touch-action:none;cursor:crosshair}"
"table{border-collapse:collapse;margin:8px 0}td{width:70px;height:50px;text-align:center;border:1px solid #555;font-family:monospace}"
".g{background:#1a5c1a}.r{background:#7a1f1f}.sel{background:#aa8800}#res{font-size:28px;margin:8px 0}"
"</style></head><body><h3>Container check</h3>"
"<div>1) Pick a button, then drag a box on the picture inside that tile. Tiles 1-9 = row by row (1 2 3 on top), then White patch.</div>"
"<div id='sel'></div>"
"<div id='box'><img id='p' src='/spic'><canvas id='ov' width='640' height='480'></canvas></div>"
"<div>2) With the correct order in place: <button onclick='learn()'>LEARN</button> 3) <button onclick='chk()'>CHECK</button></div>"
"<div id='res'>-</div><table id='t'></table><div id='msg'></div>"
"<script>"
"var R=[],cur=0,drag=null,cv=document.getElementById('ov'),cx=cv.getContext('2d');"
"function sel(){var h='',i;for(i=0;i<10;i++){h+='<button class=\"'+(i==cur?'sel':'')+'\" onclick=\"cur='+i+';sel();redraw();\">'+(i<9?('Tile '+(i+1)):'White')+'</button>';if(i==8)h+='<br>';}document.getElementById('sel').innerHTML=h;}"
"function redraw(){cx.clearRect(0,0,640,480);var i;for(i=0;i<R.length;i++){var r=R[i];cx.lineWidth=i==cur?3:2;cx.strokeStyle=i==cur?'#ff0':(i==9?'#0ff':'#f0f');cx.strokeRect(r[0]*2,r[1]*2,r[2]*2,r[3]*2);cx.fillStyle=cx.strokeStyle;cx.font='16px sans-serif';cx.fillText(i==9?'W':(i+1),r[0]*2+3,r[1]*2+16);}"
"if(drag){cx.strokeStyle='#fff';cx.strokeRect(drag.x0,drag.y0,drag.x1-drag.x0,drag.y1-drag.y0);}}"
"function api(u){return fetch(u).then(function(r){if(!r.ok)throw new Error('HTTP '+r.status+' for '+u);return r.json();});}"
"function say(t){document.getElementById('msg').textContent=t;}"
"function pos(e){var b=cv.getBoundingClientRect();return{x:(e.clientX-b.left)*640/b.width,y:(e.clientY-b.top)*480/b.height};}"
"cv.onpointerdown=function(e){var p=pos(e);drag={x0:p.x,y0:p.y,x1:p.x,y1:p.y};cv.setPointerCapture(e.pointerId);};"
"cv.onpointermove=function(e){if(!drag)return;var p=pos(e);drag.x1=p.x;drag.y1=p.y;redraw();};"
"cv.onpointerup=function(e){if(!drag)return;var d=drag;drag=null;var x=Math.round(Math.min(d.x0,d.x1)/2),y=Math.round(Math.min(d.y0,d.y1)/2),w=Math.round(Math.abs(d.x1-d.x0)/2),h=Math.round(Math.abs(d.y1-d.y0)/2);"
"if(w<4||h<4){redraw();return;}"
"R[cur]=[x,y,w,h];var c0=cur;if(cur<9)cur++;sel();redraw();"
"api('/rects?i='+c0+'&x='+x+'&y='+y+'&w='+w+'&h='+h).then(function(j){R=j.r;redraw();say('saved - press LEARN again');}).catch(function(e){say('save failed: '+e.message);});};"
"function pic(){document.getElementById('p').src='/spic?'+Date.now();}setInterval(pic,1500);"
"function draw(j){var t='',i;for(i=0;i<9;i++){if(i%3==0)t+='<tr>';var bad=j.bad.indexOf(i+1)>=0;"
"t+='<td class=\"'+(j.result=='WIN'||!bad?'g':'r')+'\">'+(i+1)+'<br>'+j.own[i]+' / '+j.other[i]+'</td>';if(i%3==2)t+='</tr>';}"
"document.getElementById('t').innerHTML=t;}"
"function chk(){document.getElementById('res').textContent='checking...';"
"api('/check').then(function(j){document.getElementById('res').textContent=j.result+(j.why?' ('+j.why+')':'')+(j.bad.length?' wrong: '+j.bad.join(','):'');draw(j);}).catch(function(e){document.getElementById('res').textContent='error';say(e.message);});}"
"function learn(){say('learning...');"
"api('/learn').then(function(j){say(j.learned?('learned'+(j.lookalike.length?'; look-alike cells (swap not detectable): '+j.lookalike.join(','):'')):('failed: '+j.why));}).catch(function(e){say('learn error: '+e.message+' (board may have rebooted - see serial monitor)');});}"
"sel();api('/rects').then(function(j){R=j.r;redraw();}).catch(function(e){say(e.message);});"
"</script></body></html>";

static esp_err_t page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

// ------------------------------------------------------------------ UART link to the Pico
static volatile uint32_t s_rx_bytes = 0, s_rx_lines = 0;
static char s_last_rx[96] = "";

static vision_line_cb_t s_line_cb = NULL;
void vision_set_line_handler(vision_line_cb_t cb) { s_line_cb = cb; }

// One write per line so lines from different tasks never interleave.
void vision_uart_send(const char *s) {
    char b[96];
    int n = snprintf(b, sizeof b, "%s\n", s);
    if (n > 0) uart_write_bytes(UART_PORT, b, n < (int)sizeof b ? n : (int)sizeof b - 1);
}
static void uart_send(const char *s) { vision_uart_send(s); }

static void uart_task(void *arg) {
    char line[96]; int n = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(UART_PORT, &c, 1, pdMS_TO_TICKS(100)) != 1) continue;
        s_rx_bytes++;
        if (c != '\n' && c != '\r') { if (n < (int)sizeof line - 1) line[n++] = (char)c; continue; }
        if (n == 0) continue;
        line[n] = 0; n = 0;
        s_rx_lines++;
        snprintf(s_last_rx, sizeof s_last_rx, "%s", line);
        if (strncmp(line, "ST ", 3) != 0) ESP_LOGI(TAG, "uart rx: '%s'", line);
        if (strcmp(line, "PING") == 0) { uart_send("PONG"); }
        else if (strcmp(line, "LEARN") == 0) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            const char *e = do_learn(NULL);
            xSemaphoreGive(s_lock);
            if (e) { char m[48]; snprintf(m, sizeof m, "ERR %s", e); uart_send(m); } else uart_send("LEARNED");
        } else if (strcmp(line, "CHECK") == 0) {
            vout_t o;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            do_check(&o);
            xSemaphoreGive(s_lock);
            char m[64];
            if (o.res == V_WIN) uart_send("WIN");
            else if (o.res == V_LOSE) { char b[32]; list_cells(o.bad_mask, b, sizeof b); snprintf(m, sizeof m, "LOSE %s", b); uart_send(m); }
            else { snprintf(m, sizeof m, "ERR %s", o.why); uart_send(m); }
        } else if (s_line_cb) {
            s_line_cb(line);   // ST ..., BOOT, OK, ERR ... go to the web app
        }
    }
}

// Shows what the UART link has received, so it can be tested from a phone without a serial monitor.
static esp_err_t link_handler(httpd_req_t *req) {
    char buf[640];
    int vis = 0;
    for (const char *p = s_last_rx; *p; p++) if (*p < 32 || *p > 126) vis++;
    snprintf(buf, sizeof buf,
             "<html><head><meta http-equiv='refresh' content='1'></head>"
             "<body style='background:#111;color:#eee;font:20px monospace'>"
             "UART1 (IO13 RX / IO14 TX)<br>bytes received: %u<br>lines received: %u<br>last line: '%s'%s</body></html>",
             (unsigned)s_rx_bytes, (unsigned)s_rx_lines, s_last_rx, vis ? " (has odd characters)" : "");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

// ------------------------------------------------------------------ start
void vision_start(httpd_handle_t server) {
    s_lock = xSemaphoreCreateMutex();

    sensor_t *s = esp_camera_sensor_get();
    if (s) {   // lock the automatic controls so the picture never drifts
        s->set_whitebal(s, 0); s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0); s->set_aec2(s, 0);
        s->set_gain_ctrl(s, 0);
        s->set_aec_value(s, CAM_AEC);
        s->set_agc_gain(s, CAM_GAIN);
    }
    load_rects();
    load_ref();
    ESP_LOGI(TAG, "reference %s", s_have_ref ? "loaded from NVS" : "not learned yet");

    uart_config_t cfg = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                          .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                          .source_clk = UART_SCLK_DEFAULT };
    uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "vision_uart", 6144, NULL, 4, NULL);

    static const httpd_uri_t uris[] = {
        { .uri = "/vision", .method = HTTP_GET, .handler = page_handler,  .user_ctx = NULL },
        { .uri = "/learn",  .method = HTTP_GET, .handler = learn_handler, .user_ctx = NULL },
        { .uri = "/check",  .method = HTTP_GET, .handler = check_handler, .user_ctx = NULL },
        { .uri = "/rects",  .method = HTTP_GET, .handler = rects_handler, .user_ctx = NULL },
        { .uri = "/link",   .method = HTTP_GET, .handler = link_handler,  .user_ctx = NULL },
    };
    for (size_t i = 0; i < sizeof uris / sizeof uris[0]; i++) {
        esp_err_t e = httpd_register_uri_handler(server, &uris[i]);
        if (e != ESP_OK) ESP_LOGE(TAG, "failed to register %s (%s)", uris[i].uri, esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "vision ready: http://192.168.4.1/vision");
}

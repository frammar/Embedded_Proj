// Camera placement / calibration tool for the ESP32-CAM.
// Open http://192.168.4.1/setup while connected to the Factory4 Wi-Fi.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "camera.h"
#include "setup.h"

static const char *TAG = "setup";

// fmt2rgb888() normally returns BGR order. If the colours look wrong (the red tile reads high in
// B instead of R), flip this to 0.
#define RGB_IS_BGR 1

typedef struct { int x, y, w, h; } rect_t;
typedef struct { int r, g, b, clip; } patch_t;

// ---------------------------------------------------------------- helpers
static int qint(const char *q, const char *key, int def) {
    char val[16];
    if (httpd_query_key_value(q, key, val, sizeof val) == ESP_OK) return atoi(val);
    return def;
}

static inline void px(const uint8_t *p, int *r, int *g, int *b) {
#if RGB_IS_BGR
    *b = p[0]; *g = p[1]; *r = p[2];
#else
    *r = p[0]; *g = p[1]; *b = p[2];
#endif
}

static inline int gray_at(const uint8_t *rgb, int W, int x, int y) {
    int r, g, b;
    px(rgb + (y * W + x) * 3, &r, &g, &b);
    return (r * 30 + g * 59 + b * 11) / 100;
}

static rect_t clamp_rect(rect_t a, int W, int H) {
    if (a.x < 0) a.x = 0;
    if (a.y < 0) a.y = 0;
    if (a.x >= W) a.x = W - 1;
    if (a.y >= H) a.y = H - 1;
    if (a.w < 0) a.w = 0;
    if (a.h < 0) a.h = 0;
    if (a.x + a.w > W) a.w = W - a.x;
    if (a.y + a.h > H) a.h = H - a.y;
    return a;
}

// Mean colour and clipped-pixel percentage inside a rectangle.
static patch_t patch_stats(const uint8_t *rgb, int W, int H, rect_t a) {
    patch_t o = {0, 0, 0, 0};
    a = clamp_rect(a, W, H);
    if (a.w < 1 || a.h < 1) return o;
    long sr = 0, sg = 0, sb = 0;
    int n = 0, clip = 0;
    for (int y = a.y; y < a.y + a.h; y++) {
        for (int x = a.x; x < a.x + a.w; x++) {
            int r, g, b;
            px(rgb + (y * W + x) * 3, &r, &g, &b);
            sr += r; sg += g; sb += b; n++;
            if (r >= 250 || g >= 250 || b >= 250) clip++;
        }
    }
    o.r = (int)(sr / n);
    o.g = (int)(sg / n);
    o.b = (int)(sb / n);
    o.clip = clip * 100 / n;
    return o;
}

// Focus score (mean edge strength x10), mean brightness, clipped % and dark % for a rectangle.
static void container_stats(const uint8_t *rgb, int W, int H, rect_t a,
                            int *focus10, int *mean, int *clip, int *dark) {
    *focus10 = *mean = *clip = *dark = 0;
    a = clamp_rect(a, W, H);
    if (a.w < 3 || a.h < 3) return;
    long edge = 0, sum = 0;
    int n = 0, nclip = 0, ndark = 0, ne = 0;
    for (int y = a.y; y < a.y + a.h; y++) {
        for (int x = a.x; x < a.x + a.w; x++) {
            int r, g, b;
            px(rgb + (y * W + x) * 3, &r, &g, &b);
            int gy0 = (r * 30 + g * 59 + b * 11) / 100;
            sum += gy0; n++;
            if (r >= 250 || g >= 250 || b >= 250) nclip++;
            if (gy0 <= 10) ndark++;
            if (x < a.x + a.w - 1 && y < a.y + a.h - 1) {
                int gx = abs(gray_at(rgb, W, x + 1, y) - gy0);
                int gv = abs(gray_at(rgb, W, x, y + 1) - gy0);
                edge += gx + gv; ne++;
            }
        }
    }
    *mean    = (int)(sum / n);
    *clip    = nclip * 100 / n;
    *dark    = ndark * 100 / n;
    *focus10 = ne ? (int)(edge * 10 / ne) : 0;
}

static void frame_stats(const uint8_t *rgb, int W, int H, int *mean, int *clip) {
    long sum = 0;
    int n = 0, nclip = 0;
    for (int y = 0; y < H; y += 4) {
        for (int x = 0; x < W; x += 4) {
            int r, g, b;
            px(rgb + (y * W + x) * 3, &r, &g, &b);
            sum += (r * 30 + g * 59 + b * 11) / 100; n++;
            if (r >= 250 || g >= 250 || b >= 250) nclip++;
        }
    }
    *mean = n ? (int)(sum / n) : 0;
    *clip = n ? nclip * 100 / n : 0;
}

// ---------------------------------------------------------------- handlers
// Live picture for the setup page. Does not touch the flash LED.
static esp_err_t spic_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    httpd_resp_set_hdr(req, "Connection", "close");
    if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return r;
}

// Numbers for the boxes drawn on the page.
static esp_err_t stats_handler(httpd_req_t *req) {
    char q[220] = {0};
    httpd_req_get_url_query_str(req, q, sizeof q);
    rect_t wr = { qint(q, "wx", 0), qint(q, "wy", 0), qint(q, "ww", 0), qint(q, "wh", 0) };
    rect_t gr = { qint(q, "gx", 0), qint(q, "gy", 0), qint(q, "gw", 0), qint(q, "gh", 0) };
    rect_t cr = { qint(q, "cx", 0), qint(q, "cy", 0), qint(q, "cw", 0), qint(q, "ch", 0) };

    httpd_resp_set_hdr(req, "Connection", "close");
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
    int W = fb->width, H = fb->height;

    uint8_t *rgb = heap_caps_malloc((size_t)W * H * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rgb) {
        esp_camera_fb_return(fb);
        ESP_LOGE(TAG, "no memory for RGB buffer");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
    esp_camera_fb_return(fb);
    if (!ok) {
        free(rgb);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    patch_t pw = patch_stats(rgb, W, H, wr);
    patch_t pg = patch_stats(rgb, W, H, gr);
    int focus, cmean, cclip, cdark, fmean, fclip;
    container_stats(rgb, W, H, cr, &focus, &cmean, &cclip, &cdark);
    frame_stats(rgb, W, H, &fmean, &fclip);
    free(rgb);

    char out[300];
    snprintf(out, sizeof out,
             "{\"w\":{\"r\":%d,\"g\":%d,\"b\":%d,\"clip\":%d},"
             "\"g\":{\"r\":%d,\"g\":%d,\"b\":%d,\"clip\":%d},"
             "\"c\":{\"focus\":%d,\"mean\":%d,\"clip\":%d,\"dark\":%d},"
             "\"f\":{\"mean\":%d,\"clip\":%d}}",
             pw.r, pw.g, pw.b, pw.clip,
             pg.r, pg.g, pg.b, pg.clip,
             focus, cmean, cclip, cdark,
             fmean, fclip);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

// Steady flash LED on/off (stays on until switched off).
static esp_err_t flash_handler(httpd_req_t *req) {
    char q[32] = {0};
    httpd_req_get_url_query_str(req, q, sizeof q);
    camera_flash(qint(q, "on", 0) != 0);
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
}

// Lock the automatic camera controls and set exposure / gain by hand.
static esp_err_t cam_handler(httpd_req_t *req) {
    char q[64] = {0};
    httpd_req_get_url_query_str(req, q, sizeof q);
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0);
        s->set_aec2(s, 0);
        s->set_gain_ctrl(s, 0);
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_aec_value(s, qint(q, "aec", 300));
        s->set_agc_gain(s, qint(q, "gain", 5));
    }
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------- the page
static const char PAGE[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=700'><title>Camera setup</title><style>"
"body{background:#111;color:#eee;font-family:sans-serif;margin:8px}"
"#box{position:relative;width:640px;height:480px;float:left;margin-right:12px}"
"#cam,#ov{position:absolute;left:0;top:0;width:640px;height:480px}"
"#ov{cursor:crosshair;touch-action:none}"
"#side{float:left;min-width:360px;font-family:monospace;font-size:13px;line-height:1.5}"
"button,label{margin:2px 8px 2px 0}"
".ok{color:#6f6}.bad{color:#f66}"
"</style></head><body>"
"<h3>Camera setup tool</h3>"
"<div style='margin-bottom:6px'>Draw a box on the picture for: "
"<label><input type='radio' name='m' value='w' checked> WHITE patch</label>"
"<label><input type='radio' name='m' value='g'> GRAY patch</label>"
"<label><input type='radio' name='m' value='c'> CONTAINER (all 9 tiles)</label></div>"
"<div style='margin-bottom:6px'>"
"<button id='f1'>Flash ON</button><button id='f0'>Flash OFF</button>"
"<button id='pz'>Pause</button><button id='rb'>Reset best focus</button>"
"<label>ROI size <input id='roi' type='range' min='30' max='90' value='55'></label>"
"<label>Exposure <input id='ae' type='range' min='0' max='1200' value='300'></label>"
"<label>Gain <input id='gn' type='range' min='0' max='30' value='5'></label>"
"</div>"
"<div id='box'><img id='cam'><canvas id='ov' width='640' height='480'></canvas></div>"
"<div id='side'><div id='st'>Draw the boxes first.</div><hr><div id='co'></div></div>"
"<script>"
"var S=2,mode='w',R={w:null,g:null,c:null},drag=null,pause=false,best=0;"
"var img=document.getElementById('cam'),cv=document.getElementById('ov'),cx=cv.getContext('2d');"
"var st=document.getElementById('st'),co=document.getElementById('co');"
"var roi=document.getElementById('roi'),ae=document.getElementById('ae'),gn=document.getElementById('gn');"
"var rs=document.querySelectorAll('input[name=m]');"
"for(var i=0;i<rs.length;i++){rs[i].onclick=function(){mode=this.value;};}"
"document.getElementById('f1').onclick=function(){fetch('/flash?on=1');};"
"document.getElementById('f0').onclick=function(){fetch('/flash?on=0');};"
"document.getElementById('rb').onclick=function(){best=0;};"
"document.getElementById('pz').onclick=function(){pause=!pause;this.textContent=pause?'Resume':'Pause';if(!pause)next();};"
"function applyCam(){fetch('/cam?aec='+ae.value+'&gain='+gn.value);}"
"ae.onchange=applyCam;gn.onchange=applyCam;applyCam();"
"function pos(e){var r=cv.getBoundingClientRect();return {x:e.clientX-r.left,y:e.clientY-r.top};}"
"cv.onpointerdown=function(e){var p=pos(e);drag={x:p.x,y:p.y,x2:p.x,y2:p.y};cv.setPointerCapture(e.pointerId);};"
"cv.onpointermove=function(e){if(!drag)return;var p=pos(e);drag.x2=p.x;drag.y2=p.y;draw();};"
"cv.onpointercancel=function(){drag=null;draw();};"
"cv.onpointerup=function(){if(!drag)return;"
"var x=Math.min(drag.x,drag.x2)/S,y=Math.min(drag.y,drag.y2)/S,w=Math.abs(drag.x2-drag.x)/S,h=Math.abs(drag.y2-drag.y)/S;"
"drag=null;if(w>=4&&h>=4){R[mode]={x:Math.round(x),y:Math.round(y),w:Math.round(w),h:Math.round(h)};}draw();};"
"function box(r,col){cx.strokeStyle=col;cx.strokeRect(r.x*S,r.y*S,r.w*S,r.h*S);}"
"function cells(){var c=R.c,L=[];if(!c)return L;var f=roi.value/100;"
"for(var j=0;j<3;j++)for(var i=0;i<3;i++){var cw=c.w/3,ch=c.h/3,w=cw*f,h=ch*f;"
"L.push({x:Math.round(c.x+i*cw+(cw-w)/2),y:Math.round(c.y+j*ch+(ch-h)/2),w:Math.round(w),h:Math.round(h)});}"
"return L;}"
"function r4(r){return '{'+r.x+','+r.y+','+r.w+','+r.h+'}';}"
"function info(){var t='320x240 coordinates {x,y,w,h}:<br>';"
"if(R.w)t+='WHITE_PATCH '+r4(R.w)+'<br>';"
"if(R.g)t+='GRAY_PATCH '+r4(R.g)+'<br>';"
"if(R.c){t+='CONTAINER '+r4(R.c)+'<br>ROI '+roi.value+'%<br>';var L=cells();"
"for(var n=0;n<L.length;n++)t+='CELL'+(n+1)+' '+r4(L[n])+'<br>';}"
"co.innerHTML=t;}"
"function draw(){cx.clearRect(0,0,640,480);cx.lineWidth=1;cx.strokeStyle='rgba(255,255,255,0.4)';"
"cx.beginPath();cx.moveTo(320,0);cx.lineTo(320,480);cx.moveTo(0,240);cx.lineTo(640,240);cx.stroke();"
"cx.lineWidth=2;if(R.w)box(R.w,'#0ff');if(R.g)box(R.g,'#fd0');if(R.c)box(R.c,'#0f0');"
"if(R.c){var c=R.c;cx.lineWidth=1;cx.strokeStyle='#0f0';"
"for(var k=1;k<3;k++){cx.beginPath();cx.moveTo((c.x+k*c.w/3)*S,c.y*S);cx.lineTo((c.x+k*c.w/3)*S,(c.y+c.h)*S);"
"cx.moveTo(c.x*S,(c.y+k*c.h/3)*S);cx.lineTo((c.x+c.w)*S,(c.y+k*c.h/3)*S);cx.stroke();}"
"var L=cells();for(var n=0;n<L.length;n++)box(L[n],'#f0f');}"
"if(drag){cx.lineWidth=1;cx.strokeStyle='#fff';cx.strokeRect(drag.x,drag.y,drag.x2-drag.x,drag.y2-drag.y);}"
"info();}"
"roi.oninput=draw;draw();"
"function next(){if(!pause)img.src='/spic?t='+Date.now();}"
"img.onload=function(){setTimeout(next,60);};img.onerror=function(){setTimeout(next,500);};next();"
"function q(k,r){return r?(k+'x='+r.x+'&'+k+'y='+r.y+'&'+k+'w='+r.w+'&'+k+'h='+r.h):'';}"
"function tag(c,t){return '<span class='+(c?'ok':'bad')+'>'+t+'</span><br>';}"
"function show(j){var t='Exposure '+ae.value+'  Gain '+gn.value+'<br><br>';"
"if(R.w){var w=j.w,mn=Math.min(w.r,w.g,w.b),mx=Math.max(w.r,w.g,w.b);"
"t+='WHITE  R'+w.r+' G'+w.g+' B'+w.b+'  clip '+w.clip+'%<br>';"
"t+=tag(mn>=180&&mx<=245&&w.clip<=2,'  want all 180-245, clip 2% or less');}"
"if(R.g){var g=j.g,gmin=Math.min(g.r,g.g,g.b),gmax=Math.max(g.r,g.g,g.b);"
"t+='GRAY   R'+g.r+' G'+g.g+' B'+g.b+'  clip '+g.clip+'%<br>';var okg=gmin>=90&&gmax<=210;"
"if(R.w){okg=okg&&(((j.w.r+j.w.g+j.w.b)-(g.r+g.g+g.b))/3>=30);}"
"t+=tag(okg,'  want 90-210 and 30+ darker than white');}"
"if(R.c){var c=j.c;if(c.focus>best)best=c.focus;"
"t+='<br>CONTAINER focus '+(c.focus/10).toFixed(1)+'  best '+(best/10).toFixed(1)+'<br>';"
"t+='  mean '+c.mean+'  clip '+c.clip+'%  dark '+c.dark+'%<br>';"
"t+=tag(best>0&&c.focus>=0.9*best,'  focus: turn the lens until this is near best');}"
"t+='<br>FRAME  mean '+j.f.mean+'  clip '+j.f.clip+'%<br>';st.innerHTML=t;}"
"function poll(){if(pause)return;var p=[q('w',R.w),q('g',R.g),q('c',R.c)].filter(function(s){return s;});"
"if(!p.length)return;fetch('/stats?'+p.join('&')).then(function(r){return r.json();}).then(show).catch(function(){});}"
"setInterval(poll,1000);"
"</script></body></html>";

static esp_err_t page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------- registration
void setup_register(httpd_handle_t server) {
    static const httpd_uri_t uris[] = {
        { .uri = "/setup", .method = HTTP_GET, .handler = page_handler,  .user_ctx = NULL },
        { .uri = "/spic",  .method = HTTP_GET, .handler = spic_handler,  .user_ctx = NULL },
        { .uri = "/stats", .method = HTTP_GET, .handler = stats_handler, .user_ctx = NULL },
        { .uri = "/flash", .method = HTTP_GET, .handler = flash_handler, .user_ctx = NULL },
        { .uri = "/cam",   .method = HTTP_GET, .handler = cam_handler,   .user_ctx = NULL },
    };
    for (size_t i = 0; i < sizeof uris / sizeof uris[0]; i++) {
        esp_err_t e = httpd_register_uri_handler(server, &uris[i]);
        if (e != ESP_OK) ESP_LOGE(TAG, "failed to register %s (%s)", uris[i].uri, esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "setup tool ready: http://192.168.4.1/setup");
}

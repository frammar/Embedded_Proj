#include "web.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "camera.h"
#include "esp_timer.h"
#include "setup.h"
#include "vision.h"
#include "app.h"

#define AP_SSID "Factory4"
#define AP_PASS "factory4pass"      // must be at least 8 characters

static const char *TAG = "web";

static const char PAGE[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Factory 4</title>"
"<style>"
"body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:12px}"
"#wrap{position:relative;display:inline-block;width:640px;max-width:95vw}"
"#cam{width:100%;display:block}"
"#grid{position:absolute;top:0;left:0;right:0;bottom:0;display:grid;"
"grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,1fr)}"
"#grid div{border:2px solid #0f0;box-sizing:border-box;color:#0f0;"
"font-weight:bold;padding:2px;text-align:left}"
"</style></head><body>"
"<h2>Factory 4 - Camera</h2>"
"<div id='wrap'><img id='cam' src='/photo'><div id='grid'></div></div>"
"<p><button onclick='flash=!flash'>Toggle flash</button> <span id='fps'></span></p>"
"<script>"
"var g=document.getElementById('grid');"
"for(var i=1;i<=9;i++){var d=document.createElement('div');d.textContent=i;g.appendChild(d);}"
"var flash=false,img=document.getElementById('cam'),n=0,t0=Date.now();"
"function next(){img.src=(flash?'/photof':'/photo')+'?t='+Date.now();}"
"img.onload=function(){n++;"
"if(n%10==0){document.getElementById('fps').textContent=(10000/(Date.now()-t0)).toFixed(1)+' fps';t0=Date.now();}"
"setTimeout(next,50);};"
"img.onerror=function(){setTimeout(next,500);};"
"</script></body></html>";

static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_photo(httpd_req_t *req, bool flash) {
    int64_t t0 = esp_timer_get_time();
    camera_fb_t *fb = camera_capture(flash);
    int64_t t1 = esp_timer_get_time();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t len = fb->len;
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, len);
    esp_camera_fb_return(fb);
    int64_t t2 = esp_timer_get_time();
    ESP_LOGI(TAG, "capture %d ms, send %d ms, %u bytes",
             (int)((t1 - t0) / 1000), (int)((t2 - t1) / 1000), (unsigned)len);
    return r;
}

static esp_err_t photo_handler(httpd_req_t *req)  { return send_photo(req, false); }
static esp_err_t photof_handler(httpd_req_t *req) { return send_photo(req, true);  }

static void wifi_start_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = { 0 };
    strcpy((char *)ap.ap.ssid, AP_SSID);
    ap.ap.ssid_len = strlen(AP_SSID);
    strcpy((char *)ap.ap.password, AP_PASS);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);   // no Wi-Fi power saving: faster response
}

void web_start(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    wifi_start_ap();

    httpd_handle_t server = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;   // when slots are full, drop the oldest idle connection
    cfg.recv_wait_timeout = 2;     // idle connections get cut after 2 s, not 5 s
    cfg.send_wait_timeout = 2;
    cfg.max_uri_handlers = 24;
    cfg.stack_size = 8192;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));

    setup_register(server);
    vision_start(server);
    app_register(server);

    httpd_uri_t root   = { .uri = "/",       .method = HTTP_GET, .handler = root_handler   };
    httpd_uri_t photo  = { .uri = "/photo",  .method = HTTP_GET, .handler = photo_handler  };
    httpd_uri_t photof = { .uri = "/photof", .method = HTTP_GET, .handler = photof_handler };
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &photo);
    httpd_register_uri_handler(server, &photof);

    ESP_LOGI(TAG, "Wi-Fi '%s' ready. Open http://192.168.4.1/", AP_SSID);
}
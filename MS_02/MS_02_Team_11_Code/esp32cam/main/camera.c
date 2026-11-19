#include "camera.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define FLASH_LED GPIO_NUM_4
static const char *TAG = "camera";

// AI-Thinker ESP32-CAM pin map
static camera_config_t config = {
    .pin_pwdn = 32, .pin_reset = -1, .pin_xclk = 0,
    .pin_sccb_sda = 26, .pin_sccb_scl = 27,
    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,         // 320x240
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

esp_err_t camera_start(void) {
    gpio_reset_pin(FLASH_LED);
    gpio_set_direction(FLASH_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(FLASH_LED, 0);

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) ESP_LOGE(TAG, "camera init failed: 0x%x", err);
    return err;
}

void camera_flash(bool on) {
    gpio_set_level(FLASH_LED, on ? 1 : 0);
}

camera_fb_t *camera_capture(bool with_flash) {
    if (with_flash) {
        camera_flash(true);
        vTaskDelay(pdMS_TO_TICKS(150));       // let the light and exposure settle
    }
    camera_fb_t *old = esp_camera_fb_get();   // throw away a possibly stale frame
    if (old) esp_camera_fb_return(old);

    camera_fb_t *fb = esp_camera_fb_get();    // the fresh frame
    if (with_flash) camera_flash(false);
    return fb;
}
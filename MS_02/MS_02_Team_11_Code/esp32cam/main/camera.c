#include "camera.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"

#define FLASH_DUTY_PCT   10     // try 5, 10, 25 until the photo looks clean
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
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

static void flash_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_1,      // the camera clock uses timer 0
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = FLASH_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,         // the camera clock uses channel 0
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&c);
}

esp_err_t camera_start(void) {
    flash_init();                      // replaces the 3 old gpio lines

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) ESP_LOGE(TAG, "camera init failed: 0x%x", err);
    return err;
}

void camera_flash(bool on)
{
    uint32_t duty = on ? (255 * FLASH_DUTY_PCT / 100) : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

camera_fb_t *camera_capture(bool with_flash) {
    if (with_flash) {
        camera_flash(true);
        vTaskDelay(pdMS_TO_TICKS(250));       // let the light and exposure settle
        for (int i = 0; i < 2; i++) {         // throw away frames taken before the light was fully on
            camera_fb_t *old = esp_camera_fb_get();
            if (old) esp_camera_fb_return(old);
        }
    }
    camera_fb_t *fb = esp_camera_fb_get();    // the newest frame
    if (with_flash) camera_flash(false);
    return fb;
}
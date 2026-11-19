#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "camera.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(camera_start());

    while (1) {
        camera_fb_t *fb = camera_capture(true);
        if (fb) {
            ESP_LOGI(TAG, "photo: %u bytes, %dx%d", (unsigned)fb->len, fb->width, fb->height);
            esp_camera_fb_return(fb);
        } else {
            ESP_LOGE(TAG, "capture failed");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
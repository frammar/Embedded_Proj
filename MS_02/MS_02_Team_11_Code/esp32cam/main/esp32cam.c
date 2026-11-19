#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera.h"
#include "web.h"

void app_main(void)
{
    ESP_ERROR_CHECK(camera_start());
    web_start();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));   // the web server runs in its own task
    }
}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define RED_LED GPIO_NUM_33   // small red LED on the back, active LOW (0 = on)

void app_main(void)
{
    gpio_reset_pin(RED_LED);
    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(RED_LED, 0);   // on
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(RED_LED, 1);   // off
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
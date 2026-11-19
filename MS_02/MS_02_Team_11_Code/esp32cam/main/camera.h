#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "esp_camera.h"

esp_err_t camera_start(void);                 // start the OV2640
void camera_flash(bool on);                   // the bright white LED (GPIO4)
camera_fb_t *camera_capture(bool with_flash); // caller must call esp_camera_fb_return(fb)
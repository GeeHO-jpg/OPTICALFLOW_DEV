#pragma once


#include "../camera/driver/esp_camera.h"
#include "err.h"
#include "log.h"

#ifdef __cplusplus
extern "C" {
#endif
ovcam_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz);

ovcam_err_t camera_enable_out_clock(const camera_config_t *config);

void camera_disable_out_clock(void);

#ifdef __cplusplus
}
#endif
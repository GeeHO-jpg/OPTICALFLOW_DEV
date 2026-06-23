
#if defined(ESP_PLATFORM) || (defined(ARDUINO) && defined(ARDUINO_ARCH_ESP32))
// ===== ESP-IDF / Arduino-ESP32 implementation =====

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_err.h"
#include "../camera/hal/xclk.h"

// static const char* TAG = "camera_xclk";
#define NO_CAMERA_LEDC_CHANNEL 0xFF
static ledc_channel_t g_ledc_channel = NO_CAMERA_LEDC_CHANNEL;

// #if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
// #include "esp32-hal-log.h"
// #else
// #include "esp_log.h"
// static const char* TAG = "camera_xclk";
// #endif

static const char* TAG = "camera_xclk";
static ledc_mode_t g_ledc_mode = LEDC_LOW_SPEED_MODE;
// map esp_err_t -> ovcam_err_t
// static inline ovcam_err_t map_esp(esp_err_t e) { return (e == ESP_OK) ? OVCAM_OK : OVCAM_E_FAIL; }

ovcam_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz)
{
    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
    timer_conf.freq_hz = xclk_freq_hz;
    timer_conf.speed_mode = g_ledc_mode;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)   
    timer_conf.deconfigure = false;
#endif
   
#if ESP_IDF_VERSION_MAJOR >= 4
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
#endif
    timer_conf.timer_num = (ledc_timer_t)ledc_timer;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "ledc_timer_config failed for freq %d, rc=%x", xclk_freq_hz, err);
        SCCB_LOGE(TAG, "ledc_timer_config failed for freq %d, rc=%x", xclk_freq_hz, err);
    }
    return ERR_MAP_ESP(err);
}


ovcam_err_t camera_enable_out_clock(const camera_config_t* config)
{
    ovcam_err_t err = xclk_timer_conf(config->ledc_timer, config->xclk_freq_hz);
    if (err != OVCAM_OK) {
        // ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        SCCB_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        return err;
    }

    g_ledc_channel = config->ledc_channel;
    ledc_channel_config_t ch_conf = {0};
    ch_conf.gpio_num = config->pin_xclk;
    ch_conf.speed_mode = g_ledc_mode;
    ch_conf.channel = config->ledc_channel;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = config->ledc_timer;
    ch_conf.duty = 1;
    ch_conf.hpoint = 0;
    esp_err_t err2 = ledc_channel_config(&ch_conf);
    if (err2 != ESP_OK) {
        // ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err2);
        SCCB_LOGE(TAG, "ledc_channel_config failed, rc=%x", err2);
        return ERR_MAP_ESP(err2);
    }
    return OVCAM_OK;
}

void camera_disable_out_clock(void)
{
    if (g_ledc_channel != NO_CAMERA_LEDC_CHANNEL) {
        ledc_stop(g_ledc_mode, g_ledc_channel, 0);  // speed_mode ต้องตรงกัน
        g_ledc_channel = NO_CAMERA_LEDC_CHANNEL;
    }
}

#endif
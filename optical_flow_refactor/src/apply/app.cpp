#include "app.h"

#include <Arduino.h>
#include <ovcam.h>
#include "no_warnings.h"
#include "mavlink_data.h"
#include "../protocal/mavlink/MAVLink.h"
#include "settings.h"
#include "flow.h"
#include "rtos.h"
// #include "../camera/camera_sensor_registry.h"

extern const resolution_info_t resolution[];


static const char* k_fs_names[FRAMESIZE_INVALID] = {
  "96X96", "QQVGA", "128X128", "QCIF", "HQVGA", "240X240",
  "QVGA", "320X320", "CIF", "HVGA", "VGA", "SVGA",
  "XGA", "HD", "SXGA", "UXGA",
  "FHD", "P_HD", "P_3MP", "QXGA",
  "QHD", "WQXGA", "P_FHD", "QSXGA", "5MP"
};

static void print_resolution_table() {
  printf("resolution[] base address = %p\n", (void*)resolution);
  for (int i = 0; i < (int)FRAMESIZE_INVALID; ++i) { (void)k_fs_names[i]; (void)resolution[i]; }
}

// ---------- ปรับตั้งกล้อง ----------
camera_config_t camera_config = {
  .pin_pwdn     = CAM_PIN_PWDN,
  .pin_reset    = CAM_PIN_RESET,
  .pin_xclk     = CAM_PIN_XCLK,
  .pin_sccb_sda = CAM_PIN_SIOD,
  .pin_sccb_scl = CAM_PIN_SIOC,

  .pin_d7   = CAM_PIN_D7, .pin_d6   = CAM_PIN_D6, .pin_d5   = CAM_PIN_D5,
  .pin_d4   = CAM_PIN_D4, .pin_d3   = CAM_PIN_D3, .pin_d2   = CAM_PIN_D2,
  .pin_d1   = CAM_PIN_D1, .pin_d0   = CAM_PIN_D0,
  .pin_vsync= CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,

  .xclk_freq_hz = 20000000,
  .ledc_timer   = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_GRAYSCALE,
  .frame_size   = FRAMESIZE_QVGA, // 720pแนวนอน FRAMESIZE_HD,600FRAMESIZE_VGA
  .jpeg_quality = 15,                  // << เริ่มสูงขึ้นเพื่อลด bytes/เฟรม
  .fb_count     = 2,
  .fb_location  = CAMERA_FB_IN_DRAM ,
  .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
  .sccb_i2c_port= I2C_NUM_1,
};


static void print_all_params(void)
{
    printf("\r\n=== PARAM DEFAULTS (name + value) ===\r\n");
    for (int i = 0; i < ONBOARD_PARAM_COUNT; i++) {
        printf("[%02d] %-24s = %.6f\r\n",
               i,
               global_data.param_name[i],
               (double)global_data.param[i]);
    }
    printf("=== END ===\r\n\r\n");
}

void capture_init(void){
    sccb_log_set_level(SCCB_LOG_INFO);
    print_resolution_table();
    if (esp_camera_init(&camera_config) != OVCAM_OK) {
        printf("[E] Camera init failed\n"); while(1) delay(1000);
    }
}


void Lidar_init(void){
    lidar_init();
    
}


void appInit(void){

    /* load settings and parameters */
    global_data_reset_param_defaults();
  print_all_params();
	// global_data_reset();
	
	/* init led */
  // led_step_init();

  /* camera */
  capture_init();

  /* Lidar */
  // Lidar_init();

  mav_init(&Serial2); // MAVLink over Serial2 (RX=47, TX=21)

  rtos_init();

}

void appRun(void){

}

#include "rtos.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <math.h>
#include <string.h>

#include <ovcam.h>

#include "lidar.h"
#include "../distance_sensor/SF30C/SF30C.h"
#include "../protocal/mavlink/MAVLink.h"

#include "frame_timing.h"
#include "drop_down_image_scale.h"
#include "settings.h"
#include "no_warnings.h"
#include "flow.h"
#include "mavlink_data.h"


// #include "crc32.h"
#include "../protocal/packet/image_sender.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


/*
    CAMERA
*/
extern camera_config_t camera_config;
volatile uint32_t time_between_images;



/*
        LIDAR
*/
float dist_m;

// static const uint16_t LIDAR_MIN_CM = 20;   // ปรับตามสเปกจริง
// static const uint16_t LIDAR_MAX_CM = 800;  // ปรับตามสเปกจริง



/*
    OPTICAL_FLOW
*/





/*
    MAV_LINK
*/

#define SYS_ID 0
#define COMP_ID_0 0
#define COMP_ID_1 1
#define SENSOR_ID 0
#define LOOP10Hz 100000 //us

#define FLOW_PROC_W BOTTOM_FLOW_IMAGE_WIDTH
#define FLOW_PROC_H BOTTOM_FLOW_IMAGE_HEIGHT

#define DEBUG_FLOW_PRINT_INTERVAL_US 200000

static SemaphoreHandle_t dist_mtx = nullptr;
static SemaphoreHandle_t g_pyq = nullptr;

static volatile uint16_t g_dist_cm = 0;
static volatile float    g_dist_m  = 0.0f;
static volatile uint32_t g_last_seq = 0;

static volatile  bool send_img = false;
// static const uint8_t PACKETHEADER_SIGNATURE[4] = {'R', 'C', 'S', 'A'};

struct diff_pix_t{
  float dx;
  float dy;
  float Vx;
  float Vy;
  uint8_t qual;
  uint16_t dist_cm; 
};


/*
    prototype_function
*/

// void LiDAR_task(void*);
// void capture_task(void*);
void process(void*);
void lidar_task(void*);

/* test q and mutex */
// void producer_task(void *arg);

/*
    HELPER_FUNCTION
*/


static inline int clamp_i32(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static inline gainceiling_t gain_max_to_ceiling(int gain_max)
{
    if (gain_max <= 2) return (gainceiling_t)GAINCEILING_2X;
    if (gain_max <= 4) return (gainceiling_t)GAINCEILING_4X;
    if (gain_max <= 8) return (gainceiling_t)GAINCEILING_8X;
    if (gain_max <= 16) return (gainceiling_t)GAINCEILING_16X;
    if (gain_max <= 32) return (gainceiling_t)GAINCEILING_32X;
    return (gainceiling_t)GAINCEILING_64X;
}



static uint16_t u16le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t u32le(const uint8_t *p) {
  return  (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}





/*
===========================================================================================
*/

void rtos_init(){

    dist_mtx = xSemaphoreCreateMutex();
    g_pyq = xSemaphoreCreateMutex();

    char img_ip[16];
    snprintf(img_ip, sizeof(img_ip), "%d.%d.%d.%d",
             PYTHON_PC_IP_1, PYTHON_PC_IP_2, PYTHON_PC_IP_3, PYTHON_PC_IP_4);
    img_sender_init(img_ip, PYTHON_IMG_RX_PORT);

    xTaskCreatePinnedToCore(process,         "process",  16384, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(lidar_task,      "LiDAR",     2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(img_sender_task, "img_tx",    8192, NULL, 4, NULL, 0);

}


void lidar_task(void*){
  static SF30C lidar;
  lidar.begin(/*RX*/20, /*TX*/19, /*baud*/115200);

  int64_t last = 0;
  while(true){
    int64_t now = esp_timer_get_time();
    if(now - last >= 20000){ // 20,000us = 50Hz
      uint16_t d=0;
      if(lidar.getdistance_cm(d)){
        if (xSemaphoreTake(dist_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_dist_cm = d;
          g_dist_m  = d/100.0f;
          xSemaphoreGive(dist_mtx);
        }
      }
      last = now;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


void process(void *arg){

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // เราจะคุม exposure/gain เองแบบง่าย ๆ ใน loop
        // s->set_exposure_ctrl(s, 0);  // AEC off
        // s->set_gain_ctrl(s, 0);      // AGC off
        // s->set_awb_gain(s, 0);       // AWB off (กัน pipeline แกว่งบางโหมด)

        // // ค่าเริ่มต้น (เดี๋ยวใน loop จะปรับเอง)
        // s->set_aec_value(s, 500);    // ~50..1200 (ขึ้นกับบอร์ด/เซนเซอร์)
        // s->set_agc_gain(s, 10);      // ~0..30

        // // ปรับภาพนิดหน่อยเพื่อช่วย texture
        // s->set_brightness(s, 0);
        // s->set_contrast(s, 1);

        s->set_gain_ctrl(s, FLOAT_AS_BOOL(global_data.param[PARAM_AGC]) ? 1 : 0);
        s->set_exposure_ctrl(s, FLOAT_AS_BOOL(global_data.param[PARAM_AEC]) ? 1 : 0);
        s->set_aec2(s, 1);             // ช่วยให้ AE นิ่งขึ้นในบางรุ่น
        s->set_ae_level(s, -1);        // ดันให้ภาพมืดลงเล็กน้อย (กันคลิป)
        s->set_awb_gain(s, 1);         // เปิด awb_gain กลับมาก่อน
        s->set_gainceiling(s, gain_max_to_ceiling((int)global_data.param[PARAM_GAIN_MAX]));
        s->set_contrast(s, clamp_i32((int)global_data.param[PARAM_CAM_CONTRAST], -2, 2));
        s->set_sharpness(s, clamp_i32((int)global_data.param[PARAM_CAM_SHARPNESS], -2, 2));
        // s->set_vflip(s, 1);
    }
    camera_fb_t *fb0 = esp_camera_fb_get();
    if (fb0) {
        printf("camera_fb: %u x %u len=%u fmt=%d\n",
               (unsigned)fb0->width,
               (unsigned)fb0->height,
               (unsigned)fb0->len,
               (int)fb0->format);
        esp_camera_fb_return(fb0);
    }

    printf("frame_size: %d x %d\n", (int)global_data.param[PARAM_IMAGE_WIDTH],(int)global_data.param[PARAM_IMAGE_HEIGHT]);


    /* variables */
	uint32_t counter = 0;
	uint8_t qual = 0;

	/* bottom flow variables */
	float pixel_flow_x = 0.0f;
	float pixel_flow_y = 0.0f;
	float pixel_flow_x_sum = 0.0f;
	float pixel_flow_y_sum = 0.0f;
	float velocity_x_sum = 0.0f;
	float velocity_y_sum = 0.0f;
	float velocity_x_lp = 0.0f;
	float velocity_y_lp = 0.0f;
	int valid_frame_count = 0;
	int pixel_flow_count = 0;

	static float accumulated_flow_x = 0;
	static float accumulated_flow_y = 0;
	static float accumulated_gyro_x = 0;
	static float accumulated_gyro_y = 0;
	static float accumulated_gyro_z = 0;
	static uint16_t accumulated_framecount = 0;
	static uint16_t accumulated_quality = 0;
	static uint32_t integration_timespan = 0;
	static uint32_t lasttime = 0;
	uint32_t time_since_last_sonar_update= 0;
	uint32_t time_last_pub= 0;

    const size_t image_size = (size_t)FLOW_PROC_W * (size_t)FLOW_PROC_H;

    static uint8_t *img_cur  = (uint8_t*)heap_caps_malloc(image_size, MALLOC_CAP_8BIT);
    static uint8_t *img_prev = (uint8_t*)heap_caps_malloc(image_size, MALLOC_CAP_8BIT);
    static uint8_t  img_udp[IMG_TX_W * IMG_TX_H];  // downscaled image for UDP tx
    if (!img_cur || !img_prev) {
        printf("[IMG] buffer alloc failed img_cur=%p img_prev=%p size=%u\n",
               (void*)img_cur, (void*)img_prev, (unsigned)image_size);
        vTaskDelete(NULL);
        return;
    }
    bool have_prev = false;

    // คำนวณ focal_length_px ครั้งเดียว (ใช้กับ flow->rad หรือ gyro comp)
    float hfov_deg = global_data.param[PARAM_HFOV_DEG];   // เช่น 66
    float hfov_rad = hfov_deg * (float)M_PI / 180.0f;
    float focal_length_px = ((float)FLOW_PROC_W * 0.5f) / tanf(hfov_rad * 0.5f);

    // ตัวกรองระยะ (optional)
    // float dist_f = 0.0f;

    // c->dist_m = 0.0f;
    bool distance_valid = true; // ถ้าใช้ตัวกรองระยะ อาจมีช่วงเวลาที่ระยะไม่ valid (เช่น ตอนเปลี่ยนฉาก) เราจะได้ไม่ต้องเชื่อ flow ในช่วงนั้น
    TickType_t last = xTaskGetTickCount();

    float x_rate,y_rate,z_rate;
    int64_t time_now_us;

    static int64_t last_pub_us = 0;
    static int64_t last_print_us = 0;
    static int64_t last_flow_debug_us = 0;
    static int64_t t0 = 0;
    static int64_t t1 = 0;
    static int64_t t2 = 0;
    static uint32_t cam_tune_counter = 0;
    static uint32_t bad_fb_count = 0;

    static diff_pix_t dV;
    static uint64_t frame_id = 0;


    
    // static PyPkt local_pkt;
    
    for (;;) {

        t0 = esp_timer_get_time();
        camera_fb_t *fb = esp_camera_fb_get();

        if (!fb) {
            vTaskDelay(1);
            continue;
        }

        const size_t need_len = (size_t)fb->width * (size_t)fb->height;
        if (!fb->buf || fb->len < need_len || fb->format != PIXFORMAT_GRAYSCALE) {
            if ((++bad_fb_count % 20U) == 1U) {
                printf("[CAM] drop fb fmt=%d len=%u need=%u w=%u h=%u\n",
                       (int)fb->format,
                       (unsigned)fb->len,
                       (unsigned)need_len,
                       (unsigned)fb->width,
                       (unsigned)fb->height);
            }
            esp_camera_fb_return(fb);
            vTaskDelay(1);
            continue;
        }

        downscale(img_cur, FLOW_PROC_W, FLOW_PROC_H, fb->buf, fb->width, fb->height);
        
        t1 = esp_timer_get_time();
        time_between_images = (uint32_t)(t1 - t0);
        
        // printf("so\n");
         
        time_now_us = esp_timer_get_time();
        
    
        uint16_t dist_cm = g_dist_cm;
        float dist_m = g_dist_m;





        // if (xSemaphoreTake(dist_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
        // dist_cm = g_dist_cm;
        // dist_m  = g_dist_m;
        // xSemaphoreGive(dist_mtx);
        // }
        distance_valid = (dist_cm >= 1 && dist_cm <= 10000 && dist_m > 0.0f);

        if (s) {
            int exp_max = clamp_i32((int)global_data.param[PARAM_EXPOSURE_MAX], 50, 1200);
            int gain_max = clamp_i32((int)global_data.param[PARAM_GAIN_MAX], 0, 64);
            int tune_div = clamp_i32((int)global_data.param[PARAM_CAM_TUNE_DIV], 1, 30);
            bool should_tune = ((++cam_tune_counter % (uint32_t)tune_div) == 0U);

            if (should_tune) {
                bool cam_auto_tune = FLOAT_AS_BOOL(global_data.param[PARAM_CAM_AUTO_TUNE_EN]);

                if (cam_auto_tune) {
                    uint32_t sum = 0;
                    uint32_t cnt = 0;
                    const size_t luma_step = 8;
                    for (size_t i = 0; i < image_size; i += luma_step) {
                        sum += img_cur[i];
                        cnt++;
                    }

                    int mean_luma = (cnt > 0) ? (int)(sum / cnt) : 0;
                    int luma_target = clamp_i32((int)global_data.param[PARAM_CAM_LUMA_TARGET], 0, 255);
                    int luma_db = clamp_i32((int)global_data.param[PARAM_CAM_LUMA_DEADBAND], 0, 64);

                    if (mean_luma > (luma_target + luma_db)) {
                        exp_max -= 20;
                        gain_max -= 1;
                    } else if (mean_luma < (luma_target - luma_db)) {
                        exp_max += 20;
                        gain_max += 1;
                    }

                    exp_max = clamp_i32(exp_max, 50, 1200);
                    gain_max = clamp_i32(gain_max, 0, 64);

                    settings_param(&global_data, PARAM_EXPOSURE_MAX, (float)exp_max);
                    settings_param(&global_data, PARAM_GAIN_MAX, (float)gain_max);
                }

                s->set_exposure_ctrl(s, FLOAT_AS_BOOL(global_data.param[PARAM_AEC]) ? 1 : 0);
                s->set_gain_ctrl(s, FLOAT_AS_BOOL(global_data.param[PARAM_AGC]) ? 1 : 0);

                if (!FLOAT_AS_BOOL(global_data.param[PARAM_AEC])) {
                    s->set_aec_value(s, exp_max);
                }
                if (!FLOAT_AS_BOOL(global_data.param[PARAM_AGC])) {
                    s->set_agc_gain(s, gain_max);
                }

                s->set_gainceiling(s, gain_max_to_ceiling(gain_max));
                s->set_contrast(s, clamp_i32((int)global_data.param[PARAM_CAM_CONTRAST], -2, 2));
                s->set_sharpness(s, clamp_i32((int)global_data.param[PARAM_CAM_SHARPNESS], -2, 2));
            }
        }
        // Post latest downscaled image for img_sender_task to transmit at 1fps
        downscale(img_udp, IMG_TX_W, IMG_TX_H, img_cur, FLOW_PROC_W, FLOW_PROC_H);
        img_sender_update(img_udp, sizeof(img_udp));

        esp_camera_fb_return(fb);
        fb = nullptr;

        if (!have_prev) {
            memcpy(img_prev, img_cur, image_size);
            have_prev = true;
            continue;
        }



        if(FLOAT_EQ_INT(global_data.param[PARAM_SENSOR_POSITION],BOTTOM)){
            uint32_t t0 = micros();

            qual = compute_flow(
                img_prev, img_cur,
                0, 0, 0,
                &pixel_flow_x,
                &pixel_flow_y
            );

            uint32_t flow_us = micros() - t0;

            if ((time_now_us - last_flow_debug_us) >= DEBUG_FLOW_PRINT_INTERVAL_US) {
                last_flow_debug_us = time_now_us;
                printf("flow=%lu us, max_fps=%.1f\n",flow_us,1000000.0f / flow_us);
            }
            // qual = compute_flow(img_prev, img_cur, 0,0,0, &pixel_flow_x, &pixel_flow_y);
            // printf("quality:%d flow_x:%2f flow_y:%2f dist:%2f m  frame_time: %d us\n",qual,pixel_flow_x,pixel_flow_y,c->dist_m,get_time_between_images());

            const float dt_s = fmaxf(1e-6f, (float)get_time_between_images() * 1e-6f);
            const float inv_focal_dt = 1.0f / (focal_length_px * dt_s);
            float flow_compx = pixel_flow_x * inv_focal_dt;
			float flow_compy = pixel_flow_y * inv_focal_dt;
            // printf("flow_comp_x: %3f rad/sec  flow_comp_y: %3f rad/sec\n",flow_compx,flow_compy);

            const uint64_t boot_now_us = get_boot_time_us();

            if (qual > 0)
                {
                    valid_frame_count++;

					uint64_t deltatime = (boot_now_us - lasttime);
                    integration_timespan += deltatime;
                    accumulated_flow_x += pixel_flow_y  / focal_length_px * 1.0f; //rad axis swapped to align x flow around y axis
                    accumulated_flow_y += pixel_flow_x  / focal_length_px * -1.0f;//rad
                    accumulated_gyro_x += x_rate * deltatime / 1000000.0f;	//rad
                    accumulated_gyro_y += y_rate * deltatime / 1000000.0f;	//rad
                    accumulated_gyro_z += z_rate * deltatime / 1000000.0f;	//rad
                    accumulated_framecount++;
                    accumulated_quality += qual;
                }
			/* integrate velocity and output values only if distance is valid */
			if (distance_valid)
			{
				/* calc velocity (negative of flow values scaled with distance) */
                float new_velocity_x = - flow_compx * dist_m;
                float new_velocity_y = - flow_compy * dist_m;

                time_since_last_sonar_update = (boot_now_us - tfmP.get_sonar_measure_time());

				if (qual > 0)
				{
					velocity_x_sum += new_velocity_x;
					velocity_y_sum += new_velocity_y;

					/* lowpass velocity output */
					velocity_x_lp = global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW] * new_velocity_x + (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_x_lp;
					velocity_y_lp = global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW] * new_velocity_y + (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_y_lp;
				}
				else
				{
					/* taking flow as zero */
					velocity_x_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_x_lp;
					velocity_y_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_y_lp;
				}
			}
			else
			{
				/* taking flow as zero */
				velocity_x_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_x_lp;
				velocity_y_lp = (1.0f - global_data.param[PARAM_BOTTOM_FLOW_WEIGHT_NEW]) * velocity_y_lp;
			}
			//update lasttime
            lasttime = boot_now_us;

			pixel_flow_x_sum += pixel_flow_x;
			pixel_flow_y_sum += pixel_flow_y;
			pixel_flow_count++;

        }
        counter++;

        if (FLOAT_AS_BOOL(global_data.param[PARAM_USB_SEND_FLOW]))
        {
            float flow_comp_m_x = 0.0f;
			float flow_comp_m_y = 0.0f;

            if(FLOAT_AS_BOOL(global_data.param[PARAM_BOTTOM_FLOW_LP_FILTERED]))
            {
                flow_comp_m_x = velocity_x_lp;
                flow_comp_m_y = velocity_y_lp;
            }
            else
            {
                if(valid_frame_count>0)
                {
                    flow_comp_m_x = velocity_x_sum/valid_frame_count;
                    flow_comp_m_y = velocity_y_sum/valid_frame_count;
                }
                else
                {
                    flow_comp_m_x = 0.0f;
                    flow_comp_m_y = 0.0f;
                }
            }

            float q_avg = (accumulated_framecount > 0) ? (float)accumulated_quality / accumulated_framecount : 0.0f;
            static const int64_t PUB_INTERVAL_US = 20000; // 50Hz
            if((time_now_us - last_pub_us) >= PUB_INTERVAL_US){
                last_pub_us = time_now_us;
                uint8_t q_send = (uint8_t) (q_avg < 0 ? 0 : (q_avg > 255 ? 255 : q_avg));


                    float fx = pixel_flow_x_sum * 10.0f * 1.0f;
                    float fy = pixel_flow_y_sum * 10.0f * 1.0f;

                    float cx = flow_comp_m_x;
                    float cy = flow_comp_m_y;

                    const float GROUND_LOCK_DIST_M = 0.15f;
                    const float PIXEL_DEADBAND = 2.0f;   // in 0.1 px units after scaling
                    const float VEL_DEADBAND_MPS = 0.03f;
                    uint8_t qual_send = q_send;

                    if (dist_m > 0.0f && dist_m < GROUND_LOCK_DIST_M) {
                        fx = 0.0f;
                        fy = 0.0f;
                        cx = 0.0f;
                        cy = 0.0f;
                        g_dist_m = 0.0f; 
                        dist_cm = 0;
                        dist_m = 0.0f;
                        qual_send = 0;
                    } else {
                        if (fabsf(fx) < PIXEL_DEADBAND) fx = 0.0f;
                        if (fabsf(fy) < PIXEL_DEADBAND) fy = 0.0f;
                        if (fabsf(cx) < VEL_DEADBAND_MPS) cx = 0.0f;
                        if (fabsf(cy) < VEL_DEADBAND_MPS) cy = 0.0f;
                    }

                    //convert normal axis to body axis (x forward, y right)
                    float body_x = cx; //[m/s]
                    float body_y = cy; //[m/s]
                    float body_pixel_x = fx; //[pixel/0.1s]
                    float body_pixel_y = fy; //[pixel/0.1s]

                    bool ok = mav_bridge_send_optical_flow(
                        SYS_ID, COMP_ID_0,
                        (uint64_t)get_boot_time_us(),
                        (uint8_t)global_data.param[PARAM_SENSOR_ID],

                        (int16_t)body_pixel_x,     // X_new
                        (int16_t)body_pixel_y,     // Y_new

                        body_x,     // compX_new
                        body_y,     // compY_new

                        qual_send,
                        dist_m,
                        0.0f, 0.0f
                    );



                    bool dist_ok =send_lidar_distance_cm(
                        SYS_ID,COMP_ID_1,(uint32_t)millis(),1,10000,dist_cm,0
                    );

                    dV.dx = body_pixel_x;
                    dV.dy = body_pixel_y;
                    dV.Vx = body_x;
                    dV.Vy = body_y;
                    dV.qual = qual_send;
                    dV.dist_cm = dist_cm;

                    if(ok && dist_ok && (time_now_us - last_print_us) >= DEBUG_FLOW_PRINT_INTERVAL_US){
                        last_print_us = time_now_us;
                        printf("opt flow sent: pixel=(%d, %d) comp=(%.3f, %.3f) qual=%u dist=%.2fm\n",
                            (int16_t)body_pixel_x, (int16_t)body_pixel_y, body_x, body_y, qual_send, dist_m);
                    }
                t2 = esp_timer_get_time();





            }


        }



            /* variable reset*/
            integration_timespan = 0; 
            accumulated_flow_x = 0; 
            accumulated_flow_y = 0; 
            accumulated_framecount = 0; 
            accumulated_quality = 0; 
            accumulated_gyro_x = 0; 
            accumulated_gyro_y = 0; 
            accumulated_gyro_z = 0; 
            velocity_x_sum = 0.0f; 
            velocity_y_sum = 0.0f; 
            pixel_flow_x_sum = 0.0f; 
            pixel_flow_y_sum = 0.0f; 
            valid_frame_count = 0; 
            pixel_flow_count = 0;

        memcpy(img_prev, img_cur, image_size);
        

        vTaskDelay(pdMS_TO_TICKS(1));
        
    }

}



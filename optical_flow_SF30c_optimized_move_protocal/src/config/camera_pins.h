

#pragma once

// เลือกพินตามรุ่นกล้อง (ต้องนิยาม CAMERA_MODEL_* มาจาก board_config.h)
// ใส่แค่ “หนึ่งรุ่น” เท่านั้น

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM  14
#define RESET_GPIO_NUM 32
#define XCLK_GPIO_NUM  21
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    33
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    19
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    5
#define Y2_GPIO_NUM    4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

#elif defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  4
#define SIOD_GPIO_NUM  18
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM    36
#define Y8_GPIO_NUM    37
#define Y7_GPIO_NUM    38
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    35
#define Y4_GPIO_NUM    14
#define Y3_GPIO_NUM    13
#define Y2_GPIO_NUM    34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM  27
#define PCLK_GPIO_NUM  25

#define LED_GPIO_NUM 22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_V2_PSRAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  22
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_WIDE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  22
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#define LED_GPIO_NUM 2

#elif defined(CAMERA_MODEL_M5STACK_ESP32CAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_UNITCAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_CAMS3_UNIT)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 21
#define XCLK_GPIO_NUM  11
#define SIOD_GPIO_NUM  17
#define SIOC_GPIO_NUM  41

#define Y9_GPIO_NUM    13
#define Y8_GPIO_NUM    4
#define Y7_GPIO_NUM    10
#define Y6_GPIO_NUM    5
#define Y5_GPIO_NUM    7
#define Y4_GPIO_NUM    16
#define Y3_GPIO_NUM    15
#define Y2_GPIO_NUM    6
#define VSYNC_GPIO_NUM 42
#define HREF_GPIO_NUM  18
#define PCLK_GPIO_NUM  12

#define LED_GPIO_NUM 14

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM   4

#elif defined(CAMERA_MODEL_TTGO_T_JOURNAL)
#define PWDN_GPIO_NUM  0
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23

#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_XIAO_ESP32S3)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

#elif defined(CAMERA_MODEL_ESP32_CAM_BOARD)
// The 18 pin header on the board has Y5 and Y3 swapped
#define USE_BOARD_HEADER 0
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   33
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM 36
#define Y8_GPIO_NUM 19
#define Y7_GPIO_NUM 21
#define Y6_GPIO_NUM 39
#if USE_BOARD_HEADER
#define Y5_GPIO_NUM 13
#else
#define Y5_GPIO_NUM 35
#endif
#define Y4_GPIO_NUM 14
#if USE_BOARD_HEADER
#define Y3_GPIO_NUM 35
#else
#define Y3_GPIO_NUM 13
#endif
#define Y2_GPIO_NUM    34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM  27
#define PCLK_GPIO_NUM  25

#elif defined(CAMERA_MODEL_ESP32S3_CAM_LCD)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  40
#define SIOD_GPIO_NUM  17
#define SIOC_GPIO_NUM  18

#define Y9_GPIO_NUM    39
#define Y8_GPIO_NUM    41
#define Y7_GPIO_NUM    42
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    3
#define Y4_GPIO_NUM    14
#define Y3_GPIO_NUM    47
#define Y2_GPIO_NUM    13
#define VSYNC_GPIO_NUM 21
#define HREF_GPIO_NUM  38
#define PCLK_GPIO_NUM  11

#elif defined(CAMERA_MODEL_ESP32S2_CAM_BOARD)
// The 18 pin header on the board has Y5 and Y3 swapped
#define USE_BOARD_HEADER 0
#define PWDN_GPIO_NUM    1
#define RESET_GPIO_NUM   2
#define XCLK_GPIO_NUM    42
#define SIOD_GPIO_NUM    41
#define SIOC_GPIO_NUM    18

#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 39
#define Y7_GPIO_NUM 40
#define Y6_GPIO_NUM 15
#if USE_BOARD_HEADER
#define Y5_GPIO_NUM 12
#else
#define Y5_GPIO_NUM 13
#endif
#define Y4_GPIO_NUM 5
#if USE_BOARD_HEADER
#define Y3_GPIO_NUM 13
#else
#define Y3_GPIO_NUM 12
#endif
#define Y2_GPIO_NUM    14
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  4
#define PCLK_GPIO_NUM  3

#elif defined(CAMERA_MODEL_ESP32S3_EYE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#elif defined(CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3) || defined(CAMERA_MODEL_DFRobot_Romeo_ESP32S3)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  45
#define SIOD_GPIO_NUM  1
#define SIOC_GPIO_NUM  2

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    46
#define Y7_GPIO_NUM    8
#define Y6_GPIO_NUM    7
#define Y5_GPIO_NUM    4
#define Y4_GPIO_NUM    41
#define Y3_GPIO_NUM    40
#define Y2_GPIO_NUM    39
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  42
#define PCLK_GPIO_NUM  5

#else
#error "Camera model not selected"
#endif


// ===============================
// Alias: old pin names -> CAM_PIN_*
// (numbers stay exactly the same)
// ===============================
#ifndef CAM_PIN_PWDN
  #define CAM_PIN_PWDN   PWDN_GPIO_NUM
#endif
#ifndef CAM_PIN_RESET
  #define CAM_PIN_RESET  RESET_GPIO_NUM
#endif
#ifndef CAM_PIN_XCLK
  #define CAM_PIN_XCLK   XCLK_GPIO_NUM
#endif
#ifndef CAM_PIN_SIOD
  #define CAM_PIN_SIOD   SIOD_GPIO_NUM
#endif
#ifndef CAM_PIN_SIOC
  #define CAM_PIN_SIOC   SIOC_GPIO_NUM
#endif
#ifndef CAM_PIN_VSYNC
  #define CAM_PIN_VSYNC  VSYNC_GPIO_NUM
#endif
#ifndef CAM_PIN_HREF
  #define CAM_PIN_HREF   HREF_GPIO_NUM
#endif
#ifndef CAM_PIN_PCLK
  #define CAM_PIN_PCLK   PCLK_GPIO_NUM
#endif

// Y2..Y9 -> D0..D7
#ifndef CAM_PIN_D0
  #define CAM_PIN_D0     Y2_GPIO_NUM
#endif
#ifndef CAM_PIN_D1
  #define CAM_PIN_D1     Y3_GPIO_NUM
#endif
#ifndef CAM_PIN_D2
  #define CAM_PIN_D2     Y4_GPIO_NUM
#endif
#ifndef CAM_PIN_D3
  #define CAM_PIN_D3     Y5_GPIO_NUM
#endif
#ifndef CAM_PIN_D4
  #define CAM_PIN_D4     Y6_GPIO_NUM
#endif
#ifndef CAM_PIN_D5
  #define CAM_PIN_D5     Y7_GPIO_NUM
#endif
#ifndef CAM_PIN_D6
  #define CAM_PIN_D6     Y8_GPIO_NUM
#endif
#ifndef CAM_PIN_D7
  #define CAM_PIN_D7     Y9_GPIO_NUM
#endif




// /* เพิ่มรุ่นอื่น ๆ ต่อด้วย #elif defined(CAMERA_MODEL_...) ตามต้องการ */
// // WROVER-KIT PIN Map
// #if defined(CAMERA_MODEL_WROVER_KIT)

//   #define CAM_PIN_PWDN   14
//   #define CAM_PIN_RESET  32
//   #define CAM_PIN_XCLK   21
//   #define CAM_PIN_SIOD   26
//   #define CAM_PIN_SIOC   27
//   #define CAM_PIN_VSYNC  25
//   #define CAM_PIN_HREF   23
//   #define CAM_PIN_PCLK   22

//   // Y2..Y9 → D0..D7
//   #define CAM_PIN_D0      4   // Y2
//   #define CAM_PIN_D1      5   // Y3
//   #define CAM_PIN_D2     18   // Y4
//   #define CAM_PIN_D3     19   // Y5
//   #define CAM_PIN_D4     36   // Y6
//   #define CAM_PIN_D5     39   // Y7
//   #define CAM_PIN_D6     34   // Y8
//   #define CAM_PIN_D7     33   // Y9

// #elif defined(CAMERA_MODEL_AI_THINKER)

//   #define CAM_PIN_PWDN   32
//   #define CAM_PIN_RESET  -1
//   #define CAM_PIN_XCLK    0
//   #define CAM_PIN_SIOD   26
//   #define CAM_PIN_SIOC   27
//   #define CAM_PIN_VSYNC  25
//   #define CAM_PIN_HREF   23
//   #define CAM_PIN_PCLK   22

//   // Y2..Y9 → D0..D7
//   #define CAM_PIN_D0      5   // Y2
//   #define CAM_PIN_D1     18   // Y3
//   #define CAM_PIN_D2     19   // Y4
//   #define CAM_PIN_D3     21   // Y5
//   #define CAM_PIN_D4     36   // Y6
//   #define CAM_PIN_D5     39   // Y7
//   #define CAM_PIN_D6     34   // Y8
//   #define CAM_PIN_D7     35   // Y9

// // ESP32S3 (WROOM) PIN Map
// #elif defined(BOARD_ESP32S3_WROOM)
//   #define CAM_PIN_PWDN 38
//   #define CAM_PIN_RESET -1
//   #define CAM_PIN_VSYNC 6
//   #define CAM_PIN_HREF 7
//   #define CAM_PIN_PCLK 13
//   #define CAM_PIN_XCLK 15
//   #define CAM_PIN_SIOD 4
//   #define CAM_PIN_SIOC 5
//   #define CAM_PIN_D0 11
//   #define CAM_PIN_D1 9
//   #define CAM_PIN_D2 8
//   #define CAM_PIN_D3 10
//   #define CAM_PIN_D4 12
//   #define CAM_PIN_D5 18
//   #define CAM_PIN_D6 17
//   #define CAM_PIN_D7 16

// // ESP32S3 (GOOUU TECH)
// #elif defined(BOARD_ESP32S3_GOOUUU)
//   #define CAM_PIN_PWDN -1
//   #define CAM_PIN_RESET -1
//   #define CAM_PIN_VSYNC 6
//   #define CAM_PIN_HREF 7
//   #define CAM_PIN_PCLK 13
//   #define CAM_PIN_XCLK 15
//   #define CAM_PIN_SIOD 4
//   #define CAM_PIN_SIOC 5
//   #define CAM_PIN_D0 11
//   #define CAM_PIN_D1 9
//   #define CAM_PIN_D2 8
//   #define CAM_PIN_D3 10
//   #define CAM_PIN_D4 12
//   #define CAM_PIN_D5 18
//   #define CAM_PIN_D6 17
//   #define CAM_PIN_D7 16
  
// #elif defined(CAMERA_MODEL_ESP32S3_EYE)
//   #define CAM_PIN_PWDN   -1
//   #define CAM_PIN_RESET  -1
//   #define CAM_PIN_XCLK   15
//   #define CAM_PIN_SIOD   4
//   #define CAM_PIN_SIOC   5
//   #define CAM_PIN_VSYNC  6
//   #define CAM_PIN_HREF   7
//   #define CAM_PIN_PCLK   13

//   // Y2..Y9 → D0..D7
//   #define CAM_PIN_D0     11   // Y2
//   #define CAM_PIN_D1      9   // Y3
//   #define CAM_PIN_D2      8   // Y4
//   #define CAM_PIN_D3     10   // Y5
//   #define CAM_PIN_D4     12   // Y6
//   #define CAM_PIN_D5     18  // Y7
//   #define CAM_PIN_D6     17   // Y8
//   #define CAM_PIN_D7     16   // Y9

// #else
//   #error "กรุณาเลือก CAMERA_MODEL_* ใน board_config.h (กำหนดให้เหลือรุ่นเดียว)"
// #endif

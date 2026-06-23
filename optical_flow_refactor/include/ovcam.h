#pragma once

#if defined(I2Ctest)


#include "../camera/hal/sccb.h"
#include "log.h"
#include "config/board_config.h"

#if defined(OV2640)
  #include "OV2640_reg.h"
#endif

#elif defined(XCLKtest)

  #include "config/board_config.h"  // XCLK_GPIO_NUM
  #include "../camera/hal/xclk.h"           // camera_enable_out_clock()

#else
//system
// #include <WiFi.h>
// #include "esp_wifi.h"
#include "esp32-hal-psram.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// #include <lwip/ip4_addr.h>

//handmade
#include "../camera/driver/esp_camera.h"
#include "log.h"
#include "../camera/hal/xclk.h"
#include "config/board_config.h"
#include "udp.h"
#include "../protocal/packet/UDPCommands.h"
#include "../protocal/packet/UDPPacket.h"
#include "../protocal/packet/UDPPacketHeader.h"
// #include "crc32.h"
#include "task.h"
#include "config/param_conf.h"




#endif

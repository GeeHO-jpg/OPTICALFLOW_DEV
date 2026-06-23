#pragma once

#include "Arduino.h"
#include "../distance_sensor/TFMPlus/TFMPlus.h"


extern TFMPlus tfmP;

extern int16_t tfDist;
extern int16_t tfFlux;
extern int16_t tfTemp;

// static const int LIDAR_RX_PIN = 17;  // ESP32-S3 RX (รับจาก TX ของ TF-Luna)
// static const int LIDAR_TX_PIN = 18;  // ESP32-S3 TX (ส่งไป RX ของ TF-Luna)

static const int LIDAR_RX_PIN = 20;  // ESP32-S3 RX (รับจาก TX ของ TF-Luna)
static const int LIDAR_TX_PIN = 21;  // ESP32-S3 TX (ส่งไป RX ของ TF-Luna)
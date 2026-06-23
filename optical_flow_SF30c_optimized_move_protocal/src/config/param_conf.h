#pragma once


// =========================
// WiFi config
// =========================
#define WIFI_USE_STATIC_IP  1

#define WIFI_SSID_NAME      "RCSA3_WIFI"
#define WIFI_PASSWORD       "RCSA12345678"

// #define WIFI_SSID_NAME      "iPhone"
// #define WIFI_PASSWORD       "txnn12345678"

// =========================
// ESP32 static IP
// Python จะส่ง command มาที่ IP นี้
// =========================
#define ESP_IP_1            172
#define ESP_IP_2            20
#define ESP_IP_3            10
#define ESP_IP_4            6

#define GATEWAY_IP_1        172
#define GATEWAY_IP_2        20
#define GATEWAY_IP_3        10
#define GATEWAY_IP_4        1

#define SUBNET_IP_1         255
#define SUBNET_IP_2         255
#define SUBNET_IP_3         255
#define SUBNET_IP_4         240

// =========================
// Python PC IP
// ESP จะส่งภาพไปหา IP นี้
// ต้องเปลี่ยนให้ตรงกับ IP เครื่องคอมที่รัน Python
// =========================
#define PYTHON_PC_IP_1      172
#define PYTHON_PC_IP_2      20
#define PYTHON_PC_IP_3      10
#define PYTHON_PC_IP_4      5

// =========================
// Ports
// =========================

// ESP รอรับ command จาก Python
#define ESP_CMD_RX_PORT     5432

// Python รอรับภาพจาก ESP
#define PYTHON_IMG_RX_PORT  5009

#define UDP_DATA_CHUNK      1400

// // ===== UDP target =====
// #define UDP_PC_IP       "192.168.1.145"
// #define UDP_TX_PORT     5005
// #define UDP_RX_PORT     14555
// #define UDP_DATA_CHUNK  1400   

// ===== PACKET  =====
#define ID_PACKET       0
#define CMD_PACKET      1
#define print_debug_packet     1

// ===== mini head =====
#define MODE            1
#define CHUNK           2
#define TOTAL_CHUNK     2
#define FRAME_SIDE      1
#define RESERVED        2

// ===== CRC32 =====
#define CRC             1
#define CRC32_FAST      1
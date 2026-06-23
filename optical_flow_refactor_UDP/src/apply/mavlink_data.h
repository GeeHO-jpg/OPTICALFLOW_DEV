#pragma once

#include <stdint.h>

// ไม่ต้อง include Arduino/MAVLink ใน header ก็ได้ (ลดปัญหาชน C)
class HardwareSerial;

void mav_init(HardwareSerial* ser, uint32_t baud = 115200, int rx_pin = 47, int tx_pin = 21);

bool mav_bridge_send_optical_flow(
    uint8_t sysid, uint8_t compid,
    uint64_t time_usec,
    uint8_t sensor_id,
    float flow_x_0p1px, float flow_y_0p1px,
    float flow_comp_m_x, float flow_comp_m_y,
    uint8_t quality,
    float ground_distance_m,
    float flow_rate_x_rad_s, float flow_rate_y_rad_s
);
bool mav_bridge_send_optical_flow_rad(
    uint8_t sysid, uint8_t compid,
    uint64_t time_usec,
    uint8_t sensor_id,
    uint32_t integration_time_us,
    float integrated_x_rad, float integrated_y_rad,
    float gyro_x_rad, float gyro_y_rad, float gyro_z_rad,
    int16_t temperature,
    uint8_t quality,
    uint32_t time_delta_distance_us,
    float distance_m
);
bool send_lidar_distance_cm(
    uint8_t sysid, 
    uint8_t compid,  
    uint64_t time_usec, 
    uint16_t min_dist,
    uint16_t max_dist,
    uint16_t cur_dist,
    uint8_t sen_id);
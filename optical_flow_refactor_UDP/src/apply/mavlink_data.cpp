#include "mavlink_data.h"

#include <Arduino.h>
#include "../protocal/mavlink/MAVLink.h"

static HardwareSerial* s_port = nullptr;

void mav_init(HardwareSerial* ser, uint32_t baud, int rx_pin, int tx_pin)
{
    if (!ser) return;
    ser->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    s_port = ser;
}

// static bool mav_send_msg(const mavlink_message_t& msg)
// {
//     if (!s_port) return false;
//     uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//     uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//     uint16_t send = s_port->write(buf, len);
//     return (send == len);
// }
static bool mav_send_msg(const mavlink_message_t& msg, uint32_t timeout_ms = 200)
{
    if (!s_port) return false;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    uint16_t off = 0;
    uint32_t t0 = millis();
    while (off < len) {
        size_t sent = s_port->write(buf + off, len - off);
        off += (uint16_t)sent;

        if (millis() - t0 > timeout_ms) return false;
        if (sent == 0) delay(1); // ให้โอกาส UART drain
    }
    return true;
}

bool mav_bridge_send_optical_flow(
    uint8_t sysid, uint8_t compid,
    uint64_t time_usec,
    uint8_t sensor_id,
    float flow_x_0p1px, float flow_y_0p1px,
    float flow_comp_m_x, float flow_comp_m_y,
    uint8_t quality,
    float ground_distance_m,
    float flow_rate_x_rad_s, float flow_rate_y_rad_s
){
    mavlink_message_t msg;

    int16_t fx = (int16_t)lroundf(flow_x_0p1px);
    int16_t fy = (int16_t)lroundf(flow_y_0p1px);

    mavlink_msg_optical_flow_pack(
        sysid, compid, &msg,
        time_usec,
        sensor_id,
        fx, fy,
        flow_comp_m_x, flow_comp_m_y,
        quality,
        ground_distance_m,
        flow_rate_x_rad_s, flow_rate_y_rad_s
    );

    return mav_send_msg(msg);
}


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
){
    mavlink_message_t msg;

    mavlink_msg_optical_flow_rad_pack(
        sysid, compid, &msg,
        time_usec,
        sensor_id,
        integration_time_us,
        integrated_x_rad, integrated_y_rad,
        gyro_x_rad, gyro_y_rad, gyro_z_rad,
        temperature,
        quality,
        time_delta_distance_us,
        distance_m
    );

    return mav_send_msg(msg);
}

bool mav_bridge_send_distance_m(
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
){
    mavlink_message_t msg;

    mavlink_msg_optical_flow_rad_pack(
        sysid, compid, &msg,
        time_usec,
        sensor_id,
        integration_time_us,
        integrated_x_rad, integrated_y_rad,
        gyro_x_rad, gyro_y_rad, gyro_z_rad,
        temperature,
        quality,
        time_delta_distance_us,
        distance_m
    );

    return mav_send_msg(msg);
}

bool send_lidar_distance_cm(uint8_t sysid, uint8_t compid,  uint64_t time_usec,uint16_t min_dist,uint16_t max_dist,uint16_t cur_dist,uint8_t sen_id)
{
    mavlink_message_t msg;
    static const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z

    mavlink_msg_distance_sensor_pack(
        sysid,
        compid,
        &msg,
        time_usec,              // time_boot_ms
        min_dist,                    // min_distance (cm)
        max_dist,                    // max_distance (cm)
        cur_dist,                    // current_distance (cm)
        MAV_DISTANCE_SENSOR_LASER,       // type
        sen_id,                       // id (0..255)
        MAV_SENSOR_ROTATION_PITCH_270, // orientation (ถ้าหันลง)
        UINT8_MAX,                       // covariance (255 = unknown)
        0.0f,                            // horizontal_fov (ไม่ใช้ก็ 0)
        0.0f,                            // vertical_fov   (ไม่ใช้ก็ 0)
        q,                               // quaternion
        255                                // signal_quality (%), ไม่รู้ใส่ 0
    );

    return mav_send_msg(msg);
}

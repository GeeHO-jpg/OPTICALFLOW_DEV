#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

class SF30C {
private:
  HardwareSerial& ser;
  bool haveH;
  uint8_t H;
  uint32_t baud;

public:
  SF30C(HardwareSerial& s = Serial1)
  : ser(s), haveH(false), H(0), baud(115200) {}   // เปลี่ยนเป็น 921600 ได้ถ้าตั้งเซนเซอร์แล้ว

  void begin(int rx, int tx, uint32_t b = 115200) {
    baud = b;
    ser.begin(baud, SERIAL_8N1, rx, tx);
  }

  // คืนค่า true เมื่อได้ distance ใหม่
  bool getdistance_cm(uint16_t &out_cm) {
    while (ser.available()) {
      uint8_t b = (uint8_t)ser.read();

      if (!haveH) {
        if (b & 0x80) {   // high byte
          H = b;
          haveH = true;
        }
      } else {
        if ((b & 0x80) == 0) { // low byte
          uint8_t L = b;
          out_cm = (uint16_t)(((H & 0x7F) * 128) + (L & 0x7F));
          haveH = false;
          return true;
        } else {
          // หลุด sync: เจอ high อีกตัว -> ใช้เป็น high ใหม่
          H = b;
          haveH = true;
        }
      }
    }
    return false; // ยังไม่มีค่าใหม่
  }
};

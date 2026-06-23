# Optical Flow Firmware — ESP32-S3 + OV2640 + SF30C

Firmware for ESP32-S3 that captures grayscale frames from an OV2640 camera, computes optical flow, reads distance from an SF30C LiDAR, and outputs MAVLink messages to a flight controller.

---

## Hardware

| Component | Description |
|---|---|
| MCU | ESP32-S3 (4D Systems GEN4 R8N16) |
| Camera | OV2640 (GRAYSCALE, 96×96) |
| LiDAR | SF30C (UART, 115200 baud) |
| FC Link | MAVLink over UART (Serial2, RX=47, TX=21) |

---

## System Flow

```
OV2640 Camera
   └─ Capture grayscale 96×96
       └─ Downscale to 64×64 (drop_down_image_scale)
           └─ compute_flow() — pixel dx, dy
               ├─ SF30C LiDAR (dist_m) — convert to velocity [m/s]
               ├─ MAVLink OPTICAL_FLOW → Flight Controller
               └─ MAVLink DISTANCE_SENSOR → Flight Controller
                   └─ UDP debug image → Python PC tool (optional)
```

---

## Build

Requires [PlatformIO](https://platformio.org/).

```bash
# Build only
pio run

# Build and upload
pio run -t upload --upload-port COM50    

# Serial monitor
pio device monitor -b 115200 -p COM50   
```

---

## Project Structure

```
optical_flow_SF30c/
├── platformio.ini          — PlatformIO build config
├── README.md
├── .gitignore
├── include/
│   └── ovcam.h             — Main include hub (camera + system headers)
├── src/
│   ├── main.cpp            — setup(): init serial, WiFi, camera, MAVLink, RTOS
│   ├── rtos.cpp / .h       — FreeRTOS tasks (process, lidar_task, udp_rx_task)
│   ├── mavlink_data.cpp/.h — MAVLink OPTICAL_FLOW and DISTANCE_SENSOR send
│   ├── lidar.cpp / .h      — TFMini (TFMPlus) LiDAR init
│   ├── udp_sender.cpp / .h — UDP chunked image TX to PC debug tool
│   ├── wifi_helper.cpp / .h— WiFi station init with static IP
│   ├── drop_down_image_scale.h — Bilinear downscale (96→64)
│   └── config/
│       ├── board_config.h  — Camera model selection (defines camera_pins.h)
│       ├── camera_pins.h   — GPIO pin assignments for OV2640
│       └── param_conf.h    — WiFi SSID/pass, IP addresses, UDP ports
├── lib/
│   ├── OF/         — PX4 C++ optical flow class (OpticalFlowPX4)
│   ├── TFMPlus/    — TFMini Plus LiDAR driver (3rd-party)
│   ├── flow/       — compute_flow(), settings/params, SF30C header, frame timing
│   ├── driver/     — esp_camera HAL driver (cam_hal, esp_camera)
│   ├── hal/        — SCCB I2C and XCLK HAL
│   ├── ll/         — Low-level DMA camera driver (esp32s3)
│   ├── sensor/     — Camera sensor drivers (OV2640, OV3660, etc.)
│   ├── conversion/ — Image format converters (JPEG, YUV, BMP)
│   ├── mavlink/    — MAVLink C headers (ardupilotmega dialect)
│   ├── packet/     — UDP packet protocol (UDPPacket, CRC32)
│   └── core/       — Utility libs (crc32, log, err)
└── test/
    └── README
```

---

## FreeRTOS Tasks

| Task | Core | Priority | Stack | Function |
|---|---|---|---|---|
| `process` | 1 | 5 | 16 KB | Camera capture + optical flow + MAVLink TX |
| `lidar_task` | 1 | 3 | 2 KB | SF30C distance read at 50 Hz |
| `udp_rx_task` | 0 | 4 | 8 KB | UDP command receive from Python PC |

`loop()` is intentionally empty — all logic runs in FreeRTOS tasks.

---

## Configuration

Edit `src/config/param_conf.h` to set:
- WiFi SSID and password
- ESP32 static IP address
- PC IP address (UDP target)
- UDP port numbers

Edit `src/config/board_config.h` to select the camera model (sets GPIO pins).

---

## Build Artifacts — Do Not Commit

The following are generated and should not be committed:

```
.pio/          — build output, cached dependencies
*.o / *.d      — compiled object and dependency files
*.elf / *.bin  — firmware images
*.map          — linker map
```

These are covered by `.gitignore`.

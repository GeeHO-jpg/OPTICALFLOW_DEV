#pragma once
#include <stdint.h>
#include <stddef.h>

#define IMG_TX_W          120
#define IMG_TX_H          120
#define IMG_TX_FPS_MS     1000  // send interval in ms (1 fps)
#define IMG_CHUNK_SIZE    1200  // bytes of image data per UDP chunk
#define IMG_CHUNK_PACE_MS 10    // delay between consecutive UDP chunks

#define IMG_TX_MAX_BYTES  (IMG_TX_W * IMG_TX_H)

#ifdef __cplusplus
extern "C" {
#endif

void img_sender_init(const char* dest_ip, uint16_t dest_port);

// Call from process task: copies buf into shared buffer (non-blocking, drops frame if busy)
void img_sender_update(const uint8_t* buf, size_t len);

// FreeRTOS task entry: wakes every IMG_TX_FPS_MS ms, sends latest image snapshot
void img_sender_task(void* arg);

#ifdef __cplusplus
}
#endif

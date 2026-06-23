#include "image_sender.h"
#include "UDPPacketHeader.h"
#include "UDPCommands.h"

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define MINI_HDR_SIZE 8
#define PKT_MAX       (UDPPACKETHEADER_SIZE + MINI_HDR_SIZE + IMG_CHUNK_SIZE)

static int                s_sock      = -1;
static struct sockaddr_in s_dest;
static bool               s_connected = false;

// Shared buffer written by process task, read by sender task
static uint8_t           s_shared_buf[IMG_TX_MAX_BYTES];
static size_t            s_shared_len = 0;   // actual byte count of current frame
static volatile bool     s_fresh   = false;
static SemaphoreHandle_t s_mutex   = NULL;

// Private send buffer so process task can write while we transmit
static uint8_t  s_send_buf[IMG_TX_MAX_BYTES];
static uint16_t s_frame_id = 0;

// ---------------------------------------------------------------------------

void img_sender_init(const char* dest_ip, uint16_t dest_port) {
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        printf("[img_sender] mutex alloc failed\n");
        return;
    }

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        printf("[img_sender] socket failed errno=%d\n", errno);
        return;
    }

    memset(&s_dest, 0, sizeof(s_dest));
    s_dest.sin_family      = AF_INET;
    s_dest.sin_port        = htons(dest_port);
    inet_aton(dest_ip, &s_dest.sin_addr);

    if (connect(s_sock, (struct sockaddr*)&s_dest, sizeof(s_dest)) == 0) {
        s_connected = true;
    } else {
        printf("[img_sender] connect failed errno=%d; will use sendto\n", errno);
    }

    // printf("[img_sender] ready -> %s:%u  chunk=%d bytes  pace=%d ms/chunk\n",
    //        dest_ip, (unsigned)dest_port, IMG_CHUNK_SIZE, IMG_CHUNK_PACE_MS);
}

// ---------------------------------------------------------------------------
// Called from process task — non-blocking: drops frame if sender is snapshotting
void img_sender_update(const uint8_t* buf, size_t len) {
    if (!s_mutex || !buf || len == 0 || len > IMG_TX_MAX_BYTES) return;

    if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
        memcpy(s_shared_buf, buf, len);
        s_shared_len = len;
        s_fresh = true;
        xSemaphoreGive(s_mutex);
    }
    // else: sender task is snapshotting — silently skip, next update will land
}

// ---------------------------------------------------------------------------
// Sender task: wakes every IMG_TX_FPS_MS, snapshots latest image, sends chunks
void img_sender_task(void* arg) {
    static uint8_t pkt[PKT_MAX];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(IMG_TX_FPS_MS));

        if (s_sock < 0 || !s_mutex) continue;

        // --- snapshot under mutex (fast: just a memcpy) ---
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        if (!s_fresh) { xSemaphoreGive(s_mutex); continue; }
        size_t send_len = s_shared_len;
        memcpy(s_send_buf, s_shared_buf, send_len);
        s_fresh = false;
        xSemaphoreGive(s_mutex);

        // --- send chunks ---
        uint16_t frame_id     = s_frame_id++;
        uint16_t total_chunks = (uint16_t)((send_len + IMG_CHUNK_SIZE - 1) / IMG_CHUNK_SIZE);
        uint32_t offset = 0;
        bool     ok     = true;

        for (uint16_t idx = 0; idx < total_chunks; idx++) {
            uint16_t data_len  = (uint16_t)(
                (offset + IMG_CHUNK_SIZE <= send_len)
                    ? IMG_CHUNK_SIZE
                    : (send_len - offset));
            uint16_t payload_sz = (uint16_t)(MINI_HDR_SIZE + data_len);

            // Packet header (UDPPACKETHEADER_SIZE = 9 bytes)
            pkt[0] = 'R'; pkt[1] = 'C'; pkt[2] = 'S'; pkt[3] = 'A';
            pkt[4] = (uint8_t)(frame_id & 0xFF);
            pkt[5] = (uint8_t)((frame_id >> 8) & 0xFF);
            pkt[6] = (uint8_t)UDPCOMMANDS_IMAGE_CHUNK;
            pkt[7] = (uint8_t)(payload_sz & 0xFF);
            pkt[8] = (uint8_t)((payload_sz >> 8) & 0xFF);

            // Mini image-chunk header (MINI_HDR_SIZE = 8 bytes)
            pkt[9]  = 0x01;
            pkt[10] = (uint8_t)(idx & 0xFF);
            pkt[11] = (uint8_t)((idx >> 8) & 0xFF);
            pkt[12] = (uint8_t)(total_chunks & 0xFF);
            pkt[13] = (uint8_t)((total_chunks >> 8) & 0xFF);
            pkt[14] = 0x00;
            pkt[15] = 0x00;
            pkt[16] = 0x00;

            memcpy(pkt + UDPPACKETHEADER_SIZE + MINI_HDR_SIZE,
                   s_send_buf + offset, data_len);

            size_t pkt_len = (size_t)(UDPPACKETHEADER_SIZE + payload_sz);
            int sent = s_connected
                ? send(s_sock, pkt, pkt_len, 0)
                : sendto(s_sock, pkt, pkt_len, 0,
                         (struct sockaddr*)&s_dest, sizeof(s_dest));

            if (sent < 0) {
                printf("[img_sender] send failed frame=%u chunk=%u/%u errno=%d\n",
                       (unsigned)frame_id,
                       (unsigned)(idx + 1), (unsigned)total_chunks,
                       errno);
                ok = false;
                break;
            }

            offset += data_len;
            vTaskDelay(pdMS_TO_TICKS(IMG_CHUNK_PACE_MS)); // pace: don't flood TX buffer
        }

        if (ok) {
            // printf("[img_sender] frame=%u ok chunks=%u bytes=%u\n",
            //        (unsigned)frame_id, (unsigned)total_chunks, (unsigned)send_len);
        }
    }
}

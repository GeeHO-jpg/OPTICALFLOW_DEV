#pragma once

#include <stdint.h>

#ifndef __cplusplus
extern "C" {
#endif


static inline uint8_t avg2(uint8_t a, uint8_t b) {
    return (uint8_t)(((uint16_t)a + (uint16_t)b) >> 1);
}


void downscale(uint8_t *dst, int dst_w ,int dst_h,const uint8_t *src,int src_w, int src_h)
{
    // fixed-point Q16 mapping
    uint32_t acc_y = 0;
    const uint32_t step_y = (src_h << 16) / dst_h; // 96/64 = 1.5

    for (int y = 0; y < dst_h; y++) {
        int sy  = (int)(acc_y >> 16);               // 0..95
        int sy2 = (sy < src_h - 1) ? (sy + 1) : sy;

        const uint8_t *row0 = src + sy  * src_w;
        const uint8_t *row1 = src + sy2 * src_w;

        // x mapping: 96 -> 64 (1.5) ด้วย Q16
        uint32_t acc_x = 0;
        const uint32_t step_x = (src_w << 16) / dst_w;

        for (int x = 0; x < dst_w; x++) {
            int sx = (int)(acc_x >> 16);
            // avg ระหว่าง 2 แถว (ลด noise/aliasing)
            dst[y*dst_w + x] = avg2(row0[sx], row1[sx]);
            acc_x += step_x;
        }

        acc_y += step_y;
    }
}


#ifndef __cplusplus
    }
#endif

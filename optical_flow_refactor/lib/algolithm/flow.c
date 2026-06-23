/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <tpetri@inf.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
 *   		 Samuel Zihlmann <samuezih@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "../protocal/mavlink/MAVLink.h"
#include <stdint.h>
#include <string.h>

#include "no_warnings.h"
#include "settings.h"
#include "frame_timing.h"



#define FRAME_SIZE	global_data.param[PARAM_IMAGE_WIDTH]
#define SEARCH_SIZE	global_data.param[PARAM_MAX_FLOW_PIXEL]
#define TILE_SIZE	8
#define NUM_BLOCKS	5

uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y);



/* -----------------------------
 * ESP32 replacements for ARM SIMD intrinsics
 * ----------------------------- */

static inline uint32_t load_u32_unaligned(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, 4);   // safe for unaligned + strict aliasing
    return v;
}

/* Read 4 bytes safely even if unaligned */
#define U32AT(p)  (load_u32_unaligned((const uint8_t *)(p)))

static inline uint32_t sum4_u8_from_u32(uint32_t v)
{
    return (v & 0xFFu)
         + ((v >> 8)  & 0xFFu)
         + ((v >> 16) & 0xFFu)
         + ((v >> 24) & 0xFFu);
}

/* replacement for how this file uses __UADD8: sum of 4 bytes in each word (a+b => 8 pixels sum) */
static inline uint32_t uadd8_sum(uint32_t a, uint32_t b)
{
    return sum4_u8_from_u32(a) + sum4_u8_from_u32(b);
}

#define __UADD8(a,b)   uadd8_sum((a),(b))

/* __UHADD8: Unsigned Halving Add (per byte): (a+b)>>1 */
static inline uint32_t __UHADD8(uint32_t a, uint32_t b)
{
    uint32_t r = 0;

    uint32_t a0 = (a >> 0)  & 0xFF, b0 = (b >> 0)  & 0xFF;
    uint32_t a1 = (a >> 8)  & 0xFF, b1 = (b >> 8)  & 0xFF;
    uint32_t a2 = (a >> 16) & 0xFF, b2 = (b >> 16) & 0xFF;
    uint32_t a3 = (a >> 24) & 0xFF, b3 = (b >> 24) & 0xFF;

    r |= (((a0 + b0) >> 1) & 0xFF) << 0;
    r |= (((a1 + b1) >> 1) & 0xFF) << 8;
    r |= (((a2 + b2) >> 1) & 0xFF) << 16;
    r |= (((a3 + b3) >> 1) & 0xFF) << 24;

    return r;
}

/* SAD for 4 unsigned bytes packed in uint32_t */
static inline uint32_t sad4_u8_u32(uint32_t a, uint32_t b)
{
    uint32_t s = 0;

    int d0 = (int)(a & 0xFF) - (int)(b & 0xFF);                   s += (uint32_t)(d0 < 0 ? -d0 : d0);
    int d1 = (int)((a >> 8) & 0xFF) - (int)((b >> 8) & 0xFF);     s += (uint32_t)(d1 < 0 ? -d1 : d1);
    int d2 = (int)((a >> 16) & 0xFF) - (int)((b >> 16) & 0xFF);   s += (uint32_t)(d2 < 0 ? -d2 : d2);
    int d3 = (int)((a >> 24) & 0xFF) - (int)((b >> 24) & 0xFF);   s += (uint32_t)(d3 < 0 ? -d3 : d3);

    return s;
}

/* __USAD8: Sum of Absolute Differences (4 bytes) */
static inline uint32_t __USAD8(uint32_t a, uint32_t b)
{
    return sad4_u8_u32(a, b);
}

/* __USADA8: Accumulate SAD (4 bytes) */
static inline uint32_t __USADA8(uint32_t a, uint32_t b, uint32_t acc)
{
    return acc + sad4_u8_u32(a, b);
}





/**
 * @brief Compute the average pixel gradient of all horizontal and vertical steps
 *
 * TODO compute_diff is not appropriate for low-light mode images
 *
 * @param image  the array holding pixel data
 * @param offX   x coordinate of upper left corner of 8x8 pattern in image
 * @param offY   y coordinate of upper left corner of 8x8 pattern in image
 *
 * @return       value indicating how suitable a pattern is for optical flow calculation
 */
static inline uint32_t compute_diff(uint8_t *image, uint16_t offX, uint16_t offY, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off = (offY + 2) * row_size + (offX + 2);
	/* we calculate only the center 4x4 pattern */
	uint32_t acc;

	/* calculate vertical gradient */
	acc = __USAD8 (*((uint32_t*) &image[off + 0 + 0 * row_size]), *((uint32_t*) &image[off + 0 + 1 * row_size]));
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 1 * row_size]), *((uint32_t*) &image[off + 0 + 2 * row_size]), acc);
	acc = __USADA8(*((uint32_t*) &image[off + 0 + 2 * row_size]), *((uint32_t*) &image[off + 0 + 3 * row_size]), acc);

	/* we need to get columns */
	uint32_t col1 = (image[off + 0 + 0 * row_size] << 24) | image[off + 0 + 1 * row_size] << 16 | image[off + 0 + 2 * row_size] << 8 | image[off + 0 + 3 * row_size];
	uint32_t col2 = (image[off + 1 + 0 * row_size] << 24) | image[off + 1 + 1 * row_size] << 16 | image[off + 1 + 2 * row_size] << 8 | image[off + 1 + 3 * row_size];
	uint32_t col3 = (image[off + 2 + 0 * row_size] << 24) | image[off + 2 + 1 * row_size] << 16 | image[off + 2 + 2 * row_size] << 8 | image[off + 2 + 3 * row_size];
	uint32_t col4 = (image[off + 3 + 0 * row_size] << 24) | image[off + 3 + 1 * row_size] << 16 | image[off + 3 + 2 * row_size] << 8 | image[off + 3 + 3 * row_size];

	/* calculate horizontal gradient */
	acc = __USADA8(col1, col2, acc);
	acc = __USADA8(col2, col3, acc);
	acc = __USADA8(col3, col4, acc);

	return acc;

}

/**
 * @brief Compute SAD distances of subpixel shift of two 8x8 pixel patterns.
 *
 * @param image1  an array holding pixel data
 * @param image2  an array holding pixel data
 * @param off1X   x coordinate of upper left corner of pattern in image1
 * @param off1Y   y coordinate of upper left corner of pattern in image1
 * @param off2X   x coordinate of upper left corner of pattern in image2
 * @param off2Y   y coordinate of upper left corner of pattern in image2
 * @param acc     array to store SAD distances for shift in every direction
 *
 * @return        zero
 */
static inline uint32_t compute_subpixel(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint32_t *acc, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t s0, s1, s2, s3, s4, s5, s6, s7, t1, t3, t5, t7;

	for (uint16_t i = 0; i < 8; i++)
	{
		acc[i] = 0;
	}


	/*
	 * calculate for each pixel in the 8x8 field with upper left corner (off1X / off1Y)
	 * every iteration is one line of the 8x8 field.
	 *
	 *  + - + - + - + - + - + - + - + - +
	 *  |   |   |   |   |   |   |   |   |
	 *  + - + - + - + - + - + - + - + - +
	 */

	for (uint16_t i = 0; i < 8; i++)
	{
		/*
		 * first column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  | x | x | x | x |   |   |   |   |
		 *  + - + - + - + - + - + - + - + - +
		 *
		 * the 8 s values are from following positions for each pixel (X):
		 *  + - + - + - +
		 *  +   5   7   +
		 *  + - + 6 + - +
		 *  +   4 X 0   +
		 *  + - + 2 + - +
		 *  +   3   1   +
		 *  + - + - + - +
		 *
		 *  variables (s1, ...) contains all 4 results (32bit -> 4 * 8bit values)
		 */

		/* compute average of two pixel values */
		// s0 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+0) * row_size])));
		// s1 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i+1) * row_size])));
		// s2 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i+1) * row_size])));
		// s3 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+1) * row_size])));
		// s4 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i+0) * row_size])));
		// s5 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 - 1 + (i-1) * row_size])));
		// s6 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 0 + (i-1) * row_size])));
		// s7 = (__UHADD8(*((uint32_t*) &image2[off2 +  0 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 1 + (i-1) * row_size])));

		s0 = (__UHADD8(U32AT(&image2[off2 +  0 + (i+0) * row_size]), U32AT(&image2[off2 + 1 + (i+0) * row_size])));
		s1 = (__UHADD8(U32AT(&image2[off2 +  0 + (i+1) * row_size]), U32AT(&image2[off2 + 1 + (i+1) * row_size])));
		s2 = (__UHADD8(U32AT(&image2[off2 +  0 + (i+0) * row_size]), U32AT(&image2[off2 + 0 + (i+1) * row_size])));
		s3 = (__UHADD8(U32AT(&image2[off2 +  0 + (i+1) * row_size]), U32AT(&image2[off2 - 1 + (i+1) * row_size])));
		s4 = (__UHADD8(U32AT(&image2[off2 +  0 + (i+0) * row_size]), U32AT(&image2[off2 - 1 + (i+0) * row_size])));
		s5 = (__UHADD8(U32AT(&image2[off2 +  0 + (i-1) * row_size]), U32AT(&image2[off2 - 1 + (i-1) * row_size])));
		s6 = (__UHADD8(U32AT(&image2[off2 +  0 + (i+0) * row_size]), U32AT(&image2[off2 + 0 + (i-1) * row_size])));
		s7 = (__UHADD8(U32AT(&image2[off2 +  0 + (i-1) * row_size]), U32AT(&image2[off2 + 1 + (i-1) * row_size])));


		/* these 4 t values are from the corners around the center pixel */
		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		/*
		 * finally we got all 8 subpixels (s0, t1, s2, t3, s4, t5, s6, t7):
		 *  + - + - + - +
		 *  |   |   |   |
		 *  + - 5 6 7 - +
		 *  |   4 X 0   |
		 *  + - 3 2 1 - +
		 *  |   |   |   |
		 *  + - + - + - +
		 */

		/* fill accumulation vector */
		// acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s0, acc[0]);
		// acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t1, acc[1]);
		// acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s2, acc[2]);
		// acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t3, acc[3]);
		// acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s4, acc[4]);
		// acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t5, acc[5]);
		// acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), s6, acc[6]);
		// acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 0 + i * row_size])), t7, acc[7]);

		acc[0] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), s0, acc[0]);
		acc[1] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), t1, acc[1]);
		acc[2] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), s2, acc[2]);
		acc[3] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), t3, acc[3]);
		acc[4] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), s4, acc[4]);
		acc[5] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), t5, acc[5]);
		acc[6] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), s6, acc[6]);
		acc[7] = __USADA8(U32AT(&image1[off1 + 0 + i * row_size]), t7, acc[7]);

		/*
		 * same for second column of 4 pixels:
		 *
		 *  + - + - + - + - + - + - + - + - +
		 *  |   |   |   |   | x | x | x | x |
		 *  + - + - + - + - + - + - + - + - +
		 */

		// s0 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+0) * row_size])));
		// s1 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i+1) * row_size])));
		// s2 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i+1) * row_size])));
		// s3 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+1) * row_size])));
		// s4 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i+0) * row_size])));
		// s5 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 3 + (i-1) * row_size])));
		// s6 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i+0) * row_size]), *((uint32_t*) &image2[off2 + 4 + (i-1) * row_size])));
		// s7 = (__UHADD8(*((uint32_t*) &image2[off2 + 4 + (i-1) * row_size]), *((uint32_t*) &image2[off2 + 5 + (i-1) * row_size])));

		s0 = (__UHADD8(U32AT(&image2[off2 + 4 + (i+0) * row_size]), U32AT(&image2[off2 + 5 + (i+0) * row_size])));
		s1 = (__UHADD8(U32AT(&image2[off2 + 4 + (i+1) * row_size]), U32AT(&image2[off2 + 5 + (i+1) * row_size])));
		s2 = (__UHADD8(U32AT(&image2[off2 + 4 + (i+0) * row_size]), U32AT(&image2[off2 + 4 + (i+1) * row_size])));
		s3 = (__UHADD8(U32AT(&image2[off2 + 4 + (i+1) * row_size]), U32AT(&image2[off2 + 3 + (i+1) * row_size])));
		s4 = (__UHADD8(U32AT(&image2[off2 + 4 + (i+0) * row_size]), U32AT(&image2[off2 + 3 + (i+0) * row_size])));
		s5 = (__UHADD8(U32AT(&image2[off2 + 4 + (i-1) * row_size]), U32AT(&image2[off2 + 3 + (i-1) * row_size])));
		s6 = (__UHADD8(U32AT(&image2[off2 + 4 + (i+0) * row_size]), U32AT(&image2[off2 + 4 + (i-1) * row_size])));
		s7 = (__UHADD8(U32AT(&image2[off2 + 4 + (i-1) * row_size]), U32AT(&image2[off2 + 5 + (i-1) * row_size])));


		t1 = (__UHADD8(s0, s1));
		t3 = (__UHADD8(s3, s4));
		t5 = (__UHADD8(s4, s5));
		t7 = (__UHADD8(s7, s0));

		// acc[0] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s0, acc[0]);
		// acc[1] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t1, acc[1]);
		// acc[2] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s2, acc[2]);
		// acc[3] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t3, acc[3]);
		// acc[4] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s4, acc[4]);
		// acc[5] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t5, acc[5]);
		// acc[6] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), s6, acc[6]);
		// acc[7] = __USADA8 ((*((uint32_t*) &image1[off1 + 4 + i * row_size])), t7, acc[7]);
		acc[0] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), s0, acc[0]);
		acc[1] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), t1, acc[1]);
		acc[2] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), s2, acc[2]);
		acc[3] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), t3, acc[3]);
		acc[4] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), s4, acc[4]);
		acc[5] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), t5, acc[5]);
		acc[6] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), s6, acc[6]);
		acc[7] = __USADA8(U32AT(&image1[off1 + 4 + i * row_size]), t7, acc[7]);
	}

	return 0;
}

/**
 * @brief Compute SAD of two 8x8 pixel windows.
 *
 * @param image1  an array holding pixel data
 * @param image2  an array holding pixel data
 * @param off1X   x coordinate of upper left corner of pattern in image1
 * @param off1Y   y coordinate of upper left corner of pattern in image1
 * @param off2X   x coordinate of upper left corner of pattern in image2
 * @param off2Y   y coordinate of upper left corner of pattern in image2
 *
 * @return        SAD of two 8x8 pixel windows
 */
static inline uint32_t compute_sad_8x8(uint8_t *image1, uint8_t *image2, uint16_t off1X, uint16_t off1Y, uint16_t off2X, uint16_t off2Y, uint16_t row_size)
{
	/* calculate position in image buffer */
	uint16_t off1 = off1Y * row_size + off1X; // image1
	uint16_t off2 = off2Y * row_size + off2X; // image2

	uint32_t acc;
	// acc = __USAD8 (*((uint32_t*) &image1[off1 + 0 + 0 * row_size]), *((uint32_t*) &image2[off2 + 0 + 0 * row_size]));
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 0 * row_size]), *((uint32_t*) &image2[off2 + 4 + 0 * row_size]), acc);

	// acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 1 * row_size]), *((uint32_t*) &image2[off2 + 0 + 1 * row_size]), acc);
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 1 * row_size]), *((uint32_t*) &image2[off2 + 4 + 1 * row_size]), acc);

	// acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 2 * row_size]), *((uint32_t*) &image2[off2 + 0 + 2 * row_size]), acc);
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 2 * row_size]), *((uint32_t*) &image2[off2 + 4 + 2 * row_size]), acc);

	// acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 3 * row_size]), *((uint32_t*) &image2[off2 + 0 + 3 * row_size]), acc);
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 3 * row_size]), *((uint32_t*) &image2[off2 + 4 + 3 * row_size]), acc);

	// acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 4 * row_size]), *((uint32_t*) &image2[off2 + 0 + 4 * row_size]), acc);
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 4 * row_size]), *((uint32_t*) &image2[off2 + 4 + 4 * row_size]), acc);

	// acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 5 * row_size]), *((uint32_t*) &image2[off2 + 0 + 5 * row_size]), acc);
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 5 * row_size]), *((uint32_t*) &image2[off2 + 4 + 5 * row_size]), acc);

	// acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 6 * row_size]), *((uint32_t*) &image2[off2 + 0 + 6 * row_size]), acc);
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 6 * row_size]), *((uint32_t*) &image2[off2 + 4 + 6 * row_size]), acc);

	// acc = __USADA8(*((uint32_t*) &image1[off1 + 0 + 7 * row_size]), *((uint32_t*) &image2[off2 + 0 + 7 * row_size]), acc);
	// acc = __USADA8(*((uint32_t*) &image1[off1 + 4 + 7 * row_size]), *((uint32_t*) &image2[off2 + 4 + 7 * row_size]), acc);

	acc = __USAD8 (U32AT(&image1[off1 + 0 + 0 * row_size]), U32AT(&image2[off2 + 0 + 0 * row_size]));
	acc = __USADA8(U32AT(&image1[off1 + 4 + 0 * row_size]), U32AT(&image2[off2 + 4 + 0 * row_size]), acc);

	acc = __USADA8(U32AT(&image1[off1 + 0 + 1 * row_size]), U32AT(&image2[off2 + 0 + 1 * row_size]), acc);
	acc = __USADA8(U32AT(&image1[off1 + 4 + 1 * row_size]), U32AT(&image2[off2 + 4 + 1 * row_size]), acc);

	acc = __USADA8(U32AT(&image1[off1 + 0 + 2 * row_size]), U32AT(&image2[off2 + 0 + 2 * row_size]), acc);
	acc = __USADA8(U32AT(&image1[off1 + 4 + 2 * row_size]), U32AT(&image2[off2 + 4 + 2 * row_size]), acc);

	acc = __USADA8(U32AT(&image1[off1 + 0 + 3 * row_size]), U32AT(&image2[off2 + 0 + 3 * row_size]), acc);
	acc = __USADA8(U32AT(&image1[off1 + 4 + 3 * row_size]), U32AT(&image2[off2 + 4 + 3 * row_size]), acc);

	acc = __USADA8(U32AT(&image1[off1 + 0 + 4 * row_size]), U32AT(&image2[off2 + 0 + 4 * row_size]), acc);
	acc = __USADA8(U32AT(&image1[off1 + 4 + 4 * row_size]), U32AT(&image2[off2 + 4 + 4 * row_size]), acc);

	acc = __USADA8(U32AT(&image1[off1 + 0 + 5 * row_size]), U32AT(&image2[off2 + 0 + 5 * row_size]), acc);
	acc = __USADA8(U32AT(&image1[off1 + 4 + 5 * row_size]), U32AT(&image2[off2 + 4 + 5 * row_size]), acc);

	acc = __USADA8(U32AT(&image1[off1 + 0 + 6 * row_size]), U32AT(&image2[off2 + 0 + 6 * row_size]), acc);
	acc = __USADA8(U32AT(&image1[off1 + 4 + 6 * row_size]), U32AT(&image2[off2 + 4 + 6 * row_size]), acc);

	acc = __USADA8(U32AT(&image1[off1 + 0 + 7 * row_size]), U32AT(&image2[off2 + 0 + 7 * row_size]), acc);
	acc = __USADA8(U32AT(&image1[off1 + 4 + 7 * row_size]), U32AT(&image2[off2 + 4 + 7 * row_size]), acc);


	return acc;
}

/**
 * @brief Computes pixel flow from image1 to image2
 *
 * Searches the corresponding position in the new image (image2) of max. 64 pixels from the old image (image1)
 * and calculates the average offset of all.
 *
 * @param image1  previous image buffer
 * @param image2  current (new) image buffer
 * @param x_rate  gyro x rate
 * @param y_rate  gyro y rate
 * @param z_rate  gyro z rate
 *
 * @return        quality of flow calculation
 */
uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float x_rate, float y_rate, float z_rate, float *pixel_flow_x, float *pixel_flow_y) {

	/* constants */
	const int16_t winmin = -SEARCH_SIZE;
	const int16_t winmax = SEARCH_SIZE;
	const uint16_t hist_size = 2*(winmax-winmin+1)+1;
	const uint16_t W = (uint16_t)global_data.param[PARAM_IMAGE_WIDTH];
	const uint16_t H = (uint16_t)global_data.param[PARAM_IMAGE_HEIGHT];
	const int16_t  S = (int16_t)global_data.param[PARAM_MAX_FLOW_PIXEL];
	const uint32_t feat_th = (uint32_t)global_data.param[PARAM_BOTTOM_FLOW_FEATURE_THRESHOLD];
	const uint32_t sad_th  = (uint32_t)global_data.param[PARAM_BOTTOM_FLOW_VALUE_THRESHOLD];
	const float send_vdo = global_data.param[PARAM_USB_SEND_VIDEO];
	const float hist_filter = global_data.param[PARAM_BOTTOM_FLOW_HIST_FILTER];
	const float hfov_deg = global_data.param[PARAM_HFOV_DEG];   // เช่น 66

	const float gyro_comp = global_data.param[PARAM_BOTTOM_FLOW_GYRO_COMPENSATION];
	const float gyro_comp_thd = global_data.param[PARAM_GYRO_COMPENSATION_THRESHOLD];
	/* variables */
	uint16_t pixLo = SEARCH_SIZE + 1;
	uint16_t pixHi = FRAME_SIZE - (SEARCH_SIZE + 1) - TILE_SIZE;
	uint16_t pixStep = (pixHi - pixLo) / NUM_BLOCKS + 1;
	uint16_t i, j;
	uint32_t acc[8];
	uint16_t histx[hist_size];
	uint16_t histy[hist_size];
	static int8_t  dirsx[64];
	static int8_t  dirsy[64];
	static uint8_t subdirs[64];
	float meanflowx = 0.0f;
	float meanflowy = 0.0f;
	uint16_t meancount = 0;
	float histflowx = 0.0f;
	float histflowy = 0.0f;

	/* initialize with 0 */
	for (j = 0; j < hist_size; j++) { histx[j] = 0; histy[j] = 0; }

	/* iterate over all patterns
	 */
	for (j = pixLo; j < pixHi; j += pixStep)
	{
		for (i = pixLo; i < pixHi; i += pixStep)
		{
			/* test pixel if it is suitable for flow tracking */
			uint32_t diff = compute_diff(image1, i, j, W);
			if (diff < feat_th)
			{
				continue;
			}


			const int stride = W;
			uint32_t dist = 0xFFFFFFFF;
			int8_t sumx = 0;
			int8_t sumy = 0;
			int8_t ii, jj;
			uint8_t *base1 = image1 + j * W + i;

			for (jj = winmin; jj <= winmax; jj++)
			{
				uint8_t *base2 = image2 + (j+jj) * W + i;

				for (ii = winmin; ii <= winmax; ii++)
				{
					uint32_t temp_dist = compute_sad_8x8(image1, image2, i, j, i + ii, j + jj, W);
					// uint32_t temp_dist = ABSDIFF(base1, base2 + ii);
					if (temp_dist < dist)
					{
						sumx = ii;
						sumy = jj;
						dist = temp_dist;
					}
				}
			}

			/* acceptance SAD distance threshhold */
			if (dist < sad_th)
			{
				meanflowx += (float) sumx;
				meanflowy += (float) sumy;

				compute_subpixel(image1, image2, i, j, i + sumx, j + sumy, acc, W);
				uint32_t mindist = dist; // best SAD until now
				uint8_t mindir = 8; // direction 8 for no direction
				for(uint8_t k = 0; k < 8; k++)
				{
					if (acc[k] < mindist)
					{
						// SAD becomes better in direction k
						mindist = acc[k];
						mindir = k;
					}
				}
				dirsx[meancount] = sumx;
				dirsy[meancount] = sumy;
				subdirs[meancount] = mindir;
				meancount++;

				/* feed histogram filter*/
				uint8_t hist_index_x = 2*sumx + (winmax-winmin+1);
				if (mindir == 0 || mindir == 1 || mindir == 7) hist_index_x += 1;
				if (mindir == 3 || mindir == 4 || mindir == 5) hist_index_x += -1;
				uint8_t hist_index_y = 2*sumy + (winmax-winmin+1);
				if (mindir == 5 || mindir == 6 || mindir == 7) hist_index_y += -1;
				if (mindir == 1 || mindir == 2 || mindir == 3) hist_index_y += 1;

				histx[hist_index_x]++;
				histy[hist_index_y]++;

			}
		}
	}

	/* create flow image if needed (image1 is not needed anymore)
	 * -> can be used for debugging purpose
	 */
	if (FLOAT_AS_BOOL(send_vdo))//&& global_data.param[PARAM_VIDEO_USB_MODE] == FLOW_VIDEO)
	{
		for (j = pixLo; j < pixHi; j += pixStep)
		{
			for (i = pixLo; i < pixHi; i += pixStep)
			{

				uint32_t diff = compute_diff(image1, i, j, W);
				if (diff > feat_th)
				{
					image1[j * W + i] = 255;
				}

			}
		}
	}

	/* evaluate flow calculation */
	if (meancount > 10)
	{
		meanflowx /= meancount;
		meanflowy /= meancount;

		int16_t maxpositionx = 0;
		int16_t maxpositiony = 0;
		uint16_t maxvaluex = 0;
		uint16_t maxvaluey = 0;

		/* position of maximal histogram peek */
		for (j = 0; j < hist_size; j++)
		{
			if (histx[j] > maxvaluex)
			{
				maxvaluex = histx[j];
				maxpositionx = j;
			}
			if (histy[j] > maxvaluey)
			{
				maxvaluey = histy[j];
				maxpositiony = j;
			}
		}

		/* check if there is a peak value in histogram */
		if (1) //(histx[maxpositionx] > meancount / 6 && histy[maxpositiony] > meancount / 6)
		{
			if (FLOAT_AS_BOOL(hist_filter))
			{

				/* use histogram filter peek value */
				uint16_t hist_x_min = maxpositionx;
				uint16_t hist_x_max = maxpositionx;
				uint16_t hist_y_min = maxpositiony;
				uint16_t hist_y_max = maxpositiony;

				/* x direction */
				if (maxpositionx > 1 && maxpositionx < hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 2;
				}
				else if (maxpositionx == 0)
				{
					hist_x_min = maxpositionx;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-1)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx;
				}
				else if (maxpositionx == 1)
				{
					hist_x_min = maxpositionx - 1;
					hist_x_max = maxpositionx + 2;
				}
				else  if (maxpositionx == hist_size-2)
				{
					hist_x_min = maxpositionx - 2;
					hist_x_max = maxpositionx + 1;
				}

				/* y direction */
				if (maxpositiony > 1 && maxpositiony < hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == 0)
				{
					hist_y_min = maxpositiony;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-1)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony;
				}
				else if (maxpositiony == 1)
				{
					hist_y_min = maxpositiony - 1;
					hist_y_max = maxpositiony + 2;
				}
				else if (maxpositiony == hist_size-2)
				{
					hist_y_min = maxpositiony - 2;
					hist_y_max = maxpositiony + 1;
				}

				float hist_x_value = 0.0f;
				float hist_x_weight = 0.0f;

				float hist_y_value = 0.0f;
				float hist_y_weight = 0.0f;

				for (uint8_t h = hist_x_min; h < hist_x_max+1; h++)
				{
					hist_x_value += (float) (h*histx[h]);
					hist_x_weight += (float) histx[h];
				}

				for (uint8_t h = hist_y_min; h<hist_y_max+1; h++)
				{
					hist_y_value += (float) (h*histy[h]);
					hist_y_weight += (float) histy[h];
				}

				histflowx = (hist_x_value/hist_x_weight - (winmax-winmin+1)) / 2.0f ;
				histflowy = (hist_y_value/hist_y_weight - (winmax-winmin+1)) / 2.0f;

			}
			else
			{

				/* use average of accepted flow values */
				uint32_t meancount_x = 0;
				uint32_t meancount_y = 0;

				for (uint8_t h = 0; h < meancount; h++)
				{
					float subdirx = 0.0f;
					if (subdirs[h] == 0 || subdirs[h] == 1 || subdirs[h] == 7) subdirx = 0.5f;
					if (subdirs[h] == 3 || subdirs[h] == 4 || subdirs[h] == 5) subdirx = -0.5f;
					histflowx += (float)dirsx[h] + subdirx;
					meancount_x++;

					float subdiry = 0.0f;
					if (subdirs[h] == 5 || subdirs[h] == 6 || subdirs[h] == 7) subdiry = -0.5f;
					if (subdirs[h] == 1 || subdirs[h] == 2 || subdirs[h] == 3) subdiry = 0.5f;
					histflowy += (float)dirsy[h] + subdiry;
					meancount_y++;
				}

				histflowx /= meancount_x;
				histflowy /= meancount_y;

			}

			/* compensate rotation */
			/* calculate focal length in pixels */
			// const float focal_length_px = (global_data.param[PARAM_FOCAL_LENGTH_MM]) / (4.0f * 6.0f) * 1000.0f; //original focal lenght: 12mm pixelsize: 6um, binning 4 enabled

			    // คำนวณ focal_length_px ครั้งเดียว (ใช้กับ flow->rad หรือ gyro comp)
			
			float hfov_rad = hfov_deg * (float)M_PI / 180.0f;
			float focal_length_px = ( W  * 0.5f) / tanf(hfov_rad * 0.5f);
			/*
			 * gyro compensation
			 * the compensated value is clamped to
			 * the maximum measurable flow value (PARAM_MAX_FLOW_PIXEL) +0.5
			 * (sub pixel flow can add half pixel to the value)
			 *
			 * -y_rate gives x flow
			 * x_rates gives y_flow
			 */
			if (FLOAT_AS_BOOL(gyro_comp))
			{
				if(fabsf(y_rate) > gyro_comp_thd)
				{
					/* calc pixel of gyro */
					float y_rate_pixel = y_rate * (get_time_between_images() / 1000000.0f) * focal_length_px;
					float comp_x = histflowx + y_rate_pixel;

					/* clamp value to maximum search window size plus half pixel from subpixel search */
					if (comp_x < (-SEARCH_SIZE - 0.5f))
						*pixel_flow_x = (-SEARCH_SIZE - 0.5f);
					else if (comp_x > (SEARCH_SIZE + 0.5f))
						*pixel_flow_x = (SEARCH_SIZE + 0.5f);
					else
						*pixel_flow_x = comp_x;
				}
				else
				{
					*pixel_flow_x = histflowx;
				}

				if(fabsf(x_rate) > gyro_comp_thd)
				{
					/* calc pixel of gyro */
					float x_rate_pixel = x_rate * (get_time_between_images() / 1000000.0f) * focal_length_px;
					float comp_y = histflowy - x_rate_pixel;

					/* clamp value to maximum search window size plus/minus half pixel from subpixel search */
					if (comp_y < (-SEARCH_SIZE - 0.5f))
						*pixel_flow_y = (-SEARCH_SIZE - 0.5f);
					else if (comp_y > (SEARCH_SIZE + 0.5f))
						*pixel_flow_y = (SEARCH_SIZE + 0.5f);
					else
						*pixel_flow_y = comp_y;
				}
				else
				{
					*pixel_flow_y = histflowy;
				}

//				/* alternative compensation */
//				/* compensate y rotation */
//				*pixel_flow_x = histflowx + y_rate_pixel;
//
//				/* compensate x rotation */
//				*pixel_flow_y = histflowy - x_rate_pixel;

			}
			else
			{
				/* without gyro compensation */
				*pixel_flow_x = histflowx;
				*pixel_flow_y = histflowy;
			}

		}
		else
		{
			*pixel_flow_x = 0.0f;
			*pixel_flow_y = 0.0f;
			return 0;
		}
	}
	else
	{
		*pixel_flow_x = 0.0f;
		*pixel_flow_y = 0.0f;
		return 0;
	}

	/* calculate quality */
	uint8_t qual = (uint8_t)(meancount * 255 / (NUM_BLOCKS*NUM_BLOCKS));

	return qual;
}

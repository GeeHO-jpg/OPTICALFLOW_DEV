/*
 * crc_32.h
 *
 *  Created on: Mar 7, 2026
 *      Author: Lenovo
 */

#ifndef INC_PACKET_HEADER_CRC_32_H_
#define INC_PACKET_HEADER_CRC_32_H_

#include <stddef.h>   /* for size_t */
#include <stdint.h>   /* for uint32_t */
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque CRC32 context */
typedef struct {
    uint32_t crc;
} rcsa_crc32_t;

/* Initialize a CRC32 context (sets to starting state 0xFFFFFFFF). */
void rcsa_crc32_init(rcsa_crc32_t *ctx);

/* Reset the CRC value in the context back to the initial state. */
void rcsa_crc32_reset(rcsa_crc32_t *ctx);

/*
 * Update the CRC32 with a buffer of data.
 * @param ctx       Pointer to CRC context.
 * @param data      Pointer to bytes to process.
 * @param data_size Number of bytes in the buffer.
 * @return          1 on success, 0 if data is NULL or data_size < 1.
 */
bool  rcsa_crc32_update(rcsa_crc32_t *ctx, const unsigned char *data, size_t data_size);

/*
 * Finalize and retrieve the CRC32 value.
 * @param ctx   Pointer to CRC context.
 * @return      Computed CRC32.
 */
uint32_t rcsa_crc32_get(const rcsa_crc32_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* INC_PACKET_HEADER_CRC_32_H_ */

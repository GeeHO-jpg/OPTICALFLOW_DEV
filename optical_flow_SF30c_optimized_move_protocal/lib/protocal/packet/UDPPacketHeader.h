/* 
 * File:   UDPPacketHeader.h
 * Author: Pete
 *
 * Created on August 26, 2024, 3:02 PM
 */

#ifndef UDPPACKETHEADER_H
#define	UDPPACKETHEADER_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define UDPPAYLOAD_MAX_SIZE     256
#define UDPPACKETHEADER_SIZE    9

// Global constant array for the header signature "RCSA"
static const uint8_t PACKETHEADER_SIGNATURE[4] = {'R', 'C', 'S', 'A'};

typedef struct {
    uint8_t signature[4];   // 4-byte signature
    uint16_t id;            // 2-byte ID
    uint8_t cmd;            // 1-byte command
    uint16_t payload_size;  // 2-byte payload size
} UDPPacketHeader;

bool IsPacketHeaderFirstByte(const uint8_t byte_1);
bool IsPacketHeaderSignature(const uint8_t* packet_signature);

UDPPacketHeader* CreateUDPPacketHeader(uint16_t id, uint8_t cmd, uint16_t payload_size);  //creat overheard of packet
UDPPacketHeader* GetUDPPacketHeader(uint8_t* packet_bytes, size_t packet_size);

uint8_t* ToBytesUDPPacketHeader(UDPPacketHeader* header);  //extract the packet

#endif	/* UDPPACKETHEADER_H */


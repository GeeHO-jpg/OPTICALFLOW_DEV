/* 
 * File:   UDPPacket.h
 * Author: Pete
 *
 * Created on 2 September 2024
 */

#ifndef UDPPACKET_H
#define	UDPPACKET_H

#include "UDPPacketHeader.h"

typedef struct {
    UDPPacketHeader* header;
    uint8_t* payload;
    size_t payload_tail_index;
} UDPPacket;


UDPPacket* CreateUDPPacket(UDPPacketHeader* header);

void FreeUDPPacket(UDPPacket* packet);

bool IsOperableUDPPacket(UDPPacket* packet);
bool IsPayloadCompletedUDPPacket(UDPPacket* packet);
bool AppendBytePayloadUDPPacket(UDPPacket* packet, uint8_t in_byte);

bool AppendBufferPayloadUDPPacket(UDPPacket* packet,const uint8_t* data, uint16_t len);


#endif	/* UDPPACKET_H */


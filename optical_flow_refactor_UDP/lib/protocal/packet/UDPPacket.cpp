#include "UDPPacket.h"

UDPPacket* CreateUDPPacket(UDPPacketHeader* header)
{
    if (header == NULL)
        return NULL;
    
    UDPPacket* packet = (UDPPacket*)malloc(sizeof(UDPPacket));
    if (packet == NULL)
        return NULL;
    
    if (header->payload_size > 0)
    {
        packet->payload = (uint8_t*)malloc(sizeof(uint8_t) * header->payload_size);
        if (packet->payload == NULL)
        {
            FreeUDPPacket(packet);
            packet = NULL;
            return NULL;
        }
    }
    packet->payload_tail_index = 0;
    
    // Assign here because if payload can't be allocated then we don't mistakenly free header
    packet->header = header;
    
    return packet; 
}

// Function to free a UDPPacket
void FreeUDPPacket(UDPPacket* packet)
{
    if (packet == NULL)
        return;
    
    // Free the header if it was dynamically allocated
    if (packet->header != NULL)
    {
        free(packet->header);
        packet->header = NULL;
    }
    
    // Free the payload if it was dynamically allocated
    if (packet->payload != NULL)
    {
        free(packet->payload);
        packet->payload = NULL;
    }
    
    // Free the packet structure itself
    free(packet);
}

bool IsOperableUDPPacket(UDPPacket* packet)
{
    return packet != NULL && packet->header != NULL;
}

bool IsPayloadCompletedUDPPacket(UDPPacket* packet)
{
    return packet->payload_tail_index >= packet->header->payload_size;
}

bool AppendBytePayloadUDPPacket(UDPPacket* packet, uint8_t in_byte)
{
    if (IsOperableUDPPacket(packet) && !IsPayloadCompletedUDPPacket(packet))
    {
        packet->payload[packet->payload_tail_index] = in_byte;
        packet->payload_tail_index++;
        return true;
    }
    
    return false;
}

bool AppendBufferPayloadUDPPacket(UDPPacket* packet,const uint8_t* data, uint16_t len) {
    if (!packet || !packet->header || !data) return false;

    uint16_t cap = packet->header->payload_size;
    uint16_t tail = packet->payload_tail_index;

    if ((uint32_t)tail + len > cap) return false;   // กันล้น
    
    memcpy(packet->payload + tail, data, len);
    packet->payload_tail_index = tail + len;
    return true;
}
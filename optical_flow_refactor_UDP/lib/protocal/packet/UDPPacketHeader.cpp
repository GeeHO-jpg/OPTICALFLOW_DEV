#include "UDPPacketHeader.h"

bool IsPacketHeaderFirstByte(const uint8_t byte_1)
{
    return byte_1 == PACKETHEADER_SIGNATURE[0];
}

bool IsPacketHeaderSignature(const uint8_t* packet_signature)
{
    return memcmp(packet_signature, PACKETHEADER_SIGNATURE, sizeof(PACKETHEADER_SIGNATURE)) == 0;
}

UDPPacketHeader* CreateUDPPacketHeader(uint16_t id, uint8_t cmd, uint16_t payload_size)
{
    UDPPacketHeader* header = (UDPPacketHeader*)malloc(sizeof(UDPPacketHeader));
    // Return NULL if memory allocation fails
    if (header == NULL)
        return NULL;
    
    memcpy(header->signature, PACKETHEADER_SIGNATURE, sizeof(header->signature));
    
    header->id = id;
    header->cmd = cmd;
    header->payload_size = payload_size;
    
    return header;
}

UDPPacketHeader* GetUDPPacketHeader(uint8_t* header_bytes, size_t header_size)
{
    if (header_bytes == NULL || header_size < UDPPACKETHEADER_SIZE)
        return NULL;
    
    if (!IsPacketHeaderSignature(header_bytes))
        return NULL;
    
    UDPPacketHeader* header = (UDPPacketHeader*)malloc(sizeof(UDPPacketHeader));
    // Return NULL if memory allocation fails
    if (header == NULL)
        return NULL;
    
    size_t offset = 0;
    memcpy(header->signature, header_bytes, sizeof(header->signature));
    offset += sizeof(header->signature);
    
    memcpy(&(header->id), header_bytes + offset, sizeof(header->id));
    offset += sizeof(header->id);
    
    header->cmd = header_bytes[offset];
    offset += sizeof(header->cmd);
     
    memcpy(&(header->payload_size), header_bytes + offset, sizeof(header->payload_size));
    offset += sizeof(header->payload_size);
    
    return header;
}

uint8_t* ToBytesUDPPacketHeader(UDPPacketHeader* header)
{
    if (header == NULL)
        return NULL;
    
    uint8_t* array = (uint8_t*)malloc(UDPPACKETHEADER_SIZE);
    if (array == NULL)
        return NULL;
    
    size_t offset = 0;
    memcpy(array, header->signature, sizeof(header->signature));
    offset += sizeof(header->signature);
    
    memcpy(array + offset, &(header->id), sizeof(header->id));
    offset += sizeof(header->id);
    
    memcpy(array + offset, &(header->cmd), sizeof(header->cmd));
    offset += sizeof(header->cmd);
    
    memcpy(array + offset, &(header->payload_size), sizeof(header->payload_size));
    
    return array;
}
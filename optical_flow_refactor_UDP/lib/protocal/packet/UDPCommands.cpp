#include "UDPCommands.h"

bool IsValidUDPCommand(uint8_t command_byte)
{
    return command_byte < UDPCOMMANDS_COUNT;
}

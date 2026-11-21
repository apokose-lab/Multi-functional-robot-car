#include "LidarC1Protocol.h"

bool parseHealthReply(const std::uint8_t* bytes, std::size_t size, HealthReply& reply)
{
    if (bytes == nullptr || size != 10)
    {
        return false;
    }

    constexpr std::uint8_t expectedDescriptor[] = {
        0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06
    };
    for (std::size_t i = 0; i < sizeof(expectedDescriptor); i++)
    {
        if (bytes[i] != expectedDescriptor[i])
        {
            return false;
        }
    }
    std::uint8_t rawStatus = bytes[7];
    if (rawStatus > 2)
    {
        return false;
    }
    reply.errorCode = static_cast<std::uint16_t>(bytes[9] * 256u + bytes[8]);
    reply.status = static_cast<HealthStatus>(rawStatus);
    return true;
}
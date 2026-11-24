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

bool HealthReplyReceiver::push(const std::uint8_t byte, HealthReply& reply)
{
    if (bytesReceived == 0)
    {
        if (byte == 0xA5)
        {
            buffer[0] = byte;
            bytesReceived = 1;
        }

        return false;
    }

    if (bytesReceived == 1)
    {
        if (byte == 0x5A)
        {
            buffer[1] = byte;
            bytesReceived = 2;
        }
        else if (byte != 0xA5)
        {
            bytesReceived = 0;
        }

        return false;
    }

    buffer[bytesReceived] = byte;
    ++bytesReceived;

    if (bytesReceived < packetSize)
    {
        return false;
    }

    const bool parsed = parseHealthReply(buffer, bytesReceived, reply);
    bytesReceived = 0;
    return parsed;
}

bool parseScanPoint(const std::uint8_t* bytes, std::size_t size, ScanPoint& point)
{
    if (bytes == nullptr || size != 5)
    {
        return false;
    }

    const bool newRotation = (bytes[0] & 0x01u) != 0;
    const bool invertStart = (bytes[0] & 0x02u) != 0;
    const bool checkBit = (bytes[1] & 0x01u) != 0;

    if (newRotation == invertStart || !checkBit)
    {
        return false;
    }

    const unsigned packedAngle = (static_cast<unsigned>(bytes[2]) << 8) | bytes[1];
    const unsigned angleQ6 = packedAngle >> 1;
    if (angleQ6 >= 360u * 64u)
    {
        return false;
    }

    const unsigned distanceQ2 = (static_cast<unsigned>(bytes[4]) << 8) | bytes[3];

    point.angleDeg = static_cast<float>(angleQ6) / 64.0f;
    point.distanceMm = static_cast<float>(distanceQ2) / 4.0f;
    point.quality = static_cast<std::uint8_t>(bytes[0] >> 2);
    point.newRotation = newRotation;
    return true;
}

bool ScanReceiver::push(std::uint8_t byte, ScanPoint& point)
{
    constexpr std::uint8_t scanDescriptor[] = {
        0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81
    };

    if (descriptorBytesReceived < sizeof(scanDescriptor))
    {
        if (byte == scanDescriptor[descriptorBytesReceived])
        {
            ++descriptorBytesReceived;
        }
        else
        {
            descriptorBytesReceived = (byte == scanDescriptor[0]) ? 1 : 0;
        }

        return false;
    }

    sample[sampleBytesReceived] = byte;
    ++sampleBytesReceived;
    if (sampleBytesReceived < sampleSize)
    {
        return false;
    }

    if (parseScanPoint(sample, sampleSize, point))
    {
        sampleBytesReceived = 0;
        return true;
    }

    for (std::size_t i = 1; i < sampleSize; ++i)
    {
        sample[i - 1] = sample[i];
    }

    sampleBytesReceived = sampleSize - 1;
    return false;
}
#pragma once
#include <cstdint>
#include <cstddef>

enum class HealthStatus : std::uint8_t
{
    Good = 0,
    Warning = 1,
    Error = 2
};

struct HealthReply
{
    HealthStatus status;
    std::uint16_t errorCode;
};

bool parseHealthReply(const std::uint8_t* bytes, std::size_t size, HealthReply& reply);

class HealthReplyReceiver
{
public:
    bool push(std::uint8_t byte, HealthReply& reply);

private:
    static constexpr std::size_t packetSize = 10;
    std::uint8_t buffer[packetSize]{};
    std::size_t bytesReceived = 0;
};


struct ScanPoint
{
    float angleDeg;
    float distanceMm;
    std::uint8_t quality;
    bool newRotation;
};

bool parseScanPoint(const std::uint8_t* bytes, std::size_t size, ScanPoint& point);

class ScanReceiver
{
public:
    bool push(std::uint8_t byte, ScanPoint& point);

private:
    static constexpr std::size_t sampleSize = 5;
    std::size_t descriptorBytesReceived = 0;
    std::uint8_t sample[sampleSize]{};
    std::size_t sampleBytesReceived = 0;
};


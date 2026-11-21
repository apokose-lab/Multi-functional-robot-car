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


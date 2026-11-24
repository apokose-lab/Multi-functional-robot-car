#include "LidarSectors.h"

#include <cmath>

namespace
{
    void updateNearest(float& nearestMm, float gapMm)
    {
        const float clearanceMm = gapMm > 0.0f ? gapMm : 0.0f;
        if (nearestMm < 0.0f || clearanceMm < nearestMm)
        {
            nearestMm = clearanceMm;
        }
    }
}

LidarSectorAccumulator::LidarSectorAccumulator(LidarChassisGeometry geometry)
    : geometry(geometry)
{
}

bool LidarSectorAccumulator::push(const ScanPoint& point, LidarSectors& completed)
{
    const bool rotationCompleted = point.newRotation && collecting;

    if (point.newRotation)
    {
        if (rotationCompleted)
        {
            completed = current;
        }

        current = LidarSectors{};
        collecting = true;
    }

    if (!collecting || point.distanceMm <= 0.0f)
    {
        return rotationCompleted;
    }

    constexpr float degreesToRadians = 0.01745329252f;
    const float angle = (point.angleDeg - geometry.forwardAngleDeg) * degreesToRadians;
    const float x = geometry.lidarForwardMm + point.distanceMm * std::cos(angle);
    // C1 angles grow clockwise. On the drivetrain, positive Y is left.
    const float y = geometry.lidarLeftMm - point.distanceMm * std::sin(angle);
    const float halfLength = geometry.lengthMm / 2.0f;
    const float halfWidth = geometry.widthMm / 2.0f;

    if (x > 0.0f && std::fabs(y) <= halfWidth + geometry.marginMm)
    {
        updateNearest(current.frontMm, x - halfLength);
    }

    if (std::fabs(x) <= halfLength + geometry.marginMm)
    {
        if (y > 0.0f)
        {
            updateNearest(current.leftMm, y - halfWidth);
        }
        else if (y < 0.0f)
        {
            updateNearest(current.rightMm, -y - halfWidth);
        }
    }

    return rotationCompleted;
}

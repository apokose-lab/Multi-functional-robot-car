#pragma once

#include "LidarC1Protocol.h"

struct LidarSectors
{
    // Clearance from the chassis edge. A negative value means no return.
    float frontMm = -1.0f;
    float leftMm = -1.0f;
    float rightMm = -1.0f;
};

struct LidarChassisGeometry
{
    float lengthMm;
    float widthMm;
    float marginMm;
    float lidarForwardMm;
    float lidarLeftMm;
    float forwardAngleDeg;
};

class LidarSectorAccumulator
{
public:
    explicit LidarSectorAccumulator(LidarChassisGeometry geometry);
    bool push(const ScanPoint& point, LidarSectors& completed);

private:
    LidarChassisGeometry geometry;
    bool collecting = false;
    LidarSectors current{};
};

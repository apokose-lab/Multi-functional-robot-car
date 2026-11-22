#pragma once

struct DistanceReading
{
    float mm = -1.0f;
    unsigned long measuredAt = 0;
};

struct DirectionReadings
{
    DistanceReading front;
    DistanceReading left;
    DistanceReading right;
};

enum class FrontPath
{
    Unknown,
    Clear,
    Blocked
};

inline bool isUsableDistance(DistanceReading reading, unsigned long now)
{
    constexpr unsigned long maxAgeMs = 300;
    return reading.mm >= 0.0f && now - reading.measuredAt <= maxAgeMs;
}

inline FrontPath evaluateFrontPath(DistanceReading lidar, DistanceReading ultrasonic,
                                   unsigned long now)
{
    constexpr float stopDistanceMm = 300.0f;
    const bool lidarUsable = isUsableDistance(lidar, now);
    const bool ultrasonicUsable = isUsableDistance(ultrasonic, now);

    if ((lidarUsable && lidar.mm <= stopDistanceMm) ||
        (ultrasonicUsable && ultrasonic.mm <= stopDistanceMm))
    {
        return FrontPath::Blocked;
    }

    if (lidarUsable && ultrasonicUsable)
    {
        return FrontPath::Clear;
    }

    return FrontPath::Unknown;
}

enum class MotionAction
{
    Stop,
    Forward,
    ShiftLeft,
    ShiftRight
};

class AvoidanceController
{
public:
    MotionAction update(FrontPath front, const DirectionReadings& lidar,
                        const DirectionReadings& ultrasonic, unsigned long now)
    {
        constexpr unsigned long shiftDurationMs = 300;
        constexpr unsigned long settleDurationMs = 180;
        constexpr float sideClearanceMm = 350.0f;
        constexpr unsigned int maxSideSteps = 3;

        if (front == FrontPath::Unknown)
        {
            if (action != MotionAction::Stop)
            {
                return stopAndSettle(now);
            }
            return MotionAction::Stop;
        }

        const float left = availableRoom(lidar.left, ultrasonic.left, now);
        const float right = availableRoom(lidar.right, ultrasonic.right, now);

        if (action == MotionAction::ShiftLeft || action == MotionAction::ShiftRight)
        {
            const float room = action == MotionAction::ShiftLeft ? left : right;
            if (front == FrontPath::Clear || room <= sideClearanceMm ||
                now - actionStartedAt >= shiftDurationMs)
            {
                return stopAndSettle(now);
            }
            return action;
        }

        if (settling && now - actionStartedAt < settleDurationMs)
        {
            return MotionAction::Stop;
        }
        settling = false;

        if (front == FrontPath::Clear)
        {
            sideSteps = 0;
            selectedSide = MotionAction::Stop;
            action = MotionAction::Forward;
            return action;
        }

        if (action == MotionAction::Forward)
        {
            return stopAndSettle(now);
        }

        if (sideSteps >= maxSideSteps)
        {
            return MotionAction::Stop;
        }

        if (selectedSide == MotionAction::Stop)
        {
            if (left <= sideClearanceMm && right <= sideClearanceMm)
            {
                return MotionAction::Stop;
            }
            selectedSide = left >= right ? MotionAction::ShiftLeft : MotionAction::ShiftRight;
        }

        const float room = selectedSide == MotionAction::ShiftLeft ? left : right;
        if (room <= sideClearanceMm)
        {
            return MotionAction::Stop;
        }

        action = selectedSide;
        actionStartedAt = now;
        ++sideSteps;
        return action;
    }

private:
    static float availableRoom(DistanceReading lidar, DistanceReading ultrasonic,
                               unsigned long now)
    {
        if (!isUsableDistance(lidar, now) || !isUsableDistance(ultrasonic, now))
        {
            return 0.0f;
        }
        return lidar.mm < ultrasonic.mm ? lidar.mm : ultrasonic.mm;
    }

    MotionAction stopAndSettle(unsigned long now)
    {
        action = MotionAction::Stop;
        actionStartedAt = now;
        settling = true;
        return action;
    }

    MotionAction action = MotionAction::Stop;
    MotionAction selectedSide = MotionAction::Stop;
    unsigned long actionStartedAt = 0;
    unsigned int sideSteps = 0;
    bool settling = false;
};

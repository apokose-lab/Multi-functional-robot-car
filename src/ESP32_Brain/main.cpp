#include <Arduino.h>
#include <MecanumKinematics.h>
#include <LidarC1Protocol.h>
#include <LidarSectors.h>
#include <Ultrasonic.h>
#include <ObstacleAvoidance.h>

/**
 * @file src/ESP32_Brain/main.cpp
 * @brief High-level robot brain running on the DFRobot FireBeetle 2 ESP32-S3.
 *
 * =============================================================================
 * TEAM QUICK-START GUIDE: HOW TO CONTROL THE ROBOT IN ONE FUNCTION CALL
 * =============================================================================
 * All drivetrain commands take speed/rotation values in the range [-255 .. +255]
 * and automatically enable closed-loop PID so all 4 wheels stay synchronized.
 *
 * Sign Conventions:
 *   - vx    (-255..255) : +Forward,            -Backward
 *   - vy    (-255..255) : +Strafe Left,        -Strafe Right
 *   - omega (-255..255) : +Rotate Left (CCW),  -Rotate Right (CW)
 *   - angleDeg          : 0 = Forward, 90 = Left, 180 = Backward, -90 = Right
 *
 * -----------------------------------------------------------------------------
 * A. CONTINUOUS MOVEMENT (Best inside `loop()` for line following / sensors)
 *    Note: Keep calling `robot.update()` in `loop()` so UART packets stream at 50Hz.
 * -----------------------------------------------------------------------------
 *   robot.drive(vx, vy, omega);          // Move in any (vx, vy) + spin (omega)
 *   robot.moveAngle(45, 140, 0);         // Move diagonally Forward-Left at 140 PWM
 *   robot.moveForward(150);              // Go straight forward at 150 PWM
 *   robot.moveForward(150, -30);         // Curve right while moving forward
 *   robot.moveBackward(150);             // Go straight backward at 150 PWM
 *   robot.strafeLeft(150);               // Slide sideways left at 150 PWM
 *   robot.strafeRight(150);              // Slide sideways right at 150 PWM
 *   robot.rotate(120);                   // Spin in place Left (CCW); use -120 for Right (CW)
 *   robot.stop();                        // Stop all 4 wheels immediately
 *   robot.beep();                        // Sound a short beep on the driver board
 *
 * -----------------------------------------------------------------------------
 * B. DISTANCE, ANGLE & TIMED MOVEMENT (Auto-stops when finished)
 *    Last argument `blocking`:
 *      - `true`  = Function waits until the maneuver finishes before moving to the next line.
 *      - `false` = Function returns immediately so you can read sensors while moving;
 *                  check `robot.isBusy()` and keep calling `robot.update()` in `loop()`.
 * -----------------------------------------------------------------------------
 *   robot.moveDistance(150, 0, 50.0f, 0, true);        // Move forward 50 cm at vx=150, then stop
 *   robot.moveAngleDistance(90, 140, 30.0f, 0, true);  // Strafe Left (90 deg) for 30 cm, then stop
 *   robot.rotateAngle(120, 90.0f, true);               // Turn Left 90 deg at omega=120, then stop
 *   robot.rotateAngle(-120, 90.0f, true);              // Turn Right 90 deg at omega=-120, then stop
 *   robot.driveFor(120, 0, 40, 1500, true);            // Curve for 1500 ms, then stop
 *
 * -----------------------------------------------------------------------------
 * C. OPTIONAL SETTINGS & FEEDBACK
 * -----------------------------------------------------------------------------
 *   robot.setPidEnabled(true);           // true = Closed-loop PID (default), false = Raw PWM
 *   robot.isBusy();                      // true while a non-blocking timed/distance move runs
 *   robot.isDriverAlive();               // true if Motion 2350 Pro is replying over UART
 * =============================================================================
 */

// UART1 pins wired to Cytron Motion 2350 Pro (GP16/GP17)
// Note: GPIO 16 and GPIO 17 on ESP32-S3 are reserved for the LiDAR sensor.
constexpr int UART_RX_PIN = 44; // Connects to GP16 (TX) on Motion 2350 Pro
constexpr int UART_TX_PIN = 43; // Connects to GP17 (RX) on Motion 2350 Pro

constexpr int LIDAR_RX_PIN = 17;
constexpr int LIDAR_TX_PIN = 16;

HardwareSerial MotionSerial(1);
HardwareSerial LidarSerial(2);
Drivetrain robot; // High-level Mecanum drivetrain controller

struct UltrasonicPins
{
    int trigger;
    int echo;

    constexpr bool assigned() const
    {
        return trigger >= 0 && echo >= 0 && trigger != echo;
    }
};

// TRIG, ECHO. Pair {8, 10} was traced on one connector; confirm its mounted position.
constexpr UltrasonicPins FRONT_US_PINS{-1, -1};
constexpr UltrasonicPins LEFT_US_PINS{4, 5};
constexpr UltrasonicPins RIGHT_US_PINS{-1, -1};

constexpr bool usesSerialPin(int pin)
{
    return pin == UART_RX_PIN || pin == UART_TX_PIN ||
           pin == LIDAR_RX_PIN || pin == LIDAR_TX_PIN ||
           pin == 19 || pin == 20; // ESP32-S3 native USB
}

constexpr bool separatePins(UltrasonicPins first, UltrasonicPins second)
{
    return !first.assigned() || !second.assigned() ||
           (first.trigger != second.trigger && first.trigger != second.echo &&
            first.echo != second.trigger && first.echo != second.echo);
}

constexpr bool freePins(UltrasonicPins pins)
{
    return !pins.assigned() || (!usesSerialPin(pins.trigger) && !usesSerialPin(pins.echo));
}

static_assert(freePins(FRONT_US_PINS) && freePins(LEFT_US_PINS) && freePins(RIGHT_US_PINS),
              "Ultrasonic pins must not overlap with LiDAR, motion UART or USB");
static_assert(separatePins(FRONT_US_PINS, LEFT_US_PINS) &&
              separatePins(FRONT_US_PINS, RIGHT_US_PINS) &&
              separatePins(LEFT_US_PINS, RIGHT_US_PINS),
              "Ultrasonic channels must use separate pins");

// Millimetres. Replace the provisional envelope and mounts with measured values.
constexpr LidarChassisGeometry CHASSIS{250.0f, 200.0f, 30.0f, 0.0f, 0.0f, 0.0f};
constexpr float FRONT_US_FROM_CENTER_MM = 120.0f;
constexpr float LEFT_US_FROM_CENTER_MM = 92.0f;
constexpr float RIGHT_US_FROM_CENTER_MM = 92.0f;

DirectionReadings lidarReadings{};
DirectionReadings ultrasonicReadings{};

struct UltrasonicChannel
{
    UltrasonicChannel(const char* label, UltrasonicPins pins, float insetMm,
                      DistanceReading& reading)
        : label(label),
          enabled(pins.assigned()),
          sensor(static_cast<std::uint8_t>(pins.trigger), static_cast<std::uint8_t>(pins.echo)),
          insetMm(insetMm),
          reading(reading)
    {
    }

    const char* label;
    bool enabled;
    Ultrasonic sensor;
    float insetMm;
    DistanceReading& reading;
};

UltrasonicChannel ultrasonicChannels[] = {
    {"front", FRONT_US_PINS, CHASSIS.lengthMm / 2.0f - FRONT_US_FROM_CENTER_MM, ultrasonicReadings.front},
    {"left", LEFT_US_PINS, CHASSIS.widthMm / 2.0f - LEFT_US_FROM_CENTER_MM, ultrasonicReadings.left},
    {"right", RIGHT_US_PINS, CHASSIS.widthMm / 2.0f - RIGHT_US_FROM_CENTER_MM, ultrasonicReadings.right}
};

AvoidanceController avoidance;
bool avoidanceArmed = false;

enum class LidarMode
{
    Idle,
    WaitingForHealth,
    Scanning,
    Stopping
};

void updateLidar(int key)
{
    static HealthReplyReceiver healthReceiver;
    static ScanReceiver scanReceiver;
    static LidarSectorAccumulator sectors{CHASSIS};
    static LidarMode mode = LidarMode::Idle;
    static unsigned long modeStartedAt = 0;
    static unsigned long lastPrintAt = 0;
    const unsigned long now = millis();

    for (int queued = LidarSerial.available(); queued > 0; --queued)
    {
        const int received = LidarSerial.read();
        if (received < 0)
        {
            break;
        }

        const auto byte = static_cast<std::uint8_t>(received);
        if (mode == LidarMode::WaitingForHealth)
        {
            HealthReply reply{};
            if (healthReceiver.push(byte, reply))
            {
                mode = LidarMode::Idle;
                Serial.printf("GET_HEALTH status=%u error=%u\n",
                              static_cast<unsigned>(reply.status),
                              static_cast<unsigned>(reply.errorCode));
            }
        }
        else if (mode == LidarMode::Scanning)
        {
            ScanPoint point{};
            LidarSectors completed{};

            if (scanReceiver.push(byte, point) && sectors.push(point, completed))
            {
                lidarReadings = {
                    {completed.frontMm, now},
                    {completed.leftMm, now},
                    {completed.rightMm, now}
                };

                if (now - lastPrintAt >= 250)
                {
                    lastPrintAt = now;
                    Serial.printf("SCAN clearance front=%.0f left=%.0f right=%.0f mm\n",
                                  static_cast<double>(completed.frontMm),
                                  static_cast<double>(completed.leftMm),
                                  static_cast<double>(completed.rightMm));
                }
            }
        }
    }

    if (mode == LidarMode::WaitingForHealth && now - modeStartedAt >= 500)
    {
        mode = LidarMode::Idle;
        Serial.println("GET_HEALTH timeout");
    }
    else if (mode == LidarMode::Stopping && now - modeStartedAt >= 20)
    {
        mode = LidarMode::Idle;
    }

    if (mode == LidarMode::Idle && key == 'h')
    {
        constexpr std::uint8_t command[] = {0xA5, 0x52};
        healthReceiver = HealthReplyReceiver{};
        LidarSerial.write(command, sizeof(command));
        modeStartedAt = millis();
        mode = LidarMode::WaitingForHealth;
        Serial.println("GET_HEALTH sent");
    }
    else if (mode == LidarMode::Idle && key == 's')
    {
        constexpr std::uint8_t command[] = {0xA5, 0x20};
        scanReceiver = ScanReceiver{};
        sectors = LidarSectorAccumulator{CHASSIS};
        lidarReadings = {};
        LidarSerial.write(command, sizeof(command));
        lastPrintAt = millis();
        mode = LidarMode::Scanning;
        Serial.println("SCAN sent");
    }
    else if (mode == LidarMode::Scanning && key == 'x')
    {
        constexpr std::uint8_t command[] = {0xA5, 0x25};
        LidarSerial.write(command, sizeof(command));
        modeStartedAt = millis();
        mode = LidarMode::Stopping;
        lidarReadings = {};
        Serial.println("STOP sent");
    }
}

void updateUltrasonics()
{
    constexpr unsigned long pingIntervalMs = 70;
    constexpr std::size_t channelCount = sizeof(ultrasonicChannels) / sizeof(ultrasonicChannels[0]);
    static std::size_t nextChannel = 0;
    static unsigned long lastPingAt = 0;
    const unsigned long now = millis();
    if (now - lastPingAt < pingIntervalMs)
    {
        return;
    }

    for (std::size_t checked = 0; checked < channelCount; ++checked)
    {
        auto& channel = ultrasonicChannels[nextChannel];
        nextChannel = (nextChannel + 1) % channelCount;
        if (!channel.enabled)
        {
            continue;
        }

        lastPingAt = now;
        const float distanceCm = channel.sensor.readDistanceCm();
        float clearanceMm = -1.0f;
        if (distanceCm > 0.0f)
        {
            const float gap = distanceCm * 10.0f - channel.insetMm;
            clearanceMm = gap > 0.0f ? gap : 0.0f;
        }
        channel.reading = {clearanceMm, millis()};

        if (channel.reading.mm >= 0.0f)
        {
            Serial.printf("US %s=%.0f mm\n", channel.label, static_cast<double>(channel.reading.mm));
        }
        else
        {
            Serial.printf("US %s=unknown\n", channel.label);
        }
        return;
    }
}

void reportFrontPath(FrontPath path)
{
    static bool printed = false;
    static FrontPath previous = FrontPath::Unknown;

    if (printed && path == previous)
    {
        return;
    }

    printed = true;
    previous = path;

    switch (path)
    {
        case FrontPath::Blocked:
            Serial.println("FRONT blocked");
            break;
        case FrontPath::Clear:
            Serial.println("FRONT clear");
            break;
        case FrontPath::Unknown:
            Serial.println("FRONT unknown");
            break;
    }
}

void applyMotion(MotionAction action)
{
    constexpr std::int16_t driveCommand = 65;
    static MotionAction previous = MotionAction::Stop;
    if (action == previous)
    {
        return;
    }
    previous = action;

    switch (action)
    {
        case MotionAction::Forward:
            robot.drive(driveCommand, 0);
            Serial.println("AUTO forward");
            break;
        case MotionAction::ShiftLeft:
            robot.drive(0, driveCommand);
            Serial.println("AUTO left");
            break;
        case MotionAction::ShiftRight:
            robot.drive(0, -driveCommand);
            Serial.println("AUTO right");
            break;
        case MotionAction::Stop:
            robot.stop();
            Serial.println("AUTO stop");
            break;
    }
}

void disarmAvoidance()
{
    applyMotion(MotionAction::Stop);
    if (avoidanceArmed)
    {
        avoidanceArmed = false;
        Serial.println("AUTO disarmed");
    }
}

void updateAvoidance(int key)
{
    const unsigned long now = millis();
    const FrontPath front = evaluateFrontPath(lidarReadings.front, ultrasonicReadings.front, now);
    reportFrontPath(front);
    const bool frontReady = isUsableDistance(lidarReadings.front, now) &&
                            isUsableDistance(ultrasonicReadings.front, now);

    if (key == 'a' && !avoidanceArmed)
    {
        if (!FRONT_US_PINS.assigned() || !LEFT_US_PINS.assigned() || !RIGHT_US_PINS.assigned())
        {
            Serial.println("AUTO unavailable: configure all ultrasonic pins");
            return;
        }
        if (!frontReady || !robot.isDriverAlive(250))
        {
            Serial.println("AUTO unavailable: check front sensors and motor driver");
            return;
        }

        avoidance = AvoidanceController{};
        avoidanceArmed = true;
        Serial.println("AUTO armed");
    }

    if (!avoidanceArmed)
    {
        return;
    }
    if (!robot.isDriverAlive(250))
    {
        disarmAvoidance();
        Serial.println("AUTO lost motor feedback");
        return;
    }

    const FrontPath motionPath = frontReady ? front : FrontPath::Unknown;
    applyMotion(avoidance.update(motionPath, lidarReadings, ultrasonicReadings, now));
}

void setup() {
    // Release default UART0 from GPIO 43/44 so MotionSerial (UART1) can use them cleanly
    Serial0.end();
    Serial.begin(115200);
    // Initialize high-speed UART link to the Motion 2350 Pro motor driver board
    MotionSerial.setRxBufferSize(512);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Initialize LiDAR
    LidarSerial.setRxBufferSize(2048);
    LidarSerial.begin(460800, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    for (auto& channel : ultrasonicChannels)
    {
        if (channel.enabled)
        {
            channel.sensor.begin();
        }
    }

    // Initialize drivetrain (PID mode enabled by default)
    robot.begin(MotionSerial);
}

void loop() {
    const int key = Serial.read();
    if (key == 'd' || key == 'x')
    {
        disarmAvoidance();
    }

    // Always keep this non-blocking update running on every loop iteration:
    // Streams 50Hz motor commands, handles auto-stop timers, and reads encoder feedback.
    robot.update();
    updateLidar(key);
    updateUltrasonics();
    updateAvoidance(key);
    // -------------------------------------------------------------------------
    // EXAMPLE 1 (Commented out): One-time autonomous sequence using blocking calls
    // -------------------------------------------------------------------------
    /*
    static bool sequenceDone = false;
    if (!sequenceDone) {
        delay(1000);                                         // Wait 1s for motor driver boot chime
        robot.moveDistance(140, 0, 50.0f, 0, true);          // Move forward 50 cm at PWM=140, omega=0
        robot.rotateAngle(-120, 90.0f, true);                // Rotate 90 deg Clockwise (Right)
        robot.moveAngleDistance(90.0f, 140, 30.0f, 0, true); // Strafe Left 30 cm
        robot.beep();                                        // Confirm completion
        sequenceDone = true;
    }
    */

    // -------------------------------------------------------------------------
    // EXAMPLE 2 (Commented out): Continuous driving with your own sensor logic
    // -------------------------------------------------------------------------
    /*
    // Read your sensors here...
    bool shouldStop = false;
    if (shouldStop) {
        robot.stop();
    } else {
        robot.drive(130, 0, 0); // Cruise forward continuously at vx=130, vy=0, omega=0
    }
    */
}

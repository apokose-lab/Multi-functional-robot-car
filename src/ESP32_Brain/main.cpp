#include <Arduino.h>
#include <MecanumKinematics.h>
#include <Ultrasonic.h>

/**
 * @file src/ESP32_Brain/main.cpp
 * @brief Autonomous obstacle-avoidance test program using only 3 HC-SR04 ultrasonic sensors,
 *        running on the DFRobot FireBeetle 2 ESP32-S3 brain board.
 *
 * =============================================================================
 * ULTRASONIC CONTROL BUGS IDENTIFIED & FIXED AGAINST lib/Ultrasonic:
 * =============================================================================
 * 1. Unassigned Front & Right sensor pins + negative-to-unsigned cast (-1 -> 255):
 *    - The previous code left FRONT_US_PINS{-1, -1} and RIGHT_US_PINS{-1, -1}, disabling
 *      both sensors while UltrasonicChannel cast -1 into uint8_t (255).
 *    - Fix: Use uint8_t directly to match Ultrasonic(uint8_t, uint8_t, float) and assign
 *      all 3 sensor pairs: Front {8, 10}, Left {4, 5}, Right {9, 6} (plus GPIO 11 buzzer).
 *
 * 2. Misinterpreting the -1.0f (INVALID_DISTANCE_CM) return value from readDistanceCm():
 *    - In lib/Ultrasonic/Ultrasonic.cpp, readDistanceCm() returns -1.0f whenever:
 *      (a) No echo pulse is received within the 30 ms timeout (duration == 0), or
 *      (b) The measured distance is beyond MAX_DISTANCE_CM (200.0 cm) or below MIN_DISTANCE_CM (2.0 cm).
 *    - The previous code treated any <= 0 value as "unknown" and halted the robot — meaning
 *      whenever the path ahead was wide open (> 2 m), the robot falsely stopped!
 *    - Fix: Treat -1.0f from readDistanceCm() as open space >= OPEN_SPACE_CM (200 cm),
 *      while still recording lastRawCm so debug logs clearly distinguish a measured hit from >200 cm/timeout.
 *
 * 3. Passing a negative calibrationOffsetCm causing close obstacles to vanish:
 *    - Ultrasonic::readDistanceCm() adds calibrationOffsetCm BEFORE checking >= MIN_DISTANCE_CM (2.0 cm).
 *      Passing a negative offset into the library could push very close obstacles (~2-3 cm) below
 *      2.0 cm, causing readDistanceCm() to return -1.0f (falsely treated as open space!).
 *    - Fix: Always keep calibrationOffsetCm = 0.0f inside Ultrasonic, then subtract the chassis
 *      recess inset (insetCm) externally and clamp the result to >= 0.0f.
 *
 * 4. Stale timeout (300 ms) and single-ping glitch sensitivity:
 *    - Ultrasonic::readDistanceCm() uses pulseIn() with ECHO_TIMEOUT_US = 30000 us (30 ms).
 *      Previously, polling all 3 channels at 70 ms intervals could take ~210-300 ms per cycle
 *      against a 300 ms maxAgeMs limit, frequently marking readings as stale/unknown.
 *    - Fix: Ping one sensor at a time in round-robin order every 60 ms to prevent acoustic
 *      crosstalk between sensors and keep max blocking <= 30 ms (safe for the Motion 2350 Pro
 *      350 ms safety watchdog), combined with a 3-sample median filter (median3) to reject glitches.
 * =============================================================================
 */

// =============================================================================
// 1. UART PINS FOR COMMUNICATION WITH CYTRON MOTION 2350 PRO MOTOR DRIVER
// =============================================================================
constexpr int UART_RX_PIN = 44; // Connects to GP16 (TX) on Cytron Motion 2350 Pro
constexpr int UART_TX_PIN = 43; // Connects to GP17 (RX) on Cytron Motion 2350 Pro

// Keep LiDAR UART pin constants for compile-time pin conflict verification
constexpr int LIDAR_RX_PIN = 15;
constexpr int LIDAR_TX_PIN = 16;

HardwareSerial MotionSerial(1);
Drivetrain robot; // High-level Mecanum drivetrain controller (switched to open-loop PWM in setup())

// =============================================================================
// 2. ULTRASONIC SENSOR PINS (TRIG, ECHO) & COMPILE-TIME SAFETY CHECKS
// =============================================================================
struct UltrasonicPins
{
    uint8_t trigger;
    uint8_t echo;
};

// ESP32-S3 GPIO wiring (TRIG, ECHO) for the 3 HC-SR04 ultrasonic sensors:
constexpr UltrasonicPins FRONT_US_PINS{9, 6};
constexpr UltrasonicPins LEFT_US_PINS{4, 5};
constexpr UltrasonicPins RIGHT_US_PINS{8, 10};

// External obstacle-warning buzzer connected to ESP32-S3 GPIO 11
constexpr uint8_t BUZZER_PIN = 11;

constexpr bool usesReservedPin(uint8_t pin)
{
    return static_cast<int>(pin) == UART_RX_PIN ||
           static_cast<int>(pin) == UART_TX_PIN ||
           static_cast<int>(pin) == LIDAR_RX_PIN ||
           static_cast<int>(pin) == LIDAR_TX_PIN ||
           pin == 19 ||
           pin == 20; // ESP32-S3 native USB D-/D+ pins
}

constexpr bool validPins(UltrasonicPins pins)
{
    return pins.trigger != pins.echo &&
           !usesReservedPin(pins.trigger) &&
           !usesReservedPin(pins.echo);
}

constexpr bool separatePins(UltrasonicPins first, UltrasonicPins second)
{
    return first.trigger != second.trigger &&
           first.trigger != second.echo &&
           first.echo != second.trigger &&
           first.echo != second.echo;
}

constexpr bool overlapsPins(uint8_t pin, UltrasonicPins pins)
{
    return pin == pins.trigger || pin == pins.echo;
}

static_assert(validPins(FRONT_US_PINS) && validPins(LEFT_US_PINS) && validPins(RIGHT_US_PINS),
              "Ultrasonic pins must not match each other or overlap with UART/LiDAR/USB pins");
static_assert(separatePins(FRONT_US_PINS, LEFT_US_PINS) &&
                  separatePins(FRONT_US_PINS, RIGHT_US_PINS) &&
                  separatePins(LEFT_US_PINS, RIGHT_US_PINS),
              "All 3 ultrasonic channels must use separate GPIO pins");
static_assert(!usesReservedPin(BUZZER_PIN) &&
                  !overlapsPins(BUZZER_PIN, FRONT_US_PINS) &&
                  !overlapsPins(BUZZER_PIN, LEFT_US_PINS) &&
                  !overlapsPins(BUZZER_PIN, RIGHT_US_PINS),
              "Buzzer pin must not overlap with ultrasonic or reserved UART/LiDAR/USB pins");

// =============================================================================
// 3. TUNABLE TEST PARAMETERS (EASY TO ADJUST HERE)
// =============================================================================
// Raw open-loop PWM duty cycle (-255..255). PID is temporarily disabled in setup() so all
// 4 wheels receive identical PWM directly. Note: raise TURN_SPEED to 120..140 if floor
// friction causes the robot to stall during in-place turns in open-loop mode.
constexpr int16_t FORWARD_SPEED = 100; // Moderate forward cruising PWM for safe floor testing
constexpr int16_t TURN_SPEED = 100;    // In-place rotation PWM for 90 deg / 180 deg turns

// Clearance thresholds measured from the outer chassis edge (in cm)
constexpr float OBSTACLE_STOP_CM = 30.0f;  // Stop immediately when front obstacle <= 30 cm
constexpr float OBSTACLE_CLEAR_CM = 38.0f; // Front must be >= 38 cm to count as clear (8 cm hysteresis)
constexpr float SIDE_CLEAR_CM = 40.0f;     // Left/Right side must be >= 40 cm to be considered open
constexpr float OPEN_SPACE_CM = 200.0f;    // Matches Ultrasonic::MAX_DISTANCE_CM (200 cm) when raw == -1.0f

// Distance (cm) from each sensor face to the outer edge of the chassis, subtracted after reading.
// Measure on the physical chassis and adjust these values if exact cm accuracy is needed.
constexpr float FRONT_US_INSET_CM = 0.5f;
constexpr float LEFT_US_INSET_CM = 0.8f;
constexpr float RIGHT_US_INSET_CM = 0.8f;

// Timing constants (in milliseconds)
constexpr unsigned long STARTUP_DELAY_MS = 3000;        // Wait 3s after power-on before auto-starting
constexpr unsigned long OBSTACLE_WAIT_MS = 5000;       // Wait up to 5s when an obstacle blocks the front
constexpr unsigned long CLEAR_CONFIRM_MS = 500;         // Obstacle must stay gone continuously for 500ms to resume
constexpr unsigned long TURN_SETTLE_MS = 600;           // Settle time after turning before evaluating new heading
constexpr unsigned long US_PING_INTERVAL_MS = 60;       // Minimum 60ms spacing between pings to avoid crosstalk
constexpr unsigned long DEBUG_PRINT_INTERVAL_MS = 500;  // Print Serial status telemetry every 500ms

// Obstacle-warning buzzer configuration on BUZZER_PIN (GPIO 11)
// Keep false to use tone()/noTone() (works on passive piezo buzzers and is audible on most
// active buzzers). Set to true for direct HIGH/LOW DC drive on an active buzzer module.
constexpr bool BUZZER_IS_ACTIVE = false;
constexpr unsigned int BUZZER_FREQ_HZ = 2000;    // Tone frequency (Hz) when BUZZER_IS_ACTIVE is false
constexpr unsigned long BUZZER_BEEP_ON_MS = 180; // Warning beep ON duration (ms)
constexpr unsigned long BUZZER_BEEP_OFF_MS = 320;// Warning beep pause duration (ms)

// =============================================================================
// 4. ULTRASONIC CHANNEL MANAGEMENT & 3-SAMPLE MEDIAN FILTER
// =============================================================================
enum UsIndex : uint8_t
{
    US_FRONT = 0,
    US_LEFT = 1,
    US_RIGHT = 2,
    US_COUNT = 3
};

struct UltrasonicChannel
{
    UltrasonicChannel(const char* channelLabel, UltrasonicPins pins, float sensorInsetCm)
        : label(channelLabel),
          insetCm(sensorInsetCm),
          // Always keep calibrationOffsetCm = 0.0f so obstacles at ~2 cm don't drop below MIN_DISTANCE_CM into -1.0f
          sensor(pins.trigger, pins.echo, 0.0f),
          samples{OPEN_SPACE_CM, OPEN_SPACE_CM, OPEN_SPACE_CM},
          sampleCount(0),
          sampleIndex(0),
          lastRawCm(-1.0f),
          filteredCm(OPEN_SPACE_CM)
    {
    }

    const char* label;
    float insetCm;
    Ultrasonic sensor;
    float samples[3];
    uint8_t sampleCount;
    uint8_t sampleIndex;
    float lastRawCm;
    float filteredCm;
};

UltrasonicChannel ultrasonicChannels[US_COUNT] = {
    {"front", FRONT_US_PINS, FRONT_US_INSET_CM},
    {"left", LEFT_US_PINS, LEFT_US_INSET_CM},
    {"right", RIGHT_US_PINS, RIGHT_US_INSET_CM}
};

// Returns the median of the 3 most recent samples to reject single-ping spikes or dropped echoes
float median3(float a, float b, float c)
{
    if (a > b)
    {
        const float tmp = a;
        a = b;
        b = tmp;
    }
    if (b > c)
    {
        b = c;
    }
    if (a > b)
    {
        b = a;
    }
    return b;
}

// Clears the filter buffers for all 3 sensors (called at boot and right after finishing a turn)
void resetUltrasonicFilters()
{
    for (auto& channel : ultrasonicChannels)
    {
        channel.samples[0] = OPEN_SPACE_CM;
        channel.samples[1] = OPEN_SPACE_CM;
        channel.samples[2] = OPEN_SPACE_CM;
        channel.sampleCount = 0;
        channel.sampleIndex = 0;
        channel.lastRawCm = -1.0f;
        channel.filteredCm = OPEN_SPACE_CM;
    }
}

// Returns true once every sensor has collected at least 3 fresh samples (~540ms at 60ms per channel)
bool allSensorsReady()
{
    for (const auto& channel : ultrasonicChannels)
    {
        if (channel.sampleCount < 3)
        {
            return false;
        }
    }
    return true;
}

float getFilteredDistanceCm(UsIndex index)
{
    return ultrasonicChannels[index].filteredCm;
}

// Pings one ultrasonic sensor per interval in round-robin order (keeps loop responsive for the 350ms watchdog)
void updateUltrasonics()
{
    static uint8_t nextChannel = 0;
    static unsigned long lastPingAt = 0;
    const unsigned long now = millis();

    // Wait until at least 60ms have elapsed since the previous trigger pulse
    if (now - lastPingAt < US_PING_INTERVAL_MS)
    {
        return;
    }

    // Record timestamp before blocking pulseIn() so consecutive triggers remain spaced >= 60ms apart
    lastPingAt = now;

    UltrasonicChannel& channel = ultrasonicChannels[nextChannel];
    nextChannel = (nextChannel + 1) % US_COUNT;

    // Call Ultrasonic library: returns cm in [2.0 .. 200.0], or -1.0f if > 200cm / timeout
    const float rawCm = channel.sensor.readDistanceCm();
    channel.lastRawCm = rawCm;

    float measuredCm = OPEN_SPACE_CM;
    if (rawCm > 0.0f)
    {
        // Subtract chassis recess inset to get clearance from the outer bumper/frame edge
        const float gapCm = rawCm - channel.insetCm;
        measuredCm = (gapCm > 0.0f) ? gapCm : 0.0f;
    }
    else
    {
        // rawCm == -1.0f (INVALID_DISTANCE_CM): in Ultrasonic.cpp, this occurs when no obstacle
        // is detected within 200 cm (or on 30ms echo timeout) -> treat as 200 cm open space.
        measuredCm = OPEN_SPACE_CM;
    }

    // Push the new measurement into the 3-element ring buffer and update filteredCm
    channel.samples[channel.sampleIndex] = measuredCm;
    channel.sampleIndex = (channel.sampleIndex + 1) % 3;
    if (channel.sampleCount < 3)
    {
        ++channel.sampleCount;
    }

    if (channel.sampleCount == 1)
    {
        channel.filteredCm = channel.samples[0];
    }
    else if (channel.sampleCount == 2)
    {
        channel.filteredCm = (channel.samples[0] + channel.samples[1]) * 0.5f;
    }
    else
    {
        channel.filteredCm = median3(channel.samples[0], channel.samples[1], channel.samples[2]);
    }
}

// =============================================================================
// 5. OBSTACLE BUZZER CONTROL, STATE MACHINE & SERIAL DEBUG LOGGING
// =============================================================================
void setBuzzerOutput(bool on)
{
    static bool currentlyOn = false;
    if (on == currentlyOn)
    {
        return;
    }
    currentlyOn = on;

    if (BUZZER_IS_ACTIVE)
    {
        digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
    }
    else if (on)
    {
        tone(BUZZER_PIN, BUZZER_FREQ_HZ);
    }
    else
    {
        noTone(BUZZER_PIN);
        digitalWrite(BUZZER_PIN, LOW);
    }
}

// Drives a non-blocking repeating warning beep while obstacleAlert is true
void updateBuzzer(unsigned long now, bool obstacleAlert)
{
    static bool alertActive = false;
    static bool beepOn = false;
    static unsigned long lastToggleAt = 0;

    if (!obstacleAlert)
    {
        alertActive = false;
        beepOn = false;
        setBuzzerOutput(false);
        return;
    }

    if (!alertActive)
    {
        alertActive = true;
        beepOn = true;
        lastToggleAt = now;
        setBuzzerOutput(true);
        return;
    }

    const unsigned long intervalMs = beepOn ? BUZZER_BEEP_ON_MS : BUZZER_BEEP_OFF_MS;
    if (now - lastToggleAt >= intervalMs)
    {
        beepOn = !beepOn;
        lastToggleAt = now;
        setBuzzerOutput(beepOn);
    }
}

enum class RobotState
{
    STARTUP_WAIT,     // Wait 3s after boot + wait for 3 initial samples from every sensor
    MOVING_FORWARD,   // Driving continuously straight forward
    WAITING_OBSTACLE, // Stopped in front of an obstacle, waiting up to 10s
    TURNING,          // Obstacle remained after 10s -> rotating toward an open direction (90 or 180 deg)
    POST_TURN_SETTLE, // Rotation finished -> settle briefly and gather 3 fresh readings before moving
    MANUAL_STOP       // Emergency stop triggered via Serial Monitor (send 'r' to resume)
};

RobotState currentState = RobotState::STARTUP_WAIT;
unsigned long stateStartedAt = 0;
unsigned long clearSinceMs = 0;

const char* stateToString(RobotState state)
{
    switch (state)
    {
        case RobotState::STARTUP_WAIT:
            return "STARTUP_WAIT";
        case RobotState::MOVING_FORWARD:
            return "MOVING_FORWARD";
        case RobotState::WAITING_OBSTACLE:
            return "WAITING_OBSTACLE";
        case RobotState::TURNING:
            return "TURNING";
        case RobotState::POST_TURN_SETTLE:
            return "POST_TURN_SETTLE";
        case RobotState::MANUAL_STOP:
            return "MANUAL_STOP";
    }
    return "UNKNOWN";
}

void printChannelReading(const char* tag, const UltrasonicChannel& channel)
{
    Serial.printf("%s=%.1fcm(", tag, static_cast<double>(channel.filteredCm));
    if (channel.lastRawCm < 0.0f)
    {
        Serial.print("raw=-1.0/NO_ECHO");
    }
    else
    {
        Serial.printf("raw=%.1f", static_cast<double>(channel.lastRawCm));
    }
    Serial.print(")");
}

void printDebugStatus(unsigned long now)
{
    static unsigned long lastPrintAt = 0;
    if (now - lastPrintAt < DEBUG_PRINT_INTERVAL_MS)
    {
        return;
    }
    lastPrintAt = now;

    Serial.printf("[%s] ", stateToString(currentState));
    printChannelReading("F", ultrasonicChannels[US_FRONT]);
    Serial.print(" | ");
    printChannelReading("L", ultrasonicChannels[US_LEFT]);
    Serial.print(" | ");
    printChannelReading("R", ultrasonicChannels[US_RIGHT]);
    Serial.printf(" | Driver=%s", robot.isDriverAlive() ? "OK" : "NO_REPLY");

    if (currentState == RobotState::STARTUP_WAIT)
    {
        const unsigned long elapsed = now - stateStartedAt;
        const unsigned long remainMs = (elapsed < STARTUP_DELAY_MS) ? (STARTUP_DELAY_MS - elapsed) : 0;
        Serial.printf(" | StartIn=%.1fs", static_cast<double>(remainMs) / 1000.0);
    }
    else if (currentState == RobotState::WAITING_OBSTACLE)
    {
        const unsigned long elapsed = now - stateStartedAt;
        const unsigned long remainMs = (elapsed < OBSTACLE_WAIT_MS) ? (OBSTACLE_WAIT_MS - elapsed) : 0;
        Serial.printf(" | WaitLeft=%.1fs", static_cast<double>(remainMs) / 1000.0);
    }

    Serial.println();
}

// =============================================================================
// 6. SETUP() - ONE-TIME HARDWARE INITIALIZATION ON BOOT
// =============================================================================
void setup()
{
    // Release default UART0 from GPIO 43/44 so MotionSerial (UART1) can use them cleanly.
    // Note: USB Serial Monitor remains active because ARDUINO_USB_CDC_ON_BOOT=1 is enabled.
    Serial0.end();
    Serial.begin(115200);

    // Initialize high-speed UART1 link (115200 baud) to the Cytron Motion 2350 Pro driver board
    MotionSerial.setRxBufferSize(512);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Configure Trigger/Echo pin modes for all 3 ultrasonic channels
    for (auto& channel : ultrasonicChannels)
    {
        channel.sensor.begin();
    }
    resetUltrasonicFilters();

    // Configure the external obstacle-warning buzzer on GPIO 11 and play a 120ms boot self-test beep
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    setBuzzerOutput(true);
    delay(120);
    setBuzzerOutput(false);

    // Initialize drivetrain (enables closed-loop PID by default and sends an initial stop packet)
    robot.begin(MotionSerial);

    // Temporarily disable closed-loop PID to drive all 4 wheels with direct open-loop PWM.
    // This bypasses M2 (Front-Right) PID runaway until its encoder wiring/polarity (GP6/GP7
    // on the Motion 2350 Pro) is inspected and fixed.
    robot.setPidEnabled(false);

    // Start in the 3-second startup wait state
    currentState = RobotState::STARTUP_WAIT;
    stateStartedAt = millis();
    clearSinceMs = 0;

    Serial.println("============================================================");
    Serial.println("ROBOT TEST: FORWARD CRUISE + ULTRASONIC OBSTACLE AVOIDANCE");
    Serial.println("Drive Mode: Open-loop raw PWM (PID disabled)");
    Serial.printf("Buzzer Pin: GPIO %u\n", BUZZER_PIN);
    Serial.printf("US Pins -> Front(T=%u,E=%u) | Left(T=%u,E=%u) | Right(T=%u,E=%u)\n",
                  FRONT_US_PINS.trigger, FRONT_US_PINS.echo,
                  LEFT_US_PINS.trigger, LEFT_US_PINS.echo,
                  RIGHT_US_PINS.trigger, RIGHT_US_PINS.echo);
    Serial.printf("Thresholds: Stop<=%.0fcm | Clear>=%.0fcm | SideOpen>=%.0fcm | Wait=%.0fs\n",
                  static_cast<double>(OBSTACLE_STOP_CM),
                  static_cast<double>(OBSTACLE_CLEAR_CM),
                  static_cast<double>(SIDE_CLEAR_CM),
                  static_cast<double>(OBSTACLE_WAIT_MS) / 1000.0);
    Serial.println("Auto-starting in 3s. Optional Serial: 'x' or 'd' = stop, 'r' = resume.");
    Serial.println("============================================================");
}

// =============================================================================
// 7. LOOP() - NON-BLOCKING MAIN CONTROL LOOP
// =============================================================================
void loop()
{
    // -------------------------------------------------------------------------
    // STEP 1: ALWAYS CALL robot.update() AT THE TOP OF EVERY LOOP ITERATION
    // - Streams ControlPackets at 50Hz (every 20ms) to the Motion 2350 Pro so its
    //   350ms hardware safety watchdog never trips and cuts motor power.
    // - Automatically stops non-blocking maneuvers (rotateAngle(..., false)) when complete.
    // - Parses incoming encoder RPM / PWM telemetry feedback from the motor driver.
    // -------------------------------------------------------------------------
    robot.update();

    // -------------------------------------------------------------------------
    // STEP 2: POLL ONE ULTRASONIC SENSOR EVERY 60ms (NON-BLOCKING ROUND-ROBIN)
    // - Measures Front -> Left -> Right sequentially, applies a 3-sample median filter,
    //   and prevents acoustic crosstalk between adjacent ultrasonic transducers.
    // -------------------------------------------------------------------------
    updateUltrasonics();

    // Capture current timestamp right after the sensor update
    const unsigned long now = millis();

    // -------------------------------------------------------------------------
    // STEP 3: OPTIONAL SERIAL MONITOR COMMANDS (WHEN USB IS CONNECTED)
    // - Send 'x' or 'd' at any time for an immediate emergency stop.
    // - Send 'r' while in MANUAL_STOP to restart the 3-second startup sequence.
    // -------------------------------------------------------------------------
    const int key = Serial.read();
    if (key == 'x' || key == 'd')
    {
        robot.stop();
        currentState = RobotState::MANUAL_STOP;
        Serial.println(">>> EMERGENCY STOP (send 'r' to resume)");
    }
    else if (key == 'r' && currentState == RobotState::MANUAL_STOP)
    {
        resetUltrasonicFilters();
        stateStartedAt = now;
        clearSinceMs = 0;
        currentState = RobotState::STARTUP_WAIT;
        Serial.println(">>> RESUMING IN 3 SECONDS...");
    }

    // Read the median-filtered front clearance (cm)
    const float frontCm = getFilteredDistanceCm(US_FRONT);

    // -------------------------------------------------------------------------
    // STEP 4: ROBOT BEHAVIOR STATE MACHINE
    // Important: Only call robot.moveForward() / robot.stop() / robot.rotateAngle()
    // ONCE on state transitions (do NOT call drive commands on every loop iteration)
    // to avoid flooding the 115200 baud UART; robot.update() maintains the 50Hz stream.
    // -------------------------------------------------------------------------
    switch (currentState)
    {
        // --- State 1: Wait 3s after boot and ensure all 3 sensors have 3 fresh samples ---
        case RobotState::STARTUP_WAIT:
        {
            if ((now - stateStartedAt >= STARTUP_DELAY_MS) && allSensorsReady())
            {
                robot.beep(); // Short confirmation beep before starting movement

                if (frontCm <= OBSTACLE_STOP_CM)
                {
                    // An obstacle is already right in front -> stay stopped and start the 10s wait
                    stateStartedAt = now;
                    clearSinceMs = 0;
                    currentState = RobotState::WAITING_OBSTACLE;
                    Serial.printf(">>> STARTUP: Front obstacle detected (%.1f cm), waiting 10s!\n",
                                  static_cast<double>(frontCm));
                }
                else
                {
                    // Front path is clear -> start driving straight forward
                    robot.moveForward(FORWARD_SPEED);
                    currentState = RobotState::MOVING_FORWARD;
                    Serial.printf(">>> STARTUP: Path clear (%.1f cm), moving forward!\n",
                                  static_cast<double>(frontCm));
                }
            }
            break;
        }

        // --- State 2: Cruising straight forward ---
        case RobotState::MOVING_FORWARD:
        {
            // Continuously monitor front sensor: brake immediately if obstacle <= 30 cm
            if (frontCm <= OBSTACLE_STOP_CM)
            {
                robot.stop();
                robot.beep();
                stateStartedAt = now;
                clearSinceMs = 0;
                currentState = RobotState::WAITING_OBSTACLE;
                Serial.printf(">>> OBSTACLE DETECTED (%.1f cm)! Stopped, waiting up to 10s...\n",
                              static_cast<double>(frontCm));
            }
            break;
        }

        // --- State 3: Stopped in front of an obstacle, waiting up to 10s ---
        case RobotState::WAITING_OBSTACLE:
        {
            // (a) Early clear check: if the obstacle moves away before 10s elapse,
            //     require front clearance >= 38 cm continuously for 500ms to filter out transient noise
            if (frontCm >= OBSTACLE_CLEAR_CM)
            {
                if (clearSinceMs == 0)
                {
                    clearSinceMs = now;
                }
                else if (now - clearSinceMs >= CLEAR_CONFIRM_MS)
                {
                    robot.moveForward(FORWARD_SPEED);
                    currentState = RobotState::MOVING_FORWARD;
                    Serial.printf(">>> OBSTACLE CLEARED EARLY (%.1f cm)! Resuming forward.\n",
                                  static_cast<double>(frontCm));
                    break;
                }
            }
            else
            {
                // Obstacle still present or returned -> reset the clear-confirmation timer
                clearSinceMs = 0;
            }

            // (b) If 10 seconds elapsed and the obstacle is still blocking the front -> find an open direction
            if (now - stateStartedAt >= OBSTACLE_WAIT_MS)
            {
                const float leftCm = getFilteredDistanceCm(US_LEFT);
                const float rightCm = getFilteredDistanceCm(US_RIGHT);
                const bool leftOpen = (leftCm >= SIDE_CLEAR_CM);
                const bool rightOpen = (rightCm >= SIDE_CLEAR_CM);

                robot.beep(); // Short tone to signal upcoming turn

                // Use rotateAngle(..., blocking = false) so robot.update() keeps running at 50Hz
                // and emergency stop commands ('x'/'d') remain responsive during the turn.
                if (leftOpen && rightOpen)
                {
                    // Both sides open (>= 40 cm) -> rotate 90 deg toward the wider side
                    if (leftCm >= rightCm)
                    {
                        robot.rotateAngle(TURN_SPEED, 90.0f, false); // Positive = Rotate Left (CCW)
                        Serial.printf(">>> 10s ELAPSED! L=%.1fcm, R=%.1fcm -> Rotating LEFT 90 deg.\n",
                                      static_cast<double>(leftCm), static_cast<double>(rightCm));
                    }
                    else
                    {
                        robot.rotateAngle(-TURN_SPEED, 90.0f, false); // Negative = Rotate Right (CW)
                        Serial.printf(">>> 10s ELAPSED! L=%.1fcm, R=%.1fcm -> Rotating RIGHT 90 deg.\n",
                                      static_cast<double>(leftCm), static_cast<double>(rightCm));
                    }
                }
                else if (leftOpen)
                {
                    // Only Left side is open -> rotate Left 90 deg
                    robot.rotateAngle(TURN_SPEED, 90.0f, false);
                    Serial.printf(">>> 10s ELAPSED! Only LEFT open (L=%.1fcm, R=%.1fcm) -> Rotating LEFT 90 deg.\n",
                                  static_cast<double>(leftCm), static_cast<double>(rightCm));
                }
                else if (rightOpen)
                {
                    // Only Right side is open -> rotate Right 90 deg
                    robot.rotateAngle(-TURN_SPEED, 90.0f, false);
                    Serial.printf(">>> 10s ELAPSED! Only RIGHT open (L=%.1fcm, R=%.1fcm) -> Rotating RIGHT 90 deg.\n",
                                  static_cast<double>(leftCm), static_cast<double>(rightCm));
                }
                else
                {
                    // Both sides blocked (< 40 cm) -> turn around 180 deg
                    robot.rotateAngle(TURN_SPEED, 180.0f, false);
                    Serial.printf(">>> 10s ELAPSED! Both sides blocked (L=%.1fcm, R=%.1fcm) -> Turning around 180 deg.\n",
                                  static_cast<double>(leftCm), static_cast<double>(rightCm));
                }

                stateStartedAt = now;
                currentState = RobotState::TURNING;
            }
            break;
        }

        // --- State 4: Rotating in place toward the chosen open direction ---
        case RobotState::TURNING:
        {
            // Wait until robot.update() finishes the timed rotation started by rotateAngle()
            if (!robot.isBusy())
            {
                robot.stop();
                // Discard pre-turn ultrasonic samples before evaluating the new heading
                resetUltrasonicFilters();
                stateStartedAt = now;
                currentState = RobotState::POST_TURN_SETTLE;
                Serial.println(">>> TURN COMPLETE! Settling and sampling new heading...");
            }
            break;
        }

        // --- State 5: Brief 600ms settle + wait for 3 fresh samples in the new heading ---
        case RobotState::POST_TURN_SETTLE:
        {
            if ((now - stateStartedAt >= TURN_SETTLE_MS) && allSensorsReady())
            {
                if (frontCm > OBSTACLE_STOP_CM)
                {
                    // New direction is clear -> resume driving forward
                    robot.moveForward(FORWARD_SPEED);
                    currentState = RobotState::MOVING_FORWARD;
                    Serial.printf(">>> NEW HEADING CLEAR (F=%.1f cm)! Moving forward.\n",
                                  static_cast<double>(frontCm));
                }
                else
                {
                    // New direction is also blocked -> enter 10s wait state again
                    clearSinceMs = 0;
                    stateStartedAt = now;
                    currentState = RobotState::WAITING_OBSTACLE;
                    Serial.printf(">>> NEW HEADING BLOCKED (F=%.1f cm)! Waiting 10s...\n",
                                  static_cast<double>(frontCm));
                }
            }
            break;
        }

        // --- State 6: Manual stop via Serial Monitor ---
        case RobotState::MANUAL_STOP:
        {
            // Remain stopped until the user sends 'r' over Serial
            break;
        }
    }

    // -------------------------------------------------------------------------
    // STEP 5: NON-BLOCKING OBSTACLE WARNING BUZZER ON GPIO 11
    // - Emits a repeating warning beep whenever the robot is stopped for an
    //   obstacle (WAITING_OBSTACLE), and silences immediately in all other states
    //   (including MANUAL_STOP).
    // -------------------------------------------------------------------------
    updateBuzzer(now, currentState == RobotState::WAITING_OBSTACLE);

    // -------------------------------------------------------------------------
    // STEP 6: PRINT SENSOR DISTANCES & STATE TELEMETRY EVERY 500ms
    // -------------------------------------------------------------------------
    printDebugStatus(now);
}

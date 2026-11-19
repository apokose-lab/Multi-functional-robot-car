# Mecanum Drivetrain Developer Guide

Welcome to the software guide for the **4-Wheel Mecanum Robot Car**.  
This document explains how the drivetrain architecture works under the hood, describes every control function in plain English, and provides copy-paste examples so any team member can command the robot in a single function call.

---

## 1. System Architecture & How It Works

The robot splits responsibilities across two microcontrollers communicating over a high-speed **UART** link (`115200` baud):

```text
+-----------------------------------+          UART (115200 baud)          +-----------------------------------+
|      ESP32-S3 Brain Board         |  ControlPacket (50 Hz, 13 bytes)     |   Cytron Motion 2350 Pro Driver   |
|  (src/ESP32_Brain/main.cpp)       | -----------------------------------> |  (src/Motion2350_Driver/main.cpp) |
|                                   |  GPIO 43 (TX) --------> GP17 (RX)    |                                   |
|  - MecanumKinematics (Drivetrain) |                                      |  - MotorPID (50Hz Closed-Loop)    |
|  - Your Line/Sensor/Arm Logic     |  MotionFeedbackPacket (25Hz, 23 B)   |  - 4x H-Bridge PWM (20kHz)        |
|                                   | <----------------------------------- |  - 4x Quadrature Encoders (ISR)   |
|                                   |  GPIO 44 (RX) <-------- GP16 (TX)    |  - 350ms Safety Watchdog + Buzzer |
+-----------------------------------+                                      +-----------------------------------+
```

### Where Should Team Members Write Code?
- **Write all application code in [`src/ESP32_Brain/main.cpp`](src/ESP32_Brain/main.cpp)** using the `Drivetrain` (`robot`) object alongside your own sensor/arm libraries.
- **Do NOT modify `src/Motion2350_Driver/main.cpp` or `lib/MotorPID/`** unless motor wiring or PID tuning constants change. The Motion 2350 Pro acts as a dedicated real-time motor slave.

### Key Internal Mechanisms
1. **Closed-Loop PID Enabled by Default**:
   - When you pass a speed command (`-255..+255`), the drivetrain automatically scales it into a wheel **RPM target** (`-120..+120 RPM`) and sends `pid_enable = 1`.
   - The Motion 2350 Pro runs a 50Hz PID + Feed-Forward controller using the 4 quadrature encoders (`680 pulses/rev`) so all 4 wheels spin at the exact requested speed even if battery voltage drops or floor friction changes.
   - You can switch to open-loop raw PWM anytime using `robot.setPidEnabled(false)`.
2. **Proportional Wheel Normalization**:
   - Mecanum inverse kinematics computes each wheel setpoint as:
     - `M1 (Front-Left)  = vx - vy - omega`
     - `M2 (Front-Right) = vx + vy + omega`
     - `M3 (Rear-Left)   = vx + vy - omega`
     - `M4 (Rear-Right)  = vx - vy + omega`
   - If you combine forward motion, strafing, and rotation at high values (e.g., `vx=200, vy=150, omega=100`), a raw sum would exceed `255`. Instead of hard-clipping (which distorts the travel angle), `MecanumKinematics` scales all 4 wheels down proportionally so the robot keeps the exact intended direction and turn ratio.
3. **50Hz Stream & 350ms Hardware Watchdog**:
   - The Motion 2350 Pro automatically stops all motors if it receives no UART packet for `350 ms`.
   - Calling `robot.update()` in `loop()` automatically streams `ControlPacket` every `20 ms` (50Hz) and checks any active distance/angle/duration timer so the robot stops smoothly when a maneuver finishes.

---

## 2. Coordinate System & Sign Conventions

```text
                +vx (Forward, 0 deg)
                        ^
                        |
          [M1: FL] ----------- [M2: FR]
             |                     |
+vy (Left)   |      +omega         |   -vy (Right)
<------------+       (CCW)         +------------>
  +90 deg    |                     |     -90 deg
          [M3: RL] ----------- [M4: RR]
                        |
                        v
               -vx (Backward, 180 deg)
```

| Parameter | Valid Range | Positive (`+`) Meaning | Negative (`-`) Meaning |
| :--- | :--- | :--- | :--- |
| **`vx`** | `-255 .. +255` | Move **Forward** | Move **Backward** |
| **`vy`** | `-255 .. +255` | Strafe **Left** | Strafe **Right** |
| **`omega`** | `-255 .. +255` | Rotate **Left (Counter-Clockwise / CCW)** | Rotate **Right (Clockwise / CW)** |
| **`angleDeg`** | Any float degrees | `0°` Forward, `90°` Left, `180°` Backward | `-90°` (`270°`) Right |
| **`pwm` / `speedPwm`** | `-255 .. +255` | Speed magnitude in target direction | Reverses direction when negative |

---

## 3. Drivetrain Function Reference (`Drivetrain` / `MecanumKinematics`)

Declare an instance in `src/ESP32_Brain/main.cpp`:
```cpp
#include <MecanumKinematics.h>
Drivetrain robot; // Alias for MecanumKinematics
```

### A. Setup & Loop Maintenance (Required)

| Function | Description |
| :--- | :--- |
| `robot.begin(MotionSerial)` | Initializes the drivetrain, binds it to the UART port (`MotionSerial`), enables PID mode (`1`), and sends an initial stop packet. Call once in `setup()`. |
| `robot.update()` | **Must be called on every iteration of `loop()`.** Automatically stops timed/distance/angle moves when complete, reads encoder feedback packets, and sends 50Hz `ControlPacket` updates. |
| `robot.waitUntilDone()` | Blocks in a safe loop (calling `update()`) until an active non-blocking maneuver finishes. |

---

### B. Continuous Movement Functions

Use these when your code decides what to do continuously inside `loop()` (e.g., line following or sensor-driven navigation).  
Any continuous command automatically cancels a previously running timer and keeps moving until you issue another command or call `robot.stop()`.

| Function | Parameters | Behavior |
| :--- | :--- | :--- |
| `robot.drive(vx, vy, omega = 0)` | `vx`, `vy`, `omega` in `-255..255` | **Master movement function.** Moves forward/backward (`vx`), strafes left/right (`vy`), and rotates (`omega`) simultaneously. |
| `robot.moveAngle(angleDeg, speedPwm, omega = 0)` | `angleDeg` (deg), `speedPwm` (`-255..255`), `omega` (`-255..255`) | Drives along any compass angle (`0°`=Fwd, `90°`=Left, `-90°`=Right, `180°`=Back) while optionally spinning at `omega`. |
| `robot.moveForward(pwm, omega = 0)` | `pwm`, `omega` in `-255..255` | Drives straight forward (or curves gently if `omega != 0`). |
| `robot.moveBackward(pwm, omega = 0)` | `pwm`, `omega` in `-255..255` | Drives straight backward (or curves gently if `omega != 0`). |
| `robot.strafeLeft(pwm, omega = 0)` | `pwm`, `omega` in `-255..255` | Slides sideways left without rotating (unless `omega != 0`). |
| `robot.strafeRight(pwm, omega = 0)` | `pwm`, `omega` in `-255..255` | Slides sideways right without rotating (unless `omega != 0`). |
| `robot.rotate(omega)` | `omega` in `-255..255` | Spins in place (`+` = CCW / Left, `-` = CW / Right). |
| `robot.stop()` | None | Immediately stops all 4 wheels, cancels any timer, and transmits a stop packet. |

---

### C. Distance, Angle & Timed Functions (Auto-Stop)

These functions calculate the required travel duration using the robot's physical wheel/chassis geometry and closed-loop PID RPM, run the maneuver, and **automatically stop** when finished.

All functions in this group accept an optional **`bool blocking = false`** parameter:
- **`blocking = true`**: Waits inside the function until the maneuver completes and stops, then moves to the next line of code. Great for simple step-by-step sequences.
- **`blocking = false`** (default): Starts the maneuver and returns immediately so your `loop()` can keep reading sensors in real time. Poll `robot.isBusy()` to know when it finishes, and make sure `robot.update()` keeps running!

| Function | Parameters | Behavior |
| :--- | :--- | :--- |
| `robot.moveDistance(vx, vy, distanceCm, omega = 0, blocking = false)` | `vx`, `vy` (`-255..255`), `distanceCm` (> 0), `omega` (`-255..255`), `blocking` | Moves along velocity vector `(vx, vy)` for `distanceCm` centimeters while optionally spinning at `omega`, then stops. |
| `robot.moveAngleDistance(angleDeg, speedPwm, distanceCm, omega = 0, blocking = false)` | `angleDeg` (deg), `speedPwm` (`-255..255`), `distanceCm` (> 0), `omega`, `blocking` | Moves at compass direction `angleDeg` at `speedPwm` for `distanceCm` centimeters, then stops. |
| `robot.rotateAngle(omega, angleDeg, blocking = false)` | `omega` (`-255..255`), `angleDeg` (deg), `blocking` | Rotates in place by `angleDeg` degrees at speed `omega`, then stops. **Sign rule:** Both positive = turn Left (CCW); if either `omega` or `angleDeg` is negative = turn Right (CW). |
| `robot.driveFor(vx, vy, omega, durationMs, blocking = false)` | `vx`, `vy`, `omega` (`-255..255`), `durationMs` (ms), `blocking` | Drives with `(vx, vy, omega)` for exactly `durationMs` milliseconds, then stops. |

---

### D. Buzzer, PID Mode & Diagnostics

| Function | Description |
| :--- | :--- |
| `robot.beep()` | Sounds a short confirmation tone (`1800 Hz`, `80 ms`) on the Motion 2350 Pro onboard buzzer. |
| `robot.setPidEnabled(enabled)` | `true` (default) = Closed-loop PID speed control (RPM); `false` = Open-loop raw PWM (`-255..255`). |
| `robot.isPidEnabled()` | Returns `true` if PID mode is active, `false` if raw PWM mode is active. |
| `robot.isBusy()` | Returns `true` while a non-blocking `moveDistance`, `moveAngleDistance`, `rotateAngle`, or `driveFor` is still running. |
| `robot.isDriverAlive(timeoutMs = 400)` | Returns `true` if the ESP32-S3 has received valid UART feedback from the Motion 2350 Pro within `timeoutMs`. |
| `robot.getTarget(wheelIndex)` | Latest computed setpoint (RPM or PWM) for wheel `0..3` (`0=FL, 1=FR, 2=RL, 3=RR`). |
| `robot.getActualRpm(wheelIndex)` | Latest measured encoder speed (RPM) for wheel `0..3`. |
| `robot.getActualPwm(wheelIndex)` | Latest applied H-bridge PWM (`-255..255`) for wheel `0..3`. |

---

## 4. Practical Usage Examples

### Example 1: Minimal Required Skeleton (`src/ESP32_Brain/main.cpp`)
Every sketch must initialize `MotionSerial` on pins `RX=44`, `TX=43` and call `robot.update()` inside `loop()`:

```cpp
#include <Arduino.h>
#include <MecanumKinematics.h>

constexpr int UART_RX_PIN = 44; // Connects to GP16 (TX) on Motion 2350 Pro
constexpr int UART_TX_PIN = 43; // Connects to GP17 (RX) on Motion 2350 Pro

HardwareSerial MotionSerial(1);
Drivetrain robot;

void setup() {
    Serial.end(); // Free GPIO 43/44 from default UART0
    MotionSerial.setRxBufferSize(512);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    robot.begin(MotionSerial);
}

void loop() {
    robot.update();

    // Add your movement & sensor logic below!
}
```

### Example 2: Simple Step-by-Step Autonomous Sequence (`blocking = true`)
Each line completes its exact distance or turn angle and stops automatically before the next line begins:

```cpp
void runSquarePatrol() {
    // 1. Move forward 50 cm at speed 140, omega = 0
    robot.moveDistance(140, 0, 50.0f, 0, true);

    // 2. Strafe right (-90 deg) for 30 cm at speed 130
    robot.moveAngleDistance(-90.0f, 130, 30.0f, 0, true);

    // 3. Rotate 90 degrees Clockwise (Right) at omega = -120
    robot.rotateAngle(-120, 90.0f, true);

    // 4. Move diagonally Forward-Left (+45 deg) for 40 cm while spinning gently (+20)
    robot.moveAngleDistance(45.0f, 140, 40.0f, 20, true);

    // 5. Sound buzzer to signal completion
    robot.beep();
}
```

### Example 3: Continuous Driving with External Sensors (e.g., Line Following)
Ideal for steering dynamically based on sensor readings inside `loop()`:

```cpp
void loop() {
    robot.update();

    // Read your sensors here...
    bool obstacleDetected = false;
    int16_t steerOmega = 0; // Computed steering correction from line sensors (-255..255)

    if (obstacleDetected) {
        robot.stop();
        // Trigger robotic arm grab sequence here...
    } else {
        // Drive forward at 135 PWM while applying live steering rotation
        robot.drive(135, 0, steerOmega);
    }
}
```

### Example 4: Non-Blocking Distance Move with Early Stop (`blocking = false`)
Start a `100 cm` forward move without freezing `loop()`, so your sensors can abort the move early if needed:

```cpp
bool started = false;

void loop() {
    robot.update();

    if (!started) {
        robot.moveDistance(140, 0, 100.0f, 0, false); // Non-blocking!
        started = true;
    }

    // Read your sensors here...
    bool emergencyStopCondition = false;
    if (robot.isBusy() && emergencyStopCondition) {
        robot.stop();
        robot.beep();
    }
}
```

---

## 5. Calibration & Troubleshooting

### Calibrating Distance & Rotation Angle
All physical constants live at the top of [`lib/MecanumKinematics/MecanumKinematics.h`](lib/MecanumKinematics/MecanumKinematics.h):
- **`WHEEL_DIAMETER_CM = 5.8f`**: Matches 58mm Mecanum wheels.
- **`DEFAULT_MAX_RPM = 120.0f`**: Wheel RPM corresponding to command `255` in PID mode.
- **`CHASSIS_TURN_RADIUS_CM = 16.0f`**: Effective turning radius `(Lx + Ly)` used by `rotateAngle()`.
  - If `rotateAngle(120, 90.0f, true)` turns **more** than 90°, **decrease** `CHASSIS_TURN_RADIUS_CM` slightly (e.g., `14.5f`).
  - If it turns **less** than 90°, **increase** `CHASSIS_TURN_RADIUS_CM` slightly (e.g., `17.5f`).

### Common Pitfalls to Avoid
1. **Never use `delay()` to time a movement**: Using `delay(2000)` stalls `robot.update()`, which causes the Motion 2350 Pro's 350ms safety watchdog to stop the motors! Always use `robot.driveFor(..., durationMs, true)` instead.
2. **Always call `robot.update()` in `loop()`**: Even when cruising continuously with `robot.drive()`, `robot.update()` must run frequently so 50Hz packets continue feeding the motor driver watchdog.
3. **Always ensure `Serial.end()` runs before `MotionSerial.begin(...)`**: GPIO 43 and 44 are shared with default UART0 on ESP32-S3; releasing `Serial` avoids UART contention with the Motion 2350 Pro.

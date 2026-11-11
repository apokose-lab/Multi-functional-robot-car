#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <RobotProtocol.h>
#include <Ultrasonic.h>
#include <MecanumKinematics.h>

// Wi-Fi Access Point (AP) credentials and UDP port for communication with the Laptop Dashboard
const char* AP_SSID = "Mecanum_Robot";
const char* AP_PASS = "12345678";
const uint16_t UDP_PORT = 4210;

// UART pins connected to GP16/GP17 of Motion 2350 Pro (GPIO 16, 17 on ESP32-S3 are reserved for LiDAR)
const int UART_RX_PIN = 44;
const int UART_TX_PIN = 43;

// Peripheral & module instances
WiFiUDP udp;
HardwareSerial MotionSerial(1);
UltrasonicArray ultrasonic;
MecanumKinematics kinematics;
PacketParser<MotionFeedbackPacket, 0xBB, 0x66> feedbackParser;

// Latest motion command state received from the Laptop via UDP
int16_t vx = 0, vy = 0, omega = 0;
uint8_t buzzer = 0, pidEnable = 0, testWheel = 0;
unsigned long lastInputTime = 0;

// Remote client network address for returning UDP telemetry packets
IPAddress laptopIP;
uint16_t laptopPort = 0;
bool hasWifiClient = false;

// Latest feedback data received from the Motion 2350 Pro driver board
MotionFeedbackPacket latestMotionFB;
unsigned long lastMotionFBTime = 0;

/**
 * @brief Initializes UART communication with Motion 2350 Pro, 3x HC-SR04 ultrasonic sensors,
 *        and the Wi-Fi SoftAP + UDP server for wireless control and telemetry.
 */
void setup() {
    // Release default UART0 pins (GPIO 43, 44) so they can be used cleanly by MotionSerial (UART1)
    Serial.end();

    // Configure high-speed UART1 link to the Motion 2350 Pro motor controller
    MotionSerial.setRxBufferSize(512);
    MotionSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Initialize Trigger/Echo pins and hardware interrupts for the 3 ultrasonic sensors
    ultrasonic.begin();

    // Start low-latency Wi-Fi Access Point and begin listening for UDP packets
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 2);
    WiFi.setSleep(false); // Disable Wi-Fi modem sleep to minimize UDP packet latency
    WiFi.setTxPower(WIFI_POWER_15dBm);
    udp.begin(UDP_PORT);

    // Zero-initialize feedback structure before entering main loop
    memset(&latestMotionFB, 0, sizeof(latestMotionFB));
}

/**
 * @brief Main non-blocking coordination loop running on the ESP32-S3 Brain:
 *        1. Receives gamepad commands over Wi-Fi UDP (with a 300ms safety watchdog)
 *        2. Triggers the 3 ultrasonic sensors sequentially every 35ms
 *        3. Parses incoming UART feedback packets from Motion 2350 Pro
 *        4. Computes Mecanum wheel targets and sends UART control packets every 20ms (50Hz)
 *        5. Broadcasts full diagnostic telemetry back to the Laptop over UDP every 50ms (20Hz)
 */
void loop() {
    // 1. Receive gamepad control packet from Laptop over Wi-Fi UDP
    int packetSize = udp.parsePacket();
    if (packetSize == sizeof(JoyPacket)) {
        // Record client IP and port so telemetry can be sent back to the Laptop
        laptopIP = udp.remoteIP();
        laptopPort = udp.remotePort();
        hasWifiClient = true;

        // Read binary JoyPacket payload and update active velocity/mode commands
        JoyPacket joy;
        udp.read(reinterpret_cast<uint8_t*>(&joy), sizeof(JoyPacket));
        vx = joy.vx; vy = joy.vy; omega = joy.omega;
        buzzer = joy.buzzer; pidEnable = joy.pid_enable; testWheel = joy.test_wheel;
        lastInputTime = millis();
    } else if (packetSize > 0) {
        // Discard malformed UDP packets of unexpected size
        udp.flush();
    }

    // Safety watchdog: stop all motion if no UDP packet arrives within 300ms
    if (millis() - lastInputTime > 300) {
        vx = 0; vy = 0; omega = 0; buzzer = 0; testWheel = 0;
    }

    // 2. Trigger the 3 HC-SR04 ultrasonic sensors in a round-robin cycle every 35ms
    ultrasonic.update(35);

    // 3. Read and validate incoming feedback packets from Motion 2350 Pro over UART
    while (feedbackParser.readFrom(MotionSerial, latestMotionFB)) {
        lastMotionFBTime = millis();
    }

    // 4. Compute wheel targets and transmit ControlPacket to Motion 2350 Pro every 20ms (50Hz)
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 20) {
        lastSend = millis();
        ControlPacket pkt = kinematics.buildControlPacket(vx, vy, omega, pidEnable, testWheel, buzzer);
        sendPacket(MotionSerial, pkt);
        buzzer = 0; // Clear one-shot buzzer trigger flag after sending
    }

    // 5. Transmit wireless diagnostic telemetry packet to Laptop every 50ms (20Hz)
    static unsigned long lastTele = 0;
    unsigned long now = millis();
    if (hasWifiClient && (now - lastTele >= 50)) {
        lastTele = now;

        WifiTelemetryPacket tele;
        tele.pid_mode = pidEnable;
        tele.vx = vx; tele.vy = vy; tele.omega = omega;

        // Check if Motion 2350 Pro has sent a valid UART feedback packet within the last 400ms
        bool motionAlive = (now - lastMotionFBTime < 400);
        tele.motion_alive = motionAlive ? 1 : 0;

        // Populate wheel setpoints and actual motor/encoder measurements
        for (int i = 0; i < 4; i++) {
            tele.target_val[i] = kinematics.getTarget(i);
            tele.actual_pwm[i] = motionAlive ? latestMotionFB.actual_pwm[i] : 0;
            tele.actual_rpm[i] = motionAlive ? latestMotionFB.actual_rpm[i] : 0;
            tele.enc_health[i] = motionAlive ? latestMotionFB.enc_health[i] : 0;
        }
        tele.active_gpios = motionAlive ? latestMotionFB.active_gpios : 0;

        // Populate ultrasonic distances and sensor connection status bitmask
        ultrasonic.readTelemetry(tele.us_cm, tele.us_status, 350);
        tele.uptime_ms = now;
        tele.checksum  = calculateChecksum(tele);

        // Send binary telemetry packet over Wi-Fi UDP to the Laptop
        udp.beginPacket(laptopIP, laptopPort);
        udp.write(reinterpret_cast<const uint8_t*>(&tele), sizeof(WifiTelemetryPacket));
        udp.endPacket();
    }
}

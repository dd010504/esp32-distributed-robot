#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "protocol.h"
#include "node_config.h"
#include "RobotNetwork.h"

// ============================================================
// Sensor pins
// ============================================================

constexpr uint8_t TRIG_PIN = 25;
constexpr uint8_t ECHO_PIN = 26;
constexpr uint8_t PIR_PIN = 27;

// ============================================================
// OLED configuration
// ============================================================

constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t OLED_ADDRESS = 0x3C;

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    -1
);

// ============================================================
// Timing
// ============================================================

constexpr unsigned long TELEMETRY_INTERVAL_MS = 250;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 2000;
constexpr unsigned long DISPLAY_INTERVAL_MS = 100;

// ============================================================
// State
// ============================================================

uint32_t heartbeatSequence = 0;

float latestDistanceCm = -1.0f;
bool latestMotionDetected = false;
bool latestDeliverySuccess = false;


// ============================================================
// Read HC-SR04 distance
// ============================================================

float readDistanceCm()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration =
        pulseIn(ECHO_PIN, HIGH, 30000UL);

    if (duration == 0)
    {
        return -1.0f;
    }

    return duration * 0.0343f / 2.0f;
}


// ============================================================
// Read PIR motion sensor
// ============================================================

bool readMotion()
{
    return digitalRead(PIR_PIN) == HIGH;
}


// ============================================================
// Update SENSOR OLED
// ============================================================

void updateDisplay()
{
    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // --------------------------------------------------------
    // Yellow section: rows 0-15
    // --------------------------------------------------------

    display.setCursor(26, 3);
    display.println("SENSOR NODE");

    // --------------------------------------------------------
    // Blue section: rows 16-63
    // --------------------------------------------------------

    display.setCursor(0, 18);

    display.print("DIST: ");

    if (latestDistanceCm < 0)
    {
        display.println("NO ECHO");
    }
    else
    {
        display.print(latestDistanceCm, 1);
        display.println(" cm");
    }


    display.setCursor(0, 30);

    display.print("MOTION: ");

    display.println(
        latestMotionDetected
            ? "YES"
            : "NO"
    );


    display.setCursor(0, 42);

    display.print("LINK: ");

    display.println(
        latestDeliverySuccess
            ? "OK"
            : "FAIL"
    );


    display.setCursor(0, 54);

    display.println("NODE: SENSOR");


    display.display();
}


// ============================================================
// Send sensor telemetry
// ============================================================

void sendSensorData()
{
    SensorPacket packet = {};

    packet.header.protocolVersion =
        PROTOCOL_VERSION;

    packet.header.source =
        NodeId::SENSOR;

    packet.header.destination =
        NodeId::MASTER;

    packet.header.type =
        PacketType::SENSOR_DATA;


    // --------------------------------------------------------
    // Read real sensors
    // --------------------------------------------------------

    latestDistanceCm =
        readDistanceCm();

    latestMotionDetected =
        readMotion();


    packet.distanceCm =
        latestDistanceCm;

    packet.motionDetected =
        latestMotionDetected ? 1 : 0;


    // --------------------------------------------------------
    // Future sensors
    // --------------------------------------------------------

    packet.temperature = 0.0f;
    packet.humidity = 0.0f;

    packet.lightLevel = 0;

    packet.accelX = 0.0f;
    packet.accelY = 0.0f;
    packet.accelZ = 0.0f;


    // --------------------------------------------------------
    // Send to MASTER
    // --------------------------------------------------------

    bool queued =
        RobotNetwork::send(
            MASTER_MAC,
            &packet,
            sizeof(packet)
        );


    if (!queued)
    {
        latestDeliverySuccess = false;

        Serial.println(
            "[SENSOR] Telemetry failed to queue"
        );

        return;
    }


    // Give send callback time to update.
    delay(20);

    latestDeliverySuccess =
        RobotNetwork::lastSendSucceeded();


    // --------------------------------------------------------
    // Serial debug
    // --------------------------------------------------------

    Serial.print(
        "[SENSOR] Distance: "
    );

    if (latestDistanceCm < 0)
    {
        Serial.println(
            "NO ECHO"
        );
    }
    else
    {
        Serial.print(
            latestDistanceCm,
            1
        );

        Serial.println(
            " cm"
        );
    }


    Serial.print(
        "[SENSOR] Motion: "
    );

    Serial.println(
        latestMotionDetected
            ? "YES"
            : "NO"
    );


    Serial.print(
        "[SENSOR] ESP-NOW delivery: "
    );

    Serial.println(
        latestDeliverySuccess
            ? "SUCCESS"
            : "FAILED"
    );
}


// ============================================================
// Send heartbeat
// ============================================================

void sendHeartbeat()
{
    HeartbeatPacket heartbeat = {};

    heartbeat.header.protocolVersion =
        PROTOCOL_VERSION;

    heartbeat.header.source =
        NodeId::SENSOR;

    heartbeat.header.destination =
        NodeId::MASTER;

    heartbeat.header.type =
        PacketType::HEARTBEAT;

    heartbeat.uptimeMs =
        millis();

    heartbeat.sequenceNumber =
        ++heartbeatSequence;


    RobotNetwork::send(
        MASTER_MAC,
        &heartbeat,
        sizeof(heartbeat)
    );
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println(
        "=============================="
    );

    Serial.println(
        " ESP32 DISTRIBUTED ROBOT"
    );

    Serial.println(
        "=============================="
    );

    Serial.println(
        "NODE: SENSOR"
    );


    // --------------------------------------------------------
    // Sensor setup
    // --------------------------------------------------------

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(PIR_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);


    // --------------------------------------------------------
    // OLED setup
    // --------------------------------------------------------

    Wire.begin(
        OLED_SDA,
        OLED_SCL
    );


    if (
        !display.begin(
            SSD1306_SWITCHCAPVCC,
            OLED_ADDRESS
        )
    )
    {
        Serial.println(
            "[OLED] Initialization failed"
        );

        while (true)
        {
            delay(1000);
        }
    }


    display.clearDisplay();

    display.setTextColor(
        SSD1306_WHITE
    );

    display.setTextSize(1);

    display.setCursor(
        14,
        24
    );

    display.println(
        "Starting sensor..."
    );

    display.display();


    Serial.println(
        "[OLED] ONLINE"
    );


    // --------------------------------------------------------
    // ESP-NOW setup
    // --------------------------------------------------------

    if (!RobotNetwork::begin())
    {
        Serial.println(
            "NETWORK: FAILED"
        );

        while (true)
        {
            delay(1000);
        }
    }


    Serial.println(
        "NETWORK: ONLINE"
    );

    RobotNetwork::printMacAddress();


    if (!RobotNetwork::addPeer(MASTER_MAC))
    {
        Serial.println(
            "[SENSOR] Failed to register MASTER"
        );

        while (true)
        {
            delay(1000);
        }
    }


    Serial.println(
        "[SENSOR] MASTER peer registered"
    );

    Serial.println(
        "[SENSOR] Ultrasonic telemetry online"
    );

    Serial.println(
        "[SENSOR] PIR motion sensor online"
    );


    delay(500);

    updateDisplay();
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    unsigned long now =
        millis();

    static unsigned long previousTelemetry = 0;
    static unsigned long previousHeartbeat = 0;
    static unsigned long previousDisplayUpdate = 0;


    // --------------------------------------------------------
    // Sensor telemetry
    // --------------------------------------------------------

    if (
        now - previousTelemetry >=
        TELEMETRY_INTERVAL_MS
    )
    {
        previousTelemetry = now;

        sendSensorData();
    }


    // --------------------------------------------------------
    // Heartbeat
    // --------------------------------------------------------

    if (
        now - previousHeartbeat >=
        HEARTBEAT_INTERVAL_MS
    )
    {
        previousHeartbeat = now;

        sendHeartbeat();
    }


    // --------------------------------------------------------
    // OLED refresh
    // --------------------------------------------------------

    if (
        now - previousDisplayUpdate >=
        DISPLAY_INTERVAL_MS
    )
    {
        previousDisplayUpdate = now;

        updateDisplay();
    }
}
#include <Arduino.h>

#include "protocol.h"
#include "node_config.h"
#include "RobotNetwork.h"

// ============================================================
// Ultrasonic sensor pins
// ============================================================

constexpr uint8_t TRIG_PIN = 25;
constexpr uint8_t ECHO_PIN = 26;

// Send telemetry 4 times per second
constexpr unsigned long TELEMETRY_INTERVAL_MS = 250;

// Send heartbeat every 2 seconds
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 2000;

uint32_t heartbeatSequence = 0;


// ============================================================
// Read HC-SR04 distance
// ============================================================

float readDistanceCm()
{
    // Start with trigger LOW
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Send 10 microsecond pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure ECHO pulse
    unsigned long duration =
        pulseIn(ECHO_PIN, HIGH, 30000UL);

    // Timeout / no echo
    if (duration == 0)
    {
        return -1.0f;
    }

    // Convert pulse duration to centimeters
    float distanceCm =
        duration * 0.0343f / 2.0f;

    return distanceCm;
}


// ============================================================
// Send sensor telemetry to MASTER
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


    // Real sensor data
    packet.distanceCm =
        readDistanceCm();


    // Sensors we haven't connected yet
    packet.temperature = 0.0f;
    packet.humidity = 0.0f;

    packet.lightLevel = 0;

    packet.accelX = 0.0f;
    packet.accelY = 0.0f;
    packet.accelZ = 0.0f;

    packet.motionDetected = 0;


    bool sent = RobotNetwork::send(
        MASTER_MAC,
        &packet,
        sizeof(packet)
    );


    if (sent)
    {
        Serial.print("[SENSOR] Distance: ");

        if (packet.distanceCm < 0)
        {
            Serial.println("NO ECHO");
        }
        else
        {
            Serial.print(packet.distanceCm, 1);
            Serial.println(" cm");
        }
    }
    else
    {
        Serial.println(
            "[SENSOR] Telemetry send failed"
        );
    }
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
    Serial.println("==============================");
    Serial.println(" ESP32 DISTRIBUTED ROBOT");
    Serial.println("==============================");
    Serial.println("NODE: SENSOR");


    // Ultrasonic sensor setup
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);


    // Start ESP-NOW
    if (!RobotNetwork::begin())
    {
        Serial.println("NETWORK: FAILED");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("NETWORK: ONLINE");

    RobotNetwork::printMacAddress();


    // Register MASTER
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
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    unsigned long now = millis();

    static unsigned long previousTelemetry = 0;
    static unsigned long previousHeartbeat = 0;


    // Send distance data
    if (
        now - previousTelemetry >=
        TELEMETRY_INTERVAL_MS
    )
    {
        previousTelemetry = now;

        sendSensorData();
    }


    // Send heartbeat
    if (
        now - previousHeartbeat >=
        HEARTBEAT_INTERVAL_MS
    )
    {
        previousHeartbeat = now;

        sendHeartbeat();
    }
}
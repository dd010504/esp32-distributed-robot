#include <Arduino.h>

#include "protocol.h"
#include "RobotNetwork.h"


// ============================================================
// Packet processing
// ============================================================

void processPacket(
    const NetworkMessage &message
)
{
    // Make sure we at least received a header.
    if (message.length < sizeof(PacketHeader))
    {
        Serial.println(
            "[MASTER] Packet too small"
        );

        return;
    }

    PacketHeader header;

    memcpy(
        &header,
        message.data,
        sizeof(PacketHeader)
    );


    // --------------------------------------------------------
    // Check protocol version
    // --------------------------------------------------------

    if (
        header.protocolVersion !=
        PROTOCOL_VERSION
    )
    {
        Serial.println(
            "[MASTER] Protocol version mismatch"
        );

        return;
    }


    // --------------------------------------------------------
    // Make sure packet is meant for MASTER
    // --------------------------------------------------------

    if (
        header.destination !=
        NodeId::MASTER
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Determine packet type
    // --------------------------------------------------------

    switch (header.type)
    {

        // ====================================================
        // SENSOR DATA
        // ====================================================

        case PacketType::SENSOR_DATA:
        {
            if (
                message.length !=
                sizeof(SensorPacket)
            )
            {
                Serial.println(
                    "[MASTER] Invalid SensorPacket size"
                );

                return;
            }

            SensorPacket packet;

            memcpy(
                &packet,
                message.data,
                sizeof(packet)
            );

            Serial.println();
            Serial.println(
                "------ SENSOR DATA ------"
            );

            Serial.print("Temperature: ");
            Serial.println(packet.temperature);

            Serial.print("Humidity: ");
            Serial.println(packet.humidity);

            Serial.print("Distance: ");
            Serial.print(packet.distanceCm);
            Serial.println(" cm");

            Serial.print("Light: ");
            Serial.println(packet.lightLevel);

            Serial.print("Motion: ");

            Serial.println(
                packet.motionDetected
                    ? "YES"
                    : "NO"
            );

            Serial.print("Accel X: ");
            Serial.println(packet.accelX);

            Serial.print("Accel Y: ");
            Serial.println(packet.accelY);

            Serial.print("Accel Z: ");
            Serial.println(packet.accelZ);

            break;
        }


        // ====================================================
        // HEARTBEAT
        // ====================================================

        case PacketType::HEARTBEAT:
        {
            if (
                message.length !=
                sizeof(HeartbeatPacket)
            )
            {
                Serial.println(
                    "[MASTER] Invalid heartbeat size"
                );

                return;
            }

            HeartbeatPacket packet;

            memcpy(
                &packet,
                message.data,
                sizeof(packet)
            );

            Serial.print(
                "[MASTER] Heartbeat received from node "
            );

            Serial.print(
                static_cast<int>(
                    packet.header.source
                )
            );

            Serial.print(
                " | Sequence: "
            );

            Serial.print(
                packet.sequenceNumber
            );

            Serial.print(
                " | Uptime: "
            );

            Serial.println(
                packet.uptimeMs
            );

            break;
        }


        default:
        {
            Serial.println(
                "[MASTER] Unknown packet type"
            );

            break;
        }
    }
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
        "NODE: MASTER"
    );


    // --------------------------------------------------------
    // Start networking
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

    Serial.print(
        "PROTOCOL VERSION: "
    );

    Serial.println(
        PROTOCOL_VERSION
    );
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    NetworkMessage message;

    // Process every waiting packet.
    while (
        RobotNetwork::receive(message)
    )
    {
        processPacket(message);
    }


    // Master heartbeat message for debugging.
    static unsigned long previousHeartbeat = 0;

    unsigned long now = millis();

    if (
        now - previousHeartbeat >= 2000
    )
    {
        previousHeartbeat = now;

        Serial.println(
            "[MASTER] Running"
        );
    }
}
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "protocol.h"
#include "node_config.h"
#include "RobotNetwork.h"

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
// Sensor state
// ============================================================

float latestDistanceCm = -1.0f;
bool latestMotionDetected = false;

bool sensorSeen = false;

unsigned long lastSensorPacketMs = 0;

constexpr unsigned long SENSOR_TIMEOUT_MS = 5000;

// ============================================================
// Actuator state
// ============================================================

int16_t commandedServoAngle = 90;

bool actuatorCommandDelivered = false;

constexpr unsigned long ACTUATOR_COMMAND_INTERVAL_MS = 3000;

// Alternates between 45 and 135 degrees.
bool servoTestState = false;


// ============================================================
// Update MASTER OLED
// ============================================================

void updateDisplay()
{
    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // ========================================================
    // Yellow header
    // ========================================================

    display.setCursor(22, 3);
    display.println("ROBOT CONTROL");


    // ========================================================
    // Sensor status
    // ========================================================

    bool sensorOnline =
        sensorSeen &&
        (
            millis() - lastSensorPacketMs <
            SENSOR_TIMEOUT_MS
        );


    display.setCursor(0, 18);

    display.print("SENSOR: ");

    if (sensorOnline)
    {
        display.println("ONLINE");
    }
    else
    {
        display.println("OFFLINE");
    }


    // ========================================================
    // Distance
    // ========================================================

    display.setCursor(0, 29);

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


    // ========================================================
    // Motion
    // ========================================================

    display.setCursor(0, 41);

    display.print("MOTION: ");

    display.println(
        latestMotionDetected
            ? "YES"
            : "NO"
    );


    // ========================================================
    // Servo command
    // ========================================================

    display.setCursor(0, 53);

    display.print("SERVO:");

    display.print(
        commandedServoAngle
    );

    display.print(" ");

    display.println(
        actuatorCommandDelivered
            ? "OK"
            : "WAIT"
    );


    display.display();
}


// ============================================================
// Send command to ACTUATOR
// ============================================================

void sendActuatorCommand()
{
    ActuatorCommand command = {};


    // --------------------------------------------------------
    // Packet header
    // --------------------------------------------------------

    command.header.protocolVersion =
        PROTOCOL_VERSION;

    command.header.source =
        NodeId::MASTER;

    command.header.destination =
        NodeId::ACTUATOR;

    command.header.type =
        PacketType::ACTUATOR_COMMAND;


    // --------------------------------------------------------
    // Alternate servo position
    // --------------------------------------------------------

    servoTestState =
        !servoTestState;


    if (servoTestState)
    {
        commandedServoAngle = 45;
    }
    else
    {
        commandedServoAngle = 135;
    }


    command.servoAngle =
        commandedServoAngle;


    // We haven't connected these yet.
    command.motorSpeed = 0;
    command.buzzerEnabled = 0;
    command.relayEnabled = 0;


    // --------------------------------------------------------
    // Send over ESP-NOW
    // --------------------------------------------------------

    bool queued =
        RobotNetwork::send(
            ACTUATOR_MAC,
            &command,
            sizeof(command)
        );


    if (!queued)
    {
        actuatorCommandDelivered = false;

        Serial.println(
            "[MASTER] Actuator command failed to queue"
        );

        return;
    }


    // Give ESP-NOW callback a moment to report delivery.
    delay(20);


    actuatorCommandDelivered =
        RobotNetwork::lastSendSucceeded();


    // --------------------------------------------------------
    // Serial debug
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "---- ACTUATOR COMMAND SENT ----"
    );

    Serial.print(
        "Servo Angle: "
    );

    Serial.println(
        commandedServoAngle
    );

    Serial.print(
        "Delivery: "
    );

    Serial.println(
        actuatorCommandDelivered
            ? "SUCCESS"
            : "FAILED"
    );

    Serial.println(
        "-------------------------------"
    );
}


// ============================================================
// Process received ESP-NOW packets
// ============================================================

void processPacket(
    const NetworkMessage &message
)
{
    if (
        message.length <
        sizeof(PacketHeader)
    )
    {
        return;
    }


    PacketHeader header;

    memcpy(
        &header,
        message.data,
        sizeof(PacketHeader)
    );


    // --------------------------------------------------------
    // Validate protocol
    // --------------------------------------------------------

    if (
        header.protocolVersion !=
        PROTOCOL_VERSION
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Packet must belong to MASTER
    // --------------------------------------------------------

    if (
        header.destination !=
        NodeId::MASTER
    )
    {
        return;
    }


    // ========================================================
    // SENSOR DATA
    // ========================================================

    if (
        header.type ==
        PacketType::SENSOR_DATA
    )
    {
        if (
            message.length !=
            sizeof(SensorPacket)
        )
        {
            return;
        }


        SensorPacket packet;

        memcpy(
            &packet,
            message.data,
            sizeof(packet)
        );


        latestDistanceCm =
            packet.distanceCm;

        latestMotionDetected =
            packet.motionDetected != 0;

        sensorSeen = true;

        lastSensorPacketMs =
            millis();


        // ----------------------------------------------------
        // Serial output
        // ----------------------------------------------------

        Serial.print(
            "[MASTER] Distance: "
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
            "[MASTER] Motion: "
        );

        Serial.println(
            latestMotionDetected
                ? "YES"
                : "NO"
        );


        return;
    }


    // ========================================================
    // HEARTBEAT
    // ========================================================

    if (
        header.type ==
        PacketType::HEARTBEAT
    )
    {
        if (
            message.length !=
            sizeof(HeartbeatPacket)
        )
        {
            return;
        }


        HeartbeatPacket packet;

        memcpy(
            &packet,
            message.data,
            sizeof(packet)
        );


        if (
            packet.header.source ==
            NodeId::SENSOR
        )
        {
            sensorSeen = true;

            lastSensorPacketMs =
                millis();
        }


        Serial.print(
            "[MASTER] Heartbeat from node "
        );

        Serial.print(
            static_cast<int>(
                packet.header.source
            )
        );

        Serial.print(
            " | Sequence: "
        );

        Serial.println(
            packet.sequenceNumber
        );
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


    // ========================================================
    // OLED setup
    // ========================================================

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
        10,
        20
    );

    display.println(
        "Starting robot..."
    );

    display.display();


    Serial.println(
        "[OLED] ONLINE"
    );


    // ========================================================
    // ESP-NOW setup
    // ========================================================

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


    // ========================================================
    // Register ACTUATOR as peer
    // ========================================================

    if (
        !RobotNetwork::addPeer(
            ACTUATOR_MAC
        )
    )
    {
        Serial.println(
            "[MASTER] Failed to register ACTUATOR"
        );

        while (true)
        {
            delay(1000);
        }
    }


    Serial.println(
        "[MASTER] ACTUATOR peer registered"
    );


    Serial.print(
        "PROTOCOL VERSION: "
    );

    Serial.println(
        PROTOCOL_VERSION
    );


    delay(500);

    updateDisplay();


    Serial.println(
        "[MASTER] Servo wireless test ready"
    );
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    // ========================================================
    // Receive SENSOR packets
    // ========================================================

    NetworkMessage message;

    while (
        RobotNetwork::receive(message)
    )
    {
        processPacket(message);
    }


    unsigned long now =
        millis();


    // ========================================================
    // Update OLED
    // ========================================================

    static unsigned long
        previousDisplayUpdate = 0;


    if (
        now - previousDisplayUpdate >=
        100
    )
    {
        previousDisplayUpdate = now;

        updateDisplay();
    }


    // ========================================================
    // Send servo command every 3 seconds
    // ========================================================

    static unsigned long
        previousActuatorCommand = 0;


    if (
        now - previousActuatorCommand >=
        ACTUATOR_COMMAND_INTERVAL_MS
    )
    {
        previousActuatorCommand = now;

        sendActuatorCommand();
    }
}
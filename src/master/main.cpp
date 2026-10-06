#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "protocol.h"
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
// Latest sensor state
// ============================================================

float latestDistanceCm = -1.0f;

bool latestMotionDetected = false;

bool sensorSeen = false;

unsigned long lastSensorPacketMs = 0;

constexpr unsigned long SENSOR_TIMEOUT_MS = 5000;


// ============================================================
// Update OLED
// ============================================================

void updateDisplay()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // =========================================
    // YELLOW AREA: pixels 0-15
    // =========================================

    display.setTextSize(1);
    display.setCursor(22, 3);
    display.println("ROBOT CONTROL");


    // =========================================
    // BLUE AREA: pixels 16-63
    // =========================================

    bool sensorOnline =
        sensorSeen &&
        (
            millis() - lastSensorPacketMs <
            SENSOR_TIMEOUT_MS
        );


    // Sensor status
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


    // Distance
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


    // Motion
    display.setCursor(0, 41);

    display.print("MOTION: ");

    if (latestMotionDetected)
    {
        display.println("YES");
    }
    else
    {
        display.println("NO");
    }


    // Mode
    display.setCursor(0, 53);
    display.println("MODE: MANUAL");


    display.display();
}

// ============================================================
// Process received ESP-NOW packet
// ============================================================

void processPacket(
    const NetworkMessage &message
)
{
    // Packet must at least contain a header.
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
        sizeof(header)
    );


    // --------------------------------------------------------
    // Validate protocol version
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


        // ----------------------------------------------------
        // Save latest telemetry
        // ----------------------------------------------------

        latestDistanceCm =
            packet.distanceCm;

        latestMotionDetected =
            packet.motionDetected != 0;

        sensorSeen = true;

        lastSensorPacketMs =
            millis();


        // ----------------------------------------------------
        // Serial debug
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

    display.setCursor(10, 20);

    display.println(
        "Starting robot..."
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


    Serial.print(
        "PROTOCOL VERSION: "
    );

    Serial.println(
        PROTOCOL_VERSION
    );


    delay(500);

    updateDisplay();
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Process received packets
    // --------------------------------------------------------

    NetworkMessage message;

    while (
        RobotNetwork::receive(message)
    )
    {
        processPacket(message);
    }


    unsigned long now =
        millis();


    // --------------------------------------------------------
    // Refresh OLED
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // Debug heartbeat
    // --------------------------------------------------------

    static unsigned long
        previousHeartbeat = 0;


    if (
        now - previousHeartbeat >=
        2000
    )
    {
        previousHeartbeat = now;

        Serial.println(
            "[MASTER] Running"
        );
    }
}
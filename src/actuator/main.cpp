#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "protocol.h"
#include "RobotNetwork.h"

// ============================================================
// OLED configuration
// ============================================================

constexpr uint8_t OLED_SDA = 32;
constexpr uint8_t OLED_SCL = 33;
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
// Servo configuration
// ============================================================

constexpr uint8_t SERVO_PIN = 13;

constexpr uint8_t SERVO_PWM_CHANNEL = 0;
constexpr uint16_t SERVO_PWM_FREQUENCY = 50;
constexpr uint8_t SERVO_PWM_RESOLUTION = 16;

// Typical hobby-servo pulse range
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;

// ============================================================
// Timing
// ============================================================

constexpr unsigned long DISPLAY_INTERVAL_MS = 100;
constexpr unsigned long COMMAND_TIMEOUT_MS = 5000;

// ============================================================
// Actuator state
// ============================================================

int16_t servoAngle = 90;
int16_t motorSpeed = 0;

bool buzzerEnabled = false;
bool relayEnabled = false;

bool commandReceived = false;

unsigned long lastCommandMs = 0;


// ============================================================
// Servo control
// ============================================================

void setServoAngle(int angle)
{
    angle = constrain(angle, 0, 180);

    servoAngle = angle;

    // Convert degrees to pulse width.
    uint32_t pulseUs =
        map(
            angle,
            0,
            180,
            SERVO_MIN_US,
            SERVO_MAX_US
        );

    // 50 Hz = 20,000 microsecond period.
    uint32_t duty =
        (pulseUs * 65535UL) / 20000UL;

    ledcWrite(
        SERVO_PWM_CHANNEL,
        duty
    );
}


// ============================================================
// Update OLED
// ============================================================

void updateDisplay()
{
    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Yellow section
    display.setCursor(20, 3);
    display.println("ACTUATOR NODE");


    bool masterOnline =
        commandReceived &&
        (
            millis() - lastCommandMs <
            COMMAND_TIMEOUT_MS
        );


    // Servo
    display.setCursor(0, 18);

    display.print("SERVO: ");
    display.print(servoAngle);
    display.println(" deg");


    // Motor
    display.setCursor(0, 29);

    display.print("MOTOR: ");
    display.print(motorSpeed);
    display.println("%");


    // Buzzer / Relay
    display.setCursor(0, 40);

    display.print("BUZ:");

    display.print(
        buzzerEnabled
            ? "ON "
            : "OFF"
    );

    display.print(" REL:");

    display.println(
        relayEnabled
            ? "ON"
            : "OFF"
    );


    // Link
    display.setCursor(0, 52);

    if (!commandReceived)
    {
        display.println("LINK: WAITING CMD");
    }
    else if (masterOnline)
    {
        display.println("LINK: MASTER OK");
    }
    else
    {
        display.println("LINK: TIMEOUT");
    }


    display.display();
}


// ============================================================
// Process incoming ESP-NOW packets
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


    if (
        header.protocolVersion !=
        PROTOCOL_VERSION
    )
    {
        Serial.println(
            "[ACTUATOR] Protocol version mismatch"
        );

        return;
    }


    if (
        header.destination !=
        NodeId::ACTUATOR
    )
    {
        return;
    }


    // ========================================================
    // ACTUATOR COMMAND
    // ========================================================

    if (
        header.type ==
        PacketType::ACTUATOR_COMMAND
    )
    {
        if (
            message.length !=
            sizeof(ActuatorCommand)
        )
        {
            Serial.println(
                "[ACTUATOR] Invalid command size"
            );

            return;
        }


        ActuatorCommand command;

        memcpy(
            &command,
            message.data,
            sizeof(command)
        );


        // ----------------------------------------------------
        // REAL SERVO MOVEMENT
        // ----------------------------------------------------

        setServoAngle(
            command.servoAngle
        );


        // Future actuator states
        motorSpeed =
            command.motorSpeed;

        buzzerEnabled =
            command.buzzerEnabled != 0;

        relayEnabled =
            command.relayEnabled != 0;


        commandReceived = true;

        lastCommandMs =
            millis();


        // ----------------------------------------------------
        // Serial output
        // ----------------------------------------------------

        Serial.println();
        Serial.println(
            "---- ACTUATOR COMMAND ----"
        );

        Serial.print("Servo: ");
        Serial.println(servoAngle);

        Serial.print("Motor: ");
        Serial.println(motorSpeed);

        Serial.print("Buzzer: ");

        Serial.println(
            buzzerEnabled
                ? "ON"
                : "OFF"
        );

        Serial.print("Relay: ");

        Serial.println(
            relayEnabled
                ? "ON"
                : "OFF"
        );

        Serial.println(
            "--------------------------"
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
        "NODE: ACTUATOR"
    );


    // ========================================================
    // Servo PWM setup
    // ========================================================

    ledcSetup(
        SERVO_PWM_CHANNEL,
        SERVO_PWM_FREQUENCY,
        SERVO_PWM_RESOLUTION
    );

    ledcAttachPin(
        SERVO_PIN,
        SERVO_PWM_CHANNEL
    );


    Serial.println(
        "[SERVO] PWM initialized on GPIO13"
    );


    // ========================================================
    // OLED setup
    // ========================================================

    Wire.begin(
        OLED_SDA,
        OLED_SCL
    );


    Wire.beginTransmission(
        OLED_ADDRESS
    );

    uint8_t oledResult =
        Wire.endTransmission();


    if (oledResult != 0)
    {
        Serial.println(
            "[OLED] Device not detected at 0x3C"
        );

        while (true)
        {
            delay(1000);
        }
    }


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


    Serial.println(
        "[OLED] ONLINE"
    );


    // ========================================================
    // Servo startup test
    // ========================================================

    Serial.println(
        "[SERVO] Starting self-test..."
    );


    setServoAngle(90);
    updateDisplay();
    delay(1000);


    setServoAngle(45);
    updateDisplay();
    delay(1000);


    setServoAngle(135);
    updateDisplay();
    delay(1000);


    setServoAngle(90);
    updateDisplay();


    Serial.println(
        "[SERVO] Self-test complete"
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


    Serial.println(
        "[ACTUATOR] Waiting for MASTER commands..."
    );
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    NetworkMessage message;

    while (
        RobotNetwork::receive(message)
    )
    {
        processPacket(message);
    }


    unsigned long now =
        millis();

    static unsigned long
        previousDisplayUpdate = 0;


    if (
        now - previousDisplayUpdate >=
        DISPLAY_INTERVAL_MS
    )
    {
        previousDisplayUpdate = now;

        updateDisplay();
    }
}
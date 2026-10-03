#include <Arduino.h>

#include "protocol.h"
#include "RobotNetwork.h"


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
        sizeof(header)
    );


    if (
        header.protocolVersion !=
        PROTOCOL_VERSION
    )
    {
        return;
    }


    if (
        header.destination !=
        NodeId::ACTUATOR
    )
    {
        return;
    }


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
            return;
        }

        ActuatorCommand command;

        memcpy(
            &command,
            message.data,
            sizeof(command)
        );


        Serial.println();
        Serial.println(
            "---- ACTUATOR COMMAND ----"
        );

        Serial.print(
            "Servo: "
        );

        Serial.println(
            command.servoAngle
        );

        Serial.print(
            "Motor: "
        );

        Serial.println(
            command.motorSpeed
        );

        Serial.print(
            "Buzzer: "
        );

        Serial.println(
            command.buzzerEnabled
        );

        Serial.print(
            "Relay: "
        );

        Serial.println(
            command.relayEnabled
        );
    }
}


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
}


void loop()
{
    NetworkMessage message;

    while (
        RobotNetwork::receive(message)
    )
    {
        processPacket(message);
    }
}
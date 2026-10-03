#include <Arduino.h>

#include "protocol.h"
#include "RobotNetwork.h"


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
    static unsigned long previousHeartbeat = 0;

    unsigned long now = millis();

    if (
        now - previousHeartbeat >= 2000
    )
    {
        previousHeartbeat = now;

        Serial.println(
            "[SENSOR] Running"
        );
    }
}
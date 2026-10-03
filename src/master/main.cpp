#include <Arduino.h>

#include "protocol.h"
#include "RobotNetwork.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32 DISTRIBUTED ROBOT");
    Serial.println("==============================");

    Serial.println("NODE: MASTER");

    if (!RobotNetwork::begin())
    {
        Serial.println("NETWORK: FAILED");
        return;
    }

    Serial.println("NETWORK: ONLINE");

    RobotNetwork::printMacAddress();

    Serial.print("PROTOCOL VERSION: ");
    Serial.println(PROTOCOL_VERSION);
}

void loop()
{
    Serial.println("[MASTER] Heartbeat");

    delay(2000);
}
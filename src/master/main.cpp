#include <Arduino.h>
#include <protocol.h>

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("============================");
    Serial.println(" ESP32 DISTRIBUTED ROBOT");
    Serial.println("============================");
    Serial.println("Node: MASTER");
    Serial.println("Status: ONLINE");
}

void loop()
{
    Serial.println("[MASTER] Heartbeat");

    delay(2000);
}
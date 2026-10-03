#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("============================");
    Serial.println(" ESP32 DISTRIBUTED ROBOT");
    Serial.println("============================");
    Serial.println("Node: ACTUATOR");
    Serial.println("Status: ONLINE");
}

void loop()
{
    Serial.println("[ACTUATOR] Heartbeat");

    delay(2000);
}
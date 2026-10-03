#include <Arduino.h>
#include <protocol.h>

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32 DISTRIBUTED ROBOT");
    Serial.println("==============================");
    Serial.println("NODE: SENSOR");
    Serial.println("STATUS: ONLINE");
}

void loop()
{
    Serial.println("[SENSOR] Heartbeat");

    delay(2000);
}
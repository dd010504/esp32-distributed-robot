#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

class RobotNetwork
{
public:
    static bool begin();

    static bool addPeer(const uint8_t mac[6]);

    static bool send(
        const uint8_t mac[6],
        const void *data,
        size_t size
    );

    static void printMacAddress();
};
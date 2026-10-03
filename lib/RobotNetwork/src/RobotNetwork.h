#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ESP-NOW v1 payload limit
constexpr size_t ROBOT_NETWORK_MAX_PACKET_SIZE = ESP_NOW_MAX_DATA_LEN;

// Number of received packets we can temporarily buffer
constexpr uint8_t ROBOT_NETWORK_QUEUE_SIZE = 8;

struct NetworkMessage
{
    uint8_t senderMac[6];

    uint16_t length;

    uint8_t data[ROBOT_NETWORK_MAX_PACKET_SIZE];
};

class RobotNetwork
{
public:
    // Initialize Wi-Fi + ESP-NOW
    static bool begin();

    // Register another ESP32 as a peer
    static bool addPeer(const uint8_t mac[6]);

    // Send raw packet data
    static bool send(
        const uint8_t mac[6],
        const void *data,
        size_t size
    );

    // Retrieve the next received packet
    // Returns false if the queue is empty
    static bool receive(NetworkMessage &message);

    // Print this ESP32's Wi-Fi MAC
    static void printMacAddress();

    // Status of most recent ESP-NOW transmission
    static bool lastSendSucceeded();

private:
    static QueueHandle_t receiveQueue;

    static volatile bool sendSucceeded;

    static void handleReceive(
        const uint8_t *mac,
        const uint8_t *data,
        int length
    );

    static void handleSend(
        const uint8_t *mac,
        esp_now_send_status_t status
    );
};
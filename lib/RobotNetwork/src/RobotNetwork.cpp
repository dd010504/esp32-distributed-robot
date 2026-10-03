#include "RobotNetwork.h"

bool RobotNetwork::begin()
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[NETWORK] ESP-NOW initialization failed");
        return false;
    }

    Serial.println("[NETWORK] ESP-NOW initialized");

    return true;
}

bool RobotNetwork::addPeer(const uint8_t mac[6])
{
    esp_now_peer_info_t peerInfo = {};

    memcpy(peerInfo.peer_addr, mac, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(mac))
    {
        Serial.println("[NETWORK] Peer already registered");
        return true;
    }

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("[NETWORK] Failed to add peer");
        return false;
    }

    Serial.println("[NETWORK] Peer added");

    return true;
}

bool RobotNetwork::send(
    const uint8_t mac[6],
    const void *data,
    size_t size
)
{
    esp_err_t result = esp_now_send(
        mac,
        reinterpret_cast<const uint8_t *>(data),
        size
    );

    return result == ESP_OK;
}

void RobotNetwork::printMacAddress()
{
    Serial.print("[NETWORK] MAC: ");
    Serial.println(WiFi.macAddress());
}
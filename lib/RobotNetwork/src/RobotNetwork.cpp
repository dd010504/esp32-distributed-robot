#include "RobotNetwork.h"

#include <esp_wifi.h>

constexpr uint8_t ROBOT_WIFI_CHANNEL = 1;

QueueHandle_t RobotNetwork::receiveQueue = nullptr;

volatile bool RobotNetwork::sendSucceeded = false;


// ============================================================
// Initialize network
// ============================================================

bool RobotNetwork::begin()
{
    // --------------------------------------------------------
    // Start Wi-Fi in Station mode
    // --------------------------------------------------------

    WiFi.mode(WIFI_STA);

    // Prevent the ESP32 from reconnecting to an old Wi-Fi
    // network and changing our ESP-NOW channel.
    WiFi.setAutoReconnect(false);

    WiFi.disconnect();

    delay(100);


    // --------------------------------------------------------
    // Force ESP-NOW radio onto channel 1
    // --------------------------------------------------------

    esp_err_t channelResult =
        esp_wifi_set_channel(
            ROBOT_WIFI_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        );

    if (channelResult != ESP_OK)
    {
        Serial.print(
            "[NETWORK] Failed to set channel. Error: "
        );

        Serial.println(channelResult);

        return false;
    }


    // --------------------------------------------------------
    // Verify the ACTUAL Wi-Fi channel
    // --------------------------------------------------------

    uint8_t primaryChannel = 0;

    wifi_second_chan_t secondaryChannel =
        WIFI_SECOND_CHAN_NONE;

    esp_err_t getChannelResult =
        esp_wifi_get_channel(
            &primaryChannel,
            &secondaryChannel
        );

    if (getChannelResult != ESP_OK)
    {
        Serial.print(
            "[NETWORK] Failed to read channel. Error: "
        );

        Serial.println(getChannelResult);

        return false;
    }


    Serial.print(
        "[NETWORK] Actual WiFi channel: "
    );

    Serial.println(
        primaryChannel
    );


    // --------------------------------------------------------
    // Create incoming-packet queue
    // --------------------------------------------------------

    receiveQueue =
        xQueueCreate(
            ROBOT_NETWORK_QUEUE_SIZE,
            sizeof(NetworkMessage)
        );


    if (receiveQueue == nullptr)
    {
        Serial.println(
            "[NETWORK] Failed to create receive queue"
        );

        return false;
    }


    // --------------------------------------------------------
    // Initialize ESP-NOW
    // --------------------------------------------------------

    if (esp_now_init() != ESP_OK)
    {
        Serial.println(
            "[NETWORK] ESP-NOW initialization failed"
        );

        return false;
    }


    // --------------------------------------------------------
    // Register callbacks
    // --------------------------------------------------------

    if (
        esp_now_register_recv_cb(handleReceive)
        != ESP_OK
    )
    {
        Serial.println(
            "[NETWORK] Failed to register receive callback"
        );

        return false;
    }


    if (
        esp_now_register_send_cb(handleSend)
        != ESP_OK
    )
    {
        Serial.println(
            "[NETWORK] Failed to register send callback"
        );

        return false;
    }


    Serial.println(
        "[NETWORK] ESP-NOW initialized"
    );

    return true;
}


// ============================================================
// Add ESP-NOW peer
// ============================================================

bool RobotNetwork::addPeer(
    const uint8_t mac[6]
)
{
    if (mac == nullptr)
    {
        return false;
    }


    if (esp_now_is_peer_exist(mac))
    {
        Serial.println(
            "[NETWORK] Peer already registered"
        );

        return true;
    }


    esp_now_peer_info_t peerInfo = {};


    memcpy(
        peerInfo.peer_addr,
        mac,
        6
    );


    // IMPORTANT:
    //
    // Explicitly force peer to the same channel
    // instead of using channel = 0.

    peerInfo.channel =
        ROBOT_WIFI_CHANNEL;


    peerInfo.ifidx =
        WIFI_IF_STA;


    peerInfo.encrypt =
        false;


    esp_err_t result =
        esp_now_add_peer(
            &peerInfo
        );


    if (result != ESP_OK)
    {
        Serial.print(
            "[NETWORK] Failed to add peer. Error: "
        );

        Serial.println(result);

        return false;
    }


    Serial.print(
        "[NETWORK] Peer added on channel "
    );

    Serial.println(
        ROBOT_WIFI_CHANNEL
    );


    return true;
}


// ============================================================
// Send packet
// ============================================================

bool RobotNetwork::send(
    const uint8_t mac[6],
    const void *data,
    size_t size
)
{
    if (
        mac == nullptr ||
        data == nullptr
    )
    {
        return false;
    }


    if (
        size == 0 ||
        size > ROBOT_NETWORK_MAX_PACKET_SIZE
    )
    {
        return false;
    }


    // Reset old delivery state
    sendSucceeded = false;


    esp_err_t result =
        esp_now_send(
            mac,
            static_cast<const uint8_t *>(data),
            size
        );


    if (result != ESP_OK)
    {
        Serial.print(
            "[NETWORK] esp_now_send error: "
        );

        Serial.println(result);

        return false;
    }


    return true;
}


// ============================================================
// Retrieve packet from queue
// ============================================================

bool RobotNetwork::receive(
    NetworkMessage &message
)
{
    if (receiveQueue == nullptr)
    {
        return false;
    }


    return xQueueReceive(
        receiveQueue,
        &message,
        0
    ) == pdTRUE;
}


// ============================================================
// ESP-NOW receive callback
// ============================================================

void RobotNetwork::handleReceive(
    const uint8_t *mac,
    const uint8_t *data,
    int length
)
{
    if (
        receiveQueue == nullptr ||
        mac == nullptr ||
        data == nullptr
    )
    {
        return;
    }


    if (
        length <= 0 ||
        length >
            static_cast<int>(
                ROBOT_NETWORK_MAX_PACKET_SIZE
            )
    )
    {
        return;
    }


    NetworkMessage message = {};


    memcpy(
        message.senderMac,
        mac,
        6
    );


    message.length =
        static_cast<uint16_t>(
            length
        );


    memcpy(
        message.data,
        data,
        length
    );


    // Never block the Wi-Fi task.
    xQueueSend(
        receiveQueue,
        &message,
        0
    );
}


// ============================================================
// ESP-NOW send callback
// ============================================================

void RobotNetwork::handleSend(
    const uint8_t *mac,
    esp_now_send_status_t status
)
{
    sendSucceeded =
        (
            status ==
            ESP_NOW_SEND_SUCCESS
        );
}


// ============================================================
// Return latest delivery status
// ============================================================

bool RobotNetwork::lastSendSucceeded()
{
    return sendSucceeded;
}


// ============================================================
// Print station MAC
// ============================================================

void RobotNetwork::printMacAddress()
{
    Serial.print(
        "[NETWORK] MAC: "
    );

    Serial.println(
        WiFi.macAddress()
    );
}
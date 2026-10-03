#include "RobotNetwork.h"

QueueHandle_t RobotNetwork::receiveQueue = nullptr;

volatile bool RobotNetwork::sendSucceeded = false;


// ============================================================
// Initialize network
// ============================================================

bool RobotNetwork::begin()
{
    // ESP-NOW uses the Wi-Fi radio.
    // We don't need to connect to a router.
    WiFi.mode(WIFI_STA);

    // Create queue for incoming packets.
    receiveQueue = xQueueCreate(
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

    // Initialize ESP-NOW.
    if (esp_now_init() != ESP_OK)
    {
        Serial.println(
            "[NETWORK] ESP-NOW initialization failed"
        );

        return false;
    }

    // Register internal callbacks.
    if (esp_now_register_recv_cb(handleReceive) != ESP_OK)
    {
        Serial.println(
            "[NETWORK] Failed to register receive callback"
        );

        return false;
    }

    if (esp_now_register_send_cb(handleSend) != ESP_OK)
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
// Add peer
// ============================================================

bool RobotNetwork::addPeer(const uint8_t mac[6])
{
    if (esp_now_is_peer_exist(mac))
    {
        return true;
    }

    esp_now_peer_info_t peerInfo = {};

    memcpy(
        peerInfo.peer_addr,
        mac,
        6
    );

    peerInfo.channel = 0;

    peerInfo.ifidx = WIFI_IF_STA;

    peerInfo.encrypt = false;

    esp_err_t result =
        esp_now_add_peer(&peerInfo);

    if (result != ESP_OK)
    {
        Serial.print(
            "[NETWORK] Failed to add peer: "
        );

        Serial.println(result);

        return false;
    }

    Serial.println(
        "[NETWORK] Peer added"
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
    if (mac == nullptr || data == nullptr)
    {
        return false;
    }

    if (size == 0)
    {
        return false;
    }

    if (size > ROBOT_NETWORK_MAX_PACKET_SIZE)
    {
        Serial.println(
            "[NETWORK] Packet too large"
        );

        return false;
    }

    esp_err_t result =
        esp_now_send(
            mac,
            static_cast<const uint8_t *>(data),
            size
        );

    return result == ESP_OK;
}


// ============================================================
// Get packet from queue
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
// ESP-NOW RECEIVE CALLBACK
// ============================================================

void RobotNetwork::handleReceive(
    const uint8_t *mac,
    const uint8_t *data,
    int length
)
{
    // VERY IMPORTANT:
    //
    // This callback executes inside the Wi-Fi task.
    // Don't do parsing, Serial spam, OLED drawing,
    // motor control, etc. here.

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
        static_cast<uint16_t>(length);

    memcpy(
        message.data,
        data,
        length
    );

    // Non-blocking.
    //
    // If the queue is full, the packet is dropped
    // instead of blocking the Wi-Fi task.
    xQueueSend(
        receiveQueue,
        &message,
        0
    );
}


// ============================================================
// ESP-NOW SEND CALLBACK
// ============================================================

void RobotNetwork::handleSend(
    const uint8_t *mac,
    esp_now_send_status_t status
)
{
    sendSucceeded =
        (status == ESP_NOW_SEND_SUCCESS);
}


// ============================================================
// Last send result
// ============================================================

bool RobotNetwork::lastSendSucceeded()
{
    return sendSucceeded;
}


// ============================================================
// Print local MAC
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
#pragma once

#include <stdint.h>

constexpr uint8_t PROTOCOL_VERSION = 1;

enum class NodeId : uint8_t
{
    MASTER = 1,
    SENSOR = 2,
    ACTUATOR = 3
};

enum class PacketType : uint8_t
{
    HEARTBEAT = 1,
    SENSOR_DATA = 2,
    ACTUATOR_COMMAND = 3,
    ACTUATOR_STATUS = 4
};

struct PacketHeader
{
    uint8_t protocolVersion;
    NodeId source;
    NodeId destination;
    PacketType type;
};

struct SensorPacket
{
    PacketHeader header;

    float temperature;
    float humidity;
    float distanceCm;

    int32_t lightLevel;

    float accelX;
    float accelY;
    float accelZ;

    uint8_t motionDetected;
};

struct ActuatorCommand
{
    PacketHeader header;

    int16_t servoAngle;
    int16_t motorSpeed;

    uint8_t buzzerEnabled;
    uint8_t relayEnabled;
};

struct HeartbeatPacket
{
    PacketHeader header;

    uint32_t uptimeMs;
    uint32_t sequenceNumber;
};

static_assert(sizeof(PacketHeader) == 4, "Unexpected PacketHeader size");
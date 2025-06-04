#ifndef NetwrokMsg_H
#define NetwrokMsg_H

#include <stddef.h>

enum class UdpMessageType : unsigned int {
    ERROR = 0,
    Geo = 1,
    Control = 2,
    SessionStart = 3,
};

class BaseUdpMessage {
public:
    virtual ~BaseUdpMessage() = default;

    BaseUdpMessage()
    {
        messageType = UdpMessageType::ERROR;
    }

    virtual size_t Size() = 0;

    unsigned long timestamp;
    UdpMessageType messageType;
};

class UdpGeoMessage : public BaseUdpMessage {
public:
    UdpGeoMessage() { messageType = UdpMessageType::Geo; }
    size_t Size() override { return sizeof(UdpGeoMessage); }
    char nmea[128] = { 0 };
};

class UdpControlMessage : public BaseUdpMessage {
public:
    UdpControlMessage() { messageType = UdpMessageType::Control; }
    size_t Size() override { return sizeof(UdpControlMessage); }
    char command[64] = { 0 };
};

class UdpSessionStartMessage : public BaseUdpMessage {
public:
    UdpSessionStartMessage() { messageType = UdpMessageType::SessionStart; }
    size_t Size() override { return sizeof(UdpSessionStartMessage); }
};
#endif

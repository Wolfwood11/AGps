#ifndef NetwrokMsg_H
#define NetwrokMsg_H

#include <stddef.h>

enum class UdpMessageType : unsigned int {
    ERROR = 0,
    Geo = 1,
    Control = 2,
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
#endif

#include <WiFiUdpController.h>

WiFiUdpController::WiFiUdpController(std::shared_ptr<Subscription<String>> nmeaSource)
{
    if (nmeaSource) {
        nmeaSource->Subscribe([this](const String& line) {
            UdpGeoMessage msg;
            msg.timestamp = millis();
            strncpy(msg.nmea, line.c_str(), sizeof(msg.nmea) - 1);
            msg.nmea[sizeof(msg.nmea) - 1] = 0; // защита от переполнения

            this->SendMessage(msg, sizeof(msg));
        },
            nmeaHolder);
    }
}

void WiFiUdpController::setup()
{
    apSsid = "Agps_" + String(123);
    WiFi.softAP(apSsid.c_str(), apPassword.c_str());
    udp.begin(udpPort);
}

void WiFiUdpController::loop(unsigned long dt)
{
    parseUdpPacket(nullptr);
}

void WiFiUdpController::parseUdpPacket(std::function<void(BaseUdpMessage*)> parseFunc)
{
    int packetSize = udp.parsePacket();
    while (packetSize > 0) {
        char* pack = new char[packetSize];
        int len = udp.read(pack, packetSize);
        if (len > 0) {
            BaseUdpMessage* baseMsg = (BaseUdpMessage*)pack;
            if (baseMsg) {
                notifySubscribers(baseMsg->messageType, *baseMsg);
                if (parseFunc)
                    parseFunc(baseMsg);
            }
        }
        delete[] pack;
        packetSize = udp.parsePacket();
    }
}

void WiFiUdpController::SubscribeOnDataReceive(UdpMessageType type, std::function<void(const BaseUdpMessage&)> callback, SubscriptionHolder& holder)
{
    if (callbacks.find(type) == callbacks.end()) {
        callbacks[type] = std::make_shared<Subscription<const BaseUdpMessage&>>();
    }
    callbacks[type]->Subscribe(callback, holder);
}

bool WiFiUdpController::SendMessage(const BaseUdpMessage& message, int size)
{
    // Проверяем, поднят ли AP (можно в setup() установить свой флаг при успехе softAP)
    if (WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
        return false;
    }

    udp.beginPacket(IPAddress(192, 168, 4, 2), udpPort);
    udp.write((const uint8_t*)&message, size);
    udp.endPacket();
    return true;
}

void WiFiUdpController::notifySubscribers(UdpMessageType type, BaseUdpMessage& message)
{
    auto callback = callbacks.find(type);
    if (callback != callbacks.end()) {
        callback->second->Broadcast(message);
    }
}

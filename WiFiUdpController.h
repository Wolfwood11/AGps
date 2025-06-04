#ifndef WiFiUdpController_H
#define WiFiUdpController_H

#include <BaseController.h>
#include <NetwrokMsg.h>
#include <Subscription.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <memory>

class WiFiUdpController : public BaseController {
public:
    WiFiUdpController(std::shared_ptr<Subscription<String>> nmeaSource);

    void setup() override;
    void loop(unsigned long dt) override;

    // подписка на приходящие сообщения
    void SubscribeOnDataReceive(UdpMessageType type, std::function<void(const BaseUdpMessage&)> callback, SubscriptionHolder& holder);

    // отправка сообщения через UDP
    bool SendMessage(const BaseUdpMessage& message, int size);

private:
    void notifySubscribers(UdpMessageType type, BaseUdpMessage& message);
    void parseUdpPacket(std::function<void(BaseUdpMessage*)> parseFunc);

    std::map<UdpMessageType, std::shared_ptr<Subscription<const BaseUdpMessage&>>> callbacks;
    WiFiUDP udp;
    const int udpPort = 4210;

    String apSsid;
    String apPassword = "12345678"; // можно не задавать

    SubscriptionHolder nmeaHolder;
};
#endif

#ifndef BtController_H
#define BtController_H

#include <BaseController.h>
#include <BluetoothSerial.h>
#include <Subscription.h>

class BtController : public BaseController {
public:
    BtController(std::shared_ptr<Subscription<char>> nmeaSource);

    std::shared_ptr<Subscription<String>> command = std::make_shared<Subscription<String>>();

    void setup() override;
    void loop(unsigned long dt) override;

    void sendResponse(const String& msg);
    bool isConnected();

private:
    BluetoothSerial btSerial;
    String buffer;
    SubscriptionHolder nmeaHolder;

    void sendNmeaChar(char c);
};
#endif

#ifndef GpsController_H
#define GpsController_H

#include <BaseController.h>
#include <HardwareSerial.h>
#include <Structs.h>
#include <Subscription.h>
#include <TinyGPSPlus.h>

class GpsController : public BaseController {
public:
    GpsController(HardwareSerial& serial, int rxPin = -1, int txPin = -1, int baud = 115200);

    std::shared_ptr<Subscription<char>> nmeaRaw;
    std::shared_ptr<Subscription<GpsData>> gpsProcessed;

    void setup() override;
    void loop(unsigned long dt) override;

private:
    HardwareSerial& serial;
    int rxPin, txPin;
    int baudRate;
    TinyGPSPlus gps;

    void sendUBX(const uint8_t* data, size_t len, unsigned long delayMs = 100);
};
#endif

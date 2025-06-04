#include <GpsController.h>
#include <MathUtils.h>

namespace {
    // Команды UBX для настройки GPS
    const uint8_t setBaud115200[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
        0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
        0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xC4, 0x96
    };

    const uint8_t setRate10Hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0x64, 0x00, 0x01, 0x00, 0x01, 0x00,
        0x7A, 0x12
    };
}

GpsController::GpsController(HardwareSerial& serial, int rxPin, int txPin, int baud)
    : serial(serial), rxPin(rxPin), txPin(txPin), baudRate(baud)
{
    nmeaRaw = std::make_shared<Subscription<String>>();
    gpsProcessed = std::make_shared<Subscription<GpsData>>();
}

void GpsController::setup()
{
    if (rxPin >= 0 && txPin >= 0)
    {
      serial.begin(9600, SERIAL_8N1, rxPin, txPin);
      sendUBX(setBaud115200, sizeof(setBaud115200));
      serial.flush();
      serial.updateBaudRate(baudRate);
      delay(200);
      sendUBX(setRate10Hz, sizeof(setRate10Hz));
    }
    else
    {
      serial.begin(baudRate);
    }   
}

void GpsController::sendUBX(const uint8_t* data, size_t len, unsigned long delayMs)
{
    serial.write(data, len);
    delay(delayMs);
}

void GpsController::loop(unsigned long dt)
{
    while (serial.available()) {
        char c = serial.read();
        if (nmeaRaw) {
            nmeaRaw->Broadcast(String(c)); // Отправляем каждый символ
        }
        gps.encode(c);
    }

    if (gps.location.isUpdated()) {
    GpsData data;
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.speedKmph = gps.speed.kmph();
    data.hdop = gps.hdop.hdop();
    data.satellites = gps.satellites.value();
    data.locationValid = gps.location.isValid();
    data.speedValid = gps.speed.isValid();
    data.hdopValid = gps.hdop.isValid();

    FullNmeaTime nmeaTime;
    nmeaTime.fromGps(gps);
    data.msOfDay = nmeaTime.isValid ? nmeaTime.getMsOfDay() : 0;
    data.nmeaTime = nmeaTime;                        // новое поле с полным временем
    data.millisReceived = millis();                  // время получения фикса по millis()

    if (gpsProcessed) {
        gpsProcessed->Broadcast(data);
    }
  }
}

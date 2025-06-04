#include <BtController.h>
#include <DisplayController.h>
#include <GpsController.h>
#include <HardwareSerial.h>
#include <Logger.h>
#include <LogicController.h> // <-- Наш основной контроллер логики
#include <Preferences.h>
#include <WiFiUdpController.h>
#include <memory>

#ifdef USE_GPS_HARDWARE
#define GPS_RX 25
#define GPS_TX 26
HardwareSerial gpsSerial(2);
GpsController gpsController(gpsSerial, GPS_RX, GPS_TX);
#else
GpsController gpsController(Serial);
#endif

BtController btController(gpsController.nmeaRaw);
WiFiUdpController wifiController(gpsController.nmeaRaw);
Preferences prefs;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
DisplayController display(u8g2);

// --- Основной контроллер логики ---
std::unique_ptr<LogicController> logic;

void setup()
{
    Serial.begin(115200);
    Logger::setLogger(Serial);
    // --- Настройка железа и сервисов ---
    gpsController.setup();
    btController.setup();
    wifiController.setup();
    display.setup();

    // --- Логика ---
    logic = std::make_unique<LogicController>(gpsController.gpsProcessed, btController.command, wifiController, prefs);
    if (logic) {
        logic->setup();
    }

    // После этого вся логика — в LogicController и связанных State/UiPage классах!
}

void loop()
{
    unsigned long now = millis();
    gpsController.loop(now);
    btController.loop(now);
    wifiController.loop(now);
    logic->loop(now);
    display.loop(now); // UI всегда обновляется независимо от логики
}

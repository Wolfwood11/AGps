#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <math.h>
#include <memory>

#include "Structs.h"
#include "MathUtils.h"
#include "GpsController.h"
#include "BtController.h"
#include "Subscription.h"
#include "Binding.h"
#include "IUiPage.h"
#include "DisplayController.h"
#include "GlobalMsg.h"
#include "TrackingPage.h"
#include "WaitLapStartPage.h"
#include "SpeedPage.h" 
#include "WaitGpsPage.h" 

#ifdef USE_GPS_HARDWARE
#define GPS_RX 25
#define GPS_TX 26
HardwareSerial gpsSerial(2);
GpsController gpsController(gpsSerial, GPS_RX, GPS_TX);
#else
GpsController gpsController(Serial);
#endif

BtController btController(gpsController.nmeaRaw);
Preferences prefs;

// --- Глобальные переменные ---
GpsData latestGpsData;
bool gpsDataUpdated = false;
SubscriptionHolder gpsSubHolder;

unsigned long lastLapTime = 0;
unsigned long bestLapTime = 0;
unsigned long storedBestLap = 0;
bool lapStarted = false;

FullNmeaTime prevFixNmeaTime_G;
bool prevFixNmeaTime_G_isValid = false;
FullNmeaTime lapStartNmeaTime_G;

double prevLat = 0.0;
double prevLon = 0.0;

TrapLine trapLines[] = {
  {"Start/Finish", 52.444978239573054, 20.639983209455288,
                   52.44479509375817, 20.640014790479068,
                   0, 0, 0, 0, 264.0 }
};
const int numTraps = sizeof(trapLines) / sizeof(TrapLine);

enum TrackerState {
  STATE_WAIT_GPS,
  STATE_WAIT_LAP_START,
  STATE_TRACKING,
  STATE_SPEEDOMETER
};
TrackerState currentState = STATE_WAIT_GPS;

// --- UI ---
// Пример PageId:
constexpr int PAGE_WAIT_GPS      = 1;
constexpr int PAGE_SPEEDOMETER   = 2;
constexpr int PAGE_WAIT_LAP      = 3;
constexpr int PAGE_TRACKING      = 4;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
DisplayController display(u8g2);

// Создаем страницы через std::shared_ptr
auto waitGpsPage     = std::make_shared<WaitGpsPage>();
auto speedPage       = std::make_shared<SpeedPage>();
auto waitLapPage     = std::make_shared<WaitLapStartPage>();
auto trackingPage    = std::make_shared<TrackingPage>();

void setup() {
  Serial.begin(115200);
  gpsController.setup();
  gpsController.gpsProcessed->Subscribe([&](const GpsData& data) {
    latestGpsData = data;
    gpsDataUpdated = true;
  }, gpsSubHolder);

  btController.setup();
  btController.command->Subscribe([&](const String& cmd) {
    if (cmd == "SETMODE TRACK") {
      currentState = STATE_WAIT_LAP_START;
      lapStarted = false;
      prevFixNmeaTime_G_isValid = false;
      btController.sendResponse("ACK: MODE TRACK");
      GlobalMsg::Broadcast(PAGE_WAIT_LAP);
    } else if (cmd == "SETMODE SPD") {
      currentState = STATE_SPEEDOMETER;
      btController.sendResponse("ACK: MODE SPD");
      GlobalMsg::Broadcast(PAGE_SPEEDOMETER);
    } else if (cmd == "RESET PB") {
      prefs.begin("lapdata", false);
      prefs.putULong("bestLap", 0);
      prefs.end();
      storedBestLap = 0;
      bestLapTime = 0;
      btController.sendResponse("ACK: PB RESET");
    } else if (cmd.startsWith("SET PB ")) {
      unsigned long newPB = cmd.substring(7).toInt();
      if (newPB > 0) {
        storedBestLap = newPB;
        prefs.begin("lapdata", true);
        prefs.putULong("bestLap", storedBestLap);
        prefs.end();
        btController.sendResponse("ACK: PB SET TO " + String(storedBestLap));
      } else btController.sendResponse("ERR: INVALID PB VALUE");
    } else {
      btController.sendResponse("ERR: UNKNOWN CMD " + cmd);
    }
  }, gpsSubHolder);

  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_logisoso20_tf);

  prefs.begin("lapdata", false);
  storedBestLap = prefs.getULong("bestLap", 0);
  bestLapTime = storedBestLap;
  prefs.end();

  for (int i = 0; i < numTraps; ++i) {
    MathUtils::latLonToMeters(trapLines[i].lat1, trapLines[i].lon1, trapLines[i].x1, trapLines[i].y1);
    MathUtils::latLonToMeters(trapLines[i].lat2, trapLines[i].lon2, trapLines[i].x2, trapLines[i].y2);
  }

  // --- Привязка биндингов к страницам ---
  waitGpsPage->satellites = [&]() { return latestGpsData.satellites; };
  waitGpsPage->btConnected = [&]() { return btController.isConnected(); };

  speedPage->speedKmph = [&]() { return latestGpsData.speedKmph; };
  speedPage->satellites = [&]() { return latestGpsData.satellites; };

  waitLapPage->pb = [&]() { return storedBestLap; };
  waitLapPage->satellites = [&]() { return latestGpsData.satellites; };

  trackingPage->lapTime = [&]() { return (lapStartNmeaTime_G.isValid && latestGpsData.nmeaTime.isValid)
        ? MathUtils::calculate_lap_duration_ms(latestGpsData.nmeaTime, lapStartNmeaTime_G)
        : 0; };
  trackingPage->bestLapTime = [&]() { return bestLapTime; };
  trackingPage->storedBestLap = [&]() { return storedBestLap; };

  // --- Регистрируем страницы ---
  display.registerPage(PAGE_WAIT_GPS, waitGpsPage);
  display.registerPage(PAGE_SPEEDOMETER, speedPage);
  display.registerPage(PAGE_WAIT_LAP, waitLapPage);
  display.registerPage(PAGE_TRACKING, trackingPage);

  GlobalMsg::Broadcast(PAGE_WAIT_LAP);
}

void loop() {
  gpsController.loop(millis());
  btController.loop(millis());
  unsigned long now = millis();
  bool gpsOk = latestGpsData.locationValid && (now - latestGpsData.millisReceived <= MathUtils::gpsTimeoutMs);

  double lat = latestGpsData.latitude;
  double lon = latestGpsData.longitude;
  int sats = latestGpsData.satellites;
  FullNmeaTime curTime = latestGpsData.nmeaTime;
  bool nmeaValid = curTime.isValid;

  // State transitions
  if (currentState == STATE_WAIT_GPS && gpsOk && sats >= 4 && nmeaValid) {
    currentState = STATE_WAIT_LAP_START;
    Serial.println("WAIT LAP");
    GlobalMsg::Broadcast(PAGE_WAIT_LAP);
  }
  if (!gpsOk) {
    currentState = STATE_WAIT_GPS;
    prevFixNmeaTime_G_isValid = false;
    lapStarted = false;
    Serial.println("!gpsOk");
    GlobalMsg::Broadcast(PAGE_WAIT_GPS);
  }

  // Основная логика круга
  switch (currentState) {
    case STATE_WAIT_LAP_START:
    case STATE_TRACKING: {
      if (!gpsOk || !nmeaValid) break;
      double tRatio;
      if (prevFixNmeaTime_G_isValid &&
          MathUtils::crossedLineInterpolated(prevLat, prevLon, lat, lon, trapLines[0], tRatio)) {
        Serial.println("crossedLineInterpolated ");
        long dt = MathUtils::calculate_nmea_fix_interval_ms(curTime, prevFixNmeaTime_G);
        if (dt >= 10 && dt <= 1000) {
          FullNmeaTime crossTime = MathUtils::add_ms_to_nmea_time(prevFixNmeaTime_G, long(tRatio * dt));
          if (currentState == STATE_WAIT_LAP_START) {
            lapStartNmeaTime_G = crossTime;
            lapStartNmeaTime_G.isValid = true;
            lapStarted = true;
            Serial.println("STATE_TRACKING 1");
            currentState = STATE_TRACKING;
            GlobalMsg::Broadcast(PAGE_TRACKING);
          } else if (currentState == STATE_TRACKING) {
            unsigned long lapTime = MathUtils::calculate_lap_duration_ms(crossTime, lapStartNmeaTime_G);
            if (lapTime >= MathUtils::minLapTimeMs) {
              lastLapTime = lapTime;
              if (bestLapTime == 0 || lapTime < bestLapTime) {
                bestLapTime = lapTime;
                if (storedBestLap == 0 || bestLapTime < storedBestLap) {
                  storedBestLap = bestLapTime;
                  prefs.begin("lapdata", false);
                  prefs.putULong("bestLap", bestLapTime);
                  prefs.end();
                }
              }
              lapStartNmeaTime_G = crossTime;
            }
          }
        }
      }
      break;
    }
  }

  display.loop(millis());

  // --- Сохраняем предыдущие значения для интерполяции ---
  if (gpsOk && nmeaValid) {
    prevLat = lat;
    prevLon = lon;
    prevFixNmeaTime_G = curTime;
    prevFixNmeaTime_G_isValid = true;
  }
}

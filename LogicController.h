#ifndef LogicController_H
#define LogicController_H

#include <BtController.h>
#include <Enums.h>
#include <ObjectStateMachine.h>
#include <Preferences.h>
#include <WiFiUdpController.h>
#include <Structs.h>
#include <memory>

// Стейты вперёд объявляем:
class WaitGpsState;
class WaitLapStartState;
class TrackingState;
class SpeedometerState;

class LogicController : public BaseController {
public:
    LogicController(const std::shared_ptr<Subscription<GpsData>>& gpsProcessed,
        const std::shared_ptr<Subscription<String>>& btCommand,
        WiFiUdpController& wifiCtrl,
        Preferences& pref);
    ~LogicController() override;

    void setup() override;
    void FillDisplayFacade();
    void loop(unsigned long dt) override;
    void onGpsUpdate(const GpsData& data);
    void onBtCommand(const String& cmd);
    void sendSessionStartMessage();

    SubscriptionHolder gpsHolder;
    SubscriptionHolder btCommandHolder;
    SubscriptionHolder wifiHolder;
    WiFiUdpController* wifi;
    // --- Доступ к данным для стейтов ---
    Preferences& prefs;

    TrapLine trapLines[1] = { // если хочешь больше — размер меняй
        { "Start/Finish", 52.444978239573054, 20.639983209455288,
            52.44479509375817, 20.640014790479068,
            0, 0, 0, 0, 264.0 }
    };
    const int numTraps = sizeof(trapLines) / sizeof(TrapLine);

    GpsData latestGpsData;
    unsigned long lastLapTime;
    unsigned long bestLapTime;
    unsigned long storedBestLap;
    bool lapStarted;

    FullNmeaTime prevFixNmeaTime_G;
    bool prevFixNmeaTime_G_isValid;
    FullNmeaTime lapStartNmeaTime_G;

    double prevLat;
    double prevLon;

    // --- State Machine ---
    ObjectStateMachine<LogicStateId> stateMachine;
    int gpsDataUpdated;
};
#endif

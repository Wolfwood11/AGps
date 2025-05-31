#pragma once

#include "BtController.h"
#include "DisplayController.h"
#include "Enums.h"
#include "GpsController.h"
#include "IState.h"
#include "MathUtils.h"
#include "ObjectStateMachine.h"
#include "Structs.h"
#include <Preferences.h>
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
        Preferences& pref);
    ~LogicController() override;

    void setup() override;
    void FillDisplayFacade();
    void loop(unsigned long dt) override;
    void onGpsUpdate(const GpsData& data);
    void onBtCommand(const String& cmd);

    SubscriptionHolder gpsHolder;
    SubscriptionHolder btCommandHolder;
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

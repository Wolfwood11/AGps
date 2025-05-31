#include "LogicController.h"

#include "DisplayFacade.h"
#include "GlobalMsg.h"
#include "Logger.h"
#include "SpeedometerState.h"
#include "TrackingState.h"
#include "WaitGpsState.h"
#include "WaitLapStartState.h"

LogicController::LogicController(
    const std::shared_ptr<Subscription<GpsData>>& gpsProcessed,
    const std::shared_ptr<Subscription<String>>& btCommand,
    Preferences& pref)
    : prefs(pref)
{
    if (gpsProcessed) {
        gpsProcessed->Subscribe([this](const GpsData& data) {
            onGpsUpdate(data);
        },
            gpsHolder);
    }

    if (btCommand) {
        btCommand->Subscribe([this](const String& cmd) {
            onBtCommand(cmd);
        },
            btCommandHolder);
    }

    for (int i = 0; i < numTraps; ++i) {
        MathUtils::latLonToMeters(trapLines[i].lat1, trapLines[i].lon1, trapLines[i].x1, trapLines[i].y1);
        MathUtils::latLonToMeters(trapLines[i].lat2, trapLines[i].lon2, trapLines[i].x2, trapLines[i].y2);
    }
}

LogicController::~LogicController()
{
    gpsHolder.reset();
    btCommandHolder.reset();
}

void LogicController::setup()
{
    // Регистрируем все стейты
    stateMachine.registerState(LOGIC_WAIT_GPS, std::make_shared<WaitGpsState>(this));
    stateMachine.registerState(LOGIC_WAIT_LAP_START, std::make_shared<WaitLapStartState>(this));
    stateMachine.registerState(LOGIC_TRACKING, std::make_shared<TrackingState>(this));
    stateMachine.registerState(LOGIC_SPEEDOMETER, std::make_shared<SpeedometerState>(this));
    stateMachine.setState(LOGIC_WAIT_GPS);
}

void LogicController::FillDisplayFacade()
{
    auto& display = DisplayFacade::instance();
    display.setSatellites(latestGpsData.satellites);
    display.setSpeedKmph(latestGpsData.speedKmph);
    display.setPb(storedBestLap);
    display.setLapTime(
        (lapStartNmeaTime_G.isValid && latestGpsData.nmeaTime.isValid)
            ? MathUtils::calculate_lap_duration_ms(latestGpsData.nmeaTime, lapStartNmeaTime_G)
            : 0);
    display.setBestLapTime(bestLapTime);
    display.setStoredBestLap(storedBestLap);
}

void LogicController::loop(unsigned long dt)
{
    unsigned long now = millis();
    bool gpsOk = latestGpsData.locationValid && (now - latestGpsData.millisReceived <= MathUtils::gpsTimeoutMs);

    double lat = latestGpsData.latitude;
    double lon = latestGpsData.longitude;
    FullNmeaTime curTime = latestGpsData.nmeaTime;
    bool nmeaValid = curTime.isValid;

    if (!gpsOk) {
        prevFixNmeaTime_G_isValid = false;
        lapStarted = false;
        stateMachine.setState(LOGIC_WAIT_GPS);
        Logger::log("gpsOk LOGIC_WAIT_GPS");
    }

    stateMachine.loop(dt);
    FillDisplayFacade();

    if (gpsOk && nmeaValid) {
        prevLat = lat;
        prevLon = lon;
        prevFixNmeaTime_G = curTime;
        prevFixNmeaTime_G_isValid = true;
    }
}

void LogicController::onGpsUpdate(const GpsData& data)
{
    latestGpsData = data;
    gpsDataUpdated = true;
}

void LogicController::onBtCommand(const String& cmd)
{
    if (cmd == "SETMODE TRACK") {
        stateMachine.setState(LOGIC_WAIT_LAP_START);
        lapStarted = false;
        prevFixNmeaTime_G_isValid = false;
    } else if (cmd == "SETMODE SPD") {
        stateMachine.setState(LOGIC_SPEEDOMETER);
    } else if (cmd == "RESET PB") {
        prefs.begin("lapdata", false);
        prefs.putULong("bestLap", 0);
        prefs.end();
        storedBestLap = 0;
        bestLapTime = 0;
    } else if (cmd.startsWith("SET PB ")) {
        unsigned long newPB = cmd.substring(7).toInt();
        if (newPB > 0) {
            storedBestLap = newPB;
            prefs.begin("lapdata", true);
            prefs.putULong("bestLap", storedBestLap);
            prefs.end();
        }
    }
}

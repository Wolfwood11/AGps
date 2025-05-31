// TrackingState.cpp
#include "TrackingState.h"
#include "LogicController.h"
#include "MathUtils.h"
#include "GlobalMsg.h"
#include "Enums.h"

void TrackingState::enter() {
    GlobalMsg::Broadcast(PAGE_TRACKING);
}

void TrackingState::loop(unsigned long) {
    auto& lc = *logic;
    auto& data = lc.latestGpsData;
    if (!data.locationValid || (millis() - data.millisReceived > MathUtils::gpsTimeoutMs) || !data.nmeaTime.isValid) {
        stateMachine.setState(LOGIC_WAIT_GPS);
        return;
    }
    double tRatio;
    if (lc.prevFixNmeaTime_G_isValid &&
        MathUtils::crossedLineInterpolated(
            lc.prevLat, lc.prevLon, data.latitude, data.longitude, lc.trapLines[0], tRatio))
    {
        long dt = MathUtils::calculate_nmea_fix_interval_ms(data.nmeaTime, lc.prevFixNmeaTime_G);
        if (dt >= 10 && dt <= 1000) {
            FullNmeaTime crossTime = MathUtils::add_ms_to_nmea_time(lc.prevFixNmeaTime_G, long(tRatio * dt));
            unsigned long lapTime = MathUtils::calculate_lap_duration_ms(crossTime, lc.lapStartNmeaTime_G);
            if (lapTime >= MathUtils::minLapTimeMs) {
                lc.lastLapTime = lapTime;
                if (lc.bestLapTime == 0 || lapTime < lc.bestLapTime) {
                    lc.bestLapTime = lapTime;
                    if (lc.storedBestLap == 0 || lc.bestLapTime < lc.storedBestLap) {
                        lc.storedBestLap = lc.bestLapTime;
                        lc.prefs.begin("lapdata", false);
                        lc.prefs.putULong("bestLap", lc.bestLapTime);
                        lc.prefs.end();
                    }
                }
                lc.lapStartNmeaTime_G = crossTime;
            }
        }
    }
}

// WaitLapStartState.cpp
#include <WaitLapStartState.h>
#include <Enums.h>
#include <GlobalMsg.h>
#include <Logger.h>
#include <LogicController.h>
#include <MathUtils.h>
#include <NetwrokMsg.h>

void WaitLapStartState::enter()
{
    GlobalMsg::Broadcast(PAGE_WAIT_LAP);
    Logger::log("enter PAGE_WAIT_LAP");
}

void WaitLapStartState::loop(unsigned long)
{
    if (!logic) {
        return;
    }
    auto& lc = *logic;
    auto& data = lc.latestGpsData;
    if (!data.locationValid || (millis() - data.millisReceived > MathUtils::gpsTimeoutMs) || !data.nmeaTime.isValid) {
        stateMachine.setState(LOGIC_WAIT_GPS);
        Logger::log("loop LOGIC_WAIT_GPS");
        return;
    }
    // Проверка на пересечение линии старта (вход в круг)
    double tRatio;
    if (lc.prevFixNmeaTime_G_isValid && MathUtils::crossedLineInterpolated(lc.prevLat, lc.prevLon, data.latitude, data.longitude, lc.trapLines[0], tRatio)) {
        Logger::log("loop crossedLineInterpolated");
        long dt = MathUtils::calculate_nmea_fix_interval_ms(data.nmeaTime, lc.prevFixNmeaTime_G);
        if (dt >= 10 && dt <= 1000) {
            if (millis() - MathUtils::lastCrossMillis >= MathUtils::minLapTimeMs / 4) {
                lc.lapStartNmeaTime_G = MathUtils::add_ms_to_nmea_time(lc.prevFixNmeaTime_G, long(tRatio * dt));
                lc.lapStartNmeaTime_G.isValid = true;
                lc.lapStarted = true;
                MathUtils::lastCrossMillis = millis();
                lc.sendSessionStartMessage();
                Logger::log("loop LOGIC_TRACKING");
                stateMachine.setState(LOGIC_TRACKING);
            }
        }
    }
}

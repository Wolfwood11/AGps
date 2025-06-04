// WaitGpsState.cpp
#include <WaitGpsState.h>
#include <Enums.h>
#include <GlobalMsg.h>
#include <Logger.h>
#include <LogicController.h>
#include <MathUtils.h>

void WaitGpsState::enter()
{
    GlobalMsg::Broadcast(PAGE_WAIT_GPS);
    Logger::log("PAGE_WAIT_GPS");
}
void WaitGpsState::loop(unsigned long)
{
    auto& data = logic->latestGpsData;
    bool gpsOk = data.locationValid && (millis() - data.millisReceived <= MathUtils::gpsTimeoutMs);
    int sats = data.satellites;
    bool nmeaValid = data.nmeaTime.isValid;
    if (gpsOk && sats >= 4 && nmeaValid) {
        stateMachine.setState(LOGIC_WAIT_LAP_START);
        Logger::log("setState LOGIC_WAIT_LAP_START");
    }
}

#ifndef Enums_H
#define Enums_H

constexpr int PAGE_WAIT_GPS = 1;
constexpr int PAGE_SPEEDOMETER = 2;
constexpr int PAGE_WAIT_LAP = 3;
constexpr int PAGE_TRACKING = 4;

enum LogicStateId {
    LOGIC_WAIT_GPS,
    LOGIC_WAIT_LAP_START,
    LOGIC_TRACKING,
    LOGIC_SPEEDOMETER
};
#endif

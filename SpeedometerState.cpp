// SpeedometerState.cpp
#include <SpeedometerState.h>
#include <LogicController.h>
#include <GlobalMsg.h>
#include <Enums.h>

void SpeedometerState::enter() {
     GlobalMsg::Broadcast(PAGE_SPEEDOMETER);
}
void SpeedometerState::loop(unsigned long) {
    // Здесь пока просто оставляем вывод скорости (всё биндингами)
    // Можно добавить авто-выход через время или по условию.
}

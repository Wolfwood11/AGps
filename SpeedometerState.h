#ifndef SpeedometerState_H
#define SpeedometerState_H

#include <BaseLogicState.h>

class SpeedometerState : public BaseLogicState {
public:
    using BaseLogicState::BaseLogicState;
    void enter() override;
    void loop(unsigned long dt) override;
};
#endif

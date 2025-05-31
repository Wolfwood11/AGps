// SpeedometerState.h
#pragma once
#include "BaseLogicState.h"

class SpeedometerState : public BaseLogicState {
public:
using BaseLogicState::BaseLogicState;
    void enter() override;
    void loop(unsigned long dt) override;
};

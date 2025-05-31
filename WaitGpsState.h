// WaitGpsState.h
#pragma once
#include "BaseLogicState.h"

class WaitGpsState : public BaseLogicState {
public:
  using BaseLogicState::BaseLogicState;
    void enter() override;
    void loop(unsigned long dt) override;
};

#ifndef WaitLapStartState_H
#define WaitLapStartState_H

#include <BaseLogicState.h>

class WaitLapStartState : public BaseLogicState {
public:
    using BaseLogicState::BaseLogicState;
    void enter() override;
    void loop(unsigned long dt) override;
};
#endif

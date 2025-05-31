#pragma once
#include "IState.h"
#include "Enums.h"

class LogicController;

class BaseLogicState : public IState<LogicStateId> {
public:
    BaseLogicState(LogicController* logic);

protected:
    LogicController* logic;
};

#ifndef BaseLogicState_H
#define BaseLogicState_H

#include <Enums.h>
#include <IState.h>

class LogicController;

class BaseLogicState : public IState<LogicStateId> {
public:
    BaseLogicState(LogicController* logic);

protected:
    LogicController* logic;
};
#endif

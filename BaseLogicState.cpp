#include "BaseLogicState.h"
#include "LogicController.h"

BaseLogicState::BaseLogicState(LogicController* logic) : IState<LogicStateId>(logic->stateMachine), logic(logic) {}
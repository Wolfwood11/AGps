#ifndef ObjectStateMachine_H
#define ObjectStateMachine_H

#include <map>
#include <memory>

template <typename EnumType>
class IState; // Forward declaration

template <typename EnumType>
class ObjectStateMachine {
public:
    void registerState(EnumType id, std::shared_ptr<IState<EnumType>> state)
    {
        states[id] = state;
    }

    void setState(EnumType id)
    {
        if (currentState && currentId != id)
            currentState->exit();
        currentState = states[id];
        currentId = id;
        if (currentState)
            currentState->enter();
    }

    void loop(unsigned long dt)
    {
        if (currentState)
            currentState->loop(dt);
    }

    EnumType getCurrentStateId() const { return currentId; }

private:
    std::map<EnumType, std::shared_ptr<IState<EnumType>>> states;
    std::shared_ptr<IState<EnumType>> currentState = nullptr;
    EnumType currentId;
};
#endif

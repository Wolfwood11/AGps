#ifndef StateMachine_H
#define StateMachine_H

#include <functional>
#include <queue>
#include <unordered_map>

template <typename EnumType>
class StateMachine {
public:
    template <typename ObjectType>
    void Bind(EnumType State, ObjectType* ObjectPtr, void (ObjectType::*FunctionPtr)(unsigned long), void (ObjectType::*EnterStateFunctionPtr)(unsigned long) = nullptr, void (ObjectType::*ExitStateFunctionPtr)(unsigned long) = nullptr)
    {
        if (ObjectPtr && FunctionPtr) {
            if (StateFunctionMap.empty()) {
                DefaultState = State;
            }

            StateFunctionMap[State] = [ObjectPtr, FunctionPtr](unsigned long dt) {
                (ObjectPtr->*FunctionPtr)(dt);
            };

            if (EnterStateFunctionPtr) {
                EnterStateFunctionMap[State] = [ObjectPtr, EnterStateFunctionPtr](unsigned long dt) {
                    (ObjectPtr->*EnterStateFunctionPtr)(dt);
                };
            }

            if (ExitStateFunctionPtr) {
                ExitStateFunctionMap[State] = [ObjectPtr, ExitStateFunctionPtr](unsigned long dt) {
                    (ObjectPtr->*ExitStateFunctionPtr)(dt);
                };
            }
        } else {
            // Handle error: Null pointer.
            Serial.println("Bind called with null ObjectPtr or FunctionPtr");
        }
    }

    void SetState(EnumType State)
    {
        if (NextState != State) {
            NextState = State;
            StateChanged = true;
            ExitStateCalled = false;
        }
    }

    void SetDefaultState(EnumType State)
    {
        if (StateFunctionMap.find(State) != StateFunctionMap.end()) {
            DefaultState = State;
        }
    }

    void SetGoToDefaultState()
    {
        SetState(DefaultState);
    }

    void ReturnToPreviousState()
    {
        if (!PreviousStates.empty()) {
            NextState = PreviousStates.front();
            PreviousStates.pop();
            StateChanged = true;
            ExitStateCalled = false;
        } else {
            SetGoToDefaultState();
        }
    }

    void UpdateState(unsigned long deltaTime)
    {
        if (!Enabled) {
            return;
        }

        if (StateChanged) {
            if (!ExitStateCalled) {
                if (ExitStateFunctionMap.find(CurrentState) != ExitStateFunctionMap.end()) {
                    ExitStateFunctionMap[CurrentState](deltaTime);
                }
                ExitStateCalled = true;
                return;
            }

            PrevState = CurrentState;

            if (StateFunctionMap.find(PrevState) != StateFunctionMap.end()) {
                PreviousStates.push(PrevState);
            }

            CurrentState = NextState;
            StateChanged = false;

            if (EnterStateFunctionMap.find(CurrentState) != EnterStateFunctionMap.end()) {
                EnterStateFunctionMap[CurrentState](deltaTime);
                return;
            }
        }

        if (StateFunctionMap.find(CurrentState) != StateFunctionMap.end()) {
            StateFunctionMap[CurrentState](deltaTime);
        }
    }

    void SetEnabled(bool value)
    {
        Enabled = value;
    }

    bool GetEnabled() const
    {
        return Enabled;
    }

    EnumType GetCurrentState() const { return CurrentState; }
    EnumType GetNextState() const { return NextState; }
    EnumType GetPrevState() const { return PrevState; }

private:
    std::unordered_map<EnumType, std::function<void(unsigned long)>> StateFunctionMap;
    std::unordered_map<EnumType, std::function<void(unsigned long)>> EnterStateFunctionMap;
    std::unordered_map<EnumType, std::function<void(unsigned long)>> ExitStateFunctionMap;

    EnumType CurrentState;
    EnumType NextState;
    EnumType PrevState;
    EnumType DefaultState;
    std::queue<EnumType> PreviousStates;

    bool Enabled = true;
    bool StateChanged = false;
    bool ExitStateCalled = false;
};
#endif

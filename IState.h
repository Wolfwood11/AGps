#ifndef IState_H
#define IState_H

template <typename EnumType>
class ObjectStateMachine; // Forward declaration

template <typename EnumType>
class IState {
public:
    IState(ObjectStateMachine<EnumType>& sm)
        : stateMachine(sm)
    {
    }
    virtual ~IState() = default;

    // Вызывается один раз при входе в стейт
    virtual void enter() { }
    // Вызывается каждый кадр (или тик)
    virtual void loop(unsigned long dt) {};
    // Вызывается при выходе из стейта
    virtual void exit() { }

protected:
    ObjectStateMachine<EnumType>& stateMachine;
};
#endif

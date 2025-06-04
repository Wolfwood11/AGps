#ifndef Binding_H
#define Binding_H

#include <functional>

template <typename T>
class Binding {
public:
    using Getter = std::function<T()>;

    Binding() = default;
    Binding(Getter g)
        : getter(g)
    {
    }

    // Позволяет присваивать Binding-у лямбду напрямую
    Binding& operator=(Getter g)
    {
        getter = g;
        return *this;
    }

    // Позволяет использовать Binding<T> как значение типа T (автоматическое преобразование)
    operator T() const
    {
        return getter ? getter() : T();
    }

    // Для совместимости оставим value() если где-то нужно явно
    T value() const { return getter ? getter() : T(); }

private:
    Getter getter;
};
#endif

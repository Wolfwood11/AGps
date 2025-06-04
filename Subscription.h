#ifndef Subscription_H
#define Subscription_H

#include <functional>
#include <map>
#include <memory>

class SubscriptionHolder {
public:
    using UnsubscribeFunction = std::function<void()>;

    SubscriptionHolder() = default;

    void SetUnsubscribeFunction(UnsubscribeFunction unsubscribeFunction)
    {
        unsubscribeFunction_ = unsubscribeFunction;
    }

    ~SubscriptionHolder()
    {
        reset();
    }

    void reset()
    {
        if (unsubscribeFunction_) {
            unsubscribeFunction_();
            unsubscribeFunction_ = nullptr;
        }
    }

private:
    UnsubscribeFunction unsubscribeFunction_;
};

template <typename T>
class Subscription : public std::enable_shared_from_this<Subscription<T>> {
public:
    using CallbackType = std::function<void(T)>;

    Subscription() = default;

    void Subscribe(CallbackType callback, SubscriptionHolder& holder)
    {
        int id = currentId++;
        callbacks_.emplace(id, callback);

        auto weakSelf = std::weak_ptr<Subscription<T>>(this->shared_from_this());
        holder.SetUnsubscribeFunction([weakSelf, id]() {
            if (auto self = weakSelf.lock()) {
                self->unsubscribe(id);
            }
        });
    }

    void Broadcast(T value)
    {
        for (const auto& [id, callback] : callbacks_) {
            if (callback) {
                callback(value);
            }
        }
    }

private:
    void unsubscribe(int id)
    {
        callbacks_.erase(id);
    }

    std::map<int, CallbackType> callbacks_;
    int currentId = 0;
};
#endif

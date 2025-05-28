// GlobalMsg.h
#pragma once
#include <functional>
#include <vector>

class GlobalMsg {
public:
    using Handler = std::function<void(int)>; // int — PageId

    static void Subscribe(Handler handler);
    static void Broadcast(int pageId);
};

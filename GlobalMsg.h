#ifndef GlobalMsg_H
#define GlobalMsg_H

#include <functional>

class GlobalMsg {
public:
    using Handler = std::function<void(int)>; // int — PageId

    static void Subscribe(Handler handler);
    static void Broadcast(int pageId);
};
#endif

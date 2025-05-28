// GlobalMsg.cpp
#include "GlobalMsg.h"

static std::vector<GlobalMsg::Handler> g_handlers;

void GlobalMsg::Subscribe(Handler handler) {
    g_handlers.push_back(handler);
}

void GlobalMsg::Broadcast(int pageId) {
    for (auto& h : g_handlers) h(pageId);
}

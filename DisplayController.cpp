// DisplayController.cpp
#include "DisplayController.h"
#include "GlobalMsg.h"

DisplayController::DisplayController(U8G2& display) : u8g2(display), currentPage(nullptr) {
    // Подписываемся на глобальные PageId
    GlobalMsg::Subscribe([this](int pageId) { this->onPageMsg(pageId); });
}

void DisplayController::setup() {
    u8g2.begin();
    u8g2.enableUTF8Print();
}

void DisplayController::loop(unsigned long dt) {
    u8g2.clearBuffer();
    if (currentPage) currentPage->render(u8g2);
    u8g2.sendBuffer();
}

void DisplayController::registerPage(int pageId, std::shared_ptr<IUiPage> page) {
    pages[pageId] = page;
}

std::shared_ptr<IUiPage> DisplayController::getCurrentPage() const {
    return currentPage;
}

void DisplayController::onPageMsg(int pageId) {
    auto it = pages.find(pageId);
    if (it != pages.end()) {
        if (currentPage) currentPage->onExit();
        currentPage = it->second;
        if (currentPage) currentPage->onEnter();
    }
}

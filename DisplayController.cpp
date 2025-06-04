// DisplayController.cpp
#include <DisplayController.h>
#include <DisplayFacade.h>
#include <Enums.h>
#include <GlobalMsg.h>
#include <SpeedPage.h>
#include <TrackingPage.h>
#include <WaitGpsPage.h>
#include <WaitLapStartPage.h>

DisplayController::DisplayController(U8G2& display)
    : u8g2(display)
    , currentPage(nullptr)
{
    // Подписываемся на глобальные PageId
    GlobalMsg::Subscribe([this](int pageId) {
        this->onPageMsg(pageId);
    });
}

void DisplayController::setup()
{
    // --- Инициализация UI ---
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_logisoso20_tf);

    // Создаем страницы через std::shared_ptr
    auto waitGpsPage = std::make_shared<WaitGpsPage>();
    auto speedPage = std::make_shared<SpeedPage>();
    auto waitLapPage = std::make_shared<WaitLapStartPage>();
    auto trackingPage = std::make_shared<TrackingPage>();

    waitGpsPage->satellites = []() {
        return DisplayFacade::instance().getSatellites();
    };
    waitGpsPage->btConnected = []() {
        return DisplayFacade::instance().getBtConnected();
    };

    speedPage->speedKmph = []() {
        return DisplayFacade::instance().getSpeedKmph();
    };
    speedPage->satellites = []() {
        return DisplayFacade::instance().getSatellites();
    };

    waitLapPage->pb = []() {
        return DisplayFacade::instance().getPb();
    };
    waitLapPage->satellites = []() {
        return DisplayFacade::instance().getSatellites();
    };

    trackingPage->lapTime = []() {
        return DisplayFacade::instance().getLapTime();
    };
    trackingPage->bestLapTime = []() {
        return DisplayFacade::instance().getBestLapTime();
    };
    trackingPage->storedBestLap = []() {
        return DisplayFacade::instance().getStoredBestLap();
    };

    registerPage(PAGE_WAIT_GPS, waitGpsPage);
    registerPage(PAGE_SPEEDOMETER, speedPage);
    registerPage(PAGE_WAIT_LAP, waitLapPage);
    registerPage(PAGE_TRACKING, trackingPage);
}

void DisplayController::loop(unsigned long dt)
{
    u8g2.clearBuffer();
    if (currentPage)
        currentPage->render(u8g2);
    u8g2.sendBuffer();
}

void DisplayController::registerPage(int pageId, std::shared_ptr<IUiPage> page)
{
    pages[pageId] = page;
}

std::shared_ptr<IUiPage> DisplayController::getCurrentPage() const
{
    return currentPage;
}

void DisplayController::onPageMsg(int pageId)
{
    auto it = pages.find(pageId);
    if (it != pages.end()) {
        if (currentPage)
            currentPage->onExit();
        currentPage = it->second;
        if (currentPage)
            currentPage->onEnter();
    }
}

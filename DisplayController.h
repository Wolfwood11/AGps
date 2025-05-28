// DisplayController.h
#pragma once
#include "BaseController.h"
#include "IUiPage.h"
#include <U8g2lib.h>
#include <memory>
#include <map>

class DisplayController : public BaseController {
public:
    DisplayController(U8G2& display);

    void setup() override;
    void loop(unsigned long dt) override;

    // Привязка страницы к id
    void registerPage(int pageId, std::shared_ptr<IUiPage> page);

    std::shared_ptr<IUiPage> getCurrentPage() const;

private:
    U8G2& u8g2;
    std::shared_ptr<IUiPage> currentPage;
    std::map<int, std::shared_ptr<IUiPage>> pages;
    void onPageMsg(int pageId);
};

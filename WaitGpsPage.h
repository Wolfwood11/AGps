// WaitGpsPage.h
#pragma once
#include "IUiPage.h"
#include "Binding.h"

class WaitGpsPage : public IUiPage {
public:
    Binding<int> satellites;
    Binding<bool> btConnected;

    WaitGpsPage() = default;
    void render(U8G2& u8g2) override {
        u8g2.setFont(u8g2_font_helvR10_tr);
        u8g2.setCursor(0, 12); u8g2.print("WAITING FOR GPS");
        u8g2.setCursor(0, 28); u8g2.print("Sats: "); u8g2.print((int)satellites);
        u8g2.setCursor(0, 44); u8g2.print("BT: "); u8g2.print((bool)btConnected ? "Connected" : "Waiting...");
    }
};

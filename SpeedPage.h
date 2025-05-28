// SpeedPage.h
#pragma once
#include "IUiPage.h"
#include "Binding.h"

class SpeedPage : public IUiPage {
public:
    Binding<float> speedKmph;
    Binding<int> satellites;

    SpeedPage() = default;
    void render(U8G2& u8g2) override {
        u8g2.setFont(u8g2_font_logisoso20_tf);
        u8g2.setCursor(0, 28);
        float speed = speedKmph;
        if (speed > 0)
            u8g2.print(String(speed, 1) + " km/h");
        else
            u8g2.print("--.- km/h");
        u8g2.setFont(u8g2_font_helvR08_tr);
        u8g2.setCursor(0, 50); u8g2.print("SAT: "); u8g2.print((int)satellites);
    }
};

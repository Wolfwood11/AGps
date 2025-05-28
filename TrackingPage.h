// TrackingPage.h
#pragma once
#include "IUiPage.h"
#include "Binding.h"

class TrackingPage : public IUiPage {
public:
    Binding<unsigned long> lapTime;
    Binding<unsigned long> bestLapTime;
    Binding<unsigned long> storedBestLap;

    TrackingPage() = default;
    void render(U8G2& u8g2) override {
        char lapBuf[20], bestBuf[20];
        unsigned long lap = lapTime, best = bestLapTime, stored = storedBestLap;
        if (lap == 0)
            sprintf(lapBuf, "--:--:--");
        else
            sprintf(lapBuf, "%lu:%02lu:%02lu", lap / 60000, (lap / 1000) % 60, (lap % 1000) / 10);

        unsigned long showBest = (best > 0) ? best : stored;
        if (showBest == 0)
            sprintf(bestBuf, "--:--:--");
        else
            sprintf(bestBuf, "%lu:%02lu:%02lu", showBest / 60000, (showBest / 1000) % 60, (showBest % 1000) / 10);

        u8g2.setFont(u8g2_font_logisoso22_tf);
        u8g2.setCursor(0, 30); u8g2.print("L "); u8g2.print(lapBuf);
        u8g2.setCursor(0, 60); u8g2.print("B "); u8g2.print(bestBuf);
    }
};

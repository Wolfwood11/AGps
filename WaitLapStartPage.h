#ifndef WaitLapStartPage_H
#define WaitLapStartPage_H
#include <Binding.h>
#include <IUiPage.h>

class WaitLapStartPage : public IUiPage {
public:
    Binding<unsigned long> pb;
    Binding<int> satellites;

    WaitLapStartPage() = default;
    void render(U8G2& u8g2) override
    {
        char buf[20];
        unsigned long pbVal = pb;
        if (pbVal == 0)
            sprintf(buf, "--:--:--");
        else
            sprintf(buf, "%lu:%02lu:%02lu", pbVal / 60000, (pbVal / 1000) % 60, (pbVal % 1000) / 10);

        u8g2.setFont(u8g2_font_helvR10_tr);
        u8g2.setCursor(0, 12);
        u8g2.print("READY TO START LAP");
        u8g2.setCursor(0, 30);
        u8g2.print("PB: ");
        u8g2.print(buf);
        u8g2.setCursor(0, 48);
        u8g2.print("SAT: ");
        u8g2.print((int)satellites);
    }
};
#endif

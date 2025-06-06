#ifndef IUiPage_H
#define IUiPage_H

#include <U8g2lib.h>

class IUiPage {
public:
    virtual ~IUiPage() = default;
    virtual void render(U8G2& u8g2) = 0;
    virtual void onEnter() { }
    virtual void onExit() { }
};
#endif

#ifndef BaseController_H
#define BaseController_H

class BaseController {
public:
    virtual ~BaseController() { }

    virtual void setup() = 0;
    virtual void loop(unsigned long dt) = 0;
};
#endif

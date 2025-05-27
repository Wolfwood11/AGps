#pragma once

class BaseController {
public:
    virtual ~BaseController() {} 

    virtual void setup() = 0; 
    virtual void loop(unsigned long dt) = 0;
};
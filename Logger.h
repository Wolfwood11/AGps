#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

class Logger {
public:
    // Устанавливаем HardwareSerial (например, Serial или Serial1)
    static void setLogger(HardwareSerial& serial)
    {
        instance().serial_ = &serial;
    }

    static bool isEnabled()
    {
        return instance().serial_ != nullptr;
    }

    static void log(const String& msg)
    {
        if (!isEnabled())
            return;
        instance().serial_->println(msg);
    }

    template <typename... Args>
    static void logf(const char* fmt, Args... args)
    {
        if (!isEnabled())
            return;
        char buf[256];
        snprintf(buf, sizeof(buf), fmt, args...);
        instance().serial_->println(buf);
    }

private:
    Logger()
        : serial_(nullptr)
    {
    }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static Logger& instance()
    {
        static Logger logger;
        return logger;
    }

    HardwareSerial* serial_;
};

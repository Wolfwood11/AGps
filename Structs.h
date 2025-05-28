#pragma once

#include <TinyGPSPlus.h>

struct TrapLine {
    const char* name;
    double lat1, lon1;
    double lat2, lon2;
    double x1, y1, x2, y2; // coords in m
    double bearing; // corossing direction
};

struct FullNmeaTime {
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint16_t hundredths; // 0-99 (1/100 секунды от TinyGPSPlus)
    bool isValid;

    FullNmeaTime()
        : year(0)
        , month(0)
        , day(0)
        , hour(0)
        , minute(0)
        , second(0)
        , hundredths(0)
        , isValid(false)
    {
    }

    void fromGps(TinyGPSPlus& gps_source)
    {
        isValid = gps_source.date.isValid() && gps_source.time.isValid();
        if (isValid) {
            year = gps_source.date.year();
            month = gps_source.date.month();
            day = gps_source.date.day();
            hour = gps_source.time.hour();
            minute = gps_source.time.minute();
            second = gps_source.time.second();
            hundredths = gps_source.time.centisecond();
        } else {
            // Явно сбрасываем значения, если данные GPS невалидны
            year = 0;
            month = 0;
            day = 0;
            hour = 0;
            minute = 0;
            second = 0;
            hundredths = 0;
        }
    }

    long getMsOfDay() const
    {
        if (!isValid)
            return 0;
        return hour * 3600000L + minute * 60000L + second * 1000L + hundredths * 10L;
    }
};

struct GpsData {
    double latitude;
    double longitude;
    double speedKmph;
    double hdop;
    int satellites;
    bool locationValid;
    bool speedValid;
    bool hdopValid;
    unsigned long msOfDay;

    // Добавляемые поля:
    unsigned long millisReceived;     // millis() во время получения фикса
    FullNmeaTime nmeaTime;  
};
#include "BtController.h"

#include <DisplayFacade.h>

BtController::BtController(std::shared_ptr<Subscription<String>> nmeaSource)
{
    if (nmeaSource) {
        nmeaSource->Subscribe([this](const String& line) {
            this->sendNmeaLine(line);
        },
            nmeaHolder);
    }
}

void BtController::sendResponse(const String& msg)
{
    if (btSerial.hasClient()) {
        btSerial.println(msg);
    }
}

bool BtController::isConnected()
{
    return btSerial.hasClient();
}

void BtController::setup()
{
    btSerial.begin("ESP32_GPS");
}

void BtController::loop(unsigned long)
{
    while (btSerial.available()) {
        char c = btSerial.read();
        if (c == '\n' || c == '\r') {
            if (!buffer.isEmpty()) {
                buffer.trim();
                if (command) {
                    command->Broadcast(buffer);
                }
                buffer = "";
            }
        } else if (buffer.length() < 50) {
            buffer += c;
        }
    }
    auto& display = DisplayFacade::instance();
    display.setBtConnected(isConnected());
}

void BtController::sendNmeaLine(const String& line)
{
    if (btSerial.hasClient()) {
        btSerial.print(line);
    }
}

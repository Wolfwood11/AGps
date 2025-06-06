#include <BtController.h>

#include <DisplayFacade.h>

BtController::BtController(std::shared_ptr<Subscription<char>> nmeaSource)
{
    if (nmeaSource) {
        nmeaSource->Subscribe([this](char c) {
            this->sendNmeaChar(c);
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

void BtController::sendNmeaChar(char c)
{
    if (btSerial.hasClient()) {
        btSerial.write(c);
    }
}

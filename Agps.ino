#include <BluetoothSerial.h>
#include <HardwareSerial.h>

#define GPS_RX 25 // D2
#define GPS_TX 26 // D3
HardwareSerial gpsSerial(2);
#define GPS_STREAM gpsSerial

BluetoothSerial SerialBT;

// Установить 115200 бод
uint8_t setBaud115200[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC4, 0x96,
    0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22
};

// Установить частоту обновления 10 Гц
uint8_t setRate10Hz[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12,
    0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30
};

struct TrapLine {
    const char* name;
    double lat1, lon1;
    double lat2, lon2;
};

void sendUBXCommand(HardwareSerial& serial, const uint8_t* cmd, size_t len, uint16_t postDelayMs = 100)
{
    for (size_t i = 0; i < len; i++)
        serial.write(cmd[i]);
    serial.flush();
    delay(postDelayMs);
}

void setup()
{
    Serial.begin(115200);

    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

    Serial.println("Отправка UBX команды на 115200...");
    sendUBXCommand(gpsSerial, setBaud115200, sizeof(setBaud115200));
    gpsSerial.updateBaudRate(115200);
    Serial.println("Отправка UBX команды на 10hz...");
    sendUBXCommand(gpsSerial, setRate10Hz, sizeof(setRate10Hz));

    SerialBT.begin("ESP32_GPS");
}

void loop()
{
    while (GPS_STREAM.available()) {
        char c = GPS_STREAM.read();
        SerialBT.write(c);
    }
}

#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <math.h> // Для fabs, fmin, fmax, sqrt, atan2, cos, M_PI
#include "Structs.h"
#include "MathUtils.h"

// Раскомментируйте, если используете аппаратный GPS UART
// #define USE_GPS_HARDWARE 1 

#ifdef USE_GPS_HARDWARE
  #define GPS_RX 25 // Укажите ваши пины RX для GPS
  #define GPS_TX 26 // Укажите ваши пины TX для GPS
  HardwareSerial gpsSerial(2); // Используем UART2 для GPS
  #define GPS_STREAM gpsSerial
#else
  #define GPS_STREAM Serial // Используем Serial для отладки или симуляции
#endif

TinyGPSPlus gps;
BluetoothSerial SerialBT;
Preferences prefs;

// Замените U8G2_SH1106_128X64_NONAME_F_HW_I2C на конструктор для вашего дисплея, если он другой
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Глобальные переменные для отладки NMEA строк (опционально)
String currentLine = "";
String lastLine = "";

String btCommand = ""; // Для команд Bluetooth

// Предыдущие значения для интерполяции
double prevLat = 0.0;
double prevLon = 0.0;
unsigned long lastValidFixTime = 0; // Время последнего валидного фикса по millis() для таймаута

// Глобальные переменные для времени NMEA
FullNmeaTime prevFixNmeaTime_G;       // Время NMEA предыдущего валидного фикса
bool prevFixNmeaTime_G_isValid = false;
FullNmeaTime lapStartNmeaTime_G;      // Время NMEA старта текущего круга

// Переменные для времени круга
unsigned long lastLapTime = 0;    // Длительность последнего круга в мс
unsigned long bestLapTime = 0;    // Длительность лучшего круга в текущей сессии в мс
unsigned long storedBestLap = 0;  // Лучший круг, сохраненный в Preferences
bool lapStarted = false;          // Флаг, что круг активно отслеживается


enum TrackerState {
  STATE_WAIT_GPS,
  STATE_WAIT_LAP_START,
  STATE_TRACKING,
  STATE_SPEEDOMETER
};
TrackerState currentState = STATE_WAIT_GPS;

// Определение линии старта/финиша
TrapLine trapLines[] = {
  {"Start/Finish", 52.444978239573054, 20.639983209455288, /* lat1, lon1 */
                   52.44479509375817, 20.640014790479068, /* lat2, lon2 */
                   0, 0, 0, 0,                          /* x1,y1,x2,y2 - заполнятся в setup */
                   264.0                                /* bearing - направление пересечения */
  }
  // Можно добавить другие линии (чекпоинты) сюда
};
const int numTraps = sizeof(trapLines) / sizeof(TrapLine);

// Команды для настройки GPS модуля (u-blox)
uint8_t setBaud115200[] = { // Команда для установки скорости 115200 бод
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
  0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x96
};
uint8_t ackBaud115200[] = { // Ответ модуля на команду смены скорости (для полноты)
  0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22
};

uint8_t setRate10Hz[] = { // Команда для установки частоты обновления 10 Гц
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12
};
uint8_t ackRate10Hz[] = { // Ответ модуля на команду смены частоты
  0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30
};

// Отображение времени на дисплее
void displayTimeRow(const char* prefix, unsigned long ms, int y_pos) {
  char buf[20]; // Увеличил буфер на всякий случай
  if (ms == 0 && bestLapTime != 0 && strcmp(prefix,"L ") == 0 ) { // Если текущий круг 0, но есть лучший (для L)
      sprintf(buf, "--:--:--");
  } else if (ms == 0 && strcmp(prefix,"B ") == 0 && storedBestLap == 0){ // Если лучший круг 0 (для B)
      sprintf(buf, "--:--:--");
  }
  else {
     sprintf(buf, "%lu:%02lu:%02lu", ms / 60000, (ms / 1000) % 60, (ms % 1000) / 10);
  }
  u8g2.setCursor(0, y_pos);
  u8g2.print(prefix); u8g2.print(buf);
}

// Обработка команд Bluetooth
void handleBluetoothCommands() {
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') { // Добавил \r как возможный терминатор
      if (btCommand.length() > 0) { // Обрабатываем только непустые команды
        btCommand.trim();
        Serial.print("BT CMD: "); Serial.println(btCommand); // Отладка
        if (btCommand == "SETMODE TRACK") {
          currentState = STATE_WAIT_LAP_START; // При смене режима, сбрасываем на ожидание старта
          lapStarted = false;
          prevFixNmeaTime_G_isValid = false; // Сброс, чтобы требовался новый валидный фикс
          SerialBT.println("ACK: MODE TRACK");
        } else if (btCommand == "SETMODE SPD") {
          currentState = STATE_SPEEDOMETER;
          SerialBT.println("ACK: MODE SPD");
        } else if (btCommand == "RESET PB") { // Изменил команду для ясности
          prefs.begin("lapdata", false);
          prefs.putULong("bestLap", 0);
          prefs.end();
          storedBestLap = 0;
          bestLapTime = 0; // Также сбрасываем лучшее время текущей сессии
          SerialBT.println("ACK: PB RESET");
        } else if (btCommand.startsWith("SET PB ")) { // Команда с параметром
          String pbValueStr = btCommand.substring(7);
          unsigned long newPB = (unsigned long)pbValueStr.toInt(); // Преобразование строки в unsigned long
          if (pbValueStr.length() > 0 && newPB > 0) { // Проверка, что значение было передано и оно положительное
             storedBestLap = newPB;
             prefs.begin("lapdata", true);
             prefs.putULong("bestLap", storedBestLap);
             prefs.end();
             SerialBT.print("ACK: PB SET TO "); SerialBT.println(storedBestLap);
          } else {
             SerialBT.println("ERR: INVALID PB VALUE");
          }
        } else {
          SerialBT.print("ERR: UNKNOWN CMD "); SerialBT.println(btCommand);
        }
        btCommand = ""; // Очищаем буфер команды
      }
    } else if (btCommand.length() < 50) { // Ограничение длины команды
        btCommand += c;
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200); // Для отладки
  Serial.println("GPS Lap Timer NMEA Time Booting...");

  #ifdef USE_GPS_HARDWARE
    GPS_STREAM.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // Начальная скорость GPS
    Serial.println("Configuring GPS module...");
    sendUBXCommand(GPS_STREAM, setBaud115200, sizeof(setBaud115200));
    GPS_STREAM.flush(); // Ждем отправки команды
    GPS_STREAM.updateBaudRate(115200); // Переключаем скорость UART ESP32
    delay(200); // Даем GPS модулю время на переключение
    sendUBXCommand(GPS_STREAM, setRate10Hz, sizeof(setRate10Hz), 200); // Увеличил задержку
    Serial.println("GPS module configured for 115200 baud and 10Hz.");
  #else
    Serial.println("Using Serial for GPS stream (simulation mode).");
  #endif

  SerialBT.begin("ESP32_GPS"); // Имя Bluetooth устройства
  Serial.println("Bluetooth started.");

  u8g2.begin();
  u8g2.enableUTF8Print(); // Для поддержки UTF-8 символов, если нужны
  u8g2.setFont(u8g2_font_nerhoe_tn); // Выберите подходящий шрифт. u8g2_font_logisoso20_tf довольно большой.
  Serial.println("Display initialized.");

  // prefs.begin("lapdata", false); // Открываем Preferences в режиме чтения-записи
  // storedBestLap = prefs.getULong("bestLap", 0);
  // prefs.end();
  Serial.print("Stored best lap: "); Serial.println(storedBestLap);

  // Предварительный расчет координат линий в метрах
  for (int i = 0; i < numTraps; ++i) {
    MathUtils::latLonToMeters(trapLines[i].lat1, trapLines[i].lon1, trapLines[i].x1, trapLines[i].y1);
    MathUtils::latLonToMeters(trapLines[i].lat2, trapLines[i].lon2, trapLines[i].x2, trapLines[i].y2);
  }
  Serial.println("Setup complete. Waiting for GPS fix...");
}

// --- Loop ---
void loop() {
  unsigned long now_millis = millis(); // millis() для общей логики, таймаутов, дебаунсинга
  handleBluetoothCommands();
  
  bool sentenceProcessedThisLoop = false;
  while (GPS_STREAM.available() > 0) {
    char c = GPS_STREAM.read();
    if (SerialBT.hasClient()) { // Отправляем, только если есть подключение
      SerialBT.write(c);
    }
    if (gps.encode(c)) {
      sentenceProcessedThisLoop = true; // Хотя бы одно NMEA предложение было успешно обработано
    }
  }

  // Обновляем время последнего валидного фикса, если местоположение обновилось
  if (sentenceProcessedThisLoop && gps.location.isUpdated()) {
      lastValidFixTime = now_millis;
  }
  
  FullNmeaTime currentNmeaDataPoint_L; // Локальная переменная для текущего полного времени NMEA
  bool currentNmeaTimeIsValidThisLoop = false;
  // Проверяем валидность и получаем NMEA время, только если были обработаны предложения
  // И TinyGPSPlus считает все компоненты валидными
  if (sentenceProcessedThisLoop) { // Убрал gps.location.isValid() отсюда, т.к. оно ниже проверяется
      if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
          currentNmeaDataPoint_L.fromGps(gps);
          currentNmeaTimeIsValidThisLoop = currentNmeaDataPoint_L.isValid;
      }
  }

  // Основная проверка состояния GPS (валидность координат + таймаут по millis)
  bool basicGpsHealthOK = gps.location.isValid() && (now_millis - lastValidFixTime <= MathUtils::gpsTimeoutMs);

  if (!basicGpsHealthOK) {
      currentState = STATE_WAIT_GPS;
      prevFixNmeaTime_G_isValid = false; // Важно: если GPS потерян, предыдущие данные нерелевантны для нового сегмента
      lapStarted = false; // Если потеряли GPS, круг прерван
  }

  // Получаем текущие координаты, если GPS здоров (для использования в switch)
  double currentLat = 0.0, currentLon = 0.0;
  int currentSats = 0;
  if (basicGpsHealthOK) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
      currentSats = gps.satellites.value();
  }

  u8g2.clearBuffer(); // Очищаем буфер дисплея один раз в начале отрисовки

  switch (currentState) {
    case STATE_WAIT_GPS:
      u8g2.setFont(u8g2_font_helvR10_tr); // Шрифт поменьше для этого экрана
      u8g2.setCursor(0, 12); u8g2.print("WAITING FOR GPS");
      u8g2.setCursor(0, 28); u8g2.print("Sats: "); u8g2.print(currentSats > 0 ? currentSats : gps.satellites.value());
      u8g2.setCursor(0, 44); u8g2.print("BT: "); SerialBT.hasClient() ? u8g2.print("Connected") : u8g2.print("Waiting...");

      if (basicGpsHealthOK && currentSats >= 4 && currentNmeaTimeIsValidThisLoop) {
          currentState = STATE_WAIT_LAP_START;
          // prevFixNmeaTime_G_isValid здесь еще false, оно установится в конце loop()
          // если currentNmeaTimeIsValidThisLoop было true. Это нормально для первого перехода.
      }
      break;

    case STATE_SPEEDOMETER:
      u8g2.setFont(u8g2_font_logisoso20_tf); // Большой шрифт для скорости
      u8g2.setCursor(0, 28);
      if (basicGpsHealthOK) {
        u8g2.print(gps.speed.kmph(), 1); u8g2.print(" km/h");
      } else {
        u8g2.print("--.- km/h");
      }
      u8g2.setFont(u8g2_font_helvR08_tr); // Маленький шрифт для спутников
      u8g2.setCursor(0, 50); u8g2.print("SAT: "); u8g2.print(currentSats);
      break;

    case STATE_WAIT_LAP_START: {
      u8g2.setFont(u8g2_font_helvR10_tr); // Шрифт для этого экрана
      u8g2.setCursor(0, 12); u8g2.println("READY TO START LAP");
      displayTimeRow("PB: ", storedBestLap, 30);
      u8g2.setCursor(0, 48); u8g2.print("SAT: "); u8g2.print(currentSats);

      double tRatio;
      if (basicGpsHealthOK && currentNmeaTimeIsValidThisLoop && prevFixNmeaTime_G_isValid) {
        if (MathUtils::crossedLineInterpolated(prevLat, prevLon, currentLat, currentLon, trapLines[0], tRatio)) {
          long nmea_fix_interval = MathUtils::calculate_nmea_fix_interval_ms(currentNmeaDataPoint_L, prevFixNmeaTime_G);
          
          if (nmea_fix_interval >= 10 && nmea_fix_interval <= 1000) { // Допустимый интервал (10мс до 1с)
            lapStartNmeaTime_G = MathUtils::add_ms_to_nmea_time(prevFixNmeaTime_G, (long)(tRatio * nmea_fix_interval));
            // Важно: убедиться, что дата в lapStartNmeaTime_G корректна.
            // add_ms_to_nmea_time в этой реализации не меняет дату, так что она будет от prevFixNmeaTime_G.
            // Если tRatio * nmea_fix_interval переносит время через полночь, дата должна обновиться.
            // Для простоты, предполагаем, что это не происходит или обрабатывается внешне.
            lapStartNmeaTime_G.isValid = true; 
            
            currentState = STATE_TRACKING;
            lapStarted = true;
            // Serial.println("--- LAP STARTED (NMEA Time) ---");
          } else {
            // Serial.print("Unusual NMEA fix interval: "); Serial.println(nmea_fix_interval);
          }
        }
      }
      break;
    }

    case STATE_TRACKING: {
      u8g2.setFont(u8g2_font_logisoso22_tf); // Большой шрифт для времени круга
      
      if (lapStartNmeaTime_G.isValid) {
        if (basicGpsHealthOK && currentNmeaTimeIsValidThisLoop) {
            displayTimeRow("L ", MathUtils::calculate_lap_duration_ms(currentNmeaDataPoint_L, lapStartNmeaTime_G), 30);
        } else if (basicGpsHealthOK && prevFixNmeaTime_G_isValid) {
            // Показываем время на основе последнего валидного NMEA фикса (будет "заморожено")
            displayTimeRow("L ", MathUtils::calculate_lap_duration_ms(prevFixNmeaTime_G, lapStartNmeaTime_G), 30);
        } else {
            u8g2.setCursor(0,30); u8g2.print("L --:--:---");
        }
      } else {
         u8g2.setCursor(0,30); u8g2.print("L START ERR");
      }
      displayTimeRow("B ", bestLapTime > 0 ? bestLapTime : storedBestLap, 60); // Показываем лучшее сессии или сохраненное

      double tRatio;
      if (basicGpsHealthOK && currentNmeaTimeIsValidThisLoop && prevFixNmeaTime_G_isValid && lapStartNmeaTime_G.isValid) {
        if (MathUtils::crossedLineInterpolated(prevLat, prevLon, currentLat, currentLon, trapLines[0], tRatio)) {
            long nmea_fix_interval = MathUtils::calculate_nmea_fix_interval_ms(currentNmeaDataPoint_L, prevFixNmeaTime_G);

            if (nmea_fix_interval >= 10 && nmea_fix_interval <= 1000) {
                FullNmeaTime crossingNmeaTime = MathUtils::add_ms_to_nmea_time(prevFixNmeaTime_G, (long)(tRatio * nmea_fix_interval));
                // Важно: Убедитесь, что дата в crossingNmeaTime корректна.
                
                unsigned long current_lap_duration = MathUtils::calculate_lap_duration_ms(crossingNmeaTime, lapStartNmeaTime_G);
                
                if (current_lap_duration >= MathUtils::minLapTimeMs) { // Проверка на минимально допустимое время круга
                    lastLapTime = current_lap_duration;

                    char lapTimeBuf[32];
                    sprintf(lapTimeBuf, "Lap: %lu:%02lu:%03lu",
                            lastLapTime / 60000,
                            (lastLapTime / 1000) % 60,
                            lastLapTime % 1000);
                    Serial.println(lapTimeBuf); 
                    if (bestLapTime == 0 || lastLapTime < bestLapTime) {
                        bestLapTime = lastLapTime;
                        if (storedBestLap == 0 || bestLapTime < storedBestLap) {
                            prefs.begin("lapdata", false);
                            prefs.putULong("bestLap", bestLapTime);
                            prefs.end();
                            storedBestLap = bestLapTime;
                            // Serial.print("New PB: "); Serial.println(storedBestLap);
                        }
                    }
                } else {
                    lastLapTime = 0; // Слишком короткий круг, не засчитываем
                    // Serial.println("Lap too short, ignored.");
                }
                lapStartNmeaTime_G = crossingNmeaTime; // Новый круг начался с этого момента
            } else {
                 // Serial.print("Unusual NMEA fix interval on lap end: "); Serial.println(nmea_fix_interval);
            }
        }
      }
      break;
    }
  }

  u8g2.sendBuffer(); // Отправляем буфер на дисплей один раз в конце цикла

  // Обновляем предыдущие значения для следующей итерации,
  // только если GPS здоров и мы получили валидное NMEA время в этой итерации.
  if (basicGpsHealthOK && currentNmeaTimeIsValidThisLoop) {
    prevLat = currentLat; 
    prevLon = currentLon;
    prevFixNmeaTime_G = currentNmeaDataPoint_L; 
    prevFixNmeaTime_G_isValid = true;
  } else if (!basicGpsHealthOK) {
    // Если GPS не здоров, инвалидируем предыдущие NMEA данные,
    // так как последовательность фиксов нарушена.
    prevFixNmeaTime_G_isValid = false;
  }
  // Если basicGpsHealthOK, но currentNmeaTimeIsValidThisLoop == false:
  // prevLat, prevLon, prevFixNmeaTime_G и prevFixNmeaTime_G_isValid НЕ МЕНЯЮТСЯ.
  // Они остаются от последнего валидного полного фикса.
}
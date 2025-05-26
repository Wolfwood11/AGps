#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <math.h> // Для fabs, fmin, fmax, sqrt, atan2, cos, M_PI

// Раскомментируйте, если используете аппаратный GPS UART
// #define USE_GPS_HARDWARE 1 

struct TrapLine {
  const char* name;
  double lat1, lon1;
  double lat2, lon2;
  double x1, y1, x2, y2; // Координаты в метрах
  double bearing;       // Ожидаемое направление пересечения
};

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

// Референсные координаты для проекции Меркатора (центр вашей трассы)
const double lat0 = 52.4449; // Пример: широта
const double lon0 = 20.6400; // Пример: долгота

// Предыдущие значения для интерполяции
double prevLat = 0.0;
double prevLon = 0.0;
unsigned long lastValidFixTime = 0; // Время последнего валидного фикса по millis() для таймаута

// Структура для хранения полного времени NMEA
struct FullNmeaTime {
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    uint16_t hundredths; // 0-99 (1/100 секунды от TinyGPSPlus)
    bool isValid;

    FullNmeaTime() : year(0), month(0), day(0), hour(0), minute(0), second(0), hundredths(0), isValid(false) {}

    void fromGps(TinyGPSPlus& gps_source) {
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
            year = 0; month = 0; day = 0; hour = 0; minute = 0; second = 0; hundredths = 0;
        }
    }

    long getMsOfDay() const {
        if (!isValid) return 0;
        return hour * 3600000L + minute * 60000L + second * 1000L + hundredths * 10L;
    }
};

// Глобальные переменные для времени NMEA
FullNmeaTime prevFixNmeaTime_G;       // Время NMEA предыдущего валидного фикса
bool prevFixNmeaTime_G_isValid = false;
FullNmeaTime lapStartNmeaTime_G;      // Время NMEA старта текущего круга

// Переменные для времени круга
unsigned long lastLapTime = 0;    // Длительность последнего круга в мс
unsigned long bestLapTime = 0;    // Длительность лучшего круга в текущей сессии в мс
unsigned long storedBestLap = 0;  // Лучший круг, сохраненный в Preferences
bool lapStarted = false;          // Флаг, что круг активно отслеживается
unsigned long lastCrossMillis = 0;// Для дебаунсинга пересечения линии (на основе millis)

// Константы для логики таймера
const float trackLengthMeters = 1200.0; // Примерная длина трассы
const float maxSpeedKmh = 300.0;        // Максимальная ожидаемая скорость
const unsigned long minLapTimeMs = (unsigned long)((trackLengthMeters / (maxSpeedKmh / 3.6)) * 1000.0); // Минимальное разумное время круга
const unsigned long gpsTimeoutMs = 5000; // Таймаут отсутствия GPS фикса в мс

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


// --- Функции для работы с координатами и временем ---

void latLonToMeters(double lat, double lon, double &x, double &y) {
  const double R = 6378137.0; // Радиус Земли в метрах
  x = R * (lon - lon0) * M_PI / 180.0 * cos(lat0 * M_PI / 180.0);
  y = R * (lat - lat0) * M_PI / 180.0;
}

double calculateBearing(double x1, double y1, double x2, double y2) {
  double bearing = atan2(x2 - x1, y2 - y1) * 180.0 / M_PI; // В atan2 сначала Y потом X для навигационного азимута
                                                          // Здесь X2-X1, Y2-Y1 (dx, dy). Если нужно от севера по часовой: atan2(dx, dy)
                                                          // atan2(y2-y1, x2-x1) -> угол от оси X против часовой
                                                          // Для навигационного (от севера по часовой): atan2(x2-x1, y2-y1)
  bearing = 90.0 - bearing; // Конвертация в навигационный азимут (0 - север, 90 - восток)
  if (bearing < 0) bearing += 360.0;
  return bearing;
}


// Расчет разницы в мс между двумя метками времени NMEA (для интервалов между фиксами GPS)
// Упрощенная версия, корректно обрабатывает один переход через полночь.
long calculate_nmea_fix_interval_ms(const FullNmeaTime& t_current, const FullNmeaTime& t_previous) {
    if (!t_current.isValid || !t_previous.isValid) return 100; // Стандартный интервал для 10Гц

    long prev_ms_of_day = t_previous.getMsOfDay();
    long current_ms_of_day = t_current.getMsOfDay();
    
    // Проверка на тот же день или переход через одну полночь
    if (t_current.year == t_previous.year && t_current.month == t_previous.month) {
        if (t_current.day == t_previous.day) {
            return current_ms_of_day - prev_ms_of_day;
        } else if (t_current.day == t_previous.day + 1) { // Переход на следующий день
            return (24 * 3600000L - prev_ms_of_day) + current_ms_of_day;
        }
    }
    // Если даты сильно расходятся или предыдущий день "позже" текущего (ошибка данных),
    // возвращаем стандартный интервал или разницу в тот же день (может быть отрицательной).
    // Для последовательных фиксов это маловероятно.
    long diff = current_ms_of_day - prev_ms_of_day;
    if (abs(diff) > 12 * 3600000L) return 100; // Если разница > 12 часов, вероятно ошибка, вернуть 100мс
    return diff; // Может быть отрицательным, если время "прыгнуло"
}

// Расчет длительности круга в мс.
// ВНИМАНИЕ: Эта функция УПРОЩЕНА. Для полной корректности при пересечении сложных границ дат
// (несколько дней, конец месяца/года) нужна полноценная арифметика дат или библиотека.
// Эта версия корректна для кругов в пределах одного дня или с одним пересечением полуночи.
unsigned long calculate_lap_duration_ms(const FullNmeaTime& t_end, const FullNmeaTime& t_start) {
    if (!t_end.isValid || !t_start.isValid) return 0;

    // Простой случай: тот же день, месяц и год
    if (t_end.year == t_start.year && t_end.month == t_start.month && t_end.day == t_start.day) {
        long diff = t_end.getMsOfDay() - t_start.getMsOfDay();
        return (diff >= 0) ? (unsigned long)diff : 0;
    }

    // Случай: пересечение одной полуночи (t_end на следующий день)
    // Это очень упрощенная проверка! Не учитывает конец месяца/года.
    if (t_end.year == t_start.year && t_end.month == t_start.month && t_end.day == t_start.day + 1) {
        long diff = (24 * 3600000L - t_start.getMsOfDay()) + t_end.getMsOfDay();
        return (unsigned long)diff;
    }
    
    // Если даты более сложные, эта функция вернет 0 или неверный результат.
    // Требуется более надежный метод (например, конвертация в секунды с эпохи).
    // Serial.println("Warning: Complex date difference in calculate_lap_duration_ms not fully supported.");
    return 0; 
}

// Добавление мс к времени NMEA (для интерполяции).
// ВНИМАНИЕ: Упрощенная версия. Корректно для небольших ms_to_add (не меняющих дату).
// Не обрабатывает изменение дня, месяца, года.
FullNmeaTime add_ms_to_nmea_time(FullNmeaTime base_time, long ms_to_add) {
    if (!base_time.isValid) return base_time;

    long current_total_ms_in_day = base_time.getMsOfDay();
    current_total_ms_in_day += ms_to_add;

    FullNmeaTime result_time = base_time; // Копируем исходную дату (Y,M,D)

    // Обработка перехода мс в пределах суток (упрощенно, без изменения даты)
    if (current_total_ms_in_day >= 24 * 3600000L) {
        current_total_ms_in_day -= 24 * 3600000L;
        // ДАТА НЕ ОБНОВЛЯЕТСЯ! Для интерполяции (<100мс) это обычно не меняет дату.
    } else if (current_total_ms_in_day < 0) {
        current_total_ms_in_day += 24 * 3600000L;
        // ДАТА НЕ ОБНОВЛЯЕТСЯ!
    }

    result_time.hour = current_total_ms_in_day / 3600000L;
    current_total_ms_in_day %= 3600000L;
    result_time.minute = current_total_ms_in_day / 60000L;
    current_total_ms_in_day %= 60000L;
    result_time.second = current_total_ms_in_day / 1000L;
    current_total_ms_in_day %= 1000L;
    result_time.hundredths = current_total_ms_in_day / 10L;
    
    return result_time;
}

// Проверка пересечения отрезков (геометрия)
bool segmentsIntersect(double p0_x, double p0_y, double p1_x, double p1_y,
                       double p2_x, double p2_y, double p3_x, double p3_y,
                       double& i_x, double& i_y, double& tRatio) {
    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x; s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x; s2_y = p3_y - p2_y;

    double s, t;
    double det = -s2_x * s1_y + s1_x * s2_y;

    // Если отрезки параллельны или коллинеарны (det близок к 0)
    if (fabs(det) < 1e-9) return false; 

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / det;
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / det;

    if (s >= -1e-9 && s <= 1.0 + 1e-9 && t >= -1e-9 && t <= 1.0 + 1e-9) { // с допуском
        i_x = p0_x + (t * s1_x);
        i_y = p0_y + (t * s1_y);
        tRatio = t; // t - это отношение для первого отрезка (траектории GPS)
        tRatio = fmax(0.0, fmin(1.0, tRatio)); // Ограничиваем tRatio [0,1]
        return true;
    }
    return false; // Нет пересечения в пределах отрезков
}

// Проверка пересечения линии с интерполяцией
bool crossedLineInterpolated(double lat_prev, double lon_prev, double lat_curr, double lon_curr, 
                             TrapLine& trap, double& tRatioCalculated) {
  double x1_m, y1_m, x2_m, y2_m;
  latLonToMeters(lat_prev, lon_prev, x1_m, y1_m);
  latLonToMeters(lat_curr, lon_curr, x2_m, y2_m);
  
  double ix, iy; // Координаты точки пересечения (не используются дальше, но нужны функции)
  
  if (!segmentsIntersect(x1_m, y1_m, x2_m, y2_m, trap.x1, trap.y1, trap.x2, trap.y2, ix, iy, tRatioCalculated)) {
    return false;
  }

  // Проверка направления пересечения
  double gps_segment_bearing = calculateBearing(x1_m, y1_m, x2_m, y2_m);
  double bearing_diff = fabs(trap.bearing - gps_segment_bearing);
  if (bearing_diff > 180.0) bearing_diff = 360.0 - bearing_diff;
  
  if (bearing_diff > 90.0) return false; // Пересечение не в ту сторону

  // Дебаунсинг на основе millis() - предотвращает многократные срабатывания на одной линии
  unsigned long now_millis_debounce = millis();
  if (now_millis_debounce - lastCrossMillis < (minLapTimeMs / 4)) { // Уменьшил время дебаунсинга
      // Serial.println("Debounced cross");
      return false;
  }
  lastCrossMillis = now_millis_debounce;
  return true;
}

// Отображение времени на дисплее
void displayTimeRow(const char* prefix, unsigned long ms, int y_pos) {
  char buf[20]; // Увеличил буфер на всякий случай
  if (ms == 0 && bestLapTime != 0 && strcmp(prefix,"L ") == 0 ) { // Если текущий круг 0, но есть лучший (для L)
      sprintf(buf, "--:--:--");
  } else if (ms == 0 && strcmp(prefix,"B ") == 0 && storedBestLap == 0){ // Если лучший круг 0 (для B)
      sprintf(buf, "--:--:--");
  }
  else {
     sprintf(buf, "%lu:%02lu:%03lu", ms / 60000, (ms / 1000) % 60, ms % 1000); // Теперь с миллисекундами
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
    latLonToMeters(trapLines[i].lat1, trapLines[i].lon1, trapLines[i].x1, trapLines[i].y1);
    latLonToMeters(trapLines[i].lat2, trapLines[i].lon2, trapLines[i].x2, trapLines[i].y2);
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
  bool basicGpsHealthOK = gps.location.isValid() && (now_millis - lastValidFixTime <= gpsTimeoutMs);

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
        if (crossedLineInterpolated(prevLat, prevLon, currentLat, currentLon, trapLines[0], tRatio)) {
          long nmea_fix_interval = calculate_nmea_fix_interval_ms(currentNmeaDataPoint_L, prevFixNmeaTime_G);
          
          if (nmea_fix_interval >= 10 && nmea_fix_interval <= 1000) { // Допустимый интервал (10мс до 1с)
            lapStartNmeaTime_G = add_ms_to_nmea_time(prevFixNmeaTime_G, (long)(tRatio * nmea_fix_interval));
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
      u8g2.setFont(u8g2_font_logisoso20_tf); // Большой шрифт для времени круга
      
      if (lapStartNmeaTime_G.isValid) {
        if (basicGpsHealthOK && currentNmeaTimeIsValidThisLoop) {
            displayTimeRow("L ", calculate_lap_duration_ms(currentNmeaDataPoint_L, lapStartNmeaTime_G), 28);
        } else if (basicGpsHealthOK && prevFixNmeaTime_G_isValid) {
            // Показываем время на основе последнего валидного NMEA фикса (будет "заморожено")
            displayTimeRow("L ", calculate_lap_duration_ms(prevFixNmeaTime_G, lapStartNmeaTime_G), 28);
        } else {
            u8g2.setCursor(0,28); u8g2.print("L --:--:---");
        }
      } else {
         u8g2.setCursor(0,28); u8g2.print("L START ERR");
      }
      displayTimeRow("B ", bestLapTime > 0 ? bestLapTime : storedBestLap, 52); // Показываем лучшее сессии или сохраненное

      double tRatio;
      if (basicGpsHealthOK && currentNmeaTimeIsValidThisLoop && prevFixNmeaTime_G_isValid && lapStartNmeaTime_G.isValid) {
        if (crossedLineInterpolated(prevLat, prevLon, currentLat, currentLon, trapLines[0], tRatio)) {
            long nmea_fix_interval = calculate_nmea_fix_interval_ms(currentNmeaDataPoint_L, prevFixNmeaTime_G);

            if (nmea_fix_interval >= 10 && nmea_fix_interval <= 1000) {
                FullNmeaTime crossingNmeaTime = add_ms_to_nmea_time(prevFixNmeaTime_G, (long)(tRatio * nmea_fix_interval));
                // Важно: Убедитесь, что дата в crossingNmeaTime корректна.
                
                unsigned long current_lap_duration = calculate_lap_duration_ms(crossingNmeaTime, lapStartNmeaTime_G);
                
                if (current_lap_duration >= minLapTimeMs) { // Проверка на минимально допустимое время круга
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
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <math.h>

//#define USE_GPS_HARDWARE 1
struct TrapLine {
  const char* name;
  double lat1, lon1;
  double lat2, lon2;
  double x1, y1, x2, y2;
  double bearing;
};

#if USE_GPS_HARDWARE
  #define GPS_RX 25
  #define GPS_TX 26
  HardwareSerial gpsSerial(2);
  #define GPS_STREAM gpsSerial
#else
  #define GPS_STREAM Serial
#endif

TinyGPSPlus gps;
BluetoothSerial SerialBT;
Preferences prefs;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

String currentLine = "";
String lastLine = "";
String btCommand = "";

const double lat0 = 52.4449;
const double lon0 = 20.6400;

double prevLat = 0, prevLon = 0;
unsigned long prevFixTime = 0;
unsigned long lastValidFixTime = 0;

void latLonToMeters(double lat, double lon, double &x, double &y) {
  const double R = 6378137.0;
  x = R * (lon - lon0) * M_PI / 180.0 * cos(lat0 * M_PI / 180.0);
  y = R * (lat - lat0) * M_PI / 180.0;
}

double calculateBearing(double x1, double y1, double x2, double y2) {
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

TrapLine trapLines[] = {
  {"Start/Finish", 52.444978239573054, 20.639983209455288, 52.44479509375817, 20.640014790479068, 0, 0, 0, 0, 264.0}
};
const int numTraps = sizeof(trapLines) / sizeof(TrapLine);

uint8_t setBaud115200[] = {
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
  0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
  0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xC4, 0x96,
  0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22
};

uint8_t setRate10Hz[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
  0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12,
  0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30
};

void sendUBXCommand(HardwareSerial &serial, const uint8_t *cmd, size_t len, uint16_t postDelayMs = 100) {
  for (size_t i = 0; i < len; i++) serial.write(cmd[i]);
  serial.flush(); delay(postDelayMs);
}

unsigned long lastLapTime = 0;
unsigned long bestLapTime = 0;
unsigned long lapStartMillis = 0;
bool lapStarted = false;
unsigned long storedBestLap = 0;
unsigned long lastCrossMillis = 0;

const float trackLengthMeters = 1200.0;
const float maxSpeedKmh = 300.0;
const unsigned long minLapTimeMs = (unsigned long)(trackLengthMeters / (maxSpeedKmh * 1000.0 / 3600.0) * 1000.0);
const unsigned long gpsTimeoutMs = 5000;

enum TrackerState {
  STATE_WAIT_GPS,
  STATE_WAIT_LAP_START,
  STATE_TRACKING,
  STATE_SPEEDOMETER
};

TrackerState currentState = STATE_WAIT_GPS;

bool segmentsIntersect(double x1, double y1, double x2, double y2,
                       double x3, double y3, double x4, double y4,
                       double& ix, double& iy, double& tRatio) {
  double d = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
  if (fabs(d) < 1e-10) return false;
  double px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / d;
  double py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / d;
  if ((px < fmin(x1,x2) || px > fmax(x1,x2)) ||
      (py < fmin(y1,y2) || py > fmax(y1,y2)) ||
      (px < fmin(x3,x4) || px > fmax(x3,x4)) ||
      (py < fmin(y3,y4) || py > fmax(y3,y4))) return false;
  double totalDist = hypot(x2 - x1, y2 - y1);
  double partDist = hypot(px - x1, py - y1);
  tRatio = (totalDist > 0) ? partDist / totalDist : 0;
  ix = px; iy = py;
  return true;
}

bool crossedLineInterpolated(double lat1, double lon1, double lat2, double lon2, TrapLine& trap, double& tRatio) {
  double x1, y1, x2, y2;
  latLonToMeters(lat1, lon1, x1, y1);
  latLonToMeters(lat2, lon2, x2, y2);
  double ix, iy;
  if (!segmentsIntersect(x1, y1, x2, y2, trap.x1, trap.y1, trap.x2, trap.y2, ix, iy, tRatio)) return false;
  double heading = calculateBearing(x1, y1, x2, y2);
  double diff = fabs(trap.bearing - heading);
  if (diff > 180.0) diff = 360.0 - diff;
  if (diff > 90.0) return false;
  unsigned long now = millis();
  if (now - lastCrossMillis < minLapTimeMs) return false;
  lastCrossMillis = now;
  return true;
}

void displayTimeRow(const char* prefix, unsigned long ms, int y) {
  char buf[16];
  sprintf(buf, "%lu:%02lu:%02lu", ms / 60000, (ms / 1000) % 60, (ms % 1000) / 10);
  u8g2.setCursor(0, y);
  u8g2.print(prefix); u8g2.println(buf);
}

void handleBluetoothCommands() {
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n') {
      btCommand.trim();
      if (btCommand == "SETMODE TRACK") {
        currentState = STATE_WAIT_LAP_START;
        SerialBT.println("MODE: TRACK");
      } else if (btCommand == "SETMODE SPD") {
        currentState = STATE_SPEEDOMETER;
        SerialBT.println("MODE: SPEEDOMETER");
      } else if (btCommand == "RESET") {
        prefs.begin("lapdata", false);
        prefs.putULong("bestLap", 0);
        prefs.end();
        storedBestLap = 0;
        SerialBT.println("PB RESET");
      } else if (btCommand.startsWith("SET ")) {
        unsigned long newPB = btCommand.substring(4).toInt();
        storedBestLap = newPB;
        prefs.begin("lapdata", false);
        prefs.putULong("bestLap", storedBestLap);
        prefs.end();
        SerialBT.println("PB SET");
      }
      btCommand = "";
    } else btCommand += c;
  }
}

void setup() {
  Serial.begin(115200);
  #if USE_GPS_HARDWARE
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    sendUBXCommand(gpsSerial, setBaud115200, sizeof(setBaud115200));
    gpsSerial.updateBaudRate(115200);
    sendUBXCommand(gpsSerial, setRate10Hz, sizeof(setRate10Hz));
  #endif
  SerialBT.begin("ESP32_GPS");
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_logisoso20_tf);
  prefs.begin("lapdata", false);
  storedBestLap = prefs.getULong("bestLap", 0);
  prefs.end();
  for (int i = 0; i < numTraps; ++i) {
    latLonToMeters(trapLines[i].lat1, trapLines[i].lon1, trapLines[i].x1, trapLines[i].y1);
    latLonToMeters(trapLines[i].lat2, trapLines[i].lon2, trapLines[i].x2, trapLines[i].y2);
  }
}

void loop() {
  unsigned long now = millis();
  handleBluetoothCommands();
  bool gotFix = false;
  while (GPS_STREAM.available()) {
    char c = GPS_STREAM.read();
    //Serial.write(c); 
    SerialBT.write(c);
    gps.encode(c);
    if (c == '\n') { lastLine = currentLine; currentLine = ""; }
    else if (c != '\r') currentLine += c;
    if (gps.location.isUpdated())
    {
      gotFix = true;
     lastValidFixTime = now;
    } 
  }
  u8g2.clearBuffer();

  if (!gps.location.isValid() || (now - lastValidFixTime > gpsTimeoutMs)) {
    currentState = STATE_WAIT_GPS;
    u8g2.setCursor(0, 24);
    u8g2.println("WAIT GPS...");
    u8g2.sendBuffer();
    return;
  }
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  int sats = gps.satellites.value();
  if (currentState == STATE_SPEEDOMETER) {
    u8g2.setCursor(0, 24);
    u8g2.print("SPD "); u8g2.print(gps.speed.kmph(), 1); u8g2.println("km/h");
    u8g2.sendBuffer(); return;
  }
  switch (currentState) {
    case STATE_WAIT_GPS:
      if (sats >= 4) currentState = STATE_WAIT_LAP_START;
      break;
    case STATE_WAIT_LAP_START: {
      lapStartMillis = now;
      u8g2.setCursor(0, 24);
      u8g2.println("READY LAP");
      displayTimeRow("PB ", storedBestLap, 48);
      u8g2.sendBuffer();
      double tRatio;
      if (gotFix && crossedLineInterpolated(prevLat, prevLon, lat, lon, trapLines[0], tRatio)) {
        unsigned long delta = now - prevFixTime;
        lapStartMillis = now - (unsigned long)(tRatio * delta);
        currentState = STATE_TRACKING;
        lapStarted = true;
      }
      break;
    }
    case STATE_TRACKING: {
      double tRatio;
      if (gotFix && crossedLineInterpolated(prevLat, prevLon, lat, lon, trapLines[0], tRatio)) {
        if (!lapStarted) {
          unsigned long delta = now - prevFixTime;
          lastLapTime = (now - lapStartMillis) - (unsigned long)(tRatio * delta);
          lapStartMillis = now - (unsigned long)(tRatio * delta);
          lapStarted = true;

          if (bestLapTime == 0 || lastLapTime < bestLapTime) {
            bestLapTime = lastLapTime;
            if (storedBestLap == 0 || bestLapTime < storedBestLap) {
              prefs.begin("lapdata", false);
              prefs.putULong("bestLap", bestLapTime);
              prefs.end();
              storedBestLap = bestLapTime;
            }
          }

          #if !USE_GPS_HARDWARE
            char buf[32];
            sprintf(buf, "L %lu:%02lu:%02lu\n", lastLapTime / 60000, (lastLapTime / 1000) % 60, (lastLapTime % 1000) / 10);
            GPS_STREAM.print(buf);
          #endif
        }
      } else lapStarted = false;
      displayTimeRow("L ", now - lapStartMillis, 24);
      displayTimeRow("B ", bestLapTime, 48);
      u8g2.sendBuffer();
      break;
    }
  }
  if (gotFix) {
    prevLat = lat;
    prevLon = lon;
    prevFixTime = now;
  }
}

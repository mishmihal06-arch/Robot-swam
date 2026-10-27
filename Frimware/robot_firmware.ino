// Простая прошивка робота (ESP32-S3).
//
// Команды по UDP (порт 8888):
//   M:a,b,c,t        a,b,c — ШИМ на моторы 1..3, целые от -255 до 255 (минус = назад)
//                    t — сколько миллисекунд выполнять (не больше MAX_T_MS);
//                    потом моторы сами выключаются мгновенно.
//                    Новая команда сразу заменяет старую.
//   STOP             остановить моторы (мгновенно)
//   PINS:p1,...,p9   пины моторов: IN1,IN2,EN мотора 1, затем мотора 2, затем мотора 3
//   GET              вернуть текущие пины
//
// Разгон и смена скорости идут плавно (RAMP_PER_SEC), остановка — мгновенная.
//
// Если 10 секунд нет команд, робот раз в секунду рассылает по сети маяк
//   ROBOT:<id>;IP:<ip>;PORT:8888
// чтобы ноутбук мог его найти. id — последние 6 символов MAC-адреса (у каждой платы свой).

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>

const char* WIFI_SSID = "ROBOT_WIFI";
const char* WIFI_PASS = "PASSWORD";

const int CMD_PORT    = 8888;
const int BEACON_PORT = 8889;
const unsigned long IDLE_MS         = 10000;  // столько молчания — и начинаем слать маяк
const unsigned long BEACON_EVERY_MS = 1000;

const long  MAX_T_MS     = 1000;    // больше этого время действия команды не берём
const float RAMP_PER_SEC = 1700.0f; // как быстро меняется ШИМ (единиц/с): 0 -> 255 примерно за 0.15 с

const int PWM_FREQ = 5000;
const int PWM_RES  = 8;  // 0..255

// Пины по умолчанию: {IN1, IN2, EN} для моторов 1, 2, 3
int pins[9] = {18, 8, 17,   15, 7, 16,   10, 11, 9};

WiFiUDP udp;
Preferences prefs;
String robotId;

unsigned long lastRx = 0;        // когда пришла последняя команда
unsigned long lastBeacon = 0;
unsigned long lastRamp = 0;
bool motorsRunning = false;
unsigned long motorsStopAt = 0;  // когда выключить моторы

float target[3]  = {0, 0, 0};    // что хотим (-255..255)
float current[3] = {0, 0, 0};    // что сейчас выдаём


// ---------------- Моторы ----------------

// n = 0..2, v = -255..255
void writeMotor(int n, int v) {
  v = constrain(v, -255, 255);
  digitalWrite(pins[n * 3],     v > 0);
  digitalWrite(pins[n * 3 + 1], v < 0);
  ledcWrite(pins[n * 3 + 2], abs(v));
}

// Мгновенная остановка
void stopMotors() {
  for (int n = 0; n < 3; n++) {
    target[n] = 0;
    current[n] = 0;
    writeMotor(n, 0);
  }
  motorsRunning = false;
}

// Плавно подтягиваем current к target
void updateRamp(unsigned long now) {
  float maxStep = RAMP_PER_SEC * (now - lastRamp) / 1000.0f;
  lastRamp = now;
  if (!motorsRunning) return;

  for (int n = 0; n < 3; n++) {
    float d = target[n] - current[n];
    if (d >  maxStep) d =  maxStep;
    if (d < -maxStep) d = -maxStep;
    current[n] += d;
    writeMotor(n, (int)roundf(current[n]));
  }
}


// ---------------- Пины ----------------

bool pinsOk(const int p[9]) {
  for (int i = 0; i < 9; i++) {
    int x = p[i];
    // запрещены: несуществующие, USB (19,20), флеш (22..32), UART0 (43,44), strapping (3,45,46)
    bool bad = x < 1 || x > 48 || x == 3 || x == 19 || x == 20 ||
               (x >= 22 && x <= 32) || x == 43 || x == 44 || x == 45 || x == 46;
    if (bad) return false;
    for (int j = i + 1; j < 9; j++)
      if (p[j] == x) return false;  // один пин дважды
  }
  return true;
}

void initPins() {
  for (int n = 0; n < 3; n++) {
    pinMode(pins[n * 3],     OUTPUT);
    pinMode(pins[n * 3 + 1], OUTPUT);
    ledcAttach(pins[n * 3 + 2], PWM_FREQ, PWM_RES);
  }
  stopMotors();
}

void releasePins() {
  stopMotors();
  for (int n = 0; n < 3; n++) ledcDetach(pins[n * 3 + 2]);
}

void setPins(const int p[9]) {
  releasePins();
  memcpy(pins, p, sizeof(pins));
  initPins();
  prefs.putBytes("pins", pins, sizeof(pins));  // запоминаем во флеш
}


// ---------------- Сеть ----------------

void reply(const String& text) {
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write((const uint8_t*)text.c_str(), text.length());
  udp.endPacket();
}

void sendBeacon() {
  String msg = "ROBOT:" + robotId + ";IP:" + WiFi.localIP().toString() + ";PORT:" + String(CMD_PORT);
  udp.beginPacket(WiFi.broadcastIP(), BEACON_PORT);
  udp.write((const uint8_t*)msg.c_str(), msg.length());
  udp.endPacket();
}

void handleCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("M:")) {
    int a, b, c;
    long t;
    if (sscanf(cmd.c_str() + 2, "%d,%d,%d,%ld", &a, &b, &c, &t) == 4) {
      if (t > MAX_T_MS) t = MAX_T_MS;
      if (t > 0) {
        target[0] = constrain(a, -255, 255);
        target[1] = constrain(b, -255, 255);
        target[2] = constrain(c, -255, 255);
        motorsRunning = true;
        motorsStopAt = millis() + t;
      } else {
        stopMotors();
      }
      reply("OK");
    } else {
      reply("ERR format");
    }

  } else if (cmd == "STOP") {
    stopMotors();
    reply("OK");

  } else if (cmd.startsWith("PINS:")) {
    int p[9];
    if (sscanf(cmd.c_str() + 5, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
               &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7], &p[8]) != 9) {
      reply("ERR need 9 pins");
    } else if (!pinsOk(p)) {
      reply("ERR bad or duplicate pin");
    } else {
      if (memcmp(p, pins, sizeof(pins)) != 0) setPins(p);
      reply("OK");
    }

  } else if (cmd == "GET") {
    String s = "PINS:";
    for (int i = 0; i < 9; i++) s += String(pins[i]) + (i < 8 ? "," : "");
    reply(s);

  } else {
    reply("ERR unknown");
  }
}


// ---------------- setup / loop ----------------

void setup() {
  Serial.begin(115200);
  delay(500);

  // Пины: из флеша, если там что-то корректное лежит
  prefs.begin("robot", false);
  int saved[9];
  if (prefs.getBytes("pins", saved, sizeof(saved)) == sizeof(saved) && pinsOk(saved)) {
    memcpy(pins, saved, sizeof(pins));
  }
  initPins();

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // без энергосбережения: команды приходят без задержек
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  robotId = WiFi.macAddress();
  robotId.replace(":", "");
  robotId = robotId.substring(6);  // последние 6 символов

  udp.begin(CMD_PORT);
  Serial.println("\nID робота: " + robotId);
  Serial.println("IP: " + WiFi.localIP().toString());

  lastRx = millis() - IDLE_MS - 1;  // сразу считаем, что тихо -> шлём маяк
  lastRamp = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. Приём команд
  if (udp.parsePacket()) {
    char buf[200];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      lastRx = now;
      handleCommand(String(buf));
    }
  }

  // 2. Время действия вышло -> выключаем моторы мгновенно
  if (motorsRunning && (long)(now - motorsStopAt) >= 0) {
    stopMotors();
  }

  // 3. Плавный разгон / смена скорости
  updateRamp(now);

  // 4. Давно не было команд -> шлём маяк
  if (now - lastRx > IDLE_MS && now - lastBeacon >= BEACON_EVERY_MS && WiFi.status() == WL_CONNECTED) {
    sendBeacon();
    lastBeacon = now;
  }

  delay(2);
}

#include <WiFi.h>
#include <WiFiUdp.h>

// ==================== 1. НАСТРОЙКИ WI-FI ====================
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
const int udpPort = 8888;

// ==================== 2. ПИНЫ МОТОРОВ ====================
#define M1_IN1 5
#define M1_IN2 6
#define M1_ENA 7

#define M2_IN1 15
#define M2_IN2 16
#define M2_ENB 17

#define M3_IN1 18
#define M3_IN2 8
#define M3_ENA 9

// ==================== 3. ПАРАМЕТРЫ ШИМ ====================
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;    // 0..255

// ==================== 4. БЕЗОПАСНОСТЬ ====================
const unsigned long watchdogTimeout = 300;  // мс без пакетов → стоп
unsigned long lastCommandTime = 0;

// Общий таймер для всех моторов
unsigned long commandEndTime = 0;   // 0 = бессрочно
bool commandActive = false;

// Флаги инверсии (если мотор физически крутится не в ту сторону)
const bool INV_M1 = false;
const bool INV_M2 = false;
const bool INV_M3 = false;

WiFiUDP udp;

// ==================== 5. УПРАВЛЕНИЕ МОТОРАМИ ====================
void setMotorRaw(int in1, int in2, int en, int speed, bool invert) {
  if (invert) speed = -speed;

  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(en, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(en, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(en, 0);
  }
}

// Применить скорости ко всем трём моторам СРАЗУ (одновременно)
void applyAll(int s1, int s2, int s3) {
  setMotorRaw(M1_IN1, M1_IN2, M1_ENA, s1, INV_M1);
  setMotorRaw(M2_IN1, M2_IN2, M2_ENB, s2, INV_M2);
  setMotorRaw(M3_IN1, M3_IN2, M3_ENA, s3, INV_M3);
}

void stopAll() {
  applyAll(0, 0, 0);
  commandEndTime = 0;
  commandActive = false;
}

// ==================== 6. ПАРСЕР ПАКЕТА ====================
// Формат: M1:<speed>;M2:<speed>;M3:<speed>;T:<duration>
// Все 4 поля обязательны. Если чего-то нет — пакет игнорируется, робот стоит.
bool parsePacket(String packet, int &s1, int &s2, int &s3, unsigned long &duration) {
  bool hasM1 = false, hasM2 = false, hasM3 = false, hasT = false;
  s1 = s2 = s3 = 0;
  duration = 0;

  int start = 0;
  while (start < packet.length()) {
    int sep = packet.indexOf(';', start);
    if (sep == -1) sep = packet.length();
    String part = packet.substring(start, sep);
    start = sep + 1;
    part.trim();
    if (part.length() == 0) continue;

    int colon = part.indexOf(':');
    if (colon <= 0) continue;

    String key = part.substring(0, colon);
    String val = part.substring(colon + 1);
    key.trim(); val.trim();

    if      (key == "M1") { s1 = val.toInt(); hasM1 = true; }
    else if (key == "M2") { s2 = val.toInt(); hasM2 = true; }
    else if (key == "M3") { s3 = val.toInt(); hasM3 = true; }
    else if (key == "T")  { duration = (unsigned long)val.toInt(); hasT = true; }
  }

  return hasM1 && hasM2 && hasM3 && hasT;
}

// ==================== 7. ОБРАБОТКА ПАКЕТА ====================
void processPacket(String packet) {
  packet.trim();

  // Аварийный стоп
  if (packet == "STOP") {
    Serial.println("Аварийный STOP");
    stopAll();
    lastCommandTime = millis();
    return;
  }

  int s1, s2, s3;
  unsigned long duration;

  if (!parsePacket(packet, s1, s2, s3, duration)) {
    Serial.println("Неполный пакет — STOP");
    stopAll();
    return;
  }

  // Сбрасываем Watchdog и запускаем новую команду — она ПРЕРЫВАЕТ старую
  lastCommandTime = millis();

  // Применяем всё одновременно
  applyAll(s1, s2, s3);

  if (duration > 0) {
    commandEndTime = millis() + duration;
    commandActive = true;
  } else {
    commandEndTime = 0;
    commandActive = false;
  }

  Serial.print("Новая команда: M1=");
  Serial.print(s1);
  Serial.print(" M2=");
  Serial.print(s2);
  Serial.print(" M3=");
  Serial.print(s3);
  Serial.print(" T=");
  Serial.println(duration);
}

// ==================== 8. SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Запуск робота...");

  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  ledcAttach(M1_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(M2_ENB, PWM_FREQ, PWM_RES);
  ledcAttach(M3_ENA, PWM_FREQ, PWM_RES);

  stopAll();

  Serial.print("Wi-Fi: "); Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi OK");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWi-Fi FAIL");
  }

  udp.begin(udpPort);
  Serial.print("UDP порт: "); Serial.println(udpPort);

  lastCommandTime = millis();
}

// ==================== 9. LOOP ====================
void loop() {
  unsigned long now = millis();

  // 1. Таймер команды: если duration истёк и новый пакет не пришёл → стоп
  if (commandActive && commandEndTime != 0 && now >= commandEndTime) {
    Serial.println("Время команды истекло → STOP");
    stopAll();
  }

  // 2. Watchdog: если пакетов нет > 300 мс → стоп
  if (now - lastCommandTime > watchdogTimeout) {
    if (commandActive) {   // останавливаем только если что-то активно
      stopAll();
    }
  }

  // 3. Приём UDP
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buf[255];
    int len = udp.read(buf, 255);
    if (len > 0) {
      buf[len] = '\0';
      String packet = String(buf);
      Serial.print("Принят: ");
      Serial.println(packet);
      processPacket(packet);

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write((const uint8_t*)"OK", 2);
      udp.endPacket();
    }
  }

  delay(5);
}

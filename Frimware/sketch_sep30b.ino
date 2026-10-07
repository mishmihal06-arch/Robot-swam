#include <WiFi.h>
#include <WiFiUdp.h>

// ==================== 1. НАСТРОЙКИ WI-FI ====================
const char* ssid = "имя сети";
const char* password = "парольсети";
const int udpPort = 8888;

// ==================== 2. ПИНЫ МОТОРОВ (ВАША РЕАЛЬНАЯ РАСПИНОВКА) ====================
#define M1_IN1 5
#define M1_IN2 6
#define M1_ENA 7

#define M2_IN1 15
#define M2_IN2 16
#define M2_ENB 17

#define M3_IN1 18
#define M3_IN2 8
#define M3_ENA 9

// ==================== 3. ПАРАМЕТРЫ ШИМ (ESP32 Core 3.x) ====================
const int PWM_FREQ = 5000;      // 5 кГц — оптимально для L298N
const int PWM_RES  = 8;         // 8 бит → 0..255

// ==================== 4. ПАРАМЕТРЫ БЕЗОПАСНОСТИ ====================
const int watchdogTimeout = 300;   // мс
unsigned long lastCommandTime = 0;

WiFiUDP udp;

// ==================== 5. ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ ====================
void setMotor(int in1, int in2, int en, int speed) {
  // Ограничиваем скорость диапазоном -255..255
  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(en, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(en, -speed);  // модуль
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(en, 0);
  }
}

void stopAll() {
  setMotor(M1_IN1, M1_IN2, M1_ENA, 0);
  setMotor(M2_IN1, M2_IN2, M2_ENB, 0);
  setMotor(M3_IN1, M3_IN2, M3_ENA, 0);
}

void moveForward(int speed) {
  setMotor(M1_IN1, M1_IN2, M1_ENA, speed);
  setMotor(M2_IN1, M2_IN2, M2_ENB, speed);
  setMotor(M3_IN1, M3_IN2, M3_ENA, speed);
}

void moveBackward(int speed) {
  setMotor(M1_IN1, M1_IN2, M1_ENA, -speed);
  setMotor(M2_IN1, M2_IN2, M2_ENB, -speed);
  setMotor(M3_IN1, M3_IN2, M3_ENA, -speed);
}

void turnLeft(int speed) {
  setMotor(M1_IN1, M1_IN2, M1_ENA, -speed);  // левый назад
  setMotor(M2_IN1, M2_IN2, M2_ENB,  speed);  // правый вперед
  setMotor(M3_IN1, M3_IN2, M3_ENA,  speed);  // задний вперед
}

void turnRight(int speed) {
  setMotor(M1_IN1, M1_IN2, M1_ENA,  speed);
  setMotor(M2_IN1, M2_IN2, M2_ENB, -speed);
  setMotor(M3_IN1, M3_IN2, M3_ENA,  speed);
}

// ==================== 6. ОБРАБОТКА КОМАНД ====================
void processCommand(String command) {
  lastCommandTime = millis();
  command.trim();

  if (command == "FWD")       moveForward(200);
  else if (command == "BACK") moveBackward(200);
  else if (command == "LEFT") turnLeft(150);
  else if (command == "RIGHT")turnRight(150);
  else if (command == "STOP") stopAll();
  else                        stopAll();  // неизвестная команда → стоп
}

// ==================== 7. SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Запуск робота...");

  // Настройка пинов направления
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  // Настройка ШИМ по новому API ESP32 Core 3.x
  ledcAttach(M1_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(M2_ENB, PWM_FREQ, PWM_RES);
  ledcAttach(M3_ENA, PWM_FREQ, PWM_RES);

  stopAll();

  // Подключение к Wi-Fi
  Serial.print("Подключение к Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi подключен!");
    Serial.print("IP-адрес робота: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nОшибка подключения к Wi-Fi!");
  }

  udp.begin(udpPort);
  Serial.print("UDP сервер запущен на порту: ");
  Serial.println(udpPort);

  lastCommandTime = millis();
}

// ==================== 8. LOOP ====================
void loop() {
  // Watchdog
  if (millis() - lastCommandTime > watchdogTimeout) {
    stopAll();
  }

  // Прием UDP
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0';
      String command = String(packetBuffer);
      Serial.print("Получена команда: ");
      Serial.println(command);

      processCommand(command);

      // Ответ "OK" (совместимо с ESP32 Core 3.x)
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write((const uint8_t*)"OK", 2);
      udp.endPacket();
    }
  }

  delay(10);
}

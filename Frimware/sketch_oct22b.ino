// Тестовый скетч для проверки моторов

void setup() {
  Serial.begin(115200);
  
  // Настраиваем все пины как выходы
  pinMode(18, OUTPUT); pinMode(8, OUTPUT);  pinMode(17, OUTPUT); // Мотор 1
  pinMode(15, OUTPUT); pinMode(7, OUTPUT);  pinMode(16, OUTPUT); // Мотор 2
  pinMode(10, OUTPUT);  pinMode(11, OUTPUT);  pinMode(9, OUTPUT);  // Мотор 3
  
  Serial.println("Начинаем тест моторов...");
}

// Функция для плавного включения мотора
void spinMotor(int in1, int in2, int en, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }
  analogWrite(en, speed); // analogWrite работает в новых версиях ESP32
}

void testMotor(String name, int in1, int in2, int en) {
  Serial.println("Тест " + name + ": ВПЕРЕД");
  spinMotor(in1, in2, en, 150); // Скорость 150 из 255
  delay(2000);
  
  spinMotor(in1, in2, en, 0);
  delay(500);
  
  Serial.println("Тест " + name + ": НАЗАД");
  spinMotor(in1, in2, en, -150);
  delay(2000);
  
  spinMotor(in1, in2, en, 0);
  delay(1000);
}

void loop() {

  // Мотор 3
  testMotor("Мотор 3", 10, 11, 9);
  
  
  // Мотор 1
  testMotor("Мотор 1", 18, 8, 17);
  
  // Мотор 2
  testMotor("Мотор 2", 15, 7, 16);
  
  
  
  Serial.println("Тест завершен! Повтор через 5 секунд...");
  delay(1000);
}

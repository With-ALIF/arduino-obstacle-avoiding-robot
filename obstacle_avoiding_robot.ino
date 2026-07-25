
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <ESP32Servo.h>


constexpr uint8_t PIN_MOTOR_ENA = 14;
constexpr uint8_t PIN_MOTOR_IN1 = 27;
constexpr uint8_t PIN_MOTOR_IN2 = 26;
constexpr uint8_t PIN_MOTOR_IN3 = 25;
constexpr uint8_t PIN_MOTOR_IN4 = 33;
constexpr uint8_t PIN_MOTOR_ENB = 32;

constexpr uint8_t PIN_US_TRIG   = 5;
constexpr uint8_t PIN_US_ECHO   = 18;

constexpr uint8_t PIN_IR_HOLE   = 34;
constexpr uint8_t HOLE_STATE    = HIGH;

constexpr uint8_t PIN_SERVO     = 19;

constexpr uint8_t PIN_I2C_SDA   = 21;
constexpr uint8_t PIN_I2C_SCL   = 22;
constexpr uint8_t LCD_I2C_ADDR  = 0x27;

constexpr float   OBSTACLE_THRESHOLD_CM = 25.0f;
constexpr uint8_t DEFAULT_SPEED         = 180;
constexpr uint8_t TURN_SPEED            = 200;

constexpr int SERVO_LEFT_ANGLE   = 150;
constexpr int SERVO_CENTER_ANGLE = 90;
constexpr int SERVO_RIGHT_ANGLE  = 30;

constexpr uint32_t LCD_UPDATE_INTERVAL_MS = 250;
constexpr uint32_t PULSE_TIMEOUT_US       = 25000;

LiquidCrystal_PCF8574 lcd(LCD_I2C_ADDR);
Servo              panServo;

uint32_t lastLcdUpdateMs = 0;

void stopMotor();
void moveForward(uint8_t speed = DEFAULT_SPEED);
void moveBackward(uint8_t speed = DEFAULT_SPEED);
void turnLeft(uint8_t speed = TURN_SPEED);
void turnRight(uint8_t speed = TURN_SPEED);

float readUltrasonic();
bool readHoleSensor();
int scanDirection();
void updateLCD(const char* line1Msg, const char* line2Msg);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println(F("\n=========================================="));
  Serial.println(F(" ESP32 Robot OS Initializing... "));
  Serial.println(F("=========================================="));

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.clear();
  updateLCD("ESP32 Robot OS", "Initializing...");

  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);
  pinMode(PIN_MOTOR_ENB, OUTPUT);
  stopMotor();

  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);

  pinMode(PIN_IR_HOLE, INPUT);
  Serial.println(F("[INIT] TCRT5000L IR Hole Sensor Initialized on GPIO 34."));

  panServo.setPeriodHertz(50);
  panServo.attach(PIN_SERVO, 500, 2400);
  panServo.write(SERVO_CENTER_ANGLE);

  delay(1500);

  updateLCD("System Ready!", "Starting Robot");
  Serial.println(F("[INIT] Initialization Complete. Starting main loop."));
  delay(1000);
}

void loop() {
  float obstacleDist  = readUltrasonic();
  bool holeDetected   = readHoleSensor();

  Serial.print(F("Obstacle Dist: "));
  if (obstacleDist < 0) {
    Serial.print(F("ERROR/TIMEOUT"));
  } else {
    Serial.print(obstacleDist, 1);
    Serial.print(F(" cm"));
  }

  Serial.print(F(" | TCRT5000L Hole IR: "));
  Serial.println(holeDetected ? F("HOLE DETECTED!") : F("CLEAR (Ground OK)"));

  if (holeDetected) {
    Serial.println(F("[SAFETY] HOLE DETECTED by TCRT5000L IR! Executing E-Stop & Backup."));
    
    stopMotor();

    updateLCD("Hole Detected!", "Backing...");

    delay(400);

    moveBackward(DEFAULT_SPEED);
    delay(800);

    stopMotor();
    delay(200);

    turnRight(TURN_SPEED);
    delay(600);

    stopMotor();
    delay(300);
    return;
  }

  if (obstacleDist > 0.0f && obstacleDist < OBSTACLE_THRESHOLD_CM) {
    Serial.printf("[AVOIDANCE] Obstacle detected at %.1f cm! Stopping & scanning...\n", obstacleDist);
    
    stopMotor();

    char obsLine1[17];
    snprintf(obsLine1, sizeof(obsLine1), "Obstacle %.0fcm", obstacleDist);

    updateLCD(obsLine1, "Scanning...");

    int bestDirection = scanDirection();

    if (bestDirection > 0) {
      Serial.println(F("[ACTION] Free path on LEFT. Turning left."));
      updateLCD(obsLine1, "Turning Left");
      turnLeft(TURN_SPEED);
      delay(500);
    } else if (bestDirection < 0) {
      Serial.println(F("[ACTION] Free path on RIGHT. Turning right."));
      updateLCD(obsLine1, "Turning Right");
      turnRight(TURN_SPEED);
      delay(500);
    } else {
      Serial.println(F("[ACTION] Dead end! Backing up and turning."));
      updateLCD(obsLine1, "Dead End!");
      moveBackward(DEFAULT_SPEED);
      delay(700);
      turnRight(TURN_SPEED);
      delay(700);
    }

    stopMotor();
    delay(200);
    return;
  }

  moveForward(DEFAULT_SPEED);

  if (millis() - lastLcdUpdateMs >= LCD_UPDATE_INTERVAL_MS) {
    lastLcdUpdateMs = millis();
    updateLCD("System Normal", "Moving Forward");
  }

  delay(50);
}


void stopMotor() {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);
  analogWrite(PIN_MOTOR_ENA, 0);
  analogWrite(PIN_MOTOR_ENB, 0);
}

void moveForward(uint8_t speed) {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, HIGH);
  digitalWrite(PIN_MOTOR_IN4, LOW);
  analogWrite(PIN_MOTOR_ENA, speed);
  analogWrite(PIN_MOTOR_ENB, speed);
}

void moveBackward(uint8_t speed) {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, HIGH);
  analogWrite(PIN_MOTOR_ENA, speed);
  analogWrite(PIN_MOTOR_ENB, speed);
}

void turnLeft(uint8_t speed) {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  digitalWrite(PIN_MOTOR_IN3, HIGH);
  digitalWrite(PIN_MOTOR_IN4, LOW);
  analogWrite(PIN_MOTOR_ENA, speed);
  analogWrite(PIN_MOTOR_ENB, speed);
}

void turnRight(uint8_t speed) {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, HIGH);
  analogWrite(PIN_MOTOR_ENA, speed);
  analogWrite(PIN_MOTOR_ENB, speed);
}


float readUltrasonic() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_US_ECHO, HIGH, PULSE_TIMEOUT_US);

  if (duration == 0) {
    return -1.0f;
  }

  float distanceCm = (duration * 0.0343f) / 2.0f;

  if (distanceCm < 2.0f || distanceCm > 400.0f) {
    return -1.0f;
  }

  return distanceCm;
}

bool readHoleSensor() {
  int sensorVal = digitalRead(PIN_IR_HOLE);
  return (sensorVal == HOLE_STATE);
}

int scanDirection() {
  panServo.write(SERVO_LEFT_ANGLE);
  delay(400);
  float leftDistance = readUltrasonic();
  Serial.print(F("[SCAN] Left Distance: "));
  Serial.println(leftDistance);

  panServo.write(SERVO_RIGHT_ANGLE);
  delay(400);
  float rightDistance = readUltrasonic();
  Serial.print(F("[SCAN] Right Distance: "));
  Serial.println(rightDistance);

  panServo.write(SERVO_CENTER_ANGLE);
  delay(300);

  if (leftDistance > OBSTACLE_THRESHOLD_CM && leftDistance >= rightDistance) {
    return 1;
  } else if (rightDistance > OBSTACLE_THRESHOLD_CM && rightDistance > leftDistance) {
    return -1;
  }

  return 0;
}


void updateLCD(const char* line1Msg, const char* line2Msg) {
  lcd.setCursor(0, 0);
  lcd.print(line1Msg);
  for (int i = strlen(line1Msg); i < 16; i++) {
    lcd.print(F(" "));
  }

  lcd.setCursor(0, 1);
  lcd.print(line2Msg);
  for (int i = strlen(line2Msg); i < 16; i++) {
    lcd.print(F(" "));
  }
}

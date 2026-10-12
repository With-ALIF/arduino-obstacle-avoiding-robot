#include <Servo.h>

// ===== L298N MOTOR PINS =====
const byte ENA = 3;   // Left motor PWM
const byte IN1 = 7;
const byte IN2 = 6;

const byte ENB = 11;  // Right motor PWM
const byte IN3 = 5;
const byte IN4 = 4;

// ===== HC-SR04 =====
const byte TRIG_PIN = 9;
const byte ECHO_PIN = 8;

// ===== SERVO =====
const byte SERVO_PIN = 10;

// ===== SPEED SETTINGS =====
const byte NORMAL_SPEED = 130; // 0-255
const byte TURN_SPEED = 230;   // 0-255
const byte UTURN_SPEED = 230;

// ===== OBSTACLE SETTINGS =====
const float OBSTACLE_LIMIT = 30.0;
const float EQUAL_LIMIT = 5.0;

const byte SERVO_CENTER = 90;
const byte SERVO_LEFT = 150;
const byte SERVO_RIGHT = 30;

// Tune these for your floor and wheels
const unsigned int TURN_90_TIME = 400;
const unsigned int TURN_180_TIME = 800;

Servo scanner;

float leftDistance = -1;
float rightDistance = -1;

// ===== SETUP =====
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  scanner.attach(SERVO_PIN);
  scanner.write(SERVO_CENTER);

  stopCar();

  Serial.println("PWM Obstacle Avoiding Car Ready");
}

// ===== MAIN LOOP =====
void loop() {
  scanner.write(SERVO_CENTER);

  float frontDistance = getDistance();

  Serial.print("Front: ");
  if (frontDistance < 0) {
    Serial.println("INVALID");
    stopCar();
    delay(50);
    return;
  }

  Serial.print(frontDistance);
  Serial.println(" cm");

  if (frontDistance <= OBSTACLE_LIMIT) {
    // Stop before scanning
    stopCar();
    delay(150);

    scanSides();

    Serial.print("Left: ");
    Serial.print(leftDistance);
    Serial.print(" cm | Right: ");
    Serial.print(rightDistance);
    Serial.println(" cm");

    // Never choose a direction using invalid readings
    if (leftDistance < 0 || rightDistance < 0) {
      stopCar();
      delay(100);
      return;
    }

    if (abs(leftDistance - rightDistance)
        <= EQUAL_LIMIT) {
      turnAround();
    }
    else if (leftDistance > rightDistance) {
      turnLeft();
    }
    else {
      turnRight();
    }

    stopCar();
    scanner.write(SERVO_CENTER);
    delay(150);
  }
  else {
    forward();
  }

  delay(20);
}

// ===== ULTRASONIC DISTANCE =====
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, 12000UL);

  if (duration == 0) {
    return -1;
  }

  float cm = duration * 0.0343 / 2.0;

  if (cm < 2.0 || cm > 200.0) {
    return -1;
  }

  return cm;
}

// ===== SCAN SIDES =====
void scanSides() {
  scanner.write(SERVO_LEFT);
  delay(350);
  leftDistance = getDistance();

  scanner.write(SERVO_RIGHT);
  delay(450);
  rightDistance = getDistance();

  scanner.write(SERVO_CENTER);
  delay(250);
}

// ===== FORWARD: LOW SPEED =====
void forward() {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, NORMAL_SPEED);
  analogWrite(ENB, NORMAL_SPEED);
}

// ===== STOP =====
void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ===== LEFT TURN: HIGH SPEED =====
void turnLeft() {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);

  delay(TURN_90_TIME);
  stopCar();
  delay(150);
}

// ===== RIGHT TURN: HIGH SPEED =====
void turnRight() {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);

  delay(TURN_90_TIME);
  stopCar();
  delay(150);
}

// ===== 180-DEGREE TURN =====
void turnAround() {
  // Pivot in place
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, UTURN_SPEED);
  analogWrite(ENB, UTURN_SPEED);

  delay(TURN_180_TIME);
  stopCar();
  delay(250);
}

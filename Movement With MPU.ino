
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

MPU6050 mpu(Wire);

// L298N input pins
const int FR_IN1 = 3;
const int FR_IN2 = 2;
const int BR_IN1 = 4;
const int BR_IN2 = 5;
const int BL_IN1 = 9;
const int BL_IN2 = 8;
const int FL_IN1 = 11;
const int FL_IN2 = 10;

// Set a wheel's direction to -1 if it spins backward.
const int FR_FORWARD = -1;
const int BR_FORWARD = -1;
const int BL_FORWARD = -1;
const int FL_FORWARD = -1;

const int BASE_SPEED = 170;
const int MIN_SPEED = 90;
const int MAX_SPEED = 230;

const float KP = 5.0f;
const float DEADBAND = 1.5f;
const float MAX_CORRECTION = 70.0f;
const int CORRECTION_SIGN = -1;

float targetHeading = 0.0f;

void motor(int pin1, int pin2, int speed) {
speed = constrain(speed, -255, 255);

if (speed >= 0) {
analogWrite(pin2, 0);
analogWrite(pin1, speed);
} else {
analogWrite(pin1, 0);
analogWrite(pin2, -speed);
}
}

void setSideSpeeds(int leftSpeed, int rightSpeed) {
motor(FR_IN1, FR_IN2, rightSpeed * FR_FORWARD);
motor(BR_IN1, BR_IN2, rightSpeed * BR_FORWARD);
motor(BL_IN1, BL_IN2, leftSpeed * BL_FORWARD);
motor(FL_IN1, FL_IN2, leftSpeed * FL_FORWARD);
}

float normalizeAngle(float angle) {
while (angle > 180.0f) angle -= 360.0f;
while (angle < -180.0f) angle += 360.0f;
return angle;
}

void setup() {
Serial.begin(115200);

const int motorPins[] = {
FR_IN1, FR_IN2, BR_IN1, BR_IN2,
BL_IN1, BL_IN2, FL_IN1, FL_IN2
};

for (int pin : motorPins) {
pinMode(pin, OUTPUT);
}
setSideSpeeds(0, 0);

Wire.begin(); // Mega SDA = 20, SCL = 21

byte status = mpu.begin();
if (status != 0) {
Serial.print("MPU6050 initialization failed. Status = ");
Serial.println(status);
while (true) {
delay(100);
}
}

Serial.println("Keep the robot still. Calibrating MPU6050...");
delay(1000);
mpu.calcOffsets(true, true);

mpu.update();
targetHeading = mpu.getAngleZ();

Serial.print("Target heading = ");
Serial.println(targetHeading);
Serial.println("Forward motion starts in 2 seconds.");

unsigned long startWait = millis();
while (millis() - startWait < 2000UL) {
mpu.update();
delay(5);
}
}

void loop() {
mpu.update();

float yaw = mpu.getAngleZ();
float error = normalizeAngle(targetHeading - yaw);
float correction = 0.0f;

if (fabsf(error) > DEADBAND) {
correction = KP * error * CORRECTION_SIGN;
}
correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

int leftSpeed = constrain(
(int)(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED
);
int rightSpeed = constrain(
(int)(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED
);

setSideSpeeds(leftSpeed, rightSpeed);

static unsigned long lastPrint = 0;
if (millis() - lastPrint >= 100UL) {
lastPrint = millis();

Serial.print("Yaw: ");
Serial.print(yaw);
Serial.print(" | Error: ");
Serial.print(error);
Serial.print(" | Left PWM: ");
Serial.print(leftSpeed);
Serial.print(" | Right PWM: ");
Serial.println(rightSpeed);
}
}

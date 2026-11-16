#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

// =====================================
// ROBOCUP DEFENDER ROBOT
// ARDUINO MEGA 2560
// MPU6050 + 4 APDS9960 + TCA9548A
// FORWARD MOVEMENT ONLY
// =====================================

// =====================================
// MOTOR PINS - L298N
// =====================================

MPU6050 mpu(Wire);

const int FR_IN1 = 3;
const int FR_IN2 = 2;

const int BR_IN1 = 4;
const int BR_IN2 = 5;

const int BL_IN1 = 9;
const int BL_IN2 = 8;

const int FL_IN1 = 11;
const int FL_IN2 = 10;

// Motor directions
const int FR_FORWARD = -1;
const int BR_FORWARD = -1;
const int BL_FORWARD = -1;
const int FL_FORWARD = -1;

// =====================================
// SPEED SETTINGS
// =====================================

const int BASE_SPEED = 170;
const int MIN_SPEED = 90;
const int MAX_SPEED = 230;

// =====================================
// MPU6050 HEADING CORRECTION
// =====================================

float targetHeading = 0.0f;

const float KP = 5.0f;
const float DEADBAND = 1.5f;

const float MAX_CORRECTION = 70.0f;
const int CORRECTION_SIGN = -1;

// =====================================
// TCA9548A + APDS9960
// =====================================

#define TCA_ADDR 0x70
#define APDS_ADDR 0x39

const byte FRONT = 0;
const byte BACK  = 1;
const byte LEFT  = 2;
const byte RIGHT = 3;

// APDS channels
const byte CHANNELS[4] = {
    6,  // FRONT
    1,  // BACK
    2,  // LEFT
    0   // RIGHT
};

const char* SENSOR_NAMES[4] = {
    "FRONT",
    "BACK",
    "LEFT",
    "RIGHT"
};

// =====================================
// CALIBRATED WHITE THRESHOLDS
// =====================================

// RED / GREEN * 1000

const int WHITE_RG_MIN[4] = {
    675,  // FRONT
    630,  // BACK
    760,  // LEFT
    735   // RIGHT
};

// BLUE / GREEN * 1000

const int WHITE_BG_MIN[4] = {
    735,  // FRONT
    570,  // BACK
    755,  // LEFT
    715   // RIGHT
};

// =====================================
// APDS9960 REGISTERS
// =====================================

#define APDS_ENABLE  0x80
#define APDS_ATIME   0x81
#define APDS_CONTROL 0x8F
#define APDS_ID      0x92
#define APDS_STATUS  0x93

#define APDS_CDATAL  0x94
#define APDS_RDATAL  0x96
#define APDS_GDATAL  0x98
#define APDS_BDATAL  0x9A

// =====================================
// COLOR SENSOR DATA
// =====================================

struct ColorData {

    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;

    int rgRatio;
    int bgRatio;

    bool white;
    bool valid;
};

ColorData color[4];

bool sensorReady[4] = {
    false, false, false, false
};

unsigned long colorUpdated[4] = {
    0, 0, 0, 0
};

byte nextColorSensor = 0;

// =====================================
// TIMING
// =====================================

const unsigned long COLOR_INTERVAL = 2;
const unsigned long COLOR_FRESH_MS = 50;

const unsigned long START_DELAY = 3000;
const unsigned long DEBUG_INTERVAL = 250;

unsigned long lastColorRead = 0;
unsigned long lastDebug = 0;

bool robotEnabled = false;

// =====================================
// MOTOR CONTROL
// =====================================

void motor(int pin1, int pin2, int speed) {

    speed = constrain(speed, -255, 255);

    if (speed > 0) {

        analogWrite(pin1, speed);
        analogWrite(pin2, 0);

    }
    else if (speed < 0) {

        analogWrite(pin1, 0);
        analogWrite(pin2, -speed);

    }
    else {

        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
    }
}

void setSideSpeeds(int leftSpeed, int rightSpeed) {

    motor(FR_IN1, FR_IN2,
          rightSpeed * FR_FORWARD);

    motor(BR_IN1, BR_IN2,
          rightSpeed * BR_FORWARD);

    motor(BL_IN1, BL_IN2,
          leftSpeed * BL_FORWARD);

    motor(FL_IN1, FL_IN2,
          leftSpeed * FL_FORWARD);
}

void stopMotors() {

    setSideSpeeds(0, 0);
}

// =====================================
// MPU6050 HEADING CORRECTION
// =====================================

float normalizeAngle(float angle) {

    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle < -180.0f)
        angle += 360.0f;

    return angle;
}

float getCorrection() {

    float yaw = mpu.getAngleZ();

    float error = normalizeAngle(
        targetHeading - yaw
    );

    float correction = 0.0f;

    if (fabsf(error) > DEADBAND) {

        correction =
            KP * error * CORRECTION_SIGN;
    }

    correction = constrain(
        correction,
        -MAX_CORRECTION,
        MAX_CORRECTION
    );

    return correction;
}

// =====================================
// FORWARD MOVEMENT WITH MPU
// =====================================

void moveForward(int speedVal) {

    float correction = getCorrection();

    int leftSpeed =
        speedVal + correction;

    int rightSpeed =
        speedVal - correction;

    leftSpeed = constrain(
        leftSpeed,
        MIN_SPEED,
        MAX_SPEED
    );

    rightSpeed = constrain(
        rightSpeed,
        MIN_SPEED,
        MAX_SPEED
    );

    setSideSpeeds(
        leftSpeed,
        rightSpeed
    );
}

// =====================================
// TCA9548A CHANNEL SELECTION
// =====================================

bool selectChannel(byte channel) {

    Wire.beginTransmission(TCA_ADDR);
    Wire.write((byte)0);

    if (Wire.endTransmission() != 0)
        return false;

    Wire.beginTransmission(TCA_ADDR);
    Wire.write((byte)(1 << channel));

    return Wire.endTransmission() == 0;
}

// =====================================
// APDS REGISTER FUNCTIONS
// =====================================

bool writeAPDS(byte reg, byte value) {

    Wire.beginTransmission(APDS_ADDR);

    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

int readAPDS(byte reg) {

    Wire.beginTransmission(APDS_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return -1;

    if (Wire.requestFrom(
        (uint8_t)APDS_ADDR,
        (uint8_t)1) != 1)
        return -1;

    return Wire.read();
}

bool read16(byte reg, uint16_t &value) {

    int low = readAPDS(reg);
    int high = readAPDS(reg + 1);

    if (low < 0 || high < 0)
        return false;

    value = (uint16_t)low |
            ((uint16_t)high << 8);

    return true;
}

// =====================================
// INITIALIZE APDS9960
// =====================================

bool initAPDS(byte index) {

    if (!selectChannel(CHANNELS[index]))
        return false;

    int id = readAPDS(APDS_ID);

    if (id != 0x9E &&
        id != 0xAB &&
        id != 0x9C) {

        Serial.print(SENSOR_NAMES[index]);
        Serial.println(" ID ERROR");

        return false;
    }

    bool ok = true;

    ok &= writeAPDS(APDS_ENABLE, 0x00);

    delay(10);

    ok &= writeAPDS(APDS_ATIME, 0xF6);

    ok &= writeAPDS(APDS_CONTROL, 0x02);

    ok &= writeAPDS(APDS_ENABLE, 0x03);

    delay(50);

    return ok;
}

// =====================================
// WHITE DETECTION
// =====================================

bool detectWhite(
    byte index,
    uint16_t r,
    uint16_t g,
    uint16_t b,
    int &rg,
    int &bg
) {

    if (g == 0) {

        rg = 0;
        bg = 0;

        return false;
    }

    rg = (long)r * 1000L / g;
    bg = (long)b * 1000L / g;

    return (
        rg >= WHITE_RG_MIN[index] &&
        bg >= WHITE_BG_MIN[index]
    );
}

// =====================================
// READ COLOR SENSOR
// =====================================

bool readColor(byte index) {

    if (!selectChannel(CHANNELS[index]))
        return false;

    int status = readAPDS(APDS_STATUS);

    if (status < 0)
        return false;

    if (!(status & 0x01))
        return false;

    uint16_t c, r, g, b;

    bool ok =
        read16(APDS_CDATAL, c) &&
        read16(APDS_RDATAL, r) &&
        read16(APDS_GDATAL, g) &&
        read16(APDS_BDATAL, b);

    if (!ok)
        return false;

    int rg, bg;

    bool white = detectWhite(
        index,
        r,
        g,
        b,
        rg,
        bg
    );

    color[index].clear = c;
    color[index].red = r;
    color[index].green = g;
    color[index].blue = b;

    color[index].rgRatio = rg;
    color[index].bgRatio = bg;

    color[index].white = white;
    color[index].valid = true;

    colorUpdated[index] = millis();

    return true;
}

// =====================================
// UPDATE ALL COLOR SENSORS
// =====================================

void updateColorSensors() {

    unsigned long now = millis();

    if (now - lastColorRead < COLOR_INTERVAL)
        return;

    lastColorRead = now;

    byte index = nextColorSensor;

    if (sensorReady[index]) {
        readColor(index);
    }

    nextColorSensor =
        (nextColorSensor + 1) % 4;
}

// =====================================
// CHECK SENSOR HEALTH
// =====================================

bool colorsHealthy() {

    unsigned long now = millis();

    for (byte i = 0; i < 4; i++) {

        if (!sensorReady[i])
            return false;

        if (!color[i].valid)
            return false;

        if (now - colorUpdated[i] > COLOR_FRESH_MS)
            return false;
    }

    return true;
}

// =====================================
// INITIAL COLOR READINGS
// =====================================

bool initializeColorReadings() {

    Serial.println(
        "Waiting for color sensor data..."
    );

    for (byte i = 0; i < 4; i++) {

        bool success = false;

        for (byte attempt = 0; attempt < 10; attempt++) {

            if (readColor(i)) {

                success = true;
                break;
            }

            delay(30);
        }

        if (!success) {

            Serial.print("COLOR READ FAILED: ");
            Serial.println(SENSOR_NAMES[i]);

            return false;
        }

        Serial.print(SENSOR_NAMES[i]);
        Serial.println(" COLOR DATA READY");
    }

    return true;
}

// =====================================
// CHECK ALL SENSORS FOR GREEN
// =====================================

bool allSensorsGreen() {

    if (!colorsHealthy())
        return false;

    for (byte i = 0; i < 4; i++) {

        if (color[i].white)
            return false;
    }

    return true;
}

// =====================================
// SERIAL DEBUG
// =====================================

void printDebug() {

    Serial.print("YAW: ");
    Serial.print(mpu.getAngleZ());

    Serial.print(" | TARGET: ");
    Serial.print(targetHeading);

    for (byte i = 0; i < 4; i++) {

        Serial.print(" | ");
        Serial.print(SENSOR_NAMES[i]);
        Serial.print(": ");

        if (!color[i].valid) {

            Serial.print("INVALID");
            continue;
        }

        Serial.print(
            color[i].white ? "WHITE" : "GREEN"
        );

        Serial.print(" RG:");
        Serial.print(color[i].rgRatio);

        Serial.print(" BG:");
        Serial.print(color[i].bgRatio);
    }

    Serial.print(" | MOVEMENT: ");

    Serial.println(
        allSensorsGreen() ? "FORWARD" : "STOPPED"
    );
}

// =====================================
// SETUP
// =====================================

void setup() {

    Serial.begin(115200);

    // Motor pins

    const int motorPins[] = {
        FR_IN1, FR_IN2,
        BR_IN1, BR_IN2,
        BL_IN1, BL_IN2,
        FL_IN1, FL_IN2
    };

    for (int pin : motorPins) {
        pinMode(pin, OUTPUT);
    }

    stopMotors();

    // I2C initialization

    Wire.begin();
    Wire.setClock(100000);
    Wire.setWireTimeout(30000, false);

    // MPU initialization

    byte status = mpu.begin();

    if (status != 0) {

        Serial.print("MPU6050 FAILED: ");
        Serial.println(status);

        while (true) {
            stopMotors();
            delay(100);
        }
    }

    Serial.println("MPU6050 READY");

    Serial.println("Keep robot stationary.");
    Serial.println("Calibrating MPU6050...");

    delay(1000);

    mpu.calcOffsets(true, true);

    mpu.update();

    targetHeading = mpu.getAngleZ();

    Serial.print("TARGET HEADING: ");
    Serial.println(targetHeading);

    // Initialize colour sensors

    Serial.println("Initializing APDS9960 sensors...");

    bool allReady = true;

    for (byte i = 0; i < 4; i++) {

        sensorReady[i] = initAPDS(i);

        Serial.print(SENSOR_NAMES[i]);
        Serial.print(": ");

        Serial.println(
            sensorReady[i] ? "READY" : "FAILED"
        );

        if (!sensorReady[i])
            allReady = false;
    }

    if (!allReady) {

        Serial.println("COLOR SENSOR INIT ERROR");

        while (true) {
            stopMotors();
            delay(100);
        }
    }

    // Initial readings

    if (!initializeColorReadings()) {

        Serial.println("COLOR INITIALIZATION FAILED");

        while (true) {
            stopMotors();
            delay(100);
        }
    }

    Serial.println("ALL COLOR SENSORS READY");

    // Starting delay

    Serial.println("STARTING IN 3 SECONDS");

    unsigned long startTime = millis();

    while (millis() - startTime < START_DELAY) {

        mpu.update();

        updateColorSensors();

        delay(2);
    }

    mpu.update();

    targetHeading = mpu.getAngleZ();

    robotEnabled = true;

    Serial.print("FINAL TARGET HEADING: ");
    Serial.println(targetHeading);

    Serial.println("DEFENDER STARTED");
}

// =====================================
// MAIN LOOP
// =====================================

void loop() {

    mpu.update();

    if (!robotEnabled) {

        stopMotors();
        return;
    }

    unsigned long now = millis();

    // PRIORITY 1: Update color sensors

    updateColorSensors();

    // PRIORITY 2: Check sensor health

    if (!colorsHealthy()) {

        stopMotors();

        if (now - lastDebug >= DEBUG_INTERVAL) {

            lastDebug = now;

            Serial.println(
                "WAITING FOR COLOR SENSOR DATA"
            );

            printDebug();
        }

        return;
    }

    // PRIORITY 3: Check for white boundary

    if (!allSensorsGreen()) {

        stopMotors();

    }

    // PRIORITY 4: Forward movement with MPU

    else {

        moveForward(BASE_SPEED);
    }

    // Debug output

    if (now - lastDebug >= DEBUG_INTERVAL) {

        lastDebug = now;

        printDebug();
    }
}
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// Configuration
#define DEBUG_MODE true           // Set to false to disable Serial debug output
#define SIMPLE_IMPLEMENTATION false
#define MPU_INTERRUPT_PIN 2      // Pin connected to MPU interrupt
#define I2C_CLOCK_RATE 400000    // 400kHz I2C clock
#define FIFO_TIMEOUT_MS 1000     // Timeout for FIFO operations
#define LED_UPDATE_INTERVAL_MS 20 // Minimum time between LED updates

// LED Configuration
const int frontLed = 3;
const int bottomLed = 5;
const int rightLed = 10;
const int leftLed = 9;

// Timing variables
unsigned long lastPrintTime = 0;
unsigned long lastLedUpdateTime = 0;

// LED Configuration Structure
typedef struct {
    byte pin;
    byte positionInsideGroup;
    char thePosition;     // 'u' = up, 'd' = down, 'l' = left, 'r' = right
    byte minAngle;       // Minimum angle to trigger LED
    byte maxAngle;       // Maximum angle for scaling brightness
} ledConfig;

// LED Configuration Array
ledConfig leds[] = {
    {3, 1, 'u', 31, 45},
    {12, 2, 'u', 16, 30},
    {11, 3, 'u', 5, 15},
    {5, 1, 'd', 5, 15},
    {6, 2, 'd', 16, 30},
    {7, 3, 'd', 31, 45},
    {8, 1, 'r', 5, 23},
    {9, 2, 'r', 24, 45},
    {10, 1, 'l', 5, 23},
    {4, 2, 'l', 24, 45},
};

const int NUM_LEDS = sizeof(leds) / sizeof(leds[0]);

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;
volatile uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
volatile uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;

// Debug print macro
#define DEBUG_PRINT(x) if(DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if(DEBUG_MODE) { Serial.println(x); }

// Interrupt handler for MPU data ready
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    if (DEBUG_MODE) {
        Serial.begin(115200);
        while (!Serial); // Wait for Serial to be ready
    }

    DEBUG_PRINTLN(F("Initializing I2C devices..."));
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(I2C_CLOCK_RATE);

    // Initialize MPU6050
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        DEBUG_PRINTLN(F("MPU6050 connection failed! Halting..."));
        while (1); // Don't continue if MPU6050 init fails
    }

    DEBUG_PRINTLN(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(101);
    mpu.setYGyroOffset(41);
    mpu.setZGyroOffset(6);
    mpu.setZAccelOffset(1273);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        DEBUG_PRINTLN(F("Enabling interrupt detection..."));
        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        DEBUG_PRINT(F("DMP Initialization failed (code "));
        DEBUG_PRINT(devStatus);
        DEBUG_PRINTLN(F(")"));
        while (1); // Don't continue if DMP init fails
    }

    // Initialize LEDs
    if (SIMPLE_IMPLEMENTATION) {
        initializeLEDsSimple();
    } else {
        initializeLEDsMultiple();
    }
}

void loop() {
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) return;

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        DEBUG_PRINTLN(F("FIFO overflow!"));
        return;
    }

    // Wait for correct available data length
    unsigned long startWait = millis();
    while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
        if (millis() - startWait > FIFO_TIMEOUT_MS) {
            DEBUG_PRINTLN(F("FIFO timeout!"));
            return;
        }
    }

    // Read packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Calculate Euler angles
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert to degrees
    int x = (int)(ypr[0] * 180/M_PI);
    int y = (int)(ypr[1] * 180/M_PI);
    int z = (int)(ypr[2] * 180/M_PI);

    // Update LEDs if enough time has passed
    if (millis() - lastLedUpdateTime >= LED_UPDATE_INTERVAL_MS) {
        if (SIMPLE_IMPLEMENTATION) {
            flashLEDsSimple(x, y, z);
        } else {
            flashLEDsMultiple(x, y, z);
        }
        lastLedUpdateTime = millis();
    }

    // Debug output
    if (DEBUG_MODE && millis() - lastPrintTime >= 1000) {
        DEBUG_PRINT(F("Orientation: "));
        DEBUG_PRINT(y);
        DEBUG_PRINT(F("\t"));
        DEBUG_PRINTLN(z);
        lastPrintTime = millis();
    }
}

void initializeLEDsSimple() {
    pinMode(frontLed, OUTPUT);
    pinMode(bottomLed, OUTPUT);
    pinMode(rightLed, OUTPUT);
    pinMode(leftLed, OUTPUT);
}

void initializeLEDsMultiple() {
    for (int i = 0; i < NUM_LEDS; i++) {
        pinMode(leds[i].pin, OUTPUT);
        digitalWrite(leds[i].pin, LOW);
    }
    delay(100); // Short delay to ensure pins are initialized
}

void flashLEDsSimple(int x, int y, int z) {
    // Calculate LED brightness with bounds checking
    int rightBrightness = (y > 0) ? constrain(y * 4, 0, 255) : 0;
    int leftBrightness = (y < 0) ? constrain(abs(y) * 4, 0, 255) : 0;
    int bottomBrightness = (z > 0) ? constrain(z * 4, 0, 255) : 0;
    int frontBrightness = (z < 0) ? constrain(abs(z) * 4, 0, 255) : 0;

    analogWrite(rightLed, rightBrightness);
    analogWrite(leftLed, leftBrightness);
    analogWrite(bottomLed, bottomBrightness);
    analogWrite(frontLed, frontBrightness);
}

int calculateLEDBrightness(int angle, int minAngle, int maxAngle) {
    if (abs(angle) < minAngle) return 0;
    if (abs(angle) > maxAngle) return 255;
    return map(abs(angle), minAngle, maxAngle, 0, 255);
}

void flashLEDsMultiple(int x, int y, int z) {
    for (int i = 0; i < NUM_LEDS; i++) {
        int brightness = 0;
        
        // Calculate brightness based on position and angle
        switch (leds[i].thePosition) {
            case 'u':
                if (z < 0) brightness = calculateLEDBrightness(z, leds[i].minAngle, leds[i].maxAngle);
                break;
            case 'd':
                if (z > 0) brightness = calculateLEDBrightness(z, leds[i].minAngle, leds[i].maxAngle);
                break;
            case 'l':
                if (y < 0) brightness = calculateLEDBrightness(y, leds[i].minAngle, leds[i].maxAngle);
                break;
            case 'r':
                if (y > 0) brightness = calculateLEDBrightness(y, leds[i].minAngle, leds[i].maxAngle);
                break;
        }
        
        analogWrite(leds[i].pin, brightness);
    }
}
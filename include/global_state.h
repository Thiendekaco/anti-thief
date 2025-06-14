#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include <RCSwitch.h>
#include <BlynkSimpleEsp32.h>  // Include trực tiếp BlynkSimpleEsp32.h

#include "config.h"

// Khai báo các đối tượng toàn cục
extern Adafruit_MPU6050 mpu;
extern TinyGPSPlus gps;
extern RCSwitch rfReceiver;
extern BlynkTimer timer;
extern HardwareSerial SerialGPS;
extern HardwareSerial SerialSIM;

// Không cần extern cho Blynk nữa, vì đã được định nghĩa trong BlynkSimpleEsp32.h

// Các biến trạng thái hệ thống
extern bool wifiConnected;
extern bool simActive;
extern bool alarmState;
extern bool ownerPresent;
extern bool motionDetected;
extern bool vehicleMoving;
extern bool gpsActive;
extern bool gpsPowerState;
extern bool rfEnabled;
extern bool rfPowerState;
extern bool hadValidFix;
extern bool signalLost;
extern bool lowBatteryAlerted;
extern bool blynkOverSIM;
extern float lastLat, lastLng;
extern float currentLat, currentLng;
extern unsigned long lastWifiCheck;
extern unsigned long lastGpsUpdate;
extern unsigned long lastRfCheck;
extern unsigned long lastRfActivation;
extern unsigned long alarmStartTime;
extern unsigned long lastDistanceCheck;
extern unsigned long motionDetectedTime;
extern unsigned long sequenceEndTime;
extern unsigned long gpsActivationTime;
extern unsigned long lastBatteryCheck;
extern unsigned long lastBlynk4GSync;
extern float batteryVoltage;
extern int batteryPercentage;
extern int stage2GpsUpdateCount;
extern bool manualReset;

// Biến cho kết nối theo yêu cầu
extern bool networkModeActive;
extern unsigned long networkActivationTime;
extern bool usingSIM;

// Biến cho trạng thái báo động
extern AlarmStage alarmStage;
extern int beepCount;
extern int beepSequenceCount;
extern unsigned long lastBeepTime;
extern bool notificationSent;
extern bool initialPositionSet;

// Biến xử lý ngắt MPU6050
extern volatile bool mpuInterrupt;

// Khai báo các hàm được sử dụng ở nhiều module
extern float getBatteryVoltage();
extern int calculateBatteryPercentage(float voltage);

#endif // GLOBAL_STATE_H
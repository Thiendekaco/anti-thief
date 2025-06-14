#include "global_state.h"

// Khởi tạo các đối tượng toàn cục
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
RCSwitch rfReceiver = RCSwitch();
BlynkTimer timer;
HardwareSerial SerialGPS(1);  // UART1 cho GPS ATGM336H
HardwareSerial SerialSIM(2);  // UART2 cho module SIM A7682S

// Khởi tạo các biến trạng thái hệ thống
bool wifiConnected = false;
bool simActive = false;
bool alarmState = false;
bool ownerPresent = false;
bool motionDetected = false;
bool vehicleMoving = false;
bool gpsActive = false;
bool gpsPowerState = false;
bool rfEnabled = true;
bool rfPowerState = false;
bool hadValidFix = false;
bool signalLost = false;
bool lowBatteryAlerted = false;
bool blynkOverSIM = false;
float lastLat = 0, lastLng = 0;
float currentLat = 0, currentLng = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastGpsUpdate = 0;
unsigned long lastRfCheck = 0;
unsigned long lastRfActivation = 0;
unsigned long alarmStartTime = 0;
unsigned long lastDistanceCheck = 0;
unsigned long motionDetectedTime = 0;
unsigned long sequenceEndTime = 0;
unsigned long gpsActivationTime = 0;
unsigned long lastBatteryCheck = 0;
unsigned long lastBlynk4GSync = 0;
float batteryVoltage = 0;
int batteryPercentage = 0;
int stage2GpsUpdateCount = 0;
bool manualReset = false;

// Biến cho kết nối theo yêu cầu
bool networkModeActive = false;
unsigned long networkActivationTime = 0;
bool usingSIM = false;

// Biến cho trạng thái báo động
AlarmStage alarmStage = STAGE_NONE;
int beepCount = 0;
int beepSequenceCount = 0;
unsigned long lastBeepTime = 0;
bool notificationSent = false;
bool initialPositionSet = false;

// Biến xử lý ngắt MPU6050
volatile bool mpuInterrupt = false;
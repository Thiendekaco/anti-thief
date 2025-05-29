/*
 * Hệ thống Theo dõi và Chống trộm Xe ESP32 - Phiên bản Tích Hợp 2025
 * Sử dụng Blynk v1 (Legacy) - PlatformIO Edition
 *
 * Tính năng:
 * - Kết nối chỉ theo yêu cầu (WiFi/4G) khi kích hoạt qua RF433
 * - Phát hiện chuyển động sử dụng cảm biến gia tốc MPU6050
 * - Theo dõi vị trí GPS (ATGM336H) chỉ khi cần thiết (chỉ gửi qua SMS)
 * - Nhận dạng người dùng RF433MHz với 2 mã khác nhau
 * - Quản lý năng lượng tối ưu để kéo dài thời lượng pin
 * - Tích hợp Blynk thông qua WiFi hoặc 4G (A7682S) (không gửi GPS)
 * - Báo động 3 giai đoạn theo cấu hình cụ thể
 * - Cảnh báo pin thấp qua SMS khi dưới 20%
 * - Chế độ ngủ cho module GPS ATGM336H
 *
 * Cập nhật lần cuối: 2025-05-29 07:35:16
 * Người phát triển: nguongthienTieu
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <RCSwitch.h>
#include "config.h"

// Giao diện Serial
HardwareSerial SerialGPS(1);  // UART1 cho GPS ATGM336H
HardwareSerial SerialSIM(2);  // UART2 cho module SIM A7682S

// Khởi tạo đối tượng
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
RCSwitch rfReceiver = RCSwitch();
BlynkTimer timer;  // Timer cho các sự kiện định kỳ với Blynk

// Biến toàn cục
bool wifiConnected = false;
bool simActive = false;
bool alarmState = false;
bool ownerPresent = false;
bool motionDetected = false;
bool vehicleMoving = false;
bool gpsActive = false;
bool gpsPowerState = false;     // Trạng thái nguồn GPS
bool rfEnabled = true;          // RF433 enabled by default
bool rfPowerState = false;      // Trạng thái nguồn RF
bool hadValidFix = false;       // Đã từng có tín hiệu GPS hợp lệ
bool signalLost = false;        // Đánh dấu khi mất tín hiệu GPS
bool lowBatteryAlerted = false; // Theo dõi đã gửi cảnh báo pin thấp hay chưa
bool blynkOverSIM = false;      // Đang sử dụng Blynk qua 4G
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
unsigned long lastBlynk4GSync = 0;  // Thời gian cuối cùng đồng bộ Blynk qua 4G
float batteryVoltage = 0;
int batteryPercentage = 0;
int stage2GpsUpdateCount = 0;    // Đếm số lần cập nhật GPS ở giai đoạn 2

// Biến mới cho kết nối theo yêu cầu
bool networkModeActive = false;          // Trạng thái kết nối mạng theo yêu cầu
unsigned long networkActivationTime = 0;  // Thời điểm kích hoạt mạng
bool usingSIM = false;                   // Đang sử dụng kết nối SIM

// Biến cho trạng thái báo động
AlarmStage alarmStage = STAGE_NONE;
int beepCount = 0;         // Đếm số tiếng bíp trong mỗi chuỗi
int beepSequenceCount = 0; // Đếm số chuỗi "3 tiếng bíp" đã thực hiện
unsigned long lastBeepTime = 0;
bool notificationSent = false;
bool initialPositionSet = false;

// Khai báo hàm
void checkWifiConnection();
void activateSim();
void deactivateSim();
void checkUserPresence();
void handleMotionDetection();
void sendSmsAlert(const char* message);
void updateGpsPosition();
void activateGPS();
void deactivateGPS();
void putGPSToSleep();
void wakeGPSFromSleep();
float calculateDistance();
void setupMPU();
void setupMPULowPower();
void setupSIM();
void handleAlarm();
float getBatteryVoltage();
int calculateBatteryPercentage(float voltage);
void updateBatteryInfo();
void sendBlynkNotification(const char* message);
void handleInitialBeeps();
void handleContinuousAlarm();
void handleGpsTracking();
void sendLocationSMS();
String createDirectionsLink();
void disableRF433();
void activateRF();
void deactivateRF();
void activateNetworkMode();
void deactivateNetworkMode();
void checkNetworkTimeout();
void updateBlynkData();
void setup4GConnection();
void checkBatteryStatus();
void syncBlynk4G();

// Xử lý ngắt MPU6050
volatile bool mpuInterrupt = false;
void IRAM_ATTR mpuISR() {
    mpuInterrupt = true;
}

// Handler cho nút kiểm tra còi từ Blynk
BLYNK_WRITE(VPIN_CONTROL_TEST) {
        int pinValue = param.asInt();
        if (pinValue) {
            digitalWrite(BUZZER_PIN, HIGH);
            Serial.println("Kiểm tra còi: BẬT");
        } else {
            digitalWrite(BUZZER_PIN, LOW);
            Serial.println("Kiểm tra còi: TẮT");
        }
}

// Đồng bộ dữ liệu với Blynk qua 4G - không bao gồm dữ liệu GPS
void syncBlynk4G() {
    if (usingSIM && blynkOverSIM) {
        // Gửi dữ liệu lên Blynk qua 4G (không gửi GPS)
        updateBlynkData();

        // Xử lý dữ liệu đến
        Blynk.run();

        Serial.println("Đã đồng bộ dữ liệu với Blynk qua 4G");
    }
}

// Cập nhật dữ liệu lên Blynk (không bao gồm GPS)
void updateBlynkData() {
    if (Blynk.connected() || blynkOverSIM) {
        Blynk.virtualWrite(VPIN_BATTERY, batteryPercentage);
        Blynk.virtualWrite(VPIN_OWNER, ownerPresent ? 1 : 0);
        Blynk.virtualWrite(VPIN_ALARM_STATE, (int)alarmStage);
        Blynk.virtualWrite(VPIN_MOTION, motionDetected ? 1 : 0);
    }
}

// Hàm gửi thông báo qua Blynk
void sendBlynkNotification(const char* message) {
    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
        Blynk.notify(message);
        Serial.print("Đã gửi thông báo Blynk: ");
        Serial.println(message);
    }
}

void setupMPU() {
    Serial.println("Khởi tạo MPU6050...");

    // Thử khởi tạo MPU6050
    if (!mpu.begin()) {
        Serial.println("Không thể tìm thấy chip MPU6050");
        while (1) {
            delay(10);
        }
    }

    // Thiết lập phát hiện chuyển động
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Cấu hình ngắt
    mpu.setMotionDetectionThreshold(MOTION_THRESHOLD_G);
    mpu.setMotionDetectionDuration(20);  // 20ms
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    Serial.println("MPU6050 đã khởi tạo thành công");
}

// Cấu hình MPU6050 ở chế độ cycle để tiết kiệm điện
void setupMPULowPower() {
    Serial.println("Cấu hình MPU6050 ở chế độ tiết kiệm điện (cycle)...");

    // Thiết lập chu kỳ thức dậy (1.25Hz - thức dậy 1.25 lần/giây)
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x20); // Set cycle bit
    Wire.endTransmission(true);

    Wire.beginTransmission(0x68);
    Wire.write(0x6C); // PWR_MGMT_2 register
    Wire.write(0x37); // Set STBY_XA, STBY_YA, STBY_ZA bits to 0 (enable accel)
    Wire.endTransmission(true);

    // Thiết lập tần số thức dậy
    Wire.beginTransmission(0x68);
    Wire.write(0x1D); // ACCEL_CONFIG2 register
    Wire.write(0x09); // Set cycle rate
    Wire.endTransmission(true);

    // Vẫn giữ ngưỡng phát hiện chuyển động
    mpu.setMotionDetectionThreshold(MOTION_THRESHOLD_G);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    Serial.println("MPU6050 đã được cấu hình ở chế độ tiết kiệm điện");
}

// Khởi tạo module SIM A7682S
void setupSIM() {
    Serial.println("Khởi tạo module SIM A7682S...");

    // Bật nguồn module SIM
    digitalWrite(SIM_PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(SIM_PWRKEY_PIN, LOW);
    delay(5000);  // Cho thời gian khởi động

    // Reset module
    SerialSIM.println("AT+CRESET");
    delay(5000);

    // Kiểm tra module có phản hồi không
    SerialSIM.println("AT");
    delay(1000);

    // Đặt chế độ văn bản cho SMS
    SerialSIM.println("AT+CMGF=1");
    delay(1000);

    // Cấu hình cài đặt SMS
    SerialSIM.println("AT+CNMI=2,1,0,0,0");
    delay(1000);

    // Kiểm tra mạng
    SerialSIM.println("AT+CSQ");
    delay(1000);

    Serial.println("Module A7682S đã khởi tạo");
}

void checkWifiConnection() {
    bool previousWifiState = wifiConnected;
    wifiConnected = (WiFi.status() == WL_CONNECTED);

    // Nếu trạng thái WiFi thay đổi
    if (previousWifiState != wifiConnected) {
        if (wifiConnected) {
            Serial.println("WiFi đã kết nối");
            Serial.print("Địa chỉ IP: ");
            Serial.println(WiFi.localIP());

            // Tắt SIM nếu không ở giai đoạn 3
            if (simActive && alarmStage != STAGE_TRACKING) {
                deactivateSim();
            }

            // Khởi động Blynk nếu chưa khởi động
            if (!Blynk.connected()) {
                Blynk.connect();
            }

            blynkOverSIM = false;
            usingSIM = false;

            if (Blynk.connected()) {
                Blynk.virtualWrite(VPIN_CONNECTION, 1); // 1 = WiFi
            }
        } else {
            Serial.println("WiFi đã ngắt kết nối");

            // Chỉ kích hoạt SIM ở giai đoạn 3 hoặc nếu đang ở chế độ kết nối mạng
            if ((alarmStage == STAGE_TRACKING || networkModeActive) && !simActive) {
                activateSim();
                if (networkModeActive) {
                    setup4GConnection();
                    usingSIM = true;
                }
            }
        }
    }
}

void activateRF() {
    if (!rfPowerState) {
        digitalWrite(RF_POWER_PIN, HIGH);
        rfPowerState = true;
        delay(50);  // Đợi RF khởi động
    }
}

void deactivateRF() {
    if (rfPowerState) {
        digitalWrite(RF_POWER_PIN, LOW);
        rfPowerState = false;
    }
}

void disableRF433() {
    rfEnabled = false;
    deactivateRF(); // Tắt nguồn RF module
    Serial.println("Đã vô hiệu hóa RF433 do xe đã bị lấy trộm");
}

// Kích hoạt module SIM A7682S
void activateSim() {
    if (!simActive) {
        Serial.println("Kích hoạt module SIM A7682S");

        // Bật nguồn module SIM nếu đã tắt
        digitalWrite(SIM_PWRKEY_PIN, HIGH);
        delay(1000);
        digitalWrite(SIM_PWRKEY_PIN, LOW);
        delay(5000);  // Cho thời gian khởi động

        // Kiểm tra module
        SerialSIM.println("AT");
        delay(500);

        // Đặt chế độ văn bản cho SMS
        SerialSIM.println("AT+CMGF=1");
        delay(1000);

        simActive = true;
    }
}

// Tắt module SIM A7682S
void deactivateSim() {
    if (simActive && !networkModeActive && alarmStage != STAGE_TRACKING) {
        Serial.println("Tắt module SIM để tiết kiệm năng lượng");

        // Gửi lệnh tắt nguồn
        SerialSIM.println("AT+CPOF");
        delay(1000);

        simActive = false;
        blynkOverSIM = false;
    }
}

// Kích hoạt module GPS ATGM336H
void activateGPS() {
    if (!gpsActive) {
        Serial.println("Kích hoạt module GPS ATGM336H");

        // Bật nguồn GPS nếu đã tắt
        if (!gpsPowerState) {
            digitalWrite(GPS_POWER_PIN, HIGH);
            gpsPowerState = true;
            delay(500); // Cho thời gian ổn định nguồn
        }

        // Đánh thức GPS từ chế độ ngủ nếu cần
        wakeGPSFromSleep();

        gpsActive = true;
        gpsActivationTime = millis();  // Ghi lại thời điểm bắt đầu kích hoạt

        // Đọc dữ liệu GPS ban đầu để lấy vị trí hiện tại
        unsigned long startTime = millis();
        while (millis() - startTime < 5000 && !gps.location.isValid()) {
            while (SerialGPS.available() > 0) {
                gps.encode(SerialGPS.read());
            }
            delay(10);
        }

        if (gps.location.isValid()) {
            updateGpsPosition();
            hadValidFix = true;
            signalLost = false;
        } else {
            hadValidFix = false;
            signalLost = false;
        }
    }
}

// Tắt hoàn toàn GPS ATGM336H
void deactivateGPS() {
    if (gpsActive && alarmStage == STAGE_NONE && !networkModeActive) {
        Serial.println("Tắt hoàn toàn module GPS ATGM336H để tiết kiệm năng lượng");

        // Đặt GPS vào chế độ ngủ trước
        putGPSToSleep();

        // Sau đó tắt nguồn hoàn toàn
        digitalWrite(GPS_POWER_PIN, LOW);
        gpsPowerState = false;

        gpsActive = false;
        gpsActivationTime = 0;
        hadValidFix = false;
        signalLost = false;
    }
}

// Đặt GPS vào chế độ ngủ - ATGM336H
void putGPSToSleep() {
    if (gpsActive && gpsPowerState) {
        Serial.println("Đặt ATGM336H vào chế độ ngủ");

        // Gửi lệnh PMTK để đặt ATGM336H vào chế độ ngủ
        SerialGPS.print(GPS_SLEEP_CMD);

        // GPS vẫn được coi là "active" nhưng ở chế độ ngủ
        Serial.println("ATGM336H đã vào chế độ ngủ");
    }
}

// Đánh thức GPS từ chế độ ngủ - ATGM336H
void wakeGPSFromSleep() {
    if (gpsPowerState) {
        Serial.println("Đánh thức ATGM336H từ chế độ ngủ");

        // ATGM336H có thể được đánh thức bằng bất kỳ byte nào hoặc lệnh hot start
        SerialGPS.print(GPS_WAKE_CMD);

        // Đợi GPS khởi động
        delay(GPS_WARMUP_TIME);

        Serial.println("ATGM336H đã được đánh thức");
    }
}

// Kích hoạt chế độ kết nối mạng
void activateNetworkMode() {
    if (!networkModeActive) {
        Serial.println("Kích hoạt chế độ kết nối mạng theo yêu cầu");
        networkModeActive = true;
        networkActivationTime = millis();

        // Thử kết nối WiFi trước
        Serial.println("Đang thử kết nối WiFi...");
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);

        // Chờ kết nối WiFi với timeout
        unsigned long wifiStartTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < WIFI_TIMEOUT) {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            // WiFi đã kết nối thành công
            Serial.println("\nĐã kết nối WiFi");
            Serial.print("Địa chỉ IP: ");
            Serial.println(WiFi.localIP());
            usingSIM = false;
            wifiConnected = true;
            blynkOverSIM = false;

            // Kết nối Blynk qua WiFi
            Blynk.begin(BLYNK_AUTH, WIFI_SSID, WIFI_PASS);

            // Cập nhật trạng thái lên Blynk
            if (Blynk.connected()) {
                Blynk.virtualWrite(VPIN_CONNECTION, 1); // 1 = WiFi
                updateBlynkData();
            }
        } else {
            // WiFi không khả dụng, chuyển sang SIM
            Serial.println("\nWiFi không khả dụng, chuyển sang sử dụng 4G");
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            wifiConnected = false;

            // Kích hoạt module SIM
            activateSim();
            delay(2000);

            // Thiết lập kết nối 4G và Blynk qua SIM
            setup4GConnection();
            usingSIM = true;
        }

        // Kích hoạt GPS nếu cần xem vị trí
        if (!gpsActive) {
            activateGPS();
        }
    } else {
        // Đặt lại thời gian hết hạn nếu đã được kích hoạt
        networkActivationTime = millis();
    }
}

// Thiết lập kết nối qua 4G với Blynk - cho A7682S
void setup4GConnection() {
    // Gửi AT commands để thiết lập kết nối 4G cho A7682S
    Serial.println("Thiết lập kết nối 4G với A7682S...");

    // Reset module
    SerialSIM.println("AT+CRESET");
    delay(5000); // Chờ module khởi động lại

    // Đảm bảo module đã sẵn sàng
    SerialSIM.println("AT");
    delay(1000);

    // Kiểm tra đăng ký mạng
    SerialSIM.println("AT+CREG?");
    delay(1000);

    // Thiết lập chế độ kết nối
    SerialSIM.println("AT+CGDCONT=1,\"IP\",\"" + String(SIM_APN) + "\""); // Cấu hình APN
    delay(1000);

    // Kích hoạt kết nối dữ liệu (cho A7682S)
    SerialSIM.println("AT+CGACT=1,1");
    delay(3000);

    // Lấy địa chỉ IP
    SerialSIM.println("AT+CGPADDR=1");
    delay(1000);

    // Khởi tạo Blynk với SIM module - triển khai thực tế
    Serial.println("Kết nối Blynk qua 4G...");
    Blynk.config(BLYNK_AUTH);  // Cấu hình Auth token

    // Khởi tạo kết nối TCP cho Blynk
    SerialSIM.println("AT+CIPSTART=\"TCP\",\"blynk-cloud.com\",80");
    delay(3000);

    // Kiểm tra kết nối
    SerialSIM.println("AT+CIPSTATUS");
    delay(1000);

    // Đọc và kiểm tra phản hồi
    String response = "";
    while (SerialSIM.available()) {
        response += (char)SerialSIM.read();
    }

    if (response.indexOf("CONNECT OK") >= 0 || response.indexOf("CONNECTED") >= 0) {
        blynkOverSIM = true;
        Serial.println("Kết nối TCP đến server Blynk thành công");
        if (Blynk.connected()) {
            Blynk.virtualWrite(VPIN_CONNECTION, 2); // 2 = 4G
            updateBlynkData();
        }
    } else {
        blynkOverSIM = false;
        Serial.println("Không thể kết nối Blynk qua 4G - chỉ sử dụng SMS");
    }
}

// Hủy kết nối mạng
void deactivateNetworkMode() {
    if (networkModeActive) {
        Serial.println("Hủy chế độ kết nối mạng");

        // Ngắt kết nối Blynk
        if (Blynk.connected()) {
            Blynk.disconnect();
        }

        if (usingSIM) {
            // Đóng kết nối 4G/TCP
            if (blynkOverSIM) {
                SerialSIM.println("AT+CIPCLOSE");
                delay(1000);
            }

            SerialSIM.println("AT+CGACT=0,1");
            delay(1000);

            // Tắt module SIM nếu không cần thiết
            if (alarmStage == STAGE_NONE) {
                deactivateSim();
            }

            blynkOverSIM = false;
        } else {
            // Tắt WiFi
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            wifiConnected = false;
        }

        // Tắt GPS nếu không cần thiết
        if (gpsActive && alarmStage == STAGE_NONE) {
            deactivateGPS();
        }

        networkModeActive = false;
        usingSIM = false;
    }
}

// Kiểm tra nếu kết nối mạng đã timeout
void checkNetworkTimeout() {
    if (networkModeActive && millis() - networkActivationTime > NETWORK_SESSION_TIMEOUT) {
        Serial.println("Thời gian kết nối mạng đã hết");
        deactivateNetworkMode();
    }
}

// Cập nhật vị trí GPS (không gửi lên Blynk)
void updateGpsPosition() {
    if (gpsActive) {
        bool currentlyValid = gps.location.isValid();

        if (currentlyValid) {
            // Ghi nhớ vị trí hiện tại
            currentLat = gps.location.lat();
            currentLng = gps.location.lng();

            // Cập nhật trạng thái tín hiệu GPS
            if (!hadValidFix) {
                hadValidFix = true;
                signalLost = false;
                Serial.println("Đã nhận được tín hiệu GPS hợp lệ lần đầu tiên");
            } else if (signalLost) {
                signalLost = false;
                Serial.println("Tín hiệu GPS đã được khôi phục");
            }

            Serial.print("Vị trí GPS hiện tại: ");
            Serial.print(currentLat, 6);
            Serial.print(", ");
            Serial.println(currentLng, 6);

            if (gps.speed.isValid()) {
                Serial.print("Tốc độ: ");
                Serial.print(gps.speed.kmph());
                Serial.println(" km/h");
            }

            // Lưu vị trí này làm vị trí cuối cùng hợp lệ
            lastLat = currentLat;
            lastLng = currentLng;

            // Cập nhật lại thời gian bắt đầu kích hoạt GPS
            gpsActivationTime = millis();
        } else {
            // Kiểm tra xem đã từng có tín hiệu GPS hợp lệ chưa
            if (hadValidFix) {
                // Đánh dấu là đã mất tín hiệu
                signalLost = true;
                Serial.println("Không thể lấy vị trí GPS hiện tại - Sử dụng vị trí cuối cùng đã biết");
            } else {
                Serial.println("Không thể lấy vị trí GPS hợp lệ");
            }
        }
    }
}

float calculateDistance() {
    if (!initialPositionSet || !hadValidFix) {
        return 0;  // Không có vị trí trước đó hoặc GPS không hoạt động hoặc vị trí không hợp lệ
    }

    // Chuyển đổi vĩ độ/kinh độ từ độ sang radian
    float lat1 = lastLat * 0.01745329252;
    float lon1 = lastLng * 0.01745329252;
    float lat2 = currentLat * 0.01745329252;
    float lon2 = currentLng * 0.01745329252;

    // Công thức Haversine
    float dlon = lon2 - lon1;
    float dlat = lat2 - lat1;
    float a = pow(sin(dlat/2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = 6371000 * c;  // Bán kính Trái Đất tính bằng mét

    Serial.print("Khoảng cách di chuyển: ");
    Serial.print(distance);
    Serial.println(" mét");

    return distance;
}

// Gửi SMS qua A7682S
void sendSmsAlert(const char* message) {
    if (!simActive) {
        activateSim();
        delay(1000);
    }

    // Gửi SMS
    SerialSIM.println("AT+CMGF=1");  // Đặt chế độ văn bản
    delay(1000);

    // Đặt số điện thoại người nhận
    SerialSIM.print("AT+CMGS=\"");
    SerialSIM.print(PHONE_NUMBER);
    SerialSIM.println("\"");
    delay(1000);

    // Gửi nội dung tin nhắn
    SerialSIM.print(message);
    SerialSIM.write(26);  // Ctrl+Z để kết thúc tin nhắn
    delay(5000);  // Đợi SMS được gửi

    Serial.print("Đã gửi Cảnh báo SMS: ");
    Serial.println(message);
}

// Hàm tính toán điện áp pin thực tế
float getBatteryVoltage() {
    // Đọc điện áp analog từ chân giám sát pin
    int adcValue = analogRead(BATTERY_PIN);  // Sử dụng chân ADC đã định nghĩa

    // Chuyển đổi giá trị ADC thành điện áp
    float vOut = adcValue * (3.3 / 4095.0);

    // Tính toán điện áp pin thực tế với mạch chia áp R1=20k, R2=100k
    float voltage = vOut * 6.0;  // Vin = Vout * (R1+R2)/R1 = Vout * 6

    // Giới hạn giá trị điện áp (đề phòng đọc sai)
    if (voltage > 8.5) voltage = 8.5;
    if (voltage < 5.0) voltage = 5.0;

    return voltage;
}

// Hàm tính phần trăm pin dựa trên điện áp
int calculateBatteryPercentage(float voltage) {
    // Tính phần trăm dựa trên điện áp
    int percentage = ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100;

    // Giới hạn giá trị trong khoảng 0-100%
    if (percentage > 100) percentage = 100;
    if (percentage < 0) percentage = 0;

    return percentage;
}

// Kiểm tra pin và gửi cảnh báo
void checkBatteryStatus() {
    // Cập nhật thông tin pin
    batteryVoltage = getBatteryVoltage();
    batteryPercentage = calculateBatteryPercentage(batteryVoltage);

    Serial.print("Điện áp pin: ");
    Serial.print(batteryVoltage, 2);
    Serial.print("V (");
    Serial.print(batteryPercentage);
    Serial.println("%)");

    // Gửi thông tin lên Blynk nếu đã kết nối
    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
        Blynk.virtualWrite(VPIN_BATTERY, batteryPercentage);  // Gửi phần trăm pin
    }

    // Kiểm tra pin thấp
    if (batteryPercentage <= LOW_BATTERY_THRESHOLD && !lowBatteryAlerted) {
        // Gửi cảnh báo pin thấp qua SMS
        if (!simActive) {
            activateSim();
            delay(1000);
        }

        String batteryMessage = "CANH BAO: Pin thap, chi con ";
        batteryMessage += String(batteryPercentage);
        batteryMessage += "%. Vui long sac pin ngay.";

        sendSmsAlert(batteryMessage.c_str());

        // Đánh dấu đã gửi cảnh báo
        lowBatteryAlerted = true;

        Serial.print("Đã gửi cảnh báo pin thấp: ");
        Serial.println(batteryPercentage);
    }

    // Reset trạng thái cảnh báo pin khi pin đã được sạc trên 80%
    if (batteryPercentage >= HIGH_BATTERY_THRESHOLD && lowBatteryAlerted) {
        lowBatteryAlerted = false;
        Serial.println("Pin đã được sạc đầy, reset trạng thái cảnh báo pin thấp");
    }
}

void checkUserPresence() {
    // Nếu RF đã bị vô hiệu hóa, không kiểm tra
    if (!rfEnabled) return;

    // Đảm bảo RF đang bật
    if (!rfPowerState) {
        activateRF();
    }

    bool previousOwnerState = ownerPresent;
    bool detectedRF = false;
    bool detectedNetworkActivation = false;

    // Kiểm tra thẻ RF trong một khoảng thời gian ngắn
    unsigned long rfStartTime = millis();
    while (millis() - rfStartTime < RF_ACTIVE_DURATION) {
        if (rfReceiver.available()) {
            unsigned long receivedCode = rfReceiver.getReceivedValue();
            rfReceiver.resetAvailable();

            if (receivedCode == USER_RF_CODE) {
                // Mã 1: Nhận diện người dùng
                ownerPresent = true;
                lastRfCheck = millis();
                detectedRF = true;
                Serial.println("Đã phát hiện mã RF của chủ sở hữu");
            }
            else if (receivedCode == NETWORK_ACTIVATE_CODE) {
                // Mã 2: Kích hoạt kết nối mạng
                detectedNetworkActivation = true;
                Serial.println("Đã phát hiện mã kích hoạt kết nối mạng");
            }
        }
        delay(10);  // Đợi chút để không chiếm CPU
    }

    // Nếu không phát hiện RF và đã lâu từ lần phát hiện cuối cùng
    if (!detectedRF && millis() - lastRfCheck > RF_CHECK_INTERVAL * 3) {
        ownerPresent = false;
    }

    // Nếu phát hiện tín hiệu kích hoạt mạng
    if (detectedNetworkActivation) {
        // Kích hoạt chế độ kết nối mạng
        activateNetworkMode();
    }

    // Nếu trạng thái hiện diện của chủ sở hữu thay đổi
    if (previousOwnerState != ownerPresent) {
        Serial.print("Sự hiện diện của chủ sở hữu: ");
        Serial.println(ownerPresent ? "Đã phát hiện" : "Không phát hiện");

        if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
            Blynk.virtualWrite(VPIN_OWNER, ownerPresent ? 1 : 0);
        }

        // Nếu chủ sở hữu hiện đang có mặt, dừng mọi cảnh báo đang hoạt động
        if (ownerPresent && alarmStage != STAGE_NONE && alarmStage != STAGE_TRACKING) {
            alarmStage = STAGE_NONE;
            beepCount = 0;
            beepSequenceCount = 0;
            sequenceEndTime = 0;
            notificationSent = false;
            initialPositionSet = false;
            stage2GpsUpdateCount = 0;
            digitalWrite(BUZZER_PIN, LOW);

            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_ALARM_STATE, 0);
                Blynk.virtualWrite(VPIN_MOTION, 0);
            }

            // Tắt GPS nếu đang hoạt động
            if (gpsActive && alarmStage != STAGE_TRACKING) {
                deactivateGPS();
            }
        }
    }
}

void handleMotionDetection() {
    // Đọc giá trị gia tốc
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Tính độ lớn của vector gia tốc
    float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                a.acceleration.y * a.acceleration.y +
                                a.acceleration.z * a.acceleration.z);

    // Phát hiện chuyển động đáng kể (không bao gồm trọng lực)
    float netAccel = abs(accelMagnitude - 9.8);

    if (netAccel > MOTION_THRESHOLD && (rfEnabled && !ownerPresent)) {
        // Có chuyển động
        if (alarmStage == STAGE_NONE) {
            // Chuyển sang giai đoạn 1 lần đầu tiên
            Serial.println("Đã phát hiện chuyển động lần đầu!");
            alarmStage = STAGE_INITIAL;
            beepCount = 0;
            beepSequenceCount = 0;
            lastBeepTime = millis();
            sequenceEndTime = 0;

            // Cập nhật trạng thái phát hiện chuyển động lên Blynk
            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_MOTION, 1);
                Blynk.virtualWrite(VPIN_ALARM_STATE, 1); // Trạng thái báo động 1
            }
        }
        else if (alarmStage == STAGE_INITIAL && sequenceEndTime > 0) {
            // Nếu đã hoàn thành một chuỗi bíp và đang trong thời gian chờ 2 phút
            // Phát hiện chuyển động mới trong thời gian này, tiếp tục chuỗi bíp tiếp theo
            if (beepSequenceCount < 3) {
                Serial.println("Tiếp tục phát hiện chuyển động, bắt đầu chuỗi bíp tiếp theo");
                beepCount = 0;
                lastBeepTime = millis();
                sequenceEndTime = 0; // Reset thời gian chờ
            }
        }
        else if (alarmStage == STAGE_INITIAL && beepSequenceCount >= 3) {
            // Đã đủ 3 chuỗi bíp và vẫn phát hiện chuyển động, chuyển sang giai đoạn 2
            Serial.println("Chuyển sang GIAI ĐOẠN 2: Báo động liên tục");
            alarmStage = STAGE_WARNING;
            notificationSent = false;
            stage2GpsUpdateCount = 0;

            // Kích hoạt GPS ở giai đoạn 2 để bắt đầu theo dõi vị trí
            activateGPS();

            // Kích hoạt kết nối mạng nếu chưa
            if (!networkModeActive) {
                activateNetworkMode();
            }

            // Lưu vị trí ban đầu để so sánh khoảng cách
            if (gps.location.isValid()) {
                lastLat = gps.location.lat();
                lastLng = gps.location.lng();
                initialPositionSet = true;
                hadValidFix = true;
                signalLost = false;
                Serial.println("Đã lưu vị trí ban đầu");
            } else {
                initialPositionSet = false;
                hadValidFix = false;
                signalLost = false;
            }

            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_ALARM_STATE, 2); // Trạng thái báo động 2
            }
        }
    }
}

// Xử lý 3 lần 3 tiếng bíp ban đầu (giai đoạn 1)
void handleInitialBeeps() {
    unsigned long currentMillis = millis();

    // Xử lý tiếng bíp
    if (beepCount < 3) {
        if (currentMillis - lastBeepTime < INITIAL_BEEP_DURATION) {
            // Bật còi trong thời gian bíp
            digitalWrite(BUZZER_PIN, HIGH);
        }
        else if (currentMillis - lastBeepTime < (INITIAL_BEEP_DURATION + BEEP_INTERVAL)) {
            // Tắt còi trong khoảng thời gian giữa các tiếng bíp
            digitalWrite(BUZZER_PIN, LOW);
        }
        else {
            // Kết thúc một chu kỳ bíp
            beepCount++;
            lastBeepTime = currentMillis;
            Serial.print("Bíp #");
            Serial.println(beepCount);
        }
    }
    else {
        // Đã bíp đủ 3 lần trong một chuỗi
        digitalWrite(BUZZER_PIN, LOW);

        // Đánh dấu kết thúc chuỗi bíp và bắt đầu đếm thời gian 2 phút
        if (sequenceEndTime == 0) {
            beepSequenceCount++;
            sequenceEndTime = currentMillis;
            Serial.print("Hoàn thành chuỗi bíp #");
            Serial.print(beepSequenceCount);
            Serial.println(", bắt đầu thời gian chờ 2 phút");

            // Kiểm tra xem đã đủ 3 chuỗi bíp chưa
            if (beepSequenceCount >= 3) {
                // Đã đủ 3 chuỗi bíp, kiểm tra chuyển động để quyết định có chuyển sang giai đoạn 2 không
                sensors_event_t a, g, temp;
                mpu.getEvent(&a, &g, &temp);
                float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                            a.acceleration.y * a.acceleration.y +
                                            a.acceleration.z * a.acceleration.z);
                float netAccel = abs(accelMagnitude - 9.8);

                if (netAccel > MOTION_THRESHOLD) {
                    alarmStage = STAGE_WARNING;
                    notificationSent = false;
                    sequenceEndTime = 0;
                    stage2GpsUpdateCount = 0;
                    Serial.println("Chuyển sang GIAI ĐOẠN 2: Báo động liên tục");

                    // Kích hoạt kết nối mạng nếu chưa
                    if (!networkModeActive) {
                        activateNetworkMode();
                    }

                    // Kích hoạt GPS ở giai đoạn 2 để bắt đầu theo dõi vị trí
                    activateGPS();

                    // Lưu vị trí ban đầu để so sánh khoảng cách
                    if (gps.location.isValid()) {
                        lastLat = gps.location.lat();
                        lastLng = gps.location.lng();
                        initialPositionSet = true;
                        hadValidFix = true;
                        signalLost = false;
                        Serial.println("Đã lưu vị trí ban đầu");
                    } else {
                        initialPositionSet = false;
                        hadValidFix = false;
                        signalLost = false;
                    }

                    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                        Blynk.virtualWrite(VPIN_ALARM_STATE, 2); // Trạng thái báo động 2
                    }
                }
            }
        }
    }
}

// Xử lý báo động liên tục (giai đoạn 2)
void handleContinuousAlarm() {
    unsigned long currentMillis = millis();

    // Báo động ngắt quãng liên tục
    if (currentMillis % 1000 < 500) {
        digitalWrite(BUZZER_PIN, HIGH);
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }

    // Gửi thông báo nếu chưa gửi
    if (!notificationSent) {
        // Gửi cảnh báo qua Blynk
        if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
            sendBlynkNotification("CẢNH BÁO: Phát hiện có người đang động vào xe!");
        }

        // Gửi SMS
        if (!simActive) {
            activateSim();
            delay(1000);
        }
        sendSmsAlert("CANH BAO: Phat hien co nguoi dang dong vao xe cua ban!");

        notificationSent = true;
    }
}

// Xử lý theo dõi GPS (giai đoạn 3) - Không gửi dữ liệu qua Blynk ở giai đoạn này
void handleGpsTracking() {
    // Không làm gì với còi ở giai đoạn 3 (còi đã tắt)

    // Đảm bảo SIM đang hoạt động trong giai đoạn này
    if (!simActive) {
        activateSim();
    }

    // Đảm bảo GPS đang hoạt động
    if (!gpsActive) {
        activateGPS();
    }

    // Chỉ đảm bảo các kết nối cần thiết cho SMS, không gửi dữ liệu vị trí qua Blynk
}

// Gửi vị trí qua SMS
void sendLocationSMS() {
    if (!simActive) {
        activateSim();
        delay(1000);
    }

    if (gpsActive && (gps.location.isValid() || (hadValidFix && signalLost))) {
        // Tạo tin nhắn SMS với thông tin vị trí
        String smsText;

        if (gps.location.isValid()) {
            smsText = "Vi tri xe cua ban: ";
            smsText += "https://maps.google.com/maps?q=";
            smsText += String(gps.location.lat(), 6);
            smsText += ",";
            smsText += String(gps.location.lng(), 6);

            if (gps.speed.isValid()) {
                smsText += " - Toc do: ";
                smsText += String(gps.speed.kmph(), 1);
                smsText += " km/h";
            }
        }
        else if (hadValidFix && signalLost) {
            smsText = "Vi tri cuoi cung: ";
            smsText += "https://maps.google.com/maps?q=";
            smsText += String(lastLat, 6);
            smsText += ",";
            smsText += String(lastLng, 6);
            smsText += " (mat tin hieu GPS)";
        }

        // Thêm thông tin pin
        smsText += " - Pin: ";
        smsText += String(batteryPercentage);
        smsText += "%";

        // Gửi SMS
        sendSmsAlert(smsText.c_str());
    } else {
        // Không có dữ liệu GPS
        sendSmsAlert("Khong the xac dinh vi tri xe. Khong co tin hieu GPS.");
    }
}

void setup() {
    // Khởi tạo giao tiếp serial
    Serial.begin(115200);
    SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    SerialSIM.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

    Serial.println("\n--------------------------------------");
    Serial.println("HỆ THỐNG CHỐNG TRỘM XE ESP32");
    Serial.println("--------------------------------------");
    Serial.println("Phiên bản: " + String(VERSION));
    Serial.println("Người phát triển: " + String(DEVELOPER));
    Serial.println("Hardware: ESP32 + ATGM336H GPS + A7682S 4G");
    Serial.println("--------------------------------------");

    // Khởi tạo I2C với các chân tùy chỉnh
    Wire.begin(I2C_SDA, I2C_SCL);

    // Khởi tạo chân
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SIM_PWRKEY_PIN, OUTPUT);
    pinMode(RF_POWER_PIN, OUTPUT);
    pinMode(GPS_POWER_PIN, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(SIM_PWRKEY_PIN, LOW);
    digitalWrite(RF_POWER_PIN, LOW);  // RF ban đầu tắt
    digitalWrite(GPS_POWER_PIN, LOW); // GPS ban đầu tắt

    // Thiết lập ngắt MPU6050
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpuISR, RISING);

    // Khởi tạo bộ thu RF
    rfReceiver.enableReceive(RF_RECEIVER_PIN);

    // Khởi tạo MPU6050
    setupMPU();

    // Cấu hình MPU ở chế độ tiết kiệm điện
    setupMPULowPower();

    // Ban đầu, tắt WiFi để tiết kiệm pin
    WiFi.mode(WIFI_OFF);

    // Không kích hoạt module SIM và GPS ở trạng thái bình thường
    simActive = false;
    gpsActive = false;
    gpsPowerState = false;
    rfEnabled = true;
    rfPowerState = false;
    networkModeActive = false;
    usingSIM = false;
    blynkOverSIM = false;
    hadValidFix = false;
    signalLost = false;
    lowBatteryAlerted = false;

    // Thiết lập Timer cho Blynk
    timer.setInterval(BLYNK_4G_SYNC_INTERVAL, syncBlynk4G); // Đồng bộ Blynk qua 4G mỗi 10 giây

    // Đọc điện áp pin ban đầu và tính phần trăm
    batteryVoltage = getBatteryVoltage();
    batteryPercentage = calculateBatteryPercentage(batteryVoltage);

    Serial.print("Điện áp pin: ");
    Serial.print(batteryVoltage, 2);
    Serial.print("V (");
    Serial.print(batteryPercentage);
    Serial.println("%)");

    Serial.println("Hệ thống đã khởi tạo thành công");
    Serial.println("Sẵn sàng hoạt động ở chế độ tiết kiệm năng lượng");
    Serial.println("--------------------------------------");

    // Tiếng bíp ngắn để chỉ báo hệ thống đã sẵn sàng
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
    // Biến thời gian hiện tại
    unsigned long currentMillis = millis();

    // Kiểm tra trạng thái pin định kỳ
    if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
        checkBatteryStatus();
        lastBatteryCheck = currentMillis;
    }

    // Kiểm tra trạng thái kết nối mạng
    if (networkModeActive) {
        // Kiểm tra timeout
        checkNetworkTimeout();

        // Chạy Blynk dựa trên loại kết nối
        if (Blynk.connected() && !usingSIM) {
            // Sử dụng WiFi
            Blynk.run();
            wifiConnected = true;
        }
        else if (usingSIM && blynkOverSIM) {
            // Sử dụng 4G - thực hiện timer để định kỳ đồng bộ
            timer.run();

            // Đồng bộ Blynk qua 4G với tần suất thấp hơn
            if (currentMillis - lastBlynk4GSync >= BLYNK_4G_SYNC_INTERVAL) {
                syncBlynk4G();
                lastBlynk4GSync = currentMillis;
            }
        }
        else {
            wifiConnected = (WiFi.status() == WL_CONNECTED);
        }
    } else {
        // Không trong chế độ kết nối mạng, đảm bảo WiFi tắt để tiết kiệm pin
        if (WiFi.getMode() != WIFI_OFF && alarmStage != STAGE_TRACKING) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            wifiConnected = false;
        }
    }

    // Điều khiển chu kỳ hoạt động của RF433
    if (rfEnabled && currentMillis - lastRfActivation >= RF_CHECK_INTERVAL) {
        // Bật RF
        activateRF();

        // Kiểm tra sự hiện diện của chủ và lệnh kích hoạt mạng
        checkUserPresence();

        // Tắt RF sau khi kiểm tra
        deactivateRF();

        lastRfActivation = currentMillis;
    }

    // Xử lý phát hiện chuyển động từ MPU6050
    if (mpuInterrupt) {
        handleMotionDetection();
        mpuInterrupt = false;
    }

    // Xử lý GPS khi nó hoạt động
    if (gpsActive) {
        // Đọc dữ liệu GPS khi có sẵn
        while (SerialGPS.available() > 0) {
            gps.encode(SerialGPS.read());
        }

        // Kiểm tra trạng thái tín hiệu GPS
        bool currentlyValid = gps.location.isValid();

        // Phát hiện khi chuyển từ có tín hiệu sang mất tín hiệu
        if (hadValidFix && !currentlyValid) {
            signalLost = true;
            Serial.println("Phát hiện mất tín hiệu GPS!");
        }

        // Xử lý GPS tùy theo giai đoạn báo động
        if (alarmStage == STAGE_TRACKING) {
            // Cập nhật thường xuyên hơn trong chế độ theo dõi
            if (currentMillis - lastGpsUpdate >= GPS_ALARM_INTERVAL) {
                updateGpsPosition();
                lastGpsUpdate = currentMillis;

                // Gửi vị trí qua SMS
                sendLocationSMS();
            }
        } else if (alarmStage == STAGE_WARNING) {
            // Ở giai đoạn 2, cập nhật vị trí mỗi 30 giây để kiểm tra khoảng cách
            if (currentMillis - lastGpsUpdate >= GPS_STAGE2_INTERVAL) {
                updateGpsPosition();
                lastGpsUpdate = currentMillis;

                // Tính khoảng cách di chuyển
                if (initialPositionSet) {
                    float distance = calculateDistance();

                    // Kiểm tra quá 10m để chuyển sang giai đoạn 3
                    if (distance > DISTANCE_THRESHOLD) {
                        alarmStage = STAGE_TRACKING;
                        Serial.println("Chuyển sang GIAI ĐOẠN 3: Theo dõi GPS");

                        // Tắt còi ở giai đoạn 3
                        digitalWrite(BUZZER_PIN, LOW);

                        // Vô hiệu hóa RF433 ở giai đoạn 3
                        disableRF433();

                        // Kích hoạt kết nối mạng nếu chưa
                        if (!networkModeActive) {
                            activateNetworkMode();
                        }

                        // Gửi thông báo
                        if (Blynk.connected() || blynkOverSIM) {
                            Blynk.virtualWrite(VPIN_ALARM_STATE, 3); // Trạng thái báo động 3
                            sendBlynkNotification("XE ĐANG DI CHUYỂN! Đang theo dõi GPS...");
                        }

                        // Kích hoạt SIM và gửi SMS
                        activateSim();
                        sendLocationSMS();
                    } else {
                        // Tăng bộ đếm cập nhật GPS
                        stage2GpsUpdateCount++;

                        // Sau 3 lần cập nhật mà chưa di chuyển quá 10m, quay lại giai đoạn 1
                        if (stage2GpsUpdateCount >= 3) {
                            Serial.println("Sau 3 lần cập nhật GPS, xe chưa di chuyển quá 10m. Quay lại giai đoạn 1.");
                            alarmStage = STAGE_INITIAL;
                            beepCount = 0;
                            beepSequenceCount = 0;
                            stage2GpsUpdateCount = 0;

                            if (Blynk.connected() || blynkOverSIM) {
                                Blynk.virtualWrite(VPIN_ALARM_STATE, 1);
                            }
                        }
                    }
                }
            }
        } else {
            // Cập nhật bình thường nếu GPS đang hoạt động
            if (currentMillis - lastGpsUpdate >= GPS_UPDATE_INTERVAL) {
                updateGpsPosition();
                lastGpsUpdate = currentMillis;
            }
        }

        // Kiểm tra xem GPS có timeout không
        if (gpsActivationTime > 0 &&
            currentMillis - gpsActivationTime > GPS_TIMEOUT &&
            !gps.location.isValid() && !hadValidFix) {

            Serial.println("Timeout chờ đợi tín hiệu GPS hợp lệ");

            // Nếu đang ở giai đoạn báo động, thông báo rằng không thể xác định vị trí
            if (alarmStage == STAGE_TRACKING) {
                if (simActive) {
                    sendSmsAlert("Khong the xac dinh vi tri xe. He thong khong the nhan duoc tin hieu GPS.");
                }

                if (Blynk.connected() || blynkOverSIM) {
                    sendBlynkNotification("Không thể xác định vị trí xe. Hệ thống không thể nhận được tín hiệu GPS.");
                }
            }
        }
    }

    // Xử lý các giai đoạn báo động
    switch (alarmStage) {
        case STAGE_INITIAL:
            handleInitialBeeps();
            break;

        case STAGE_WARNING:
            handleContinuousAlarm();
            break;

        case STAGE_TRACKING:
            handleGpsTracking();
            break;

        default:
            // Kiểm tra xem có cần reset sau khi hoàn thành chuỗi bíp không
            if (beepSequenceCount > 0 && sequenceEndTime > 0 &&
                currentMillis - sequenceEndTime > SEQUENCE_WAIT_TIME) {

                // Nếu không có chuyển động trong 2 phút sau khi hoàn thành chuỗi bíp, reset
                Serial.println("Không phát hiện chuyển động sau 2 phút, reset hệ thống");
                alarmStage = STAGE_NONE;
                beepCount = 0;
                beepSequenceCount = 0;
                sequenceEndTime = 0;

                if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                    Blynk.virtualWrite(VPIN_MOTION, 0);
                    Blynk.virtualWrite(VPIN_ALARM_STATE, 0);
                }

                // Tắt GPS nếu đang hoạt động
                if (gpsActive) {
                    deactivateGPS();
                }
            }
            break;
    }

    // Xử lý dữ liệu từ module SIM
    if (simActive) {
        while (SerialSIM.available()) {
            char c = SerialSIM.read();
            Serial.write(c);  // Chuyển tiếp đến serial chính để gỡ lỗi
        }
    }

    // Cơ chế ngủ sâu để tiết kiệm năng lượng khi thích hợp
    if (alarmStage == STAGE_NONE && !networkModeActive) {
        // Đảm bảo RF đã tắt trước khi ngủ
        deactivateRF();

        // Đảm bảo GPS đã đi vào chế độ ngủ
        if (gpsActive) {
            putGPSToSleep();
        }

        // Ngủ nhẹ để tiết kiệm năng lượng nhưng vẫn thức dậy khi có ngắt
        esp_sleep_enable_timer_wakeup(5000000); // 5 giây
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_39, 1); // Thức dậy khi có ngắt từ MPU

        Serial.println("Vào chế độ light sleep để tiết kiệm pin...");
        esp_light_sleep_start();

        // Kiểm tra lý do thức dậy
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
        if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
            // Thức dậy do MPU6050 phát hiện chuyển động
            Serial.println("Thức dậy do phát hiện chuyển động!");
            handleMotionDetection();
        } else {
            Serial.println("Thức dậy theo lịch trình");
        }
    }
}

// Cập nhật lần cuối: 2025-05-29 07:41:30
// Người phát triển: nguongthienTieu
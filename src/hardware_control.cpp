#include "hardware_control.h"
#include "gps_manager.h"
#include "power_management.h"
#include "network_manager.h"
#include "security_system.h"
#include "config.h"

void initializePins() {
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(RF_POWER_PIN, OUTPUT);
    pinMode(GPS_POWER_PIN, OUTPUT);
    pinMode(SIM_PWRKEY_PIN, OUTPUT);
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    pinMode(BATTERY_PIN, INPUT);

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(RF_POWER_PIN, LOW);
    digitalWrite(GPS_POWER_PIN, LOW);
    digitalWrite(SIM_PWRKEY_PIN, LOW);

    // Khởi tạo UART cho GPS và SIM với chân tùy chỉnh
    SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    SerialSIM.begin(115200, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

    Serial.println("Đã khởi tạo chân I/O và UART");
}

void initializeI2C() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("Đã khởi tạo I2C");
}

void initializeMPU6050() {
    if (mpu.begin(0x68, &Wire)) {
        // Cấu hình MPU6050
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

        // Thiết lập ngắt
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpuISR, RISING);

        Serial.println("MPU6050 đã được khởi tạo thành công");
    } else {
        Serial.println("LỖI: Không thể khởi tạo MPU6050!");
    }
}

void initializeRF433() {
    rfReceiver.enableReceive(RF_DATA_PIN);
    Serial.println("Module RF433MHz đã được khởi tạo");
}

void setupTimers() {
    timer.setInterval(10000, checkWifiConnection);
    timer.setInterval(BATTERY_CHECK_INTERVAL, checkBatteryStatus);
    timer.setInterval(RF_CHECK_INTERVAL, checkUserPresence);
    timer.setInterval(MOTION_CHECK_INTERVAL, handleMotionDetection);
    timer.setInterval(BLYNK_SYNC_INTERVAL, syncBlynk4G);

    Serial.println("Đã thiết lập các timer");
}

void runTimers() {
    timer.run();
}

void IRAM_ATTR mpuISR() {
    mpuInterrupt = true;
}

void handleMpuInterrupt() {
    if (mpuInterrupt) {
        mpuInterrupt = false;
        handleMotionDetection();
    }
}

void activateRF() {
    if (!rfPowerState) {
        digitalWrite(RF_POWER_PIN, HIGH);
        rfPowerState = true;
        delay(50);  // Đợi RF khởi động
        Serial.println("Đã kích hoạt nguồn RF433MHz");
    }
}

void deactivateRF() {
    if (rfPowerState) {
        digitalWrite(RF_POWER_PIN, LOW);
        rfPowerState = false;
        Serial.println("Đã tắt nguồn RF433MHz");
    }
}

void disableRF433() {
    rfEnabled = false;
    deactivateRF();
    Serial.println("Đã vô hiệu hóa RF433 do xe đã bị lấy trộm");
}

void acknowledgeUserPresence() {
    // Phát ra 2 tiếng bíp ngắn để xác nhận đã nhận diện người dùng
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Đã xác nhận nhận diện chủ sở hữu với âm thanh");
}

void handleInitialBeeps() {
    // Phát 3 tiếng bíp để xác nhận hệ thống đã khởi động
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }

    // Kiểm tra các thiết bị cần thiết
    Serial.println("Kiểm tra các thiết bị:");

    // Kiểm tra MPU6050
    Serial.print("- MPU6050: ");
    if (mpu.begin(0x68, &Wire)) {
        Serial.println("OK");
    } else {
        Serial.println("KHÔNG TÌM THẤY");
        // Phát tiếng bíp dài nếu không tìm thấy MPU6050
        digitalWrite(BUZZER_PIN, HIGH);
        delay(1000);
        digitalWrite(BUZZER_PIN, LOW);
    }

    // Kiểm tra module RF
    Serial.print("- Module RF433: ");
    activateRF();
    delay(500);
    Serial.println("Đã kích hoạt");

    // Kiểm tra GPS
    Serial.print("- GPS ATGM336H: ");
    activateGPS();
    unsigned long gpsStartTime = millis();
    bool gpsResponded = false;

    while (millis() - gpsStartTime < 2000) {
        if (SerialGPS.available()) {
            gpsResponded = true;
            break;
        }
        delay(10);
    }

    if (gpsResponded) {
        Serial.println("OK");
    } else {
        Serial.println("KHÔNG PHẢN HỒI");
        // Phát 2 tiếng bíp dài nếu GPS không phản hồi
        for (int i = 0; i < 2; i++) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }
    }

    // Chỉ kích hoạt SIM khi cần
    Serial.println("- Module SIM A7682S: Chưa kích hoạt (sẽ kích hoạt khi cần)");

    // Phát 2 tiếng bíp để xác nhận hoàn tất kiểm tra
    for (int i = 0; i < 2; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
    }
}

void blinkStatusLed(unsigned long currentMillis) {
    if (currentMillis % 1000 < 100) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
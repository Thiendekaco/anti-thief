#include "power_management.h"
#include "network_manager.h"
#include "gps_manager.h"
#include "utils.h"

// Hàm tính toán điện áp pin thực tế - pin 2x18650 với module bảo vệ HX-2S-D20
float getBatteryVoltage() {
    // Đọc điện áp analog từ chân giám sát pin
    int adcValue = analogRead(BATTERY_PIN);

    // ESP32 ADC là 12-bit (0-4095)
    // Chuyển đổi giá trị ADC thành điện áp (3.3V reference)
    float vOut = adcValue * (3.3 / 4095.0);

    // Tính toán điện áp pin thực tế với mạch chia áp R1=20k, R2=100k
    // Vin = Vout * (R1+R2)/R1 = Vout * 6
    float voltage = vOut * 6.0;

    // Giới hạn giá trị điện áp (đề phòng đọc sai)
    if (voltage > 8.5) voltage = 8.5;
    if (voltage < 5.0) voltage = 5.0;

    Serial.print("ADC: ");
    Serial.print(adcValue);
    Serial.print(", Điện áp đọc: ");
    Serial.print(voltage, 2);
    Serial.println("V");

    return voltage;
}

// Hàm tính phần trăm pin dựa trên điện áp - pin 2x18650
int calculateBatteryPercentage(float voltage) {
    // Tính phần trăm dựa trên điện áp
    // Pin 2x18650 mắc nối tiếp: 8.4V (đầy) - 6.0V (cạn)
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

// Kích hoạt module SIM A7682S
void activateSim() {
    if (!simActive) {
        Serial.println("Kích hoạt module SIM A7682S");

        // Bật nguồn module SIM với PWRKEY
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
        Serial.println("Module SIM A7682S đã kích hoạt");
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

        Serial.println("Module SIM A7682S đã tắt");
    }
}

// Kiểm tra lệnh reset thủ công
void checkManualReset() {
    if (manualReset) {
        Serial.println("Đang xử lý lệnh reset thủ công...");
        manualReset = false;  // Reset biến trạng thái
        // Các bước reset đã được thực hiện trong checkUserPresence()
    }
}
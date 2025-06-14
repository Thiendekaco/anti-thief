#include "utils.h"
#include "hardware_control.h"
#include "config.h"
#include "power_management.h"
#include <time.h>


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

// ----------------------
// Các tiện ích về thời gian
// ----------------------

// Chuyển đổi thời gian trong mili giây thành chuỗi đọc được
String formatTimeInterval(unsigned long ms) {
    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;

    String result = "";
    if (days > 0) {
        result += String(days) + " ngày ";
    }

    hours %= 24;
    if (hours > 0 || days > 0) {
        result += String(hours) + " giờ ";
    }

    minutes %= 60;
    if (minutes > 0 || hours > 0 || days > 0) {
        result += String(minutes) + " phút ";
    }

    seconds %= 60;
    result += String(seconds) + " giây";

    return result;
}

// Ghi thời gian chạy vào Serial
void logRuntime() {
    static unsigned long lastLog = 0;
    unsigned long now = millis();

    // Log mỗi giờ
    if (now - lastLog >= 3600000 || lastLog == 0) {
        Serial.print("Thời gian hoạt động: ");
        Serial.println(formatTimeInterval(now));
        lastLog = now;
    }
}

// ----------------------
// Các tiện ích về chuỗi
// ----------------------

// Tạo chuỗi tin nhắn vị trí GPS theo định dạng Google Maps
String createGoogleMapsLink(float lat, float lng) {
    String link = "http://maps.google.com/maps?q=";
    link += String(lat, 6);
    link += ",";
    link += String(lng, 6);
    return link;
}

// Tạo chuỗi thông báo pin theo phần trăm còn lại
String createBatteryMessage(int percentage) {
    String status;
    if (percentage <= 10) {
        status = "KHẨN CẤP: Pin cực thấp, chỉ còn ";
    } else if (percentage <= 20) {
        status = "CẢNH BÁO: Pin thấp, chỉ còn ";
    } else if (percentage <= 50) {
        status = "LƯU Ý: Pin còn ";
    } else {
        status = "THÔNG BÁO: Pin còn ";
    }

    status += String(percentage) + "%";

    if (percentage <= 20) {
        status += ". Vui lòng sạc pin ngay!";
    } else if (percentage <= 50) {
        status += ". Nên sạc pin khi có điều kiện.";
    }

    return status;
}

// ----------------------
// Các tiện ích về đơn vị
// ----------------------

// Chuyển đổi từ m/s sang km/h
float msToKmh(float speedMs) {
    return speedMs * 3.6;
}

// Chuyển đổi từ km/h sang m/s
float kmhToMs(float speedKmh) {
    return speedKmh / 3.6;
}

// ----------------------
// Các tiện ích về ghi nhật ký
// ----------------------

// Xóa bỏ định nghĩa enum LogLevel ở đây vì đã có trong utils.h

// Ghi log với thời gian và mức độ nghiêm trọng
void logMessage(LogLevel level, const char* message) {
    String prefix;
    switch (level) {
        case LOG_DEBUG:
            prefix = "[DEBUG] ";
            break;
        case LOG_INFO:
            prefix = "[INFO] ";
            break;
        case LOG_WARNING:
            prefix = "[WARN] ";
            break;
        case LOG_ERROR:
            prefix = "[ERROR] ";
            break;
        case LOG_CRITICAL:
            prefix = "[CRIT] ";
            break;
    }

    unsigned long currentMillis = millis();
    unsigned long seconds = currentMillis / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;

    minutes %= 60;
    seconds %= 60;

    char timeStr[12];
    sprintf(timeStr, "%02lu:%02lu:%02lu", hours, minutes, seconds);

    Serial.print(timeStr);
    Serial.print(" ");
    Serial.print(prefix);
    Serial.println(message);
}

// ----------------------
// Tiện ích kiểm tra và chẩn đoán
// ----------------------

// Thực hiện chẩn đoán hệ thống và ghi kết quả
void performSystemDiagnostics() {
    // Ghi thông tin phiên bản
    logMessage(LOG_INFO, "Bắt đầu chẩn đoán hệ thống");
    logMessage(LOG_INFO, ("Phiên bản: " + String(APP_VERSION)).c_str());

    // Kiểm tra điện áp
    float voltage = getBatteryVoltage();
    int percentage = calculateBatteryPercentage(voltage);

    String batteryInfo = "Pin: " + String(voltage, 2) + "V (" + String(percentage) + "%)";
    logMessage(LOG_INFO, batteryInfo.c_str());

    // Kiểm tra trạng thái RF
    logMessage(LOG_INFO, rfEnabled ? "RF433: Hoạt động" : "RF433: Vô hiệu hóa");

    // Kiểm tra trạng thái GPS
    if (gpsActive) {
        if (hadValidFix) {
            String gpsInfo = "GPS: Hoạt động, vị trí: " + String(currentLat, 6) + ", " + String(currentLng, 6);
            logMessage(LOG_INFO, gpsInfo.c_str());
        } else {
            logMessage(LOG_WARNING, "GPS: Hoạt động nhưng chưa có tín hiệu hợp lệ");
        }
    } else {
        logMessage(LOG_INFO, "GPS: Không hoạt động");
    }

    // Kiểm tra trạng thái SIM
    logMessage(LOG_INFO, simActive ? "SIM: Hoạt động" : "SIM: Không hoạt động");

    // Kiểm tra trạng thái Blynk
    if (networkModeActive) {
        if (Blynk.connected()) {
            logMessage(LOG_INFO, "Blynk: Kết nối qua WiFi");
        } else if (blynkOverSIM) {
            logMessage(LOG_INFO, "Blynk: Kết nối qua 4G");
        } else {
            logMessage(LOG_WARNING, "Blynk: Không kết nối mặc dù chế độ mạng đang hoạt động");
        }
    } else {
        logMessage(LOG_INFO, "Blynk: Không kết nối (chế độ mạng không hoạt động)");
    }

    // Kiểm tra trạng thái báo động
    String alarmInfo = "Trạng thái báo động: ";
    switch (alarmStage) {
        case STAGE_NONE:
            alarmInfo += "Không có báo động";
            break;
        case STAGE_WARNING:
            alarmInfo += "Cảnh báo (Giai đoạn 1)";
            break;
        case STAGE_ALERT:
            alarmInfo += "Báo động (Giai đoạn 2)";
            break;
        case STAGE_TRACKING:
            alarmInfo += "Theo dõi (Giai đoạn 3)";
            break;
    }
    logMessage(LOG_INFO, alarmInfo.c_str());

    // Kiểm tra trạng thái chủ sở hữu
    logMessage(LOG_INFO, ownerPresent ? "Chủ sở hữu: Hiện diện" : "Chủ sở hữu: Vắng mặt");

    logMessage(LOG_INFO, "Chẩn đoán hệ thống hoàn tất");
}

// Handler cho các sự kiện Blynk
void handleBlynkEvents() {
    // Các sự kiện Blynk được xử lý thông qua các macro BLYNK_WRITE
}
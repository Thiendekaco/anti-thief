#include "gps_manager.h"
#include "config.h"
#include "utils.h"

// Kích hoạt module GPS ATGM336H
void activateGPS() {
    if (!gpsActive) {
        Serial.println("Kích hoạt module GPS ATGM336H");

        // Bật nguồn GPS thông qua LDO với chân EN
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

        Serial.println("Module GPS ATGM336H đã kích hoạt");
    }
}

// Tắt hoàn toàn GPS ATGM336H
void deactivateGPS() {
    if (gpsActive && alarmStage == STAGE_NONE && !networkModeActive) {
        Serial.println("Tắt hoàn toàn module GPS ATGM336H để tiết kiệm năng lượng");

        // Đặt GPS vào chế độ ngủ trước
        putGPSToSleep();

        // Sau đó tắt nguồn hoàn toàn qua LDO
        digitalWrite(GPS_POWER_PIN, LOW);
        gpsPowerState = false;

        gpsActive = false;
        gpsActivationTime = 0;
        hadValidFix = false;
        signalLost = false;

        Serial.println("Module GPS ATGM336H đã tắt");
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

        // ATGM336H có thể được đánh thức bằng lệnh hot start
        SerialGPS.print(GPS_WAKE_CMD);

        // Đợi GPS khởi động
        delay(GPS_WARMUP_TIME);

        Serial.println("ATGM336H đã được đánh thức");
    }
}

// Cập nhật vị trí GPS
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

// Xử lý dữ liệu GPS
void processGpsData() {
    if (gpsActive) {
        while (SerialGPS.available() > 0) {
            gps.encode(SerialGPS.read());
        }
    }
}

// Tính khoảng cách di chuyển
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
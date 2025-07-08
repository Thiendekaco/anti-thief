/**
 * Hệ thống quản lý cooldown cho SMS
 *
 * Cập nhật lần cuối: 2025-06-18 17:43:54
 * Người phát triển: nguongthienTieu
 */

#ifndef SMS_COOLDOWN_H
#define SMS_COOLDOWN_H

#include "config.h"

// Thời gian cooldown cho từng loại SMS (mili giây)
const unsigned long SMS_COOLDOWN[] = {
        60000,   // SMS_ALERT_ALARM - 1 phút
        300000,  // SMS_ALERT_MOVEMENT - 5 phút
        300000,  // SMS_ALERT_POSITION - 5 phút
        600000,  // SMS_ALERT_GPS_LOST - 10 phút
        1800000, // SMS_ALERT_LOW_BATTERY - 30 phút
        900000,  // SMS_ALERT_MODULE_RESET - 15 phút
        1800000  // SMS_ALERT_SYSTEM_ERROR - 30 phút
};

// Thời gian gửi SMS cuối cùng cho mỗi loại
unsigned long lastSMSSentTime[7] = {0, 0, 0, 0, 0, 0, 0};

// Khởi tạo giá trị cooldown để có thể gửi SMS ngay sau khi khởi động
inline void initSMSCooldown() {
    unsigned long currentTime = millis();
    for (int i = 0; i < 7; ++i) {
        // Đặt thời gian gửi cuối cùng đủ xa để không bị cooldown ban đầu
        lastSMSSentTime[i] = currentTime - SMS_COOLDOWN[i];
    }
}
// Kiểm tra và cập nhật cooldown, trả về true nếu có thể gửi SMS
bool checkSMSCooldown(SMSAlertType alertType) {
    unsigned long currentTime = millis();

    // Kiểm tra nếu đã qua thời gian cooldown
    if (currentTime - lastSMSSentTime[alertType] >= SMS_COOLDOWN[alertType]) {
        lastSMSSentTime[alertType] = currentTime;
        return true;
    }

    return false;
}

// Gửi SMS với cơ chế cooldown
bool sendSMSWithCooldown(const char* message, SMSAlertType alertType) {
    // Kiểm tra cooldown
    if (checkSMSCooldown(alertType)) {
        // Gửi SMS
        sendSmsAlert(message);
        return true;
    } else {
        // Không thể gửi do đang trong thời gian cooldown
        Serial.print("Không thể gửi SMS (cooldown): ");
        Serial.println(message);
        return false;
    }
}

#endif // SMS_COOLDOWN_H
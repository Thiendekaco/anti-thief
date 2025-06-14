#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include "global_state.h"
#include "power_management.h"

// Enum cho mức độ nghiêm trọng của log
enum LogLevel {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_CRITICAL
};

// Các tiện ích về thời gian
String formatTimeInterval(unsigned long ms);
void logRuntime(); // Thêm khai báo rõ ràng cho hàm này

// Các tiện ích về chuỗi
String createGoogleMapsLink(float lat, float lng);
String createBatteryMessage(int percentage);

// Các tiện ích về đơn vị
float msToKmh(float speedMs);
float kmhToMs(float speedKmh);

// Các tiện ích về ghi nhật ký
void logMessage(LogLevel level, const char* message);

// Tiện ích kiểm tra và chẩn đoán
void performSystemDiagnostics();

// Các handler cho Blynk
void handleBlynkEvents();

#endif // UTILS_H
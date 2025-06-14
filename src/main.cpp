/*
 * Hệ thống Theo dõi và Chống trộm Xe ESP32 - Phiên bản Tích Hợp 2025
 * Sử dụng Blynk v1 (Legacy) - PlatformIO Edition
 *
 * Tính năng:
 * - Kết nối chỉ theo yêu cầu (WiFi/4G) khi kích hoạt qua RF433
 * - Phát hiện chuyển động sử dụng cảm biến gia tốc MPU6050
 * - Theo dõi vị trí GPS (ATGM336H) chỉ khi cần thiết (chỉ gửi qua SMS)
 * - Nhận dạng người dùng RF433MHz với 3 mã khác nhau (user, network, reset)
 * - Quản lý năng lượng tối ưu để kéo dài thời lượng pin
 * - Tích hợp Blynk thông qua WiFi hoặc 4G (A7682S) (không gửi GPS)
 * - Báo động 3 giai đoạn theo cấu hình cụ thể
 * - Cảnh báo pin thấp qua SMS khi dưới 20%
 * - Chế độ ngủ cho module GPS ATGM336H
 *
 * Ngày cập nhật: 2025-06-14 11:15:03
 * Người phát triển: nguongthienTieu
 */

#include <Arduino.h>
#include "config.h"
#include "global_state.h"
#include "power_management.h"
#include "hardware_control.h"
#include "gps_manager.h"
#include "network_manager.h"
#include "security_system.h"
#include "utils.h"

// Các khai báo hàm bổ sung nếu cần thiết
void logRuntime(); // Thêm khai báo hàm ở đây

void setup() {
    // Khởi tạo Serial
    Serial.begin(115200);

    // Hiển thị thông tin phiên bản
    Serial.println("\n==============================");
    Serial.println("Hệ thống Chống trộm Xe ESP32");
    Serial.print("Phiên bản: ");
    Serial.println(APP_VERSION);
    Serial.print("Ngày cập nhật: ");
    Serial.println("2025-06-14 11:15:03");
    Serial.print("Người phát triển: ");
    Serial.println(DEVELOPER);
    Serial.println("==============================\n");

    // Khởi tạo phần cứng
    initializePins();
    initializeI2C();
    initializeMPU6050();
    initializeRF433();

    // Tiếng bíp khởi động và kiểm tra thiết bị
    handleInitialBeeps();

    // Kích hoạt RF để bắt đầu kiểm tra
    activateRF();

    // Bắt đầu các timer cho Blynk
    setupTimers();

    // Cập nhật trạng thái pin ban đầu
    checkBatteryStatus();

    // Đặt chế độ WiFi là tắt ban đầu (chỉ kích hoạt khi cần)
    disableWiFi();

    Serial.println("Hệ thống đã sẵn sàng và đang chờ tín hiệu...");
}

void loop() {
    // Biến thời gian hiện tại
    unsigned long currentMillis = millis();

    // Kiểm tra lệnh reset thủ công
    checkManualReset();

    // Chạy timer của Blynk
    runTimers();

    // Xử lý ngắt MPU6050
    handleMpuInterrupt();

    // Xử lý dữ liệu GPS
    processGpsData();

    // Xử lý dữ liệu SMS
    processSmsCommands();

    // Xử lý Blynk
    handleBlynkConnection();

    // Kiểm tra timeout kết nối mạng
    checkNetworkTimeout();

    // Xử lý các giai đoạn báo động
    handleAlarmStages();

    // Nhấp nháy LED để chỉ báo hệ thống đang hoạt động
    blinkStatusLed(currentMillis);

    // Ghi log thời gian chạy
    logRuntime();
}
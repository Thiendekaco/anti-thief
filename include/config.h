#ifndef CONFIG_H
#define CONFIG_H

// Thông tin phiên bản
#define APP_VERSION "2.1.0"
#define DEVELOPER "nguongthienTieu"

// Định nghĩa các chân - ESP32 38 pin
// Chân kỹ thuật số
#define BUZZER_PIN 12        // Chân điều khiển còi buzzer 5V
#define LED_PIN 2            // LED tích hợp trên ESP32
#define RF_POWER_PIN 26      // Chân điều khiển nguồn module RF433MHz
#define RF_DATA_PIN 27       // Chân nhận dữ liệu từ module RF433MHz
#define GPS_POWER_PIN 25     // Chân EN của LDO để bật/tắt GPS ATGM336H
#define SIM_PWRKEY_PIN 33    // Chân PWRKEY của module SIM A7682S
#define MPU_INT_PIN 14       // Chân ngắt từ MPU6050

// Chân Analog
#define BATTERY_PIN 34       // Chân đọc điện áp pin

// UART Pins - Thiếu các định nghĩa này gây lỗi
#define GPS_RX_PIN 16        // UART1 RX cho GPS ATGM336H
#define GPS_TX_PIN 17        // UART1 TX cho GPS ATGM336H
#define SIM_RX_PIN 18        // UART2 RX cho module SIM A7682S
#define SIM_TX_PIN 19        // UART2 TX cho module SIM A7682S

// I2C Pins cho MPU6050
#define SDA_PIN 21           // I2C SDA
#define SCL_PIN 22           // I2C SCL

// Mã RF
#define USER_RF_CODE 12345678            // Mã RF nhận diện chủ sở hữu
#define NETWORK_ACTIVATE_CODE 87654321   // Mã RF kích hoạt kết nối mạng
#define RESET_RF_CODE 13579246           // Mã RF reset hệ thống

// Mã SMS
#define SMS_SOS_CODE "SOS"               // Lệnh kích hoạt báo động
#define SMS_STATUS_CODE "STATUS"         // Lệnh kiểm tra trạng thái
#define SMS_GPS_CODE "GPS"               // Lệnh yêu cầu vị trí GPS
#define SMS_STOP_CODE "STOP"             // Lệnh dừng báo động

// Thông tin WiFi và Blynk
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASS "YourWiFiPassword"
#define BLYNK_AUTH_TOKEN "YourBlynkAuthToken"

// Thông tin SIM
#define PHONE_NUMBER "+84123456789"
#define SIM_APN "viettel"

// Các chân ảo Blynk
#define VPIN_BATTERY V0                  // Hiển thị % pin
#define VPIN_OWNER V1                    // Trạng thái hiện diện chủ sở hữu
#define VPIN_ALARM_STATE V2              // Trạng thái báo động
#define VPIN_MOTION V3                   // Trạng thái phát hiện chuyển động
#define VPIN_CONNECTION V4               // Loại kết nối (WiFi/4G)
#define VPIN_CONTROL_TEST V5             // Nút kiểm tra còi

// Các thông số hệ thống
#define BATTERY_CHECK_INTERVAL 60000     // 1 phút
#define RF_CHECK_INTERVAL 5000           // 5 giây
#define MOTION_CHECK_INTERVAL 200        // 200ms
#define WIFI_TIMEOUT 10000               // 10 giây
#define GPS_WARMUP_TIME 1000             // 1 giây
#define POSITION_UPDATE_INTERVAL 30000   // 30 giây
#define NETWORK_SESSION_TIMEOUT 300000   // 5 phút
#define BLYNK_SYNC_INTERVAL 30000        // 30 giây
#define RF_ACTIVE_DURATION 1000          // 1 giây

// Thông số pin 2x18650 mắc nối tiếp
#define BATTERY_MAX_VOLTAGE 8.4          // 2 cell LiPo đầy
#define BATTERY_MIN_VOLTAGE 6.0          // 2 cell LiPo cạn
#define LOW_BATTERY_THRESHOLD 20         // Cảnh báo khi pin dưới 20%
#define HIGH_BATTERY_THRESHOLD 80        // Pin đã sạc đủ

// Lệnh GPS ATGM336H
#define GPS_SLEEP_CMD "$PMTK161,0*28\r\n"   // Lệnh đặt GPS vào chế độ ngủ
#define GPS_WAKE_CMD "$PMTK101*32\r\n"      // Lệnh đánh thức GPS (hot start)

// Enum trạng thái báo động
enum AlarmStage {
    STAGE_NONE = 0,     // Không có báo động
    STAGE_WARNING = 1,  // Giai đoạn 1: Cảnh báo
    STAGE_ALERT = 2,    // Giai đoạn 2: Báo động
    STAGE_TRACKING = 3  // Giai đoạn 3: Theo dõi
};

#endif // CONFIG_H
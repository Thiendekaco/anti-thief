#ifndef CONFIG_H
#define CONFIG_H

// Thông tin xác thực Blynk v1
#define BLYNK_PRINT Serial  // In thông tin debug Blynk ra Serial Monitor

// Thông tin đăng nhập WiFi
#define WIFI_SSID "TEN_WIFI_CUA_BAN"
#define WIFI_PASS "MAT_KHAU_WIFI"

// Cấu hình SIM
#define SIM_APN "internet"      // Thay đổi thành APN của nhà mạng của bạn
#define SIM_USER ""             // Để trống nếu không yêu cầu
#define SIM_PASS ""             // Để trống nếu không yêu cầu

// Số điện thoại để nhận cảnh báo SMS
#define PHONE_NUMBER "+SO_DIEN_THOAI_CUA_BAN"

// Định nghĩa chân
#define BUZZER_PIN 13
#define RF_RECEIVER_PIN 14    // Chân dữ liệu của module RF433
#define RF_POWER_PIN 32       // Thêm chân điều khiển nguồn cho RF433
#define SIM_PWRKEY_PIN 5      // Giữ nguyên hoặc điều chỉnh nếu cần
#define MPU_INT_PIN 39        // Giữ nguyên chân ngắt MPU
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_POWER_PIN 33      // Thêm chân điều khiển nguồn cho GPS ATGM336H
#define SIM_RX_PIN 26
#define SIM_TX_PIN 27
#define BATTERY_PIN 35        // Thay đổi thành GPIO35 theo yêu cầu

// I2C pins cho MPU6050
#define I2C_SDA 21
#define I2C_SCL 22

// Hằng số cho phát hiện trộm và báo động
#define MOTION_THRESHOLD_G 1.5     // Ngưỡng gia tốc tính bằng g
#define MOTION_THRESHOLD (MOTION_THRESHOLD_G * 9.8) // Chuyển đổi sang m/s²
#define INITIAL_BEEP_DURATION 500  // 0.5 giây cho mỗi tiếng bíp ban đầu
#define BEEP_INTERVAL 500          // 0.5 giây giữa các tiếng bíp
#define SEQUENCE_WAIT_TIME 120000  // 2 phút chờ giữa các chuỗi bíp
#define DISTANCE_THRESHOLD 10.0    // Ngưỡng di chuyển 10 mét cho giai đoạn 3
#define WIFI_TIMEOUT 10000         // Thời gian chờ WiFi 10 giây
#define NETWORK_SESSION_TIMEOUT 300000 // 5 phút kết nối mạng
#define GPS_UPDATE_INTERVAL 60000  // 1 phút giữa các lần cập nhật vị trí thông thường
#define GPS_ALARM_INTERVAL 300000  // 5 phút giữa các lần cập nhật khi báo động
#define GPS_STAGE2_INTERVAL 30000  // 30 giây giữa các lần cập nhật ở giai đoạn 2
#define RF_CHECK_INTERVAL 5000     // Kiểm tra thẻ RF mỗi 5 giây
#define RF_ACTIVE_DURATION 150     // RF bật trong 150ms mỗi chu kỳ
#define GPS_TIMEOUT 60000          // 60 giây chờ đợi tín hiệu GPS
#define BATTERY_CHECK_INTERVAL 900000 // 15 phút kiểm tra pin một lần
#define BLYNK_4G_SYNC_INTERVAL 10000 // 10 giây cho đồng bộ Blynk qua 4G
#define GPS_WARMUP_TIME 2000       // 2 giây để GPS khởi động từ chế độ ngủ

// Hằng số cho pin
#define BATTERY_MAX_VOLTAGE 8.4  // Điện áp tối đa của pin 2S Li-ion/Li-po
#define BATTERY_MIN_VOLTAGE 6.0  // Điện áp tối thiểu an toàn
#define LOW_BATTERY_THRESHOLD 20   // Ngưỡng pin thấp 20%
#define HIGH_BATTERY_THRESHOLD 80  // Ngưỡng pin cao 80%

// Mã RF433
#define USER_RF_CODE 12345678      // Mã nhận diện người dùng
#define NETWORK_ACTIVATE_CODE 87654321 // Mã kích hoạt kết nối mạng

// Lệnh NMEA để điều khiển chế độ ngủ ATGM336H
#define GPS_SLEEP_CMD "$PMTK161,0*28\r\n"       // Lệnh ngủ ATGM336H
#define GPS_WAKE_CMD "$PMTK010,002*2D\r\n"      // Lệnh đánh thức ATGM336H

// Chân ảo Blynk - Chỉ sử dụng các chân thiết yếu (đã bỏ GPS)
#define VPIN_ALARM_STATE V3    // Trạng thái cảnh báo
#define VPIN_BATTERY V4        // Phần trăm pin
#define VPIN_CONTROL_TEST V5   // Nút kiểm tra còi
#define VPIN_MOTION V7         // Phát hiện chuyển động
#define VPIN_OWNER V8          // Sự hiện diện của chủ sở hữu
#define VPIN_CONNECTION V9     // Loại kết nối (WiFi/4G)

// Trạng thái báo động
enum AlarmStage {
    STAGE_NONE = 0,    // Không báo động
    STAGE_INITIAL,     // Giai đoạn 1: Bíp 3 lần
    STAGE_WARNING,     // Giai đoạn 2: Báo động liên tục, gửi thông báo
    STAGE_TRACKING     // Giai đoạn 3: Theo dõi GPS, gửi vị trí
};

#endif // CONFIG_H
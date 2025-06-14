#include "network_manager.h"
#include "power_management.h"
#include "gps_manager.h"
#include "security_system.h"
#include "utils.h"

// Quản lý WiFi
void enableWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("Đang kết nối WiFi...");
}

void disableWiFi() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiConnected = false;
    Serial.println("Đã tắt WiFi");
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
    Blynk.config(BLYNK_AUTH_TOKEN);  // Cấu hình Auth token

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

// Đồng bộ dữ liệu với Blynk qua 4G
void syncBlynk4G() {
    if (usingSIM && blynkOverSIM) {
        // Gửi dữ liệu lên Blynk qua 4G (không gửi GPS)
        updateBlynkData();

        // Xử lý dữ liệu đến
        Blynk.run();

        Serial.println("Đã đồng bộ dữ liệu với Blynk qua 4G");
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
        enableWiFi();

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
            Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);

            // Cập nhật trạng thái lên Blynk
            if (Blynk.connected()) {
                Blynk.virtualWrite(VPIN_CONNECTION, 1); // 1 = WiFi
                updateBlynkData();
            }
        } else {
            // WiFi không khả dụng, chuyển sang SIM
            Serial.println("\nWiFi không khả dụng, chuyển sang sử dụng 4G");
            disableWiFi();

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
            disableWiFi();
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

// Cập nhật dữ liệu lên Blynk
void updateBlynkData() {
    if (Blynk.connected() || blynkOverSIM) {
        Blynk.virtualWrite(VPIN_BATTERY, batteryPercentage);
        Blynk.virtualWrite(VPIN_OWNER, ownerPresent ? 1 : 0);
        Blynk.virtualWrite(VPIN_ALARM_STATE, (int)alarmStage);
        Blynk.virtualWrite(VPIN_MOTION, motionDetected ? 1 : 0);
    }
}

// Quản lý kết nối Blynk
void handleBlynkConnection() {
    if (Blynk.connected()) {
        Blynk.run();
    }
}

// Gửi thông báo qua Blynk
void sendBlynkNotification(const char* message) {
    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
        Blynk.notify(message);
        Serial.print("Đã gửi thông báo Blynk: ");
        Serial.println(message);
    }
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

// Xử lý dữ liệu SMS từ module SIM
void processSmsCommands() {
    if (simActive) {
        String smsData = "";
        while (SerialSIM.available()) {
            char c = SerialSIM.read();
            smsData += c;
        }

        // Xử lý các lệnh SMS
        if (smsData.indexOf("+CMT:") >= 0) {
            Serial.println("Đã nhận được SMS:");
            Serial.println(smsData);

            // Xử lý lệnh SOS
            if (smsData.indexOf(SMS_SOS_CODE) >= 0) {
                // Kích hoạt chế độ báo động khẩn cấp
                if (alarmStage == STAGE_NONE) {
                    alarmStage = STAGE_ALERT;
                    alarmStartTime = millis();

                    // Kích hoạt GPS và gửi vị trí
                    activateGPS();

                    // Phản hồi rằng đã nhận được lệnh
                    String response = "Da nhan lenh SOS. Dang kich hoat bao dong va gui vi tri.";
                    sendSmsAlert(response.c_str());

                    Serial.println("Đã kích hoạt báo động SOS từ SMS");
                }
            }
                // Xử lý lệnh STATUS
            else if (smsData.indexOf(SMS_STATUS_CODE) >= 0) {
                // Gửi trạng thái hiện tại
                String statusMessage = "Trang thai: ";
                statusMessage += "Pin: " + String(batteryPercentage) + "%, ";
                statusMessage += "Bao dong: " + String((int)alarmStage) + ", ";
                statusMessage += "Chu so huu: " + String(ownerPresent ? "Co mat" : "Vang mat");

                sendSmsAlert(statusMessage.c_str());

                Serial.println("Đã gửi trạng thái qua SMS");
            }
                // Xử lý lệnh GPS
            else if (smsData.indexOf(SMS_GPS_CODE) >= 0) {
                // Kích hoạt GPS nếu chưa hoạt động
                if (!gpsActive) {
                    activateGPS();
                    delay(5000);  // Đợi GPS lấy được vị trí
                }

                // Cập nhật vị trí
                updateGpsPosition();

                // Gửi vị trí qua SMS
                if (hadValidFix) {
                    String positionMessage = "Vi tri hien tai: ";
                    positionMessage += "http://maps.google.com/maps?q=";
                    positionMessage += String(currentLat, 6);
                    positionMessage += ",";
                    positionMessage += String(currentLng, 6);

                    sendSmsAlert(positionMessage.c_str());

                    Serial.println("Đã gửi vị trí GPS qua SMS theo yêu cầu");
                } else {
                    sendSmsAlert("Khong the xac dinh vi tri GPS. Vui long thu lai sau.");
                    Serial.println("Không thể xác định vị trí GPS");
                }
            }
                // Xử lý lệnh STOP
            else if (smsData.indexOf(SMS_STOP_CODE) >= 0) {
                // Dừng báo động nếu đang ở giai đoạn 1 hoặc 2
                if (alarmStage == STAGE_WARNING || alarmStage == STAGE_ALERT) {
                    alarmStage = STAGE_NONE;
                    digitalWrite(BUZZER_PIN, LOW);  // Tắt còi

                    // Phản hồi rằng đã dừng báo động
                    sendSmsAlert("Da dung bao dong theo yeu cau.");

                    Serial.println("Đã dừng báo động theo yêu cầu SMS");
                }
                    // Không thể dừng ở giai đoạn 3 (TRACKING)
                else if (alarmStage == STAGE_TRACKING) {
                    sendSmsAlert("Khong the dung che do theo doi - Phuong tien dang bi lay trom!");
                    Serial.println("Không thể dừng chế độ theo dõi qua SMS");
                }
                else {
                    sendSmsAlert("Hien tai khong co bao dong nao dang hoat dong.");
                    Serial.println("Không có báo động nào để dừng");
                }
            }
        }
    }
}
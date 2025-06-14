#include "security_system.h"
#include "gps_manager.h"
#include "network_manager.h"
#include "hardware_control.h"
#include "power_management.h"

void checkUserPresence() {
    // Nếu RF đã bị vô hiệu hóa, không kiểm tra
    if (!rfEnabled) return;

    // Đảm bảo RF đang bật
    if (!rfPowerState) {
        activateRF();
    }

    bool previousOwnerState = ownerPresent;
    bool detectedRF = false;
    bool detectedNetworkActivation = false;

    // Kiểm tra thẻ RF trong một khoảng thời gian ngắn
    unsigned long rfStartTime = millis();
    while (millis() - rfStartTime < RF_ACTIVE_DURATION) {
        if (rfReceiver.available()) {
            unsigned long receivedCode = rfReceiver.getReceivedValue();
            rfReceiver.resetAvailable();

            if (receivedCode == USER_RF_CODE) {
                // Mã 1: Nhận diện người dùng
                ownerPresent = true;
                lastRfCheck = millis();
                detectedRF = true;
                Serial.println("Đã phát hiện mã RF của chủ sở hữu");

                // Phát âm thanh xác nhận
                acknowledgeUserPresence();
            }
            else if (receivedCode == NETWORK_ACTIVATE_CODE) {
                // Mã 2: Kích hoạt kết nối mạng
                detectedNetworkActivation = true;
                Serial.println("Đã phát hiện mã kích hoạt kết nối mạng");
            }
            else if (receivedCode == RESET_RF_CODE) {
                // Mã 3: Reset hệ thống về chế độ chờ
                manualReset = true;
                Serial.println("Đã phát hiện mã reset hệ thống");

                // Reset trạng thái hiện diện của chủ sở hữu
                ownerPresent = false;

                // Reset hệ thống về trạng thái chờ
                alarmStage = STAGE_NONE;
                beepCount = 0;
                beepSequenceCount = 0;
                sequenceEndTime = 0;
                notificationSent = false;
                initialPositionSet = false;
                stage2GpsUpdateCount = 0;
                digitalWrite(BUZZER_PIN, LOW);

                if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                    Blynk.virtualWrite(VPIN_ALARM_STATE, 0);
                    Blynk.virtualWrite(VPIN_MOTION, 0);
                    Blynk.virtualWrite(VPIN_OWNER, 0); // Đã reset, chủ sở hữu không hiện diện
                }

                // Phát âm thanh xác nhận reset (3 tiếng bíp)
                for (int i = 0; i < 3; i++) {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(100);
                    digitalWrite(BUZZER_PIN, LOW);
                    delay(100);
                }

                // Tắt GPS nếu đang hoạt động
                if (gpsActive && alarmStage != STAGE_TRACKING) {
                    deactivateGPS();
                }
            }
        }
        delay(10);  // Đợi chút để không chiếm CPU
    }

    // Nếu phát hiện tín hiệu kích hoạt mạng
    if (detectedNetworkActivation) {
        // Kích hoạt chế độ kết nối mạng
        activateNetworkMode();
    }

    // Nếu trạng thái hiện diện của chủ sở hữu thay đổi
    if (previousOwnerState != ownerPresent) {
        Serial.print("Sự hiện diện của chủ sở hữu: ");
        Serial.println(ownerPresent ? "Đã phát hiện" : "Không phát hiện");

        if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
            Blynk.virtualWrite(VPIN_OWNER, ownerPresent ? 1 : 0);
        }

        // Nếu chủ sở hữu hiện đang có mặt, dừng mọi cảnh báo đang hoạt động
        if (ownerPresent && alarmStage != STAGE_NONE && alarmStage != STAGE_TRACKING) {
            alarmStage = STAGE_NONE;
            beepCount = 0;
            beepSequenceCount = 0;
            sequenceEndTime = 0;
            notificationSent = false;
            initialPositionSet = false;
            stage2GpsUpdateCount = 0;
            digitalWrite(BUZZER_PIN, LOW);

            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_ALARM_STATE, 0);
                Blynk.virtualWrite(VPIN_MOTION, 0);
            }

            // Tắt GPS nếu đang hoạt động
            if (gpsActive && alarmStage != STAGE_TRACKING) {
                deactivateGPS();
            }
        }
    }
}

void handleMotionDetection() {
    // Đọc giá trị gia tốc từ MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Tính toán độ lớn của vector gia tốc
    float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                a.acceleration.y * a.acceleration.y +
                                a.acceleration.z * a.acceleration.z);

    // Kiểm tra nếu có chuyển động
    if (abs(accelMagnitude - 9.8) > 2.0) {  // Ngưỡng phát hiện chuyển động
        if (!motionDetected) {
            motionDetected = true;
            motionDetectedTime = millis();
            Serial.println("Đã phát hiện chuyển động!");

            // Cập nhật trạng thái lên Blynk nếu đang kết nối
            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_MOTION, 1);
            }

            // Nếu chủ sở hữu không có mặt, bắt đầu quy trình báo động
            if (!ownerPresent && alarmStage == STAGE_NONE) {
                alarmStage = STAGE_WARNING;  // Bắt đầu giai đoạn 1: Cảnh báo
                alarmStartTime = millis();
                beepCount = 0;
                beepSequenceCount = 0;
                notificationSent = false;

                Serial.println("Bắt đầu giai đoạn CẢNH BÁO do phát hiện chuyển động khi chủ sở hữu không có mặt");

                // Cập nhật trạng thái lên Blynk nếu đang kết nối
                if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                    Blynk.virtualWrite(VPIN_ALARM_STATE, 1);
                }
            }
        }
    } else {
        // Nếu không phát hiện chuyển động trong một khoảng thời gian
        if (motionDetected && (millis() - motionDetectedTime > 5000)) {
            motionDetected = false;
            Serial.println("Chuyển động đã dừng");

            // Cập nhật trạng thái lên Blynk nếu đang kết nối
            if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                Blynk.virtualWrite(VPIN_MOTION, 0);
            }
        }
    }
}

void handleAlarmStages() {
    switch(alarmStage) {
        case STAGE_NONE:
            // Không có gì xảy ra
            break;

        case STAGE_WARNING:
            // Giai đoạn 1: Cảnh báo
            // Phát ra tiếng bíp liên tục trong 10 giây
            if (millis() - alarmStartTime < 10000) {
                // Phát ra tiếng bíp ngắt quãng
                if (millis() - lastBeepTime >= 500) {
                    lastBeepTime = millis();
                    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));  // Đảo trạng thái còi
                }
            } else {
                // Chuyển sang giai đoạn 2 nếu không có chủ sở hữu xuất hiện
                if (!ownerPresent) {
                    alarmStage = STAGE_ALERT;
                    alarmStartTime = millis();
                    beepCount = 0;
                    beepSequenceCount = 0;
                    sequenceEndTime = 0;
                    notificationSent = false;
                    digitalWrite(BUZZER_PIN, LOW);  // Tắt còi tạm thời

                    // Kích hoạt GPS để bắt đầu theo dõi vị trí
                    activateGPS();

                    // Gửi thông báo qua Blynk nếu đang kết nối
                    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                        Blynk.virtualWrite(VPIN_ALARM_STATE, 2);  // Giai đoạn 2
                        sendBlynkNotification("CẢNH BÁO: Phương tiện đang bị xâm nhập!");
                    }

                    // Gửi cảnh báo qua SMS
                    activateSim();
                    delay(1000);

                    String alertMessage = "CANH BAO: Phuong tien dang bi xam nhap bat hop phap!";
                    sendSmsAlert(alertMessage.c_str());

                    Serial.println("Chuyển sang giai đoạn BÁO ĐỘNG - Đã gửi thông báo và SMS");
                } else {
                    // Nếu chủ sở hữu xuất hiện, trở về trạng thái bình thường
                    alarmStage = STAGE_NONE;
                    digitalWrite(BUZZER_PIN, LOW);  // Tắt còi
                    Serial.println("Chủ sở hữu đã xuất hiện - Dừng cảnh báo");

                    if (networkModeActive && (Blynk.connected() || blynkOverSIM)) {
                        Blynk.virtualWrite(VPIN_ALARM_STATE, 0);
                    }
                }
            }
            break;

        case STAGE_ALERT:
            // Triển khai mã cho giai đoạn STAGE_ALERT
            // ...
            break;

        case STAGE_TRACKING:
            // Triển khai mã cho giai đoạn STAGE_TRACKING
            // ...
            break;
    }
}
#ifndef MPU_POWER_MANAGEMENT_H
#define MPU_POWER_MANAGEMENT_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>

// Thanh ghi MPU6050 cho chế độ ngủ
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

// Địa chỉ I2C của MPU6050
#define MPU6050_I2C_ADDR 0x68

// Đặt MPU6050 vào chế độ ngủ
void putMPUToSleep() {
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x40); // Bit 6 (sleep) = 1
    Wire.endTransmission();

    mpuSleepState = true;
    Serial.println("MPU6050 đã vào chế độ ngủ");
}

// Đánh thức MPU6050 từ chế độ ngủ
void wakeMPUFromSleep() {
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00); // Bit 6 (sleep) = 0
    Wire.endTransmission();

    mpuSleepState = false;
    Serial.println("MPU6050 đã được đánh thức");
}

// Chuyển đổi chế độ MPU dựa trên sự hiện diện của chủ sở hữu
void toggleMPUSleepMode(bool ownerPresent) {
    if (ownerPresent) {
        // Khi chủ sở hữu có mặt, đặt MPU vào chế độ ngủ để tiết kiệm pin
        if (!mpuSleepState) {
            putMPUToSleep();
        }
    } else {
        // Khi chủ sở hữu không có mặt, đánh thức MPU để phát hiện chuyển động
        if (mpuSleepState) {
            wakeMPUFromSleep();
        }
    }
}

#endif // MPU_POWER_MANAGEMENT_H
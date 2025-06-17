/*
 * Quản lý năng lượng cho MPU6050
 * Cho phép đặt MPU6050 vào chế độ sleep để tiết kiệm pin
 *
 * Cập nhật lần cuối: 2025-06-17 02:41:14
 * Người phát triển: nguongthienTieu
 */

#ifndef MPU_POWER_MANAGEMENT_H
#define MPU_POWER_MANAGEMENT_H

#include <Arduino.h>
#include <Wire.h>

// Địa chỉ I2C của MPU6050
#define MPU6050_ADDR          0x68
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_PWR_MGMT_2    0x6C

// Khai báo biến ngoài - đã khai báo trong main.cpp
extern bool mpuSleepState;
extern void mpuISR();
extern Adafruit_MPU6050 mpu;

// Đặt MPU6050 vào chế độ sleep
void putMPUToSleep() {
    if (!mpuSleepState) {
        Serial.println("Đặt MPU6050 vào chế độ sleep để tiết kiệm năng lượng");

        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(MPU6050_PWR_MGMT_1);
        Wire.write(0x40);  // Đặt bit SLEEP (bit 6)
        Wire.endTransmission(true);

        mpuSleepState = true;

        // Tắt ngắt MPU6050 khi vào chế độ sleep
        detachInterrupt(digitalPinToInterrupt(MPU_INT_PIN));

        Serial.println("MPU6050 đã vào chế độ sleep, ngắt đã bị tắt");
    }
}

// Đánh thức MPU6050 từ chế độ sleep
void wakeMPUFromSleep() {
    if (mpuSleepState) {
        Serial.println("Đánh thức MPU6050 từ chế độ sleep");

        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(MPU6050_PWR_MGMT_1);
        Wire.write(0x00);  // Xóa bit SLEEP, đặt clock source là internal oscillator
        Wire.endTransmission(true);

        // Đợi MPU6050 thức dậy hoàn toàn
        delay(100);

        // Kích hoạt lại ngắt MPU6050
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpuISR, RISING);

        mpuSleepState = false;

        Serial.println("MPU6050 đã được đánh thức, ngắt đã được kích hoạt lại");

        // Đọc giá trị từ thanh ghi để xóa bất kỳ ngắt nào chưa được xử lý
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
    }
}

// Chuyển đổi chế độ MPU dựa trên sự hiện diện của chủ sở hữu
void toggleMPUSleepMode(bool ownerPresent) {
    if (ownerPresent) {
        // Khi chủ sở hữu có mặt, đặt MPU vào chế độ sleep
        putMPUToSleep();
    } else {
        // Khi chủ sở hữu không có mặt, đánh thức MPU để phát hiện chuyển động
        wakeMPUFromSleep();
    }
}

#endif // MPU_POWER_MANAGEMENT_H
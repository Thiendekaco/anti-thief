#ifndef HARDWARE_CONTROL_H
#define HARDWARE_CONTROL_H

#include <Arduino.h>
#include "global_state.h"

// Khởi tạo phần cứng
void initializePins();
void initializeI2C();
void initializeMPU6050();
void initializeRF433();
void setupTimers();

// Xử lý RF433
void activateRF();
void deactivateRF();
void disableRF433();
void acknowledgeUserPresence();

// Xử lý còi và LED
void handleInitialBeeps();
void blinkStatusLed(unsigned long currentMillis);

// Ngắt MPU6050
void IRAM_ATTR mpuISR();
void handleMpuInterrupt();

// Hàm quản lý timer
void runTimers();

#endif // HARDWARE_CONTROL_H
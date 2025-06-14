#ifndef SECURITY_SYSTEM_H
#define SECURITY_SYSTEM_H

#include <Arduino.h>
#include "global_state.h"

// Kiểm tra sự hiện diện của chủ sở hữu
void checkUserPresence();

// Xử lý phát hiện chuyển động
void handleMotionDetection();

// Quản lý báo động
void handleAlarmStages();

#endif // SECURITY_SYSTEM_H
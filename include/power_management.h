#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include <Arduino.h>
#include "global_state.h"

// Kiểm tra pin
float getBatteryVoltage();
int calculateBatteryPercentage(float voltage);
void checkBatteryStatus();

// Quản lý nguồn các module
void activateSim();
void deactivateSim();
void checkManualReset();

#endif // POWER_MANAGEMENT_H
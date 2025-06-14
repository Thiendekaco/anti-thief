#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include "global_state.h"

// Quản lý WiFi
void enableWiFi();
void disableWiFi();
void checkWifiConnection();

// Quản lý kết nối 4G
void setup4GConnection();
void syncBlynk4G();

// Quản lý chế độ kết nối mạng
void activateNetworkMode();
void deactivateNetworkMode();
void checkNetworkTimeout();

// Quản lý Blynk
void updateBlynkData();
void handleBlynkConnection();
void sendBlynkNotification(const char* message);

// Quản lý SMS
void sendSmsAlert(const char* message);
void processSmsCommands();

#endif // NETWORK_MANAGER_H
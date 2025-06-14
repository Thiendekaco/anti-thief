#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include "global_state.h"

// Quản lý GPS
void activateGPS();
void deactivateGPS();
void wakeGPSFromSleep();
void putGPSToSleep();
void updateGpsPosition();
void processGpsData();
float calculateDistance();

#endif // GPS_MANAGER_H
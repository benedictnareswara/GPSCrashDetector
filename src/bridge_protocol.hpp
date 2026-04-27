#pragma once

#include <Arduino.h>

void publishBridgeEvent(const char *eventType, float tiltDeg, float accelMagnitudeG);

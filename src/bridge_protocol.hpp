// ---------------------------------------------
// FILE: bridge_protocol.hpp
// PURPOSE: Public interface for the ESP32 bridge communication layer.
// ⚠️ PROTECTED LAYER: Do not change the function signature without
//    updating the ESP32 firmware and dashboard.
// ---------------------------------------------
#pragma once

#include <Arduino.h>

void publishBridgeEvent(const char *eventType, float tiltDeg, float accelMagnitudeG);

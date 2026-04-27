#pragma once

#include <Arduino.h>

#include "types.hpp"

void initSafetyStateMachine();
void updateSafetyStateMachine(bool buttonPressed,
                              bool crashTrigger,
                              float lastTiltDeg,
                              float lastAccelMagnitudeG,
                              void (*setBuzzer)(bool),
                              void (*publishEvent)(const char *, float, float));

AlertState getAlertState();

// ---------------------------------------------
// FILE: safety_fsm.hpp
// PURPOSE: Public interface for the safety alert state machine.
// HOW IT WORKS: Exposes functions to initialize and update a finite
//               state machine that transitions between STANDBY,
//               CRASH_PENDING, and CRASH_CONFIRMED based on sensor
//               triggers and button input.
// ---------------------------------------------
#pragma once

#include <Arduino.h>

#include "types.hpp"

void initSafetyStateMachine();
void updateSafetyStateMachine(bool buttonPressed,
                              bool crashTrigger,
                              float lastAccelMagnitudeG,
                              void (*setBuzzer)(bool),
                              void (*publishEvent)(const char *, float, float));

AlertState getAlertState();

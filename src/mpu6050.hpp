// ---------------------------------------------
// FILE: mpu6050.hpp
// PURPOSE: Public interface for the MPU6050 accelerometer/gyroscope sensor.
// HOW IT WORKS: Declares functions to initialize the sensor, read raw data,
//               and compute useful values like G-force magnitude and temperature.
// ---------------------------------------------
#pragma once

#include <Arduino.h>

#include "types.hpp"

bool initMpu6050();
bool isMpuReady();
bool readMpuRaw(MpuRawData &data);

float computeAccelMagnitudeG(const MpuRawData &data);
float rawTempToC(int16_t rawTemp);

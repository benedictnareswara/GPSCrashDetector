#pragma once

#include <Arduino.h>

#include "types.hpp"

bool initMpu6050();
bool isMpuReady();
bool readMpuRaw(MpuRawData &data);

float computeTiltDegrees(const MpuRawData &data);
float computeAccelMagnitudeG(const MpuRawData &data);
float rawTempToC(int16_t rawTemp);

int getMpuLastTxError();
int getMpuLastRxCount();

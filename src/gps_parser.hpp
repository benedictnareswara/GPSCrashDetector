#pragma once

#include <Arduino.h>

#include "types.hpp"

void processGpsStream();
bool hasFreshFix();
const GpsFix &getLatestFix();

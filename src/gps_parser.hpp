// ---------------------------------------------
// FILE: gps_parser.hpp
// PURPOSE: Public interface for the GPS NMEA sentence parser.
// HOW IT WORKS: Provides functions to process incoming GPS bytes from
//               Serial1 and query the latest valid fix.
// ---------------------------------------------
#pragma once

#include <Arduino.h>

#include "types.hpp"

void processGpsStream();
bool hasFreshFix();
const GpsFix &getLatestFix();

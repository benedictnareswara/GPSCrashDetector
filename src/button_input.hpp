// ---------------------------------------------
// FILE: button_input.hpp
// PURPOSE: Public interface for debounced button edge detection.
// ---------------------------------------------
#pragma once

#include <Arduino.h>

void initButtonInput();
bool detectButtonPressedEdge();

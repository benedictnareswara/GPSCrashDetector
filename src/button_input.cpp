// ---------------------------------------------
// FILE: button_input.cpp
// PURPOSE: Debounced push-button edge detection.
// HOW IT WORKS: Samples the button GPIO each loop iteration and applies
//               a debounce delay. Returns true only on the rising edge
//               (transition from not-pressed to pressed) of the stable state.
// ---------------------------------------------
#include "button_input.hpp"

#include "app_config.hpp"

namespace {
bool gButtonRawState = false;
bool gButtonStableState = false;
unsigned long gButtonLastEdgeMs = 0;
}

void initButtonInput() {
  gButtonRawState = (digitalRead(BUTTON_PIN) == LOW);
  gButtonStableState = gButtonRawState;
  gButtonLastEdgeMs = millis();
}

bool detectButtonPressedEdge() {
  const bool rawPressed = (digitalRead(BUTTON_PIN) == LOW);

  if (rawPressed != gButtonRawState) {
    gButtonRawState = rawPressed;
    gButtonLastEdgeMs = millis();
  }

  if ((millis() - gButtonLastEdgeMs) < BUTTON_DEBOUNCE_MS) {
    return false;
  }

  if (gButtonStableState == gButtonRawState) {
    return false;
  }

  gButtonStableState = gButtonRawState;
  if (!gButtonStableState) {
    return false;
  }

  Serial.print("BUTTON edge at ms=");
  Serial.println(millis());
  return true;
}

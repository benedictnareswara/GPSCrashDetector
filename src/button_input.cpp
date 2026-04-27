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

  if ((millis() - gButtonLastEdgeMs) >= BUTTON_DEBOUNCE_MS && gButtonStableState != gButtonRawState) {
    gButtonStableState = gButtonRawState;
    if (gButtonStableState) {
      Serial.print("BUTTON edge detected at ms=");
      Serial.println(millis());
      return true;
    }
  }
  return false;
}

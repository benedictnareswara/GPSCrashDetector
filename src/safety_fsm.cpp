// ---------------------------------------------
// FILE: safety_fsm.cpp
// PURPOSE: Implements the safety alert state machine for crash handling.
// HOW IT WORKS: Manages three states (STANDBY → CRASH_PENDING → CRASH_CONFIRMED).
//               When a crash trigger fires, a 5-second cancel window starts.
//               If not cancelled by button press, the event is published to
//               the ESP bridge and the buzzer latches on until acknowledged.
// ---------------------------------------------
#include "safety_fsm.hpp"

#include "app_config.hpp"

namespace {

AlertState gAlertState = ALERT_STANDBY;
bool gManualBuzzerLatched = false;
bool gIncidentPublished = false;
unsigned long gCrashWindowStartMs = 0;
unsigned long gLastPendingCountdownLogMs = 0;
unsigned long gLastCrashExitMs = 0;
PendingSource gPendingSource = PENDING_SOURCE_CRASH;

void enterStandbyFromCrash(void (*setBuzzer)(bool)) {
  setBuzzer(false);
  gManualBuzzerLatched = false;
  gAlertState = ALERT_STANDBY;
  gLastCrashExitMs = millis();
  gIncidentPublished = false;
}

void handleStandbyState(bool buttonPressed, bool crashTrigger, void (*setBuzzer)(bool)) {
  if (crashTrigger && (millis() - gLastCrashExitMs >= EVENT_COOLDOWN_MS)) {
    gAlertState = ALERT_CRASH_PENDING;
    gPendingSource = PENDING_SOURCE_CRASH;
    gCrashWindowStartMs = millis();
    gLastPendingCountdownLogMs = 0;
    gManualBuzzerLatched = false;
    setBuzzer(true);
    Serial.println("STATE: CRASH_PENDING (5s cancel window, waiting before ESP send)");
    return;
  }

  if (!buttonPressed) {
    return;
  }

  if (!gManualBuzzerLatched) {
    gAlertState = ALERT_CRASH_PENDING;
    gPendingSource = PENDING_SOURCE_MANUAL;
    gCrashWindowStartMs = millis();
    gLastPendingCountdownLogMs = 0;
    gManualBuzzerLatched = false;
    setBuzzer(true);
    Serial.println("STATE: MANUAL_PENDING (5s cancel window, press again to cancel)");
    return;
  }

  gManualBuzzerLatched = false;
  setBuzzer(false);
  gIncidentPublished = false;
  Serial.println("STATE: STANDBY manual clear -> buzzer OFF (local only, no ESP send)");
}

void handleCrashPendingState(bool buttonPressed,
                             float lastAccelMagnitudeG,
                             void (*setBuzzer)(bool),
                             void (*publishEvent)(const char *, float, float)) {
  const unsigned long now = millis();
  const unsigned long elapsedMs = now - gCrashWindowStartMs;

  // Log countdown once per second
  if (gLastPendingCountdownLogMs == 0 || (now - gLastPendingCountdownLogMs) >= 1000) {
    const unsigned long remainingMs =
        (elapsedMs < CRASH_CANCEL_WINDOW_MS) ? (CRASH_CANCEL_WINDOW_MS - elapsedMs) : 0;
    Serial.print("COUNTDOWN ");
    Serial.print((gPendingSource == PENDING_SOURCE_MANUAL) ? "MANUAL" : "CRASH");
    Serial.print(" remaining_ms=");
    Serial.println(remainingMs);
    gLastPendingCountdownLogMs = now;
  }

  if (buttonPressed) {
    setBuzzer(false);
    gManualBuzzerLatched = false;
    gAlertState = ALERT_STANDBY;
    gLastCrashExitMs = millis();
    if (gPendingSource == PENDING_SOURCE_MANUAL) {
      Serial.println("STATE: MANUAL canceled within 5s -> nothing sent to ESP");
    } else {
      Serial.println("STATE: CANCELED within 5s -> nothing sent to ESP");
    }
    gPendingSource = PENDING_SOURCE_CRASH;
    gIncidentPublished = false;
    return;
  }

  if (elapsedMs < CRASH_CANCEL_WINDOW_MS) {
    return;
  }

  // Window expired — confirm the incident
  gAlertState = ALERT_CRASH_CONFIRMED;
  gManualBuzzerLatched = true;
  setBuzzer(true);

  if (!gIncidentPublished) {
    if (gPendingSource == PENDING_SOURCE_MANUAL) {
      publishEvent("MANUAL", 0.0f, lastAccelMagnitudeG);
      Serial.println("STATE: MANUAL_CONFIRMED -> buzzer latched ON");
    } else {
      publishEvent("CRASH_CONFIRMED", 0.0f, lastAccelMagnitudeG);
      Serial.println("STATE: CRASH_CONFIRMED -> buzzer latched ON");
    }
    gIncidentPublished = true;
  }
  gPendingSource = PENDING_SOURCE_CRASH;
}

void handleCrashConfirmedState(bool buttonPressed, void (*setBuzzer)(bool)) {
  if (!buttonPressed) {
    return;
  }
  enterStandbyFromCrash(setBuzzer);
  Serial.println("STATE: CRASH_CONFIRMED acknowledged -> buzzer OFF (local clear, no ESP send)");
}

}  // namespace

void initSafetyStateMachine() {
  gAlertState = ALERT_STANDBY;
  gManualBuzzerLatched = false;
  gIncidentPublished = false;
  gCrashWindowStartMs = 0;
  gLastPendingCountdownLogMs = 0;
  gLastCrashExitMs = 0;
  gPendingSource = PENDING_SOURCE_CRASH;
}

void updateSafetyStateMachine(bool buttonPressed,
                              bool crashTrigger,
                              float lastAccelMagnitudeG,
                              void (*setBuzzer)(bool),
                              void (*publishEvent)(const char *, float, float)) {
  switch (gAlertState) {
    case ALERT_STANDBY:
      handleStandbyState(buttonPressed, crashTrigger, setBuzzer);
      break;
    case ALERT_CRASH_PENDING:
      handleCrashPendingState(buttonPressed, lastAccelMagnitudeG, setBuzzer, publishEvent);
      break;
    case ALERT_CRASH_CONFIRMED:
      handleCrashConfirmedState(buttonPressed, setBuzzer);
      break;
    default:
      gAlertState = ALERT_STANDBY;
      setBuzzer(false);
      break;
  }
}

AlertState getAlertState() {
  return gAlertState;
}

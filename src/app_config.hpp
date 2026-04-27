#pragma once

#include <Arduino.h>

inline constexpr uint8_t LED_PIN = LED_BUILTIN;
inline constexpr uint8_t BUZZER_PIN = 6;
inline constexpr uint8_t BUTTON_PIN = 7;
inline constexpr uint16_t BUZZER_TONE_HZ = 1760;

inline constexpr uint32_t MPU_SAMPLE_INTERVAL_MS = 1000;
inline constexpr uint32_t GPS_BAUD = 9600;
inline constexpr uint32_t BRIDGE_BAUD = 115200;
inline constexpr uint32_t GPS_FIX_STALE_MS = 10000;
inline constexpr uint32_t CRASH_CANCEL_WINDOW_MS = 5000;
inline constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
inline constexpr uint32_t EVENT_COOLDOWN_MS = 3000;

inline constexpr float MPU_ACCEL_LSB_PER_G = 16384.0f;
inline constexpr float TILT_TRIGGER_DEG = 35.0f;
inline constexpr float ACCEL_TRIGGER_G = 1.35f;

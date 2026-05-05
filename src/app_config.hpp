// ---------------------------------------------
// FILE: app_config.hpp
// PURPOSE: Central configuration for all hardware pins, timing,
//          and crash-detection thresholds used across the system.
// HOW IT WORKS: Defines compile-time constants so every module
//               shares the same tuning values without magic numbers.
// ---------------------------------------------
#pragma once

#include <Arduino.h>

inline constexpr uint8_t LED_PIN = LED_BUILTIN;
inline constexpr uint8_t BUZZER_PIN = 6;
inline constexpr uint8_t BUTTON_PIN = 7;
inline constexpr uint16_t BUZZER_TONE_HZ = 1760;

inline constexpr uint32_t MPU_SAMPLE_INTERVAL_MS = 100;
inline constexpr uint32_t GPS_BAUD = 9600;
inline constexpr uint32_t BRIDGE_BAUD = 115200;
inline constexpr uint32_t GPS_FIX_STALE_MS = 10000;
inline constexpr uint32_t CRASH_CANCEL_WINDOW_MS = 5000;
inline constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
inline constexpr uint32_t EVENT_COOLDOWN_MS = 3000;

inline constexpr float MPU_ACCEL_LSB_PER_G = 16384.0f;

// I2C bit-bang configuration (Mega 2560: SDA=pin 20/PD1, SCL=pin 21/PD0)
inline constexpr uint8_t I2C_SDA_PIN = 20;
inline constexpr uint8_t I2C_SCL_PIN = 21;
inline constexpr uint8_t I2C_HALF_CYCLE_US = 5;  // ~100 kHz clock

// MPU6050 I2C addresses (AD0 pin selects between the two)
inline constexpr uint8_t MPU6050_DEFAULT_ADDRESS = 0x68;
inline constexpr uint8_t MPU6050_ALT_ADDRESS = 0x69;

// G-force impact detection thresholds
// A crash is detected when acceleration spikes above GFORCE_SPIKE_THRESHOLD_G
// and then drops below GFORCE_STOP_THRESHOLD_G within GFORCE_SPIKE_WINDOW_MS.
inline constexpr float GFORCE_SPIKE_THRESHOLD_G = 2.5f;
inline constexpr float GFORCE_STOP_THRESHOLD_G = 1.2f;
inline constexpr uint32_t GFORCE_SPIKE_WINDOW_MS = 500;

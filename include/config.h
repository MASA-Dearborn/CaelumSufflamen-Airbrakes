#pragma once

#include <Arduino.h>

/*
config.h
===============================================================================
ROLE
  Central compile-time configuration for Caelum Sufflamen.

ENGINEERING INTENT
  Hardware pins, sensor availability, scheduler periods, telemetry cadence,
  estimator tuning, SD logging cadence, and safety gates are grouped here so
  mission-level changes do not require editing algorithmic modules.

SAFETY DEFAULTS
  Airbrake actuation and the airbrake policy are disabled by default. A build
  must explicitly define ACTUATION_ENABLED=1 and AIRBRAKE_POLICY_ENABLED=1 before
  non-idle airbrake commands can be issued.

API CONSISTENCY NOTE
  This finalized branch uses RuntimeConfig/SystemState/KfAlt2State. Older names
  such as Config, EepromBlob, FlightState, AuxVertLinAccel, and KalmanAlt2 are
  intentionally not part of this API set.
===============================================================================
*/

//==============================================================================
// Compile-time feature switches
//==============================================================================

#define STATUS_LED LED_BUILTIN

#ifndef BMI088_ENABLED
#define BMI088_ENABLED 1
#endif

#ifndef BMI088_USE_SPI
#define BMI088_USE_SPI 0
#endif

#ifndef LIS2DU12_ENABLED
#define LIS2DU12_ENABLED 1
#endif

#ifndef LIS2MDL_ENABLED
#define LIS2MDL_ENABLED 0
#endif

#ifndef ACTUATION_ENABLED
#define ACTUATION_ENABLED 0
#endif

#ifndef AIRBRAKE_POLICY_ENABLED
#define AIRBRAKE_POLICY_ENABLED 0
#endif

//==============================================================================
// Runtime and telemetry constants
//==============================================================================

static const float DEFAULT_SEA_LEVEL_HPA = 1013.25f;

static const uint32_t SERIAL_BAUD = 115200UL;
static const uint32_t BOOT_SERIAL_TIMEOUT_MS = 2000UL;

static const uint32_t LOOP_HZ = 50UL;
static const uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_HZ;

static const uint32_t HEARTBEAT_MS = 250UL;
static const uint32_t DIAG_PERIOD_MS = 1000UL;
static const uint32_t TLM_PERIOD_MS = 100UL;
static const uint32_t CMD_BUF_N = 96UL;

//==============================================================================
// SD logging constants
//==============================================================================

static const uint32_t SD_LOG_HZ = 50UL;
static const uint32_t SD_LOG_PERIOD_US = 1000000UL / SD_LOG_HZ;
static const uint32_t SD_FLUSH_EVERY_LINES = 50UL;
static const uint32_t SD_FLUSH_EVERY_MS = 500UL;

/*
Warning mask allocation
------------------------------------------------------------------------------
Bits 0..12 are reserved for core health and validity flags. The SD warning bit
is placed after this range to preserve compatibility with larger Caelum branches.
*/
static const uint32_t WARN_SD_FAULT_BIT = 13UL;

//==============================================================================
// Physical constants and estimator tuning
//==============================================================================

static const float kG = 9.8f;
static const float kAlphaG = 0.02f;

static const float kTs = 1.0f / (float)SD_LOG_HZ;
static const float kSigmaH2 = 5.71e-03f;
static const float kR = kSigmaH2;
static const float kSigmaA2 = 2.73e-03f;

static const float kQ00 = kSigmaA2 * (kTs * kTs * kTs * kTs / 4.0f);
static const float kQ01 = kSigmaA2 * (kTs * kTs * kTs / 2.0f);
static const float kQ10 = kSigmaA2 * (kTs * kTs * kTs / 2.0f);
static const float kQ11 = kSigmaA2 * (kTs * kTs);

//==============================================================================
// Optional actuator pin and servo mapping
//==============================================================================

static const uint8_t PIN_AIRBRAKE_SERVO = 9;

static const int SERVO_US_MIN_DEFAULT = 1000;
static const int SERVO_US_MAX_DEFAULT = 2000;
static const int SERVO_US_IDLE_DEFAULT = 1000;

//==============================================================================
// Policy constants
//==============================================================================

static const float POLICY_START_VZ_MPS = 35.0f;
static const float POLICY_FULL_SCALE_VZ_MPS = 50.0f;
static const float POLICY_MAX_COMMAND01 = 1.0f;
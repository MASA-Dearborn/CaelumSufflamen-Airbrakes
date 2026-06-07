#pragma once

#include <Arduino.h>

/*
config.h
===============================================================================
ROLE
  Central compile-time and runtime-tuning configuration for Caelum Sufflamen.

ENGINEERING INTENT
  Hardware feature switches, scheduler periods, telemetry cadence, estimator
  tuning, Madgwick tuning, barometer calibration constants, SD logging cadence,
  and actuator safety gates are concentrated here.

SAFETY DEFAULTS
  ACTUATION_ENABLED and AIRBRAKE_POLICY_ENABLED default to zero. A normal build
  therefore cannot command non-idle airbrake motion unless those switches are
  explicitly enabled at compile time.

API CONSISTENCY NOTE
  This migrated branch uses RuntimeConfig, SystemState, AttitudeSample,
  KfAlt2State, and EstimatorSample. Older branch names such as Config,
  EepromBlob, FlightState, AuxVertLinAccel, and KalmanAlt2 are intentionally not
  part of the public API.
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
// Barometer calibration constants
//==============================================================================

static const uint16_t BARO_CALIB_SAMPLES = 50U;
static const uint32_t BARO_CALIB_SAMPLE_DELAY_MS = 20UL;

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

static const float kG = 9.80665f;

/*
Madgwick gain
------------------------------------------------------------------------------
MADGWICK_BETA controls accelerometer correction strength. Larger values correct
roll/pitch drift faster but pass more accelerometer disturbance into attitude.
*/
static const float MADGWICK_BETA = 0.1f;

/*
Measured-IMU-dt guards
------------------------------------------------------------------------------
The estimator rejects unrealistic IMU sample intervals. The lower bound avoids
near-zero dt numerical sensitivity. The upper bound rejects stalled or delayed
samples that would create an excessively large prediction jump.
*/
static const float EST_MIN_IMU_DT_S = 0.0005f;
static const float EST_MAX_IMU_DT_S = 0.1000f;

/*
Kalman tuning
------------------------------------------------------------------------------
kSigmaA2 is acceleration process-noise intensity. kR is the barometric altitude
measurement variance. Both should be validated with bench and flight data.
*/
static const float kSigmaH2 = 5.71e-03f;
static const float kR = kSigmaH2;
static const float kSigmaA2 = 2.73e-03f;

static const uint32_t EST_MAX_AGE_MS = 200UL;

//==============================================================================
// Optional actuator pin and servo mapping
//==============================================================================

static const uint8_t PIN_AIRBRAKE_SERVO = 9;

static const int SERVO_US_MIN_DEFAULT = 1000;
static const int SERVO_US_MAX_DEFAULT = 2000;
static const int SERVO_US_IDLE_DEFAULT = 1000;

//==============================================================================
// Simple placeholder airbrake policy constants
//==============================================================================

static const float POLICY_START_VZ_MPS = 35.0f;
static const float POLICY_FULL_SCALE_VZ_MPS = 50.0f;
static const float POLICY_MAX_COMMAND01 = 1.0f;
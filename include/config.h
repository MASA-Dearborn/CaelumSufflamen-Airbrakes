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
#define ACTUATION_ENABLED 1
#endif

#ifndef AIRBRAKE_POLICY_ENABLED
#define AIRBRAKE_POLICY_ENABLED 1
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
// Apogee-prediction airbrake policy constants
//==============================================================================

/*
POLICY_TARGET_APOGEE_M
------------------------------------------------------------------------------
Target apogee altitude above the estimator reference frame.

If the estimator is using pad-relative altitude, this target is pad-relative.
If the estimator is using sea-level-referenced altitude, this target must be
defined in the same altitude frame.
*/
static const float POLICY_TARGET_APOGEE_M = 300.0f;

/*
POLICY_MIN_ALT_M
------------------------------------------------------------------------------
Minimum altitude gate for policy activation.

This prevents pad, rail, launch-transient, or early-flight estimator noise from
authorizing airbrake deployment.
*/
static const float POLICY_MIN_ALT_M = 30.0f;

/*
POLICY_MIN_VZ_MPS
------------------------------------------------------------------------------
Minimum upward vertical-speed gate for policy activation.

The apogee-prediction law is a coast-phase upward-flight law. It should not
command deployment when the vehicle is near rest, descending, or not clearly in
upward coast.
*/
static const float POLICY_MIN_VZ_MPS = 15.0f;

/*
POLICY_APOGEE_DEADBAND_M
------------------------------------------------------------------------------
No-command deadband around target apogee.

If the predicted closed-brake apogee is only slightly above the target, the
policy remains idle. This prevents command chatter caused by estimator noise and
small prediction changes.
*/
static const float POLICY_APOGEE_DEADBAND_M = 5.0f;

/*
Aerodynamic model constants
------------------------------------------------------------------------------
The policy uses:

  k(u) = rho * (CDA_body + u*CDA_brake) / (2*m)

where k has units of 1/m.

These are initial placeholders. Flight logs, CFD, simulation, wind-tunnel data,
or system-identification tests should replace them with vehicle-specific values.
*/
static const float POLICY_VEHICLE_MASS_KG = 2.50f;
static const float POLICY_RHO_KGPM3 = 1.225f;
static const float POLICY_CDA_BODY_M2 = 0.0040f;
static const float POLICY_CDA_BRAKE_M2 = 0.0200f;

/*
Command shaping
------------------------------------------------------------------------------
POLICY_MAX_COMMAND01 limits maximum normalized deployment.
POLICY_SLEW_PER_SEC limits command-rate change in command units per second.
*/
static const float POLICY_MAX_COMMAND01 = 1.0f;
static const float POLICY_SLEW_PER_SEC = 1.5f;

/*
Estimator freshness
------------------------------------------------------------------------------
The policy must not act on stale estimator state. This gate is independent of
the lower-level safety module and intentionally duplicates the freshness check
near the command source.
*/
static const uint32_t POLICY_MAX_EST_AGE_MS = 200UL;

/*
Bisection solver count
------------------------------------------------------------------------------
A fixed iteration count keeps runtime deterministic. Eighteen iterations gives
sub-micro command resolution over [0,1], far beyond practical servo precision.
*/
static const uint8_t POLICY_BISECTION_STEPS = 18U;

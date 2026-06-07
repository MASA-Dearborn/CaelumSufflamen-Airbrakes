#pragma once

#include <Arduino.h>
#include <SD.h>
#include <math.h>
#include <stdint.h>

/*
data_types.h
===============================================================================
ROLE
  Shared data contracts for the modular Caelum Sufflamen firmware.

SNAPSHOT CONTRACT
  Runtime state is exchanged through published snapshots. A snapshot is a plain
  struct containing:

    valid   : semantic usability of payload values.
    updated : new sample/event flag for estimator consumption.
    t_ms    : millisecond timestamp for diagnostics and freshness checks.
    t_us    : microsecond timestamp for measured-dt estimator propagation.
    seq     : monotonically increasing successful-publication counter.
    data    : numerical payload with documented units.

READING RULE
  Numeric payload fields are meaningful only when valid == true and the values
  consumed by a reader are finite. Invalid snapshots may contain stale numeric
  payloads and must not be interpreted as current measurements.

OWNERSHIP RULE
  Each snapshot has exactly one logical writer. Other modules may read snapshots
  but must not mutate fields owned by another module.
===============================================================================
*/

struct RuntimeConfig {
  bool valid = true;
  bool serial_header_enable = true;
  float sea_level_hpa = 1013.25f;
  float baro_baseline_hpa = NAN;
};

struct BaroSample {
  bool valid = false;
  bool updated = false;

  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float temp_c = NAN;
  float press_hpa = NAN;
  float alt_m = NAN;
};

struct ImuSample {
  bool valid = false;
  bool updated = false;

  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float ax = NAN;
  float ay = NAN;
  float az = NAN;

  float gx = NAN;
  float gy = NAN;
  float gz = NAN;

  float a_norm = NAN;
};

struct AuxSample {
  bool valid = false;
  bool updated = false;

  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float ax = NAN;
  float ay = NAN;
  float az = NAN;
  float a_norm = NAN;
};

struct AuxVzSample {
  bool valid = false;
  bool updated = false;

  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float a_vertical = NAN;
};

struct AttitudeSample {
  bool valid = false;
  bool updated = false;

  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
};

struct EstimatorSample {
  bool valid = false;
  bool seeded = false;

  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float h_m = 0.0f;
  float v_mps = 0.0f;
  float a_mps2 = NAN;

  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;
};

struct SensorHealth {
  bool bmp_ok = false;
  bool bmi_accel_ok = false;
  bool bmi_gyro_ok = false;
  bool lis_ok = false;
};

struct SdLoggerState {
  bool enabled = false;
  bool card_ok = false;
  bool file_open = false;
  bool runtime_failed = false;

  uint32_t fail_count = 0;
  uint32_t next_log_us = 0;
  uint32_t line_count = 0;
  uint32_t last_flush_ms = 0;

  char filename[16] = "NONE";

  File file;
};

enum PlotMode {
  PLOT_MODE_OFF = 0,
  PLOT_MODE_OVERVIEW = 1,
  PLOT_MODE_IMU = 2,
  PLOT_MODE_APOGEE = 3,
  PLOT_MODE_UNIFIED = 4
};

enum PlotGroupMask {
  PLOT_GROUP_OVERVIEW = 0x01,
  PLOT_GROUP_IMU = 0x02,
  PLOT_GROUP_APOGEE = 0x04,
  PLOT_GROUP_UNIFIED = 0x08
};

struct PlotField {
  const char *label;
  float value;
  uint8_t groups;
};

struct AirbrakePolicyOutput {
  bool valid = false;
  float command01 = 0.0f;
};

struct ActuatorConfig {
  int servo_us_min = 1000;
  int servo_us_max = 2000;
  int servo_us_idle = 1000;
};

struct SystemState {
  RuntimeConfig cfg;
  SensorHealth health;

  BaroSample baro;
  ImuSample imu;
  AuxSample aux;
  AuxVzSample auxvz;
  AttitudeSample attitude;
  EstimatorSample est;

  SdLoggerState sdlog;
  PlotMode plot_mode = PLOT_MODE_OFF;

  AirbrakePolicyOutput policy;
  ActuatorConfig actuator_cfg;
};
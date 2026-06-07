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
  Runtime state is exchanged through published snapshots.

COMMON FIELDS
  valid   semantic usability gate
  updated true only when the current service call produced a new publication
  t_ms    millisecond timestamp
  t_us    microsecond timestamp
  seq     successful-publication counter
===============================================================================
*/

struct RuntimeConfig {
  bool valid = true;
  bool serial_header_enable = true;
  float sea_level_hpa = 1013.25f;
  float baro_baseline_hpa = NAN;
};

struct MagCal {
  float off_x = 0.0f;
  float off_y = 0.0f;
  float off_z = 0.0f;

  float scl_x = 1.0f;
  float scl_y = 1.0f;
  float scl_z = 1.0f;

  bool valid = false;

  float rx = NAN;
  float ry = NAN;
  float rz = NAN;
};

struct Config {
  float sea_level_hpa = 1013.25f;
  float baro_baseline_hpa = NAN;
  float declination_deg = 0.0f;

  MagCal mag_cal;

  int16_t servo_us_min = 1000;
  int16_t servo_us_max = 2000;
  int16_t servo_us_idle = 1000;

  bool valid = true;
  bool serial_header_enable = true;
};

struct ActuatorConfig {
  int16_t servo_us_min = 1000;
  int16_t servo_us_max = 2000;
  int16_t servo_us_idle = 1000;
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
  bool motion_bad = false;
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

  float roll_deg = NAN;
  float pitch_deg = NAN;
};

struct AuxSample {
  bool valid = false;
  bool updated = false;
  bool motion_bad = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float ax = NAN;
  float ay = NAN;
  float az = NAN;

  float a_norm = NAN;
};

using AuxAccelSample = AuxSample;

struct AttitudeSample {
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float qw = 1.0f;
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;

  float roll_deg = NAN;
  float pitch_deg = NAN;
  float yaw_deg = NAN;
};

struct AuxVzSample {
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float a_vertical = NAN;
  float a_mps2 = NAN;

  float a_wz_mps2 = NAN;
  float a_lin_z_mps2 = NAN;
};

using AuxVertLinAccel = AuxVzSample;

struct EstimatorSample {
  bool valid = false;
  bool updated = false;
  bool seeded = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float h_m = 0.0f;
  float v_mps = 0.0f;

  float altitude_m = NAN;
  float vz_mps = NAN;

  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;
};

struct FlightState {
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float altitude_m = NAN;
  float vz_mps = NAN;
};

struct MagSample {
  bool valid = false;
  bool updated = false;
  bool interference = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float raw_x = NAN;
  float raw_y = NAN;
  float raw_z = NAN;

  float cal_x = NAN;
  float cal_y = NAN;
  float cal_z = NAN;

  float norm_uT = NAN;
  float heading_deg = NAN;
};

struct SensorHealth {
  bool bmp_ok = false;
  bool bmi_accel_ok = false;
  bool bmi_gyro_ok = false;
  bool lis_ok = false;

  bool mpu_ok = false;
  bool aux_ok = false;
  bool mag_ok = false;
};

enum class ArmingState : uint8_t {
  DISARMED = 0,
  SAFE = 1,
  ARMED = 2
};

enum class FlightPhase : uint8_t {
  IDLE = 0,
  BOOST = 1,
  COAST = 2,
  BRAKE = 3,
  DESCENT = 4
};

struct AirbrakePolicyOutput {
  float command01 = 0.0f;
  bool valid = false;
};

struct KfAlt2State {
  bool seeded = false;

  float h_m = 0.0f;
  float v_mps = 0.0f;

  float x0 = 0.0f;
  float x1 = 0.0f;

  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;

  float q_acc = 4.0f;
  float r_meas = 4.0f;

  uint32_t t_ms = 0;
  uint32_t t_us = 0;
};

using KalmanAlt2 = KfAlt2State;

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

  bool baro_ref_set = false;
  float baro_ref_alt_m = NAN;
  float ref_basis_baseline_hpa = NAN;
  float ref_basis_slp_hpa = 1013.25f;

  float g_bx = 0.0f;
  float g_by = 0.0f;
  float g_bz = -9.8f;

  float kf_h = 0.0f;
  float kf_v = 0.0f;

  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;

  File file;
};

enum PlotMode {
  PLOT_MODE_OFF = 0,
  PLOT_MODE_OVERVIEW = 1,
  PLOT_MODE_IMU = 2,
  PLOT_MODE_APOGEE = 3
};

struct SystemState {
  RuntimeConfig cfg;

  SensorHealth health;

  BaroSample baro;
  ImuSample imu;
  AuxSample aux;
  AttitudeSample att;
  AuxVzSample auxvz;
  EstimatorSample est;
  FlightState flight;
  MagSample mag;

  KfAlt2State kf;
  SdLoggerState sdlog;

  ActuatorConfig actuator_cfg;

  AirbrakePolicyOutput policy;
  ArmingState arm_state = ArmingState::DISARMED;
  FlightPhase phase = FlightPhase::IDLE;

  bool policy_runtime_enabled = false;
  bool software_arm_token = false;

  PlotMode plot_mode = PLOT_MODE_OFF;
};
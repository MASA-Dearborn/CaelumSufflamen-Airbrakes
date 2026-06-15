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
  updated true only when the owning module produced a fresh publication during
          its most recent service call; it is an observability flag, not a
          cross-module consume latch
  t_ms    millisecond timestamp
  t_us    microsecond timestamp
  seq     successful-publication counter
===============================================================================
*/

struct RuntimeConfig {
  // Global configuration validity gate. Downstream logic may refuse actuation or
  // estimation-dependent decisions when this contract is false.
  bool valid = true;

  // When true, the firmware emits the high-rate CSV telemetry stream. When
  // false, diagnostics and command responses still remain available.
  bool serial_header_enable = true;

  // Reference sea-level pressure used by the standard-atmosphere inversion from
  // measured pressure to altitude.
  float sea_level_hpa = 1013.25f;

  // Pad-reference pressure captured on the ground. NAN means "no explicit pad
  // baseline has been captured yet", so the estimator may fall back to its
  // first valid live altitude sample as the zero reference.
  float baro_baseline_hpa = NAN;
};

struct ActuatorConfig {
  // Pulse-equivalent lower travel limit for the servo mapping.
  int16_t servo_us_min = 1000;

  // Pulse-equivalent upper travel limit for the servo mapping.
  int16_t servo_us_max = 2000;

  // Fail-safe idle command written whenever deployment is not authorized.
  int16_t servo_us_idle = 1000;
};

struct BaroSample {
  // Publication metadata for the latest barometer snapshot.
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  // Payload in physical engineering units.
  float temp_c = NAN;
  float press_hpa = NAN;
  float alt_m = NAN;
};

struct ImuSample {
  // Publication metadata for the BMI088 IMU snapshot.
  bool valid = false;
  bool updated = false;

  // Reserved for future motion-quality screening, such as saturation or shock
  // detection. The current branch does not yet assert this flag.
  bool motion_bad = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  // Specific force in the IMU body frame [m/s^2].
  float ax = NAN;
  float ay = NAN;
  float az = NAN;

  // Angular rate in the IMU body frame [rad/s].
  float gx = NAN;
  float gy = NAN;
  float gz = NAN;

  // Euclidean norm of body acceleration. This is useful for coarse boost
  // detection and sensor sanity checks.
  float a_norm = NAN;

  // Optional Euler-angle placeholders retained for compatibility with broader
  // branches. The current estimator primarily publishes quaternion attitude.
  float roll_deg = NAN;
  float pitch_deg = NAN;
};

struct AuxSample {
  // Publication metadata for the auxiliary accelerometer snapshot.
  bool valid = false;
  bool updated = false;
  bool motion_bad = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  // Auxiliary body-frame acceleration [m/s^2].
  float ax = NAN;
  float ay = NAN;
  float az = NAN;

  // Auxiliary acceleration norm [m/s^2].
  float a_norm = NAN;
};

using AuxAccelSample = AuxSample;

struct AttitudeSample {
  // Publication metadata for the quaternion attitude estimate.
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  // Unit quaternion mapping body-frame vectors into the world frame used by the
  // estimator's vertical-axis projection.
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;

  // Euler-angle views are provided for operator interpretation and future
  // extensions, even though the estimator itself operates on quaternions.
  float roll_deg = NAN;
  float pitch_deg = NAN;
  float yaw_deg = NAN;
};

struct AuxVzSample {
  // Publication metadata for the derived world-vertical acceleration estimate.
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  // a_vertical is the world-frame vertical specific-force-derived acceleration
  // after the attitude module projects the measured acceleration vector and
  // removes gravity under the firmware sign convention.
  float a_vertical = NAN;

  // Compatibility aliases retained for older analysis tooling and larger Caelum
  // branches that used alternative naming.
  float a_mps2 = NAN;

  float a_wz_mps2 = NAN;
  float a_lin_z_mps2 = NAN;
};

using AuxVertLinAccel = AuxVzSample;

struct EstimatorSample {
  // Publication metadata for the fused altitude/vertical-speed estimate.
  bool valid = false;
  bool updated = false;

  // Seeded is true once the Kalman filter has received its first trusted
  // altitude reference and may therefore propagate meaningful state.
  bool seeded = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  // State estimate in the pad-relative or baseline-relative altitude frame [m]
  // and its first derivative [m/s].
  float h_m = 0.0f;
  float v_mps = 0.0f;

  // Latest world-vertical acceleration input used by the predictor [m/s^2].
  float a_mps2 = NAN;

  // Compatibility aliases preserved for downstream tooling that expects the
  // older naming convention.
  float altitude_m = NAN;
  float vz_mps = NAN;

  // 2x2 covariance matrix:
  //   P00 = var(h), P11 = var(v), P01/P10 = covariance(h,v).
  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;
};

struct FlightState {
  // Legacy flight-state publication retained for compatibility with analysis
  // pipelines. The current branch primarily uses EstimatorSample and phase.
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  float altitude_m = NAN;
  float vz_mps = NAN;
};

struct MagSample {
  // Publication metadata for an optional magnetometer snapshot.
  bool valid = false;
  bool updated = false;

  // True when the magnetic field is considered corrupted by interference or
  // local disturbances.
  bool interference = false;
  uint32_t t_ms = 0;
  uint32_t t_us = 0;
  uint32_t seq = 0;

  // Raw sensor outputs.
  float raw_x = NAN;
  float raw_y = NAN;
  float raw_z = NAN;

  // Calibrated magnetic vector.
  float cal_x = NAN;
  float cal_y = NAN;
  float cal_z = NAN;

  // Vector norm and derived heading for optional situational awareness.
  float norm_uT = NAN;
  float heading_deg = NAN;
};

struct SensorHealth {
  // Per-device initialization and availability flags. These are latched from
  // boot-time initialization and later reflected in diagnostics and warning
  // masks.
  bool bmp_ok = false;
  bool bmi_accel_ok = false;
  bool bmi_gyro_ok = false;
  bool lis_ok = false;

  // Compatibility slots for alternate hardware configurations.
  bool mpu_ok = false;
  bool aux_ok = false;
  bool mag_ok = false;
};

enum class ArmingState : uint8_t {
  DISARMED = 0,
  SAFE = 1,
  ARMED = 2
};

static inline const char *arming_state_name(ArmingState state) {
  switch (state) {
    case ArmingState::DISARMED:
      return "DISARMED";

    case ArmingState::SAFE:
      return "SAFE";

    case ArmingState::ARMED:
      return "ARMED";

    default:
      return "UNKNOWN";
  }
}

enum class FlightPhase : uint8_t {
  IDLE = 0,
  BOOST = 1,
  COAST = 2,
  BRAKE = 3,
  DESCENT = 4
};

struct FlightPhaseDiag {
  // Publication metadata for flight-phase transition observability.
  bool valid = false;
  bool updated = false;
  uint32_t t_ms = 0;
  uint32_t seq = 0;

  // Latched state-machine milestones.
  bool launch_latched = false;
  bool burnout_latched = false;
  bool descent_latched = false;

  // Candidate timers are active while a condition is continuously true but has
  // not yet satisfied its confirmation dwell.
  bool launch_candidate = false;
  bool burnout_candidate = false;
  bool descent_candidate = false;

  // Dwell and phase-condition gates used by the current update pass.
  bool boost_dwell_met = false;
  bool coast_dwell_met = false;
  bool brake_active = false;

  // Confirmation timer progress [ms]. Inactive candidates report zero.
  uint32_t launch_confirm_ms = 0;
  uint32_t burnout_confirm_ms = 0;
  uint32_t descent_confirm_ms = 0;

  // Time since major latched events [ms]. Unlatched events report 0xFFFFFFFF.
  uint32_t since_launch_ms = 0xFFFFFFFFUL;
  uint32_t since_burnout_ms = 0xFFFFFFFFUL;
};

struct AirbrakePolicyOutput {
  // valid means the policy considers the command both meaningful and
  // authorized-for-consideration. Safety and actuator gates still have final
  // authority over hardware motion.
  bool valid = false;

  /*
  command01
  -------------------------------------------------------------------------------
  Normalized deployment command.

  Meaning:
    0.0 = fully retracted / no deployment intent
    1.0 = maximum permitted deployment intent

  This is policy intent only. The safety and actuator modules decide whether this
  command reaches hardware.
  */
  float command01 = 0.0f;

  /*
  predicted_apogee_no_brake_m
  -------------------------------------------------------------------------------
  Predicted coast apogee if airbrakes remain fully retracted.
  */
  float predicted_apogee_no_brake_m = NAN;

  /*
  predicted_apogee_full_brake_m
  -------------------------------------------------------------------------------
  Predicted coast apogee at maximum permitted deployment.
  */
  float predicted_apogee_full_brake_m = NAN;

  /*
  target_apogee_m
  -------------------------------------------------------------------------------
  Effective target apogee used by the current policy computation.

  This may be lower than the nominal target when covariance-aware uncertainty
  margin is active.
  */
  float target_apogee_m = NAN;

  /*
  apogee_error_m
  -------------------------------------------------------------------------------
  Closed-brake predicted apogee error:

    predicted_apogee_no_brake_m - target_apogee_m
  */
  float apogee_error_m = NAN;

  /*
  target_nominal_m
  -------------------------------------------------------------------------------
  Nominal configured target before uncertainty margin.
  */
  float target_nominal_m = NAN;

  /*
  target_effective_m
  -------------------------------------------------------------------------------
  Target after subtracting the covariance-aware uncertainty margin.
  */
  float target_effective_m = NAN;

  /*
  uncertainty_margin_m
  -------------------------------------------------------------------------------
  Altitude uncertainty margin subtracted from the nominal target.
  */
  float uncertainty_margin_m = NAN;
};

struct KfAlt2State {
  // The filter becomes seeded once a trusted altitude measurement establishes
  // the reference frame for h_m.
  bool seeded = false;

  // Mean state of the constant-acceleration vertical model.
  float h_m = 0.0f;
  float v_mps = 0.0f;

  // Symmetric covariance matrix for [h, v]^T.
  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;
};

using KalmanAlt2 = KfAlt2State;

struct SdLoggerState {
  // High-level SD availability and latched-failure state.
  bool enabled = false;
  bool card_ok = false;
  bool file_open = false;
  bool runtime_failed = false;

  // Logging cadence, failure accounting, and flush bookkeeping.
  uint32_t fail_count = 0;
  uint32_t next_log_us = 0;
  uint32_t line_count = 0;
  uint32_t last_flush_ms = 0;

  // Active filename for operator visibility and post-flight traceability.
  char filename[16] = "NONE";

  // Open file handle owned exclusively by sd_logger.cpp.
  File file;
};

enum PlotMode {
  PLOT_MODE_OFF = 0,
  PLOT_MODE_OVERVIEW = 1,
  PLOT_MODE_IMU = 2,
  PLOT_MODE_APOGEE = 3
};

struct SystemState {
  // Configuration and hardware-availability layers.
  RuntimeConfig cfg;

  SensorHealth health;

  // Published sensor and derived-estimator snapshots. Each module owns exactly
  // one publication responsibility and updates only its own fields.
  BaroSample baro;
  ImuSample imu;
  AuxSample aux;
  AttitudeSample attitude;
  AuxVzSample auxvz;
  EstimatorSample est;
  // Legacy compatibility publications. The live branch maintains these so
  // downstream tooling from larger Caelum branches can still consume this
  // firmware output without becoming control-critical.
  FlightState flight;
  MagSample mag;

  // Mirror of the live private filter state for diagnostics and legacy tools.
  KfAlt2State kf;
  SdLoggerState sdlog;

  // Actuator calibration and latest policy intent.
  ActuatorConfig actuator_cfg;

  AirbrakePolicyOutput policy;

  // High-level supervisory state used by commands, policy gating, and safety.
  ArmingState arm_state = ArmingState::DISARMED;
  FlightPhase phase = FlightPhase::IDLE;
  FlightPhaseDiag phase_diag;

  // Operator/software arming gates. policy_runtime_enabled is the software
  // permission for the controller to compute non-idle deployment intent at all.
  // software_arm_token records that an explicit operator arming command was
  // accepted; the policy may require both the ARMED state and this token.
  bool policy_runtime_enabled = false;
  bool software_arm_token = false;

  // Plot-mode selection is reserved for external visualization tools and is not
  // currently consumed by the flight runtime in this branch.
  PlotMode plot_mode = PLOT_MODE_OFF;
};

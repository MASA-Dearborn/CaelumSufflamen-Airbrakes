#include "estimation.h"

#include "config.h"
#include "math_utils.h"
#include "kalman_alt2.h"
#include "attitude.h"

/*
estimation.cpp
===============================================================================
PURPOSE
  Own the live estimator kernel:
    IMU update  -> Madgwick attitude update -> vertical acceleration -> predict
    Baro update -> relative altitude measurement -> update

DESIGN CHANGE
  This module follows the validated estimator pipeline while preserving SystemState
  publication, validity flags, sequence counters, and safety-compatible estimator
  output.
===============================================================================
*/

// If no explicit pressure baseline has been captured, the estimator falls back
// to the first trusted live barometric altitude as its local zero reference.
static bool g_live_ref_set = false;
static float g_live_ref_alt = 0.0f;

// Private Kalman state and last-IMU timestamp are owned here so only this module
// controls the predict/correct sequencing.
static KfAlt2State g_kf;
static uint32_t g_last_imu_us = 0;

/*
estimation_reset(...)
------------------------------------------------------------------------------
ROLE
  Reset live relative-altitude reference, private Kalman state, and estimator
  timing.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  state.est is invalidated. Private reference and Kalman state are reset.

MECHANISM
  1. Clear published estimator validity.
  2. Clear live reference altitude.
  3. Clear last IMU timestamp.
  4. Reset private Kalman filter.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No hardware I/O. No dynamic allocation.
*/
void estimation_reset(SystemState &state) {
  // Public estimate is invalidated immediately so downstream consumers cannot
  // mistake reset state for a fresh altitude solution.
  state.est.valid = false;
  state.est.updated = false;
  state.est.seeded = false;
  state.est.h_m = 0.0f;
  state.est.v_mps = 0.0f;
  state.est.a_mps2 = NAN;
  state.est.altitude_m = NAN;
  state.est.vz_mps = NAN;
  state.est.P00 = 1.0f;
  state.est.P01 = 0.0f;
  state.est.P10 = 0.0f;
  state.est.P11 = 1.0f;

  state.flight = FlightState();
  state.kf = KfAlt2State();

  g_live_ref_set = false;
  g_live_ref_alt = 0.0f;
  g_last_imu_us = 0;

  // The private filter state is reset separately so its seed state and
  // covariance cannot leak across reference-frame changes.
  kf_alt2_reset(g_kf);
}

/*
estimation_relative_altitude(...)
------------------------------------------------------------------------------
ROLE
  Convert current barometric altitude into a relative altitude measurement.

INPUT CONTRACT
  state.baro.valid must be true and state.baro.alt_m must be finite. If
  state.cfg.baro_baseline_hpa is used, state.cfg.sea_level_hpa must also be
  finite and positive.

OUTPUT CONTRACT
  Returns relative altitude in meters. Returns NAN if no trusted altitude
  measurement exists.

MECHANISM
  1. Reject invalid barometer data.
  2. If a finite positive baseline pressure exists, convert it into baseline
     altitude and subtract it from current altitude.
  3. Otherwise, auto-capture first valid barometer altitude as live zero.
  4. Return altitude relative to selected zero.

FAILURE BEHAVIOR
  Invalid barometer data returns NAN and does not update the filter.

DETERMINISM
  Constant-time scalar math. No hardware I/O. No dynamic allocation.
*/
float estimation_relative_altitude(SystemState &state) {
  if (!state.baro.valid || !is_finite_f(state.baro.alt_m)) {
    return NAN;
  }

  if (is_finite_f(state.cfg.baro_baseline_hpa) &&
      state.cfg.baro_baseline_hpa > 0.0f) {
    // When a ground pressure baseline is available, convert that pressure to the
    // equivalent standard-atmosphere altitude and subtract it from the current
    // standard-atmosphere altitude. This preserves a consistent barometric
    // relative-altitude frame.
    const float base_alt =
      pressure_to_altitude_m(state.cfg.baro_baseline_hpa, state.cfg.sea_level_hpa);

    return state.baro.alt_m - base_alt;
  }

  if (!g_live_ref_set) {
    // Auto-capture the first trusted altitude only once; repeated resetting
    // would destroy continuity of the estimator state.
    g_live_ref_alt = state.baro.alt_m;
    g_live_ref_set = true;
  }

  return state.baro.alt_m - g_live_ref_alt;
}

/*
publish_estimator(...)
------------------------------------------------------------------------------
ROLE
  Publish private Kalman state into SystemState.

INPUT CONTRACT
  state must refer to the shared SystemState object. now_ms and now_us must be
  current monotonic timestamps.

OUTPUT CONTRACT
  state.est receives state, covariance, timestamps, validity, and sequence.

MECHANISM
  1. Validate private Kalman state.
  2. Copy state and covariance.
  3. Copy last vertical acceleration if available.
  4. Publish timestamps.
  5. Increment sequence.

FAILURE BEHAVIOR
  If the private Kalman state is invalid, state.est.valid is false.

DETERMINISM
  Constant-time. No hardware I/O. No dynamic allocation.
*/
static void publish_estimator(SystemState &state, uint32_t now_us, uint32_t now_ms) {
  // Validity is derived from the private Kalman state rather than from whether a
  // specific sensor updated this cycle.
  state.est.valid = kf_alt2_is_valid(g_kf);
  state.est.updated = true;
  state.est.seeded = g_kf.seeded;

  state.est.t_us = now_us;
  state.est.t_ms = now_ms;
  ++state.est.seq;

  state.est.h_m = g_kf.h_m;
  state.est.v_mps = g_kf.v_mps;
  state.est.a_mps2 = state.auxvz.valid ? state.auxvz.a_vertical : NAN;
  state.est.altitude_m = g_kf.h_m;
  state.est.vz_mps = g_kf.v_mps;

  state.est.P00 = g_kf.P00;
  state.est.P01 = g_kf.P01;
  state.est.P10 = g_kf.P10;
  state.est.P11 = g_kf.P11;

  state.flight.valid = state.est.valid;
  state.flight.updated = true;
  state.flight.t_us = now_us;
  state.flight.t_ms = now_ms;
  ++state.flight.seq;
  state.flight.altitude_m = g_kf.h_m;
  state.flight.vz_mps = g_kf.v_mps;

  state.kf = g_kf;
}

/*
estimation_update(...)
------------------------------------------------------------------------------
ROLE
  Execute one estimator consumption pass using updated IMU and barometer
  snapshots.

INPUT CONTRACT
  state.imu.updated indicates a new IMU sample. state.baro.updated indicates a
  new barometer sample. now_ms is the current scheduler timestamp.

OUTPUT CONTRACT
  The private Kalman filter is seeded, predicted, and/or corrected as data
  arrives. state.est is published whenever prediction or correction occurs.

MECHANISM
  1. If a new IMU sample exists, compute measured IMU dt.
  2. Reject first IMU sample as timing initialization only.
  3. Reject unreasonable dt.
  4. Update quaternion attitude using gyro and accelerometer.
  5. Compute quaternion-based vertical acceleration.
  6. Seed Kalman from barometer if needed.
  7. Predict Kalman using vertical acceleration and measured dt.
  8. If a new barometer sample exists, compute relative altitude and update.
  9. Publish estimator state if prediction or update occurred.
  10. Refresh per-pass updated observability flags.

FAILURE BEHAVIOR
  Invalid IMU data prevents prediction. Invalid barometer data prevents update.
  Stale estimator output is not newly validated unless a coherent update path ran.

DETERMINISM
  Constant-time scalar math. No hardware I/O. No dynamic allocation.
*/
bool estimation_update(SystemState &state, uint32_t now_ms) {
  const uint32_t now_us = micros();

  // updated flags are publication observability for the current scheduler pass.
  state.attitude.updated = false;
  state.auxvz.updated = false;
  state.est.updated = false;
  state.flight.updated = false;

  // estimator_changed tracks whether predict or correct math actually advanced
  // the private filter. Publication happens only when state meaningfully moves.
  bool estimator_changed = false;

  if (state.imu.updated) {
    if (g_last_imu_us == 0U) {
      // The very first IMU sample establishes the timing origin only. Without a
      // previous timestamp, no measured dt exists for safe integration.
      g_last_imu_us = state.imu.t_us;
    } else {
      const float dt_s = (state.imu.t_us - g_last_imu_us) * 1.0e-6f;
      g_last_imu_us = state.imu.t_us;

      if (dt_s > EST_MIN_IMU_DT_S && dt_s < EST_MAX_IMU_DT_S) {
        // The IMU mounting/sign convention in this branch requires negating the
        // accelerometer vector before handing it to Madgwick. This keeps the
        // gravity-direction correction consistent with the validated reference
        // estimator.
        const bool attitude_ok = attitude_update_imu(
          state,
          state.imu.gx,
          state.imu.gy,
          state.imu.gz,
          -state.imu.ax,
          -state.imu.ay,
          -state.imu.az,
          dt_s,
          state.imu.t_us,
          now_ms
        );

        if (attitude_ok) {
          // Once attitude is updated, the same cycle's accelerometer sample can
          // be rotated into world vertical acceleration for the Kalman predictor.
          const float a_vertical = attitude_compute_vertical_accel(
            state,
            state.imu.ax,
            state.imu.ay,
            state.imu.az
          );

          // The derived-acceleration publication is explicitly rebuilt here so
          // telemetry and downstream logic can tell whether this cycle produced a
          // fresh world-vertical acceleration estimate.
          state.auxvz.valid = false;
          state.auxvz.updated = false;
          state.auxvz.t_us = state.imu.t_us;
          state.auxvz.t_ms = now_ms;
          state.auxvz.a_vertical = NAN;
          state.auxvz.a_mps2 = NAN;
          state.auxvz.a_wz_mps2 = NAN;
          state.auxvz.a_lin_z_mps2 = NAN;

          if (is_finite_f(a_vertical)) {
            state.auxvz.valid = true;
            state.auxvz.updated = true;
            state.auxvz.a_vertical = a_vertical;
            state.auxvz.a_mps2 = a_vertical;
            state.auxvz.a_wz_mps2 = a_vertical;
            state.auxvz.a_lin_z_mps2 = a_vertical;
            ++state.auxvz.seq;

            if (!g_kf.seeded) {
              // The filter cannot predict meaningfully until altitude has been
              // anchored to the same reference frame used by the barometer.
              const float z0 = estimation_relative_altitude(state);

              if (is_finite_f(z0)) {
                kf_alt2_seed(g_kf, z0);
              }
            }

            if (g_kf.seeded) {
              // Predict uses measured dt and measured world-vertical
              // acceleration, which preserves real sensor timing instead of
              // assuming a perfectly fixed scheduler cadence.
              kf_alt2_predict(g_kf, a_vertical, dt_s);
              estimator_changed = true;
            }
          }
        }
      }
    }

  }

  if (state.baro.updated) {
    const float z_meas = estimation_relative_altitude(state);

    if (is_finite_f(z_meas)) {
      if (!g_kf.seeded) {
        // A valid altitude measurement can also seed the filter directly even if
        // no usable IMU-driven prediction has occurred yet.
        kf_alt2_seed(g_kf, z_meas);
      } else {
        // Barometric altitude is the absolute observation that arrests drift in
        // the integrated vertical state.
        kf_alt2_update(g_kf, z_meas);
      }

      estimator_changed = true;
    }

  }

  if (estimator_changed) {
    publish_estimator(state, now_us, now_ms);
  } else if (!g_kf.seeded) {
    // Before the filter has a trusted reference, invalid is more truthful than
    // publishing zeros that might be misread as a real estimate.
    state.est.valid = false;
    state.flight.valid = false;
  }

  return estimator_changed;
}

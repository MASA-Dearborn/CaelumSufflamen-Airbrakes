#include "estimation.h"

#include "config.h"
#include "math_utils.h"
#include "kalman_alt2.h"

/*
estimation.cpp
===============================================================================
PURPOSE
  Own the estimator epoch: barometer snapshot -> altitude measurement -> Kalman
  predict/update -> FlightState publication.

BOUNDARY
  This module does not read hardware. It consumes already-published snapshots.
  This preserves traceability: every estimator output can be traced to snapshot
  validity, timestamp, and numeric payload.

FAIL-SAFE RULE
  Invalid barometer input invalidates the published FlightState for this epoch.
  It does not reuse stale altitude or velocity.
===============================================================================
*/

static uint32_t g_last_est_ms = 0;

/*
estimation_begin(...)
------------------------------------------------------------------------------
ROLE
  Initialize the estimator module and clear the Kalman seed state.

INPUT CONTRACT
  - kf is the persistent filter object owned by SystemState.

OUTPUT CONTRACT
  - kf is initialized with conservative covariance and unseeded state.
  - The estimator scheduler timestamp is initialized to current millis().

MECHANISM
  The first valid pressure-derived altitude measurement will later seed the
  filter. This avoids inventing altitude before a valid measurement exists.

DETERMINISM
  Constant-time assignments. No hardware I/O. No heap allocation.
*/
void estimation_begin(KalmanAlt2& kf) {
  kf.x0 = 0.0f;
  kf.x1 = 0.0f;

  // Conservative prior; actual state remains unusable until seeded == true.
  kf.P00 = 25.0f;
  kf.P01 = 0.0f;
  kf.P10 = 0.0f;
  kf.P11 = 25.0f;

  // Initial tuning placeholders. These should be validated from ground data.
  kf.q_acc = 4.0f;
  kf.r_meas = 4.0f;

  // The filter must not publish valid estimates before first valid altitude.
  kf.seeded = false;
  kf.t_ms = millis();

  // Establish scheduler baseline to avoid a startup burst.
  g_last_est_ms = millis();
}

/*
estimation_update(...)
------------------------------------------------------------------------------
ROLE
  Execute one estimator epoch if the estimator period has elapsed.

INPUT CONTRACT
  - baro is consumed only if baro.valid == true and press_hpa is finite.
  - aux_vz is optional. If invalid, prediction uses zero acceleration.
  - cfg must contain a finite sea-level pressure or finite baseline pressure.

OUTPUT CONTRACT
  - flight.t_ms is updated whenever the estimator epoch is due.
  - flight.valid is true only after a coherent seed/update path.
  - flight numeric fields are meaningful only when flight.valid == true.

MECHANISM
  1. Time-gate the estimator epoch.
  2. Invalidate FlightState by default for the due epoch.
  3. Reject invalid barometer input.
  4. Select pressure reference:
       baseline pressure if available, otherwise sea-level pressure.
  5. Convert pressure to altitude.
  6. Seed Kalman filter if not yet seeded.
  7. Predict using aux vertical acceleration if valid.
  8. Update using barometric altitude.
  9. Publish fused altitude and vertical speed.

SAFETY / FAILURE BEHAVIOR
  Any invalid measurement path returns with flight.valid == false. This prevents
  stale estimator values from authorizing policy or actuation.

DETERMINISM
  Time-gated. Bounded scalar math. No sensor I/O. No dynamic allocation.
*/
bool estimation_update(
  const BaroSample& baro,
  const AuxVertLinAccel& aux_vz,
  const Config& cfg,
  KalmanAlt2& kf,
  FlightState& flight
) {
  const uint32_t now_ms = millis();

  // Epoch gate: if not due, leave the previously published FlightState untouched.
  if ((now_ms - g_last_est_ms) < EST_PERIOD_MS) return false;
  g_last_est_ms = now_ms;

  // A due estimator attempt always updates timestamp and begins invalid.
  // Validity is asserted only after all required checks pass.
  flight.t_ms = now_ms;
  flight.valid = false;

  // The estimator measurement z comes from barometric pressure. Invalid pressure
  // means no legitimate measurement exists for this epoch.
  if (!baro.valid || !isfinite(baro.press_hpa)) {
    return true;
  }

  // Baseline pressure gives pad-relative altitude. If no baseline has been
  // captured, fall back to sea-level reference pressure.
  const float ref_hpa =
    isfinite(cfg.baro_baseline_hpa) ? cfg.baro_baseline_hpa : cfg.sea_level_hpa;

  // Convert pressure measurement to altitude measurement.
  const float z_alt_m = pressure_to_altitude_m(baro.press_hpa, ref_hpa);
  if (!isfinite(z_alt_m)) {
    return true;
  }

  // First valid altitude measurement initializes the filter. No prediction is
  // performed because there is no previous trusted time/state pair yet.
  if (!kf.seeded) {
    kf_alt2_reset(kf, z_alt_m, now_ms);

    flight.altitude_m = kf.x0;
    flight.vz_mps = kf.x1;
    flight.valid = kf_alt2_is_valid(kf);
    return true;
  }

  // Compute elapsed time since the last filter update. millis() subtraction is
  // intentionally unsigned to remain well-defined across wraparound.
  const float dt_s = (now_ms - kf.t_ms) * 0.001f;

  if (isfinite(dt_s) && dt_s > 0.0001f) {
    // Acceleration is optional. If unavailable, the model reduces to a
    // constant-velocity prediction with zero control input.
    const float u_mps2 =
      (aux_vz.valid && isfinite(aux_vz.a_lin_z_mps2))
        ? aux_vz.a_lin_z_mps2
        : 0.0f;

    kf_alt2_predict_u(kf, dt_s, u_mps2);
  }

  // Correct prediction using barometric altitude.
  kf_alt2_update_z(kf, z_alt_m);
  kf.t_ms = now_ms;

  // Publish the fused estimate. Validity is tied to filter internal sanity.
  flight.altitude_m = kf.x0;
  flight.vz_mps = kf.x1;
  flight.valid = kf_alt2_is_valid(kf);

  return true;
}
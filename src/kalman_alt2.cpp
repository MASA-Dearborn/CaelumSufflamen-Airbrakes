#include "kalman_alt2.h"

#include <math.h>

/*
kalman_alt2.cpp
===============================================================================
PURPOSE
  Implements a deterministic, allocation-free, scalar 2-state Kalman filter for
  vertical flight estimation.

STATE MODEL
  x0 = altitude [m]
  x1 = vertical speed [m/s]

  x0(k+1) = x0(k) + dt*x1(k) + 0.5*dt^2*u
  x1(k+1) = x1(k) + dt*u

MEASUREMENT MODEL
  z = altitude [m]

DESIGN DECISION
  The implementation uses hand-expanded 2x2 algebra instead of a matrix library.
  This keeps runtime bounded, avoids heap allocation, and makes each covariance
  operation auditable line-by-line.

NUMERICAL POLICY
  The measurement update uses Joseph-form covariance propagation:
    P+ = (I-KH)P-(I-KH)^T + KRK^T

  Joseph form is more verbose than the simplified covariance update, but it is
  more robust against negative variances caused by finite-precision arithmetic.
===============================================================================
*/

/*
kf_alt2_reset(...)
------------------------------------------------------------------------------
ROLE
  Seed or re-seed the altitude filter from a trusted altitude measurement.

INPUT CONTRACT
  - alt0_m should be finite and expressed in meters.
  - now_ms should be the current monotonic scheduler timestamp from millis().
  - This function does not itself reject NaN altitude; upstream estimator logic
    is expected to validate the barometric altitude before seeding.

OUTPUT CONTRACT
  - State becomes [altitude = alt0_m, vertical speed = 0].
  - Covariance is reset to a conservative diagonal prior.
  - seeded is asserted.
  - t_ms is updated to align filter time with the seeding measurement.

MECHANISM
  1. Store altitude directly.
  2. Initialize vertical speed to zero because no derivative estimate exists yet.
  3. Use diagonal covariance so altitude and velocity begin uncorrelated.
  4. Mark the filter seeded.

SAFETY / FAILURE BEHAVIOR
  This function intentionally does not command hardware and does not publish
  FlightState directly. It only mutates the internal filter object.

DETERMINISM
  Constant-time scalar assignments. No loops. No I/O. No heap allocation.
*/
void kf_alt2_reset(KalmanAlt2& kf, float alt0_m, uint32_t now_ms) {
  // Store the posterior state estimate at initialization.
  // Altitude is trusted because the estimator has already validated z_alt_m.
  kf.x0 = alt0_m;

  // Vertical speed is unknown at first altitude lock. A neutral zero prior is
  // chosen so the first few baro updates can infer velocity through covariance.
  kf.x1 = 0.0f;

  // Conservative diagonal covariance:
  //   P00 = 25 m^2     -> sigma_alt ≈ 5 m
  //   P11 = 25 (m/s)^2 -> sigma_vz  ≈ 5 m/s
  // These are not universal constants; they are safe placeholders for bring-up.
  kf.P00 = 25.0f;
  kf.P01 = 0.0f;
  kf.P10 = 0.0f;
  kf.P11 = 25.0f;

  // Administrative publication state for the filter.
  kf.seeded = true;
  kf.t_ms = now_ms;
}

/*
kf_alt2_predict_u(...)
------------------------------------------------------------------------------
ROLE
  Advance the state and covariance forward in time using vertical acceleration
  as the control input.

INPUT CONTRACT
  - kf.seeded must be true.
  - dt_s must be finite and positive.
  - u_acc_mps2 must be finite.
  - kf.q_acc must be finite and non-negative because it represents process-noise
    intensity.

OUTPUT CONTRACT
  - x0 and x1 are propagated according to constant-acceleration kinematics.
  - P is propagated by F P F^T + Q.
  - Symmetry is explicitly restored by assigning P10 = P01.

MECHANISM
  1. Reject invalid time/control/noise inputs.
  2. Propagate state mean.
  3. Snapshot old covariance terms.
  4. Compute F P F^T for F = [[1,dt],[0,1]].
  5. Add discretized white-acceleration process noise.
  6. Write the predicted covariance back into the filter.

SAFETY / FAILURE BEHAVIOR
  Invalid inputs cause a no-op return. This prevents NaNs from spreading through
  the estimator state and then into policy or telemetry.

DETERMINISM
  Constant-time scalar math. No loops. No I/O. No heap allocation.
*/
void kf_alt2_predict_u(KalmanAlt2& kf, float dt_s, float u_acc_mps2) {
  // A filter that has not been seeded has no meaningful state to propagate.
  if (!kf.seeded) return;

  // Reject non-physical or numerically unsafe inputs before any state mutation.
  if (!isfinite(dt_s) || !(dt_s > 0.0f)) return;
  if (!isfinite(u_acc_mps2)) return;
  if (!isfinite(kf.q_acc) || kf.q_acc < 0.0f) return;

  // Precompute powers of dt so the equations below mirror the continuous-time
  // kinematic derivation and avoid repeated multiplication.
  const float dt2 = dt_s * dt_s;

  // --------------------------------------------------------------------------
  // 1) State prediction
  // --------------------------------------------------------------------------
  // Altitude integrates both velocity and acceleration.
  kf.x0 = kf.x0 + dt_s * kf.x1 + 0.5f * dt2 * u_acc_mps2;

  // Vertical speed integrates acceleration.
  kf.x1 = kf.x1 + dt_s * u_acc_mps2;

  // --------------------------------------------------------------------------
  // 2) Covariance prediction
  // --------------------------------------------------------------------------
  // Snapshot the previous covariance. This avoids accidental dependence on
  // partially updated terms while expanding F P F^T.
  const float P00 = kf.P00;
  const float P01 = kf.P01;
  const float P10 = kf.P10;
  const float P11 = kf.P11;

  // For F = [[1,dt],[0,1]]:
  //   Pp00 = P00 + dt*(P10 + P01) + dt^2*P11
  //   Pp01 = P01 + dt*P11
  //   Pp10 = P10 + dt*P11
  //   Pp11 = P11
  const float nP00_ap = P00 + dt_s * (P10 + P01) + dt2 * P11;
  const float nP01_ap = P01 + dt_s * P11;
  const float nP11_ap = P11;

  // --------------------------------------------------------------------------
  // 3) Process-noise discretization
  // --------------------------------------------------------------------------
  // q_acc represents continuous-time white acceleration noise. For a 2-state
  // position/velocity model, the standard discrete Q is:
  //   Q00 = q*dt^4/4
  //   Q01 = q*dt^3/2
  //   Q11 = q*dt^2
  const float dt3 = dt2 * dt_s;
  const float dt4 = dt2 * dt2;

  const float q = kf.q_acc;
  const float Q00 = 0.25f * dt4 * q;
  const float Q01 = 0.50f * dt3 * q;
  const float Q11 = dt2 * q;

  // Write the predicted covariance. Symmetry is enforced explicitly because
  // small float asymmetries can otherwise accumulate over many iterations.
  kf.P00 = nP00_ap + Q00;
  kf.P01 = nP01_ap + Q01;
  kf.P10 = kf.P01;
  kf.P11 = nP11_ap + Q11;
}

/*
kf_alt2_update_z(...)
------------------------------------------------------------------------------
ROLE
  Correct the predicted altitude/velocity state using a barometric altitude
  measurement.

INPUT CONTRACT
  - kf.seeded must be true.
  - z_alt_m must be finite.
  - kf.r_meas must be finite and positive.

OUTPUT CONTRACT
  - x0 and x1 are corrected by Kalman gain.
  - P is updated using Joseph form.
  - Off-diagonal covariance is symmetrized.

MECHANISM
  1. Reject invalid measurement/noise state.
  2. Compute innovation y = z - Hx.
  3. Compute innovation variance S = HPH^T + R = P00 + R.
  4. Compute Kalman gain K = PH^T/S.
  5. Correct state mean.
  6. Apply Joseph covariance update.
  7. Enforce symmetry.

SAFETY / FAILURE BEHAVIOR
  If S is non-positive or too small, the measurement update is skipped. This
  avoids division by zero and protects covariance from invalid arithmetic.

DETERMINISM
  Constant-time scalar math. No loops. No I/O. No heap allocation.
*/
void kf_alt2_update_z(KalmanAlt2& kf, float z_alt_m) {
  if (!kf.seeded) return;
  if (!isfinite(z_alt_m)) return;
  if (!isfinite(kf.r_meas) || !(kf.r_meas > 0.0f)) return;

  // Innovation: measurement information not already explained by prediction.
  const float y = z_alt_m - kf.x0;

  // For H = [1 0], innovation variance reduces to altitude variance + R.
  const float S = kf.P00 + kf.r_meas;

  // Guard against singular or negative innovation variance.
  if (!(S > 1.0e-9f)) return;

  const float invS = 1.0f / S;

  // Kalman gain:
  //   K0 corrects altitude directly.
  //   K1 corrects velocity through altitude-velocity covariance coupling.
  const float K0 = kf.P00 * invS;
  const float K1 = kf.P10 * invS;

  // --------------------------------------------------------------------------
  // 1) State correction
  // --------------------------------------------------------------------------
  kf.x0 = kf.x0 + K0 * y;
  kf.x1 = kf.x1 + K1 * y;

  // Snapshot predicted covariance before overwriting it.
  const float P00 = kf.P00;
  const float P01 = kf.P01;
  const float P10 = kf.P10;
  const float P11 = kf.P11;
  const float R = kf.r_meas;

  // --------------------------------------------------------------------------
  // 2) Joseph covariance correction
  // --------------------------------------------------------------------------
  // For H = [1 0]:
  //   A = I - K H = [[1-K0, 0],
  //                  [-K1,  1]]
  const float a00 = 1.0f - K0;
  const float a10 = -K1;

  // B = A * P
  const float b00 = a00 * P00;
  const float b01 = a00 * P01;
  const float b10 = a10 * P00 + P10;
  const float b11 = a10 * P01 + P11;

  // nP = B * A^T
  float nP00 = b00 * a00;
  float nP01 = b00 * a10 + b01;
  float nP10 = b10 * a00;
  float nP11 = b10 * a10 + b11;

  // Add measurement-noise contribution K R K^T.
  nP00 += K0 * R * K0;
  nP01 += K0 * R * K1;
  nP10 += K1 * R * K0;
  nP11 += K1 * R * K1;

  // Numerical symmetry repair.
  const float sym01 = 0.5f * (nP01 + nP10);

  kf.P00 = nP00;
  kf.P01 = sym01;
  kf.P10 = sym01;
  kf.P11 = nP11;
}
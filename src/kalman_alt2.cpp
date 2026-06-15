#include "kalman_alt2.h"

#include <math.h>

#include "config.h"
#include "math_utils.h"

/*
kalman_alt2.cpp
===============================================================================
PURPOSE
  Deterministic scalar Kalman filter implementation for altitude and vertical
  velocity estimation.

DESIGN CHANGE
  The live estimator uses measured IMU dt rather than fixed kTs. This follows the
  validated reference estimator and better matches multirate sensor fusion.

STATE MODEL
  h(k+1) = h(k) + v(k)*dt + 0.5*a*dt^2
  v(k+1) = v(k) + a*dt

MEASUREMENT MODEL
  z = h + noise

COVARIANCE MODEL
  P is propagated by F P F^T + Q, where Q is discretized from acceleration
  process noise intensity kSigmaA2 using measured dt.
===============================================================================
*/

/*
kf_alt2_reset(...)
------------------------------------------------------------------------------
ROLE
  Return the filter to an unseeded conservative state.

INPUT CONTRACT
  kf must refer to a valid KfAlt2State object.

OUTPUT CONTRACT
  seeded is cleared. State is reset to zero. Covariance is reset to identity.

MECHANISM
  1. Clear seed flag.
  2. Reset altitude and velocity.
  3. Reset covariance diagonal.
  4. Clear covariance off-diagonal terms.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time assignments. No loops. No hardware I/O. No dynamic allocation.
*/
void kf_alt2_reset(KfAlt2State &kf) {
  kf.seeded = false;

  kf.h_m = 0.0f;
  kf.v_mps = 0.0f;

  kf.P00 = 1.0f;
  kf.P01 = 0.0f;
  kf.P10 = 0.0f;
  kf.P11 = 1.0f;
}

/*
kf_alt2_seed(...)
------------------------------------------------------------------------------
ROLE
  Initialize the filter from the first trusted relative altitude measurement.

INPUT CONTRACT
  h0_m must be finite and expressed in meters.

OUTPUT CONTRACT
  If h0_m is finite, the filter becomes seeded with altitude h0_m and zero
  velocity.

MECHANISM
  1. Reject non-finite seed altitude.
  2. Mark filter seeded.
  3. Store altitude directly.
  4. Initialize vertical speed to zero.
  5. Reset covariance to a known diagonal prior.

FAILURE BEHAVIOR
  Non-finite seed altitude resets the filter to unseeded state.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
void kf_alt2_seed(KfAlt2State &kf, float h0_m) {
  if (!is_finite_f(h0_m)) {
    kf_alt2_reset(kf);
    return;
  }

  // Seeding defines the altitude reference frame and declares the state
  // trustworthy enough for later predict/correct operations.
  kf.seeded = true;

  kf.h_m = h0_m;
  kf.v_mps = 0.0f;

  kf.P00 = 1.0f;
  kf.P01 = 0.0f;
  kf.P10 = 0.0f;
  kf.P11 = 1.0f;
}

/*
kf_alt2_predict(...)
------------------------------------------------------------------------------
ROLE
  Advance altitude and velocity using measured vertical acceleration and measured
  sample interval.

INPUT CONTRACT
  The filter must be seeded. a_vertical_mps2 must be finite. dt_s must be finite
  and positive.

OUTPUT CONTRACT
  On valid input, h_m, v_mps, and covariance are advanced by dt_s.

MECHANISM
  1. Reject unseeded filter, invalid acceleration, or invalid dt.
  2. Compute dt powers.
  3. Propagate altitude and velocity using constant-acceleration kinematics.
  4. Snapshot prior covariance.
  5. Compute F P F^T for F = [[1,dt],[0,1]].
  6. Compute Q(dt) from acceleration process noise intensity.
  7. Commit predicted covariance and enforce symmetry.

FAILURE BEHAVIOR
  Invalid inputs cause a no-op return. This prevents NaNs from contaminating the
  estimator and downstream safety/policy layers.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
void kf_alt2_predict(KfAlt2State &kf, float a_vertical_mps2, float dt_s) {
  if (!kf.seeded) return;
  if (!is_finite_f(a_vertical_mps2)) return;
  if (!is_finite_f(dt_s) || !(dt_s > 0.0f)) return;

  // dt powers appear repeatedly in both the kinematic state update and the
  // discretized process-noise covariance.
  const float dt2 = dt_s * dt_s;
  const float dt3 = dt2 * dt_s;
  const float dt4 = dt2 * dt2;

  // State prediction under constant acceleration across the measured interval.
  kf.h_m = kf.h_m + kf.v_mps * dt_s + 0.5f * a_vertical_mps2 * dt2;
  kf.v_mps = kf.v_mps + a_vertical_mps2 * dt_s;

  // Snapshot covariance so all predicted entries use the same prior matrix.
  const float P00 = kf.P00;
  const float P01 = kf.P01;
  const float P10 = kf.P10;
  const float P11 = kf.P11;

  // Q is the continuous white-acceleration process noise discretized over dt.
  // Larger dt or larger acceleration uncertainty expands state covariance
  // because more unmodeled vertical motion could have occurred.
  const float Q00 = kSigmaA2 * dt4 / 4.0f;
  const float Q01 = kSigmaA2 * dt3 / 2.0f;
  const float Q11 = kSigmaA2 * dt2;

  // For F = [1 dt; 0 1], the predicted covariance terms can be written out
  // explicitly. This avoids general matrix code while preserving the same
  // linear-systems mathematics.
  const float P00_new = P00 + dt_s * (P10 + P01) + dt2 * P11 + Q00;
  const float P01_new = P01 + dt_s * P11 + Q01;
  const float P11_new = P11 + Q11;

  kf.P00 = P00_new;
  // P must remain symmetric for a physically meaningful covariance matrix, so
  // the off-diagonal terms are written identically.
  kf.P01 = P01_new;
  kf.P10 = P01_new;
  kf.P11 = P11_new;
}

/*
kf_alt2_update(...)
------------------------------------------------------------------------------
ROLE
  Correct predicted altitude/velocity using barometric altitude.

INPUT CONTRACT
  The filter must be seeded. z_meas_m must be finite.

OUTPUT CONTRACT
  On valid input, h_m, v_mps, and covariance are corrected using the scalar
  altitude measurement update.

MECHANISM
  1. Reject unseeded filter or invalid measurement.
  2. Compute innovation variance S = P00 + R.
  3. Reject non-positive S.
  4. Compute Kalman gain K = P H^T / S.
  5. Correct state mean.
  6. Apply Joseph-form covariance update.
  7. Symmetrize off-diagonal covariance.

FAILURE BEHAVIOR
  Invalid measurement or singular innovation variance causes a no-op update.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
void kf_alt2_update(KfAlt2State &kf, float z_meas_m) {
  if (!kf.seeded) return;
  if (!is_finite_f(z_meas_m)) return;

  const float y = z_meas_m - kf.h_m;
  const float S = kf.P00 + kR;

  if (!(S > 1.0e-9f)) return;

  const float invS = 1.0f / S;

  const float K0 = kf.P00 * invS;
  const float K1 = kf.P10 * invS;

  // The innovation y is the measured-minus-predicted altitude mismatch. The
  // Kalman gain distributes that mismatch into altitude and vertical-speed
  // corrections according to current covariance.
  kf.h_m = kf.h_m + K0 * y;
  kf.v_mps = kf.v_mps + K1 * y;

  const float P00 = kf.P00;
  const float P01 = kf.P01;
  const float P10 = kf.P10;
  const float P11 = kf.P11;

  // Joseph-form covariance update:
  //   P = (I-KH) P (I-KH)^T + K R K^T
  //
  // This form is more numerically robust than the simplified textbook update
  // because it better preserves positive semidefiniteness under finite
  // precision.
  const float a00 = 1.0f - K0;
  const float a10 = -K1;

  const float b00 = a00 * P00;
  const float b01 = a00 * P01;
  const float b10 = a10 * P00 + P10;
  const float b11 = a10 * P01 + P11;

  float nP00 = b00 * a00;
  float nP01 = b00 * a10 + b01;
  float nP10 = b10 * a00;
  float nP11 = b10 * a10 + b11;

  nP00 += K0 * kR * K0;
  nP01 += K0 * kR * K1;
  nP10 += K1 * kR * K0;
  nP11 += K1 * kR * K1;

  // Explicit symmetrization removes tiny numerical asymmetries that can build
  // up over repeated updates and later confuse downstream reasoning about P.
  const float sym01 = 0.5f * (nP01 + nP10);

  kf.P00 = nP00;
  kf.P01 = sym01;
  kf.P10 = sym01;
  kf.P11 = nP11;
}

/*
kf_alt2_is_valid(...)
------------------------------------------------------------------------------
ROLE
  Validate filter state before publishing it.

INPUT CONTRACT
  kf must refer to a valid KfAlt2State object.

OUTPUT CONTRACT
  Returns true only when seeded and all state/covariance terms are finite.

MECHANISM
  1. Check seed state.
  2. Check altitude and velocity finiteness.
  3. Check covariance finiteness.

FAILURE BEHAVIOR
  Any invalid term returns false.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
bool kf_alt2_is_valid(const KfAlt2State &kf) {
  return kf.seeded &&
         is_finite_f(kf.h_m) &&
         is_finite_f(kf.v_mps) &&
         is_finite_f(kf.P00) &&
         is_finite_f(kf.P01) &&
         is_finite_f(kf.P10) &&
         is_finite_f(kf.P11);
}

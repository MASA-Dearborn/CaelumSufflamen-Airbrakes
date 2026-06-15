#include "attitude.h"

#include <math.h>

#include "config.h"
#include "math_utils.h"

/*
attitude.cpp
===============================================================================
PURPOSE
  Own the Madgwick IMU quaternion update and quaternion-based vertical
  acceleration projection.

DESIGN 
  This file implements the estimator kernel: gyro integrates angular motion,
  accelerometer constrains gravity direction, and the quaternion rotates measured
  acceleration into the world vertical axis.
===============================================================================
*/

// Private quaternion state owned by this module. The rest of the firmware sees
// attitude only after this state has been fully updated and republished.
static float g_q0 = 1.0f;
static float g_q1 = 0.0f;
static float g_q2 = 0.0f;
static float g_q3 = 0.0f;

/*
attitude_publish(...)
------------------------------------------------------------------------------
ROLE
  Publish the private quaternion state into SystemState.

INPUT CONTRACT
  state refers to the shared SystemState object. now_us and now_ms are
  current monotonic timestamps.

OUTPUT CONTRACT
  state.attitude receives the private quaternion state, validity is asserted, and
  the attitude sequence counter increments.

MECHANISM
  1. Copy private quaternion components into the published snapshot.
  2. Stamp the snapshot with microsecond and millisecond timestamps.
  3. Assert valid/updated flags.
  4. Increment sequence counter.


DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
static void attitude_publish(SystemState &state, uint32_t now_us, uint32_t now_ms) 
{
  // Publish all four components together so downstream users observe one
  // coherent unit quaternion rather than a partially updated state.
  state.attitude.q0 = g_q0;
  state.attitude.q1 = g_q1;
  state.attitude.q2 = g_q2;
  state.attitude.q3 = g_q3;

  // These timestamps describe when the attitude estimate became available to
  // the rest of the system.
  state.attitude.t_us = now_us;
  state.attitude.t_ms = now_ms;

  state.attitude.valid = true;
  state.attitude.updated = true;
  ++state.attitude.seq;
}

/*
attitude_begin(...)
------------------------------------------------------------------------------
ROLE
  Initialize attitude estimation state.

INPUT CONTRACT
  state refers to the shared SystemState object.

OUTPUT CONTRACT
  The private quaternion and published attitude snapshot are reset to identity
  rotation.

MECHANISM
  1. Reset private quaternion state.
  2. Reset published attitude snapshot.
  3. Mark attitude as invalid until the first successful IMU update.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
void attitude_begin(SystemState &state) 
{
  attitude_reset(state);
}

/*
attitude_reset(...)
------------------------------------------------------------------------------
ROLE
  Reset the attitude estimator to identity orientation.

INPUT CONTRACT
  state refers to the shared SystemState object.

OUTPUT CONTRACT
  Private quaternion becomes [1,0,0,0]. Published attitude is marked invalid
  until the next successful Madgwick update.

MECHANISM
  1. Store identity quaternion privately.
  2. Store identity quaternion in the published snapshot.
  3. Clear validity and update flags.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
void attitude_reset(SystemState &state) {
  // Identity quaternion represents zero rotation between body and world frames
  // until real IMU integration establishes orientation.
  g_q0 = 1.0f;
  g_q1 = 0.0f;
  g_q2 = 0.0f;
  g_q3 = 0.0f;

  state.attitude.q0 = g_q0;
  state.attitude.q1 = g_q1;
  state.attitude.q2 = g_q2;
  state.attitude.q3 = g_q3;

  state.attitude.valid = false;
  state.attitude.updated = false;
  state.attitude.t_ms = 0;
  state.attitude.t_us = 0;
}

/*
attitude_update_imu(...)
------------------------------------------------------------------------------
ROLE
  Execute one Madgwick IMU attitude update from gyro and accelerometer data.

INPUT CONTRACT
  gx, gy, gz must be finite angular rates in rad/s. ax, ay, az must be finite
  acceleration components. dt_s must be finite and positive.

OUTPUT CONTRACT
  On success, the private quaternion is advanced and normalized, then published
  into state.attitude. Returns true only when an update was applied.

MECHANISM
  1. Reject invalid dt or non-finite sensor values.
  2. Normalize accelerometer vector so it represents gravity direction.
  3. Compute Madgwick gradient-descent correction terms.
  4. Normalize the correction gradient.
  5. Compute quaternion derivative from gyro integration and beta correction.
  6. Integrate quaternion using Euler integration.
  7. Renormalize quaternion to unit length.
  8. Publish attitude snapshot.

FAILURE BEHAVIOR
  Invalid dt, invalid sensor values, zero accelerometer norm, or zero quaternion
  norm causes a no-op return. The previous attitude state is preserved.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
bool attitude_update_imu(
  SystemState &state,
  float gx,
  float gy,
  float gz,
  float ax,
  float ay,
  float az,
  float dt_s,
  uint32_t now_us,
  uint32_t now_ms
) {
  // Rejecting invalid dt protects the quaternion integrator from timestamp
  // corruption or startup transients.
  if (!is_finite_f(dt_s) || !(dt_s > 0.0f)) return false;

  // The Madgwick update requires both finite gyro and finite accelerometer
  // samples because the two terms jointly determine the quaternion correction.
  if (!is_finite_f(gx) || !is_finite_f(gy) || !is_finite_f(gz)) return false;
  if (!is_finite_f(ax) || !is_finite_f(ay) || !is_finite_f(az)) return false;

  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (!is_finite_f(norm) || norm < 1.0e-6f) return false;

  // Accelerometer normalization removes magnitude so the correction uses only
  // direction of measured gravity.
  ax /= norm;
  ay /= norm;
  az /= norm;

  // These repeated products are expanded once so the implementation mirrors the
  // analytical Madgwick equations without recomputing identical terms.
  const float twoQ0 = 2.0f * g_q0;
  const float twoQ1 = 2.0f * g_q1;
  const float twoQ2 = 2.0f * g_q2;
  const float twoQ3 = 2.0f * g_q3;

  const float fourQ0 = 4.0f * g_q0;
  const float fourQ1 = 4.0f * g_q1;
  const float fourQ2 = 4.0f * g_q2;

  const float eightQ1 = 8.0f * g_q1;
  const float eightQ2 = 8.0f * g_q2;

  const float q0q0 = g_q0 * g_q0;
  const float q1q1 = g_q1 * g_q1;
  const float q2q2 = g_q2 * g_q2;
  const float q3q3 = g_q3 * g_q3;

  // Gradient-descent correction for accelerometer/gravity alignment.
  //
  // The accelerometer is treated as a measurement of gravity direction. These
  // s0..s3 terms are the gradient of the objective function that measures the
  // mismatch between measured gravity direction and quaternion-implied gravity.
  float s0 =
    fourQ0 * q2q2 +
    twoQ2 * ax +
    fourQ0 * q1q1 -
    twoQ1 * ay;

  float s1 =
    fourQ1 * q3q3 -
    twoQ3 * ax +
    4.0f * q0q0 * g_q1 -
    twoQ0 * ay -
    fourQ1 +
    eightQ1 * q1q1 +
    eightQ1 * q2q2 +
    fourQ1 * az;

  float s2 =
    4.0f * q0q0 * g_q2 +
    twoQ0 * ax +
    fourQ2 * q3q3 -
    twoQ3 * ay -
    fourQ2 +
    eightQ2 * q1q1 +
    eightQ2 * q2q2 +
    fourQ2 * az;

  float s3 =
    4.0f * q1q1 * g_q3 -
    twoQ1 * ax +
    4.0f * q2q2 * g_q3 -
    twoQ2 * ay;

  // The gradient is normalized so MADGWICK_BETA acts as a clean correction-gain
  // parameter rather than coupling to raw gradient magnitude.
  norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (is_finite_f(norm) && norm > 1.0e-6f) {
    s0 /= norm;
    s1 /= norm;
    s2 /= norm;
    s3 /= norm;
  } else {
    s0 = 0.0f;
    s1 = 0.0f;
    s2 = 0.0f;
    s3 = 0.0f;
  }

  // q_dot = gyro integration term - beta * correction gradient.
  //
  // The gyro term advances the quaternion according to rigid-body kinematics.
  // The beta-weighted term damps drift by rotating the solution back toward the
  // gravity direction implied by the accelerometer.
  const float qDot0 =
    0.5f * (-g_q1 * gx - g_q2 * gy - g_q3 * gz) -
    MADGWICK_BETA * s0;

  const float qDot1 =
    0.5f * (g_q0 * gx + g_q2 * gz - g_q3 * gy) -
    MADGWICK_BETA * s1;

  const float qDot2 =
    0.5f * (g_q0 * gy - g_q1 * gz + g_q3 * gx) -
    MADGWICK_BETA * s2;

  const float qDot3 =
    0.5f * (g_q0 * gz + g_q1 * gy - g_q2 * gx) -
    MADGWICK_BETA * s3;

  // Explicit Euler integration is acceptable here because dt is small and the
  // subsequent renormalization keeps the quaternion on the unit sphere.
  g_q0 += qDot0 * dt_s;
  g_q1 += qDot1 * dt_s;
  g_q2 += qDot2 * dt_s;
  g_q3 += qDot3 * dt_s;

  // Unit-length enforcement is essential because the quaternion is used as a
  // rotation operator. A non-unit quaternion would distort projected vectors.
  norm = sqrtf(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);
  if (!is_finite_f(norm) || norm < 1.0e-6f) {
    attitude_reset(state);
    return false;
  }

  g_q0 /= norm;
  g_q1 /= norm;
  g_q2 /= norm;
  g_q3 /= norm;

  // Only a fully integrated and renormalized quaternion is published.
  attitude_publish(state, now_us, now_ms);
  return true;
}

/*
attitude_compute_vertical_accel(...)
------------------------------------------------------------------------------
ROLE
  Rotate body-frame acceleration into the world vertical axis and remove gravity.

INPUT CONTRACT
  state.attitude must be valid. ax, ay, and az must be finite body-frame
  acceleration values in m/s^2.

OUTPUT CONTRACT
  Returns world-frame vertical linear acceleration in m/s^2, positive upward.
  Returns NAN if the computation cannot be trusted.

MECHANISM
  1. Reject invalid attitude or acceleration.
  2. Read body-to-world quaternion from state.attitude.
  3. Compute the world Z component of body acceleration using the quaternion
     rotation matrix third-row projection.
  4. Add kG to remove gravity under the validated program sign convention.

FAILURE BEHAVIOR
  Invalid attitude or non-finite acceleration returns NAN.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
float attitude_compute_vertical_accel(
  const SystemState &state,
  float ax,
  float ay,
  float az
) {
  if (!state.attitude.valid) return NAN;
  if (!is_finite_f(ax) || !is_finite_f(ay) || !is_finite_f(az)) return NAN;

  const float q0 = state.attitude.q0;
  const float q1 = state.attitude.q1;
  const float q2 = state.attitude.q2;
  const float q3 = state.attitude.q3;

  if (!is_finite_f(q0) || !is_finite_f(q1) || !is_finite_f(q2) || !is_finite_f(q3)) {
    return NAN;
  }

  // This is the third row of the quaternion-derived body-to-world rotation
  // matrix applied to the body-frame acceleration vector. The result is the
  // acceleration component along the world vertical axis.
  const float a_world_z =
    2.0f * (q1 * q3 - q0 * q2) * ax +
    2.0f * (q0 * q1 + q2 * q3) * ay +
    (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az;

  // The accelerometer measures specific force. Under this firmware's sign
  // convention, adding +g removes the gravity contribution and yields vertical
  // linear acceleration for the Kalman predictor.
  return a_world_z + kG;
}

/*
attitude_update_aux_vertical(...)
------------------------------------------------------------------------------
ROLE
  Publish the derived vertical acceleration snapshot from the current IMU
  acceleration and quaternion attitude.

INPUT CONTRACT
  state.imu must contain a valid acceleration sample. state.attitude must be
  valid. now_us and now_ms must be current monotonic timestamps.

OUTPUT CONTRACT
  state.auxvz is invalidated at the start of the attempt. It becomes valid only
  when finite quaternion-based vertical acceleration is computed.

MECHANISM
  1. Clear previous aux vertical validity.
  2. Reject invalid IMU or attitude snapshots.
  3. Compute quaternion-based vertical acceleration.
  4. Reject non-finite result.
  5. Publish timestamp, sequence, update flag, and acceleration.

FAILURE BEHAVIOR
  Invalid IMU/attitude data leaves state.auxvz.valid false.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
bool attitude_update_aux_vertical(SystemState &state, uint32_t now_us, uint32_t now_ms) {
  // Clear output flags first so any early return unambiguously means "no fresh
  // derived vertical acceleration was published this cycle."
  state.auxvz.valid = false;
  state.auxvz.updated = false;
  state.auxvz.a_vertical = NAN;
  state.auxvz.t_ms = now_ms;
  state.auxvz.t_us = now_us;

  if (!state.imu.valid || !state.attitude.valid) {
    return true;
  }

  const float a_vertical = attitude_compute_vertical_accel(
    state,
    state.imu.ax,
    state.imu.ay,
    state.imu.az
  );

  if (!is_finite_f(a_vertical)) {
    return true;
  }

  state.auxvz.valid = true;
  state.auxvz.updated = true;
  ++state.auxvz.seq;
  state.auxvz.a_vertical = a_vertical;

  return true;
}

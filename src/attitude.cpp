#include "attitude.h"

#include <math.h>
#include "config.h"
#include "math_utils.h"

/*
attitude.cpp
===============================================================================
PURPOSE
  Maintain a low-pass body-frame gravity reference and compute vertical linear
  acceleration for the altitude filter.

SIGN CONVENTION
  The gravity reference points along measured acceleration at rest. Projection
  onto that direction yields down-axis measured acceleration. Removing gravity
  and negating produces upward vertical acceleration.
===============================================================================
*/

/*
attitude_reset_gravity_reference(...)
------------------------------------------------------------------------------
ROLE
  Reset learned body-frame gravity to a conservative default.

MECHANISM
  The default vector assumes the body z axis observes approximately -g in the
  uploaded firmware's convention.
*/
void attitude_reset_gravity_reference(SystemState &state) {
  state.sdlog.g_bx = 0.0f;
  state.sdlog.g_by = 0.0f;
  state.sdlog.g_bz = -kG;
}

/*
attitude_update_gravity_reference(...)
------------------------------------------------------------------------------
ROLE
  Low-pass update the body-frame gravity reference when acceleration magnitude
  looks close to 1 g.

INPUT CONTRACT
  ax, ay, az must be finite body-frame acceleration components.

MECHANISM
  1. Compute acceleration magnitude.
  2. Accept the sample only when magnitude is within a broad 1 g window.
  3. Apply exponential moving average.
  4. Renormalize the learned vector to exactly kG magnitude.

FAILURE BEHAVIOR
  Non-finite or high-dynamic samples are ignored. This prevents thrust, vibration,
  or shock from corrupting the gravity reference.
*/
void attitude_update_gravity_reference(SystemState &state, float ax, float ay, float az) {
  if (!is_finite_f(ax) || !is_finite_f(ay) || !is_finite_f(az)) return;

  const float a_mag = sqrtf(ax * ax + ay * ay + az * az);
  if (!is_finite_f(a_mag)) return;

  // Only quasi-static samples should influence the gravity estimate.
  if (fabsf(a_mag - kG) >= 3.0f) return;

  state.sdlog.g_bx = (1.0f - kAlphaG) * state.sdlog.g_bx + kAlphaG * ax;
  state.sdlog.g_by = (1.0f - kAlphaG) * state.sdlog.g_by + kAlphaG * ay;
  state.sdlog.g_bz = (1.0f - kAlphaG) * state.sdlog.g_bz + kAlphaG * az;

  const float norm_g = sqrtf(
    state.sdlog.g_bx * state.sdlog.g_bx +
    state.sdlog.g_by * state.sdlog.g_by +
    state.sdlog.g_bz * state.sdlog.g_bz
  );

  if (norm_g > 1e-6f && is_finite_f(norm_g)) {
    const float scale = kG / norm_g;
    state.sdlog.g_bx *= scale;
    state.sdlog.g_by *= scale;
    state.sdlog.g_bz *= scale;
  }
}

/*
attitude_compute_vertical_accel(...)
------------------------------------------------------------------------------
ROLE
  Project measured body acceleration onto the learned gravity direction and
  convert it into upward linear acceleration.

MECHANISM
  1. Normalize the learned gravity direction.
  2. Dot measured acceleration with that direction.
  3. Remove gravity to obtain down-axis linear acceleration.
  4. Negate to convert down-axis linear acceleration to upward acceleration.

FAILURE BEHAVIOR
  Invalid gravity reference returns NAN.
*/
float attitude_compute_vertical_accel(const SystemState &state, float ax, float ay, float az) {
  if (!is_finite_f(ax) || !is_finite_f(ay) || !is_finite_f(az)) return NAN;

  const float norm_g = sqrtf(
    state.sdlog.g_bx * state.sdlog.g_bx +
    state.sdlog.g_by * state.sdlog.g_by +
    state.sdlog.g_bz * state.sdlog.g_bz
  );

  if (!is_finite_f(norm_g) || norm_g < 1e-6f) return NAN;

  const float zdx = state.sdlog.g_bx / norm_g;
  const float zdy = state.sdlog.g_by / norm_g;
  const float zdz = state.sdlog.g_bz / norm_g;

  const float a_down_meas = ax * zdx + ay * zdy + az * zdz;
  const float a_down_linear = a_down_meas - kG;

  return -a_down_linear;
}

/*
attitude_update_aux_vertical(...)
------------------------------------------------------------------------------
ROLE
  Publish the AuxVzSample derived snapshot from the current IMU acceleration.

OUTPUT CONTRACT
  auxvz.valid is false by default for each due attempt. It becomes true only when
  a finite vertical acceleration can be computed.
*/
bool attitude_update_aux_vertical(SystemState &state, uint32_t now_ms) {
  state.auxvz.valid = false;
  state.auxvz.a_vertical = NAN;

  if (!state.imu.valid) return true;

  const float a_vertical = attitude_compute_vertical_accel(
    state,
    state.imu.ax,
    state.imu.ay,
    state.imu.az
  );

  if (!is_finite_f(a_vertical)) return true;

  state.auxvz.valid = true;
  state.auxvz.t_ms = now_ms;
  ++state.auxvz.seq;
  state.auxvz.a_vertical = a_vertical;

  return true;
}
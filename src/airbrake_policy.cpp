#include "airbrake_policy.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "math_utils.h"

/*
airbrake_policy.cpp
===============================================================================
PURPOSE
  Compute normalized airbrake deployment intent from drag-aware apogee prediction.

CONTROL IDEA
  The policy predicts coast apogee under a candidate airbrake deployment command
  u in [0,1]. If predicted closed-brake apogee exceeds the target, the policy
  increases u until predicted apogee approaches the target.

MODEL
  During upward coast:

    dv/dt = -g - k(u)v^2

  where:

    k(u) = rho * (CDA_body + u*CDA_brake) / (2*m)

  Under locally constant density and drag area, predicted apogee is:

    h_ap(u) = h + (1/(2k)) * ln(1 + k*v^2/g)

  For k near zero, the formula falls back to the ballistic limit:

    h_ap = h + v^2/(2g)

SAFETY DEFAULT
  Policy output is invalid unless AIRBRAKE_POLICY_ENABLED is set at compile time
  and the system is explicitly armed, policy-enabled, in the permitted phase,
  and estimator state is valid, finite, fresh, above the altitude gate, and
  still climbing above the minimum vertical-speed gate.
===============================================================================
*/

// The policy owns its previous command and timestamp so slew limiting remains
// deterministic and independent of actuator implementation details.
static float g_prev_command01 = 0.0f;
static uint32_t g_prev_ms = 0;

/*
airbrake_policy_reset(...)
------------------------------------------------------------------------------
ROLE
  Reset policy output and private command memory to fail-safe state.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  state.policy.valid is false, state.policy.command01 is zero, and internal slew
  memory is reset.

MECHANISM
  1. Clear public policy validity.
  2. Clear public normalized command.
  3. Clear private previous-command memory.
  4. Reset previous timestamp for future slew limiting.

FAILURE BEHAVIOR
  No failure path exists. This function only writes fail-safe values.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
void airbrake_policy_reset(SystemState &state) {
  state.policy.valid = false;
  state.policy.command01 = 0.0f;

  state.policy.predicted_apogee_no_brake_m = NAN;
  state.policy.predicted_apogee_full_brake_m = NAN;
  state.policy.target_apogee_m = NAN;
  state.policy.apogee_error_m = NAN;

  state.policy.target_nominal_m = NAN;
  state.policy.target_effective_m = NAN;
  state.policy.uncertainty_margin_m = NAN;

  g_prev_command01 = 0.0f;
  g_prev_ms = millis();
}
/*
policy_drag_k(...)
------------------------------------------------------------------------------
ROLE
  Compute the lumped quadratic-drag coefficient k(u).

INPUT CONTRACT
  command01 may be finite, non-finite, or outside [0,1]. clamp01(...) bounds it.

OUTPUT CONTRACT
  Returns k in units of 1/m. Returns zero if model constants are invalid.

MECHANISM
  1. Clamp normalized deployment command to [0,1].
  2. Compute effective drag area:
       CDA = CDA_body + u*CDA_brake
  3. Reject invalid density, mass, or drag area.
  4. Return:
       k = rho*CDA/(2m)

FAILURE BEHAVIOR
  Invalid model parameters return zero. The apogee predictor then uses the
  ballistic fallback.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
static float policy_drag_k(float command01) {
  // Any caller-facing command is first clamped into the physically meaningful
  // normalized deployment interval.
  const float u = clamp01(command01);

  const float cda =
    POLICY_CDA_BODY_M2 + u * POLICY_CDA_BRAKE_M2;

  if (!is_finite_f(POLICY_RHO_KGPM3) || POLICY_RHO_KGPM3 <= 0.0f) {
    return 0.0f;
  }

  if (!is_finite_f(POLICY_VEHICLE_MASS_KG) || POLICY_VEHICLE_MASS_KG <= 0.0f) {
    return 0.0f;
  }

  if (!is_finite_f(cda) || cda < 0.0f) {
    return 0.0f;
  }

  // k lumps density, drag area, and mass into the coefficient that appears in
  // the quadratic-drag vertical equation dv/dt = -g - k v^2.
  return (POLICY_RHO_KGPM3 * cda) / (2.0f * POLICY_VEHICLE_MASS_KG);
}

/*
policy_predict_apogee_m(...)
------------------------------------------------------------------------------
ROLE
  Predict coast apogee for a candidate normalized airbrake deployment.

INPUT CONTRACT
  h_m must be finite altitude in meters. v_mps must be finite vertical velocity
  in m/s. command01 is the candidate normalized deployment.

OUTPUT CONTRACT
  Returns predicted apogee in meters. Returns NAN if altitude or velocity are not
  finite.

MECHANISM
  1. Reject non-finite altitude or velocity.
  2. If vertical velocity is non-positive, return current altitude.
  3. Compute drag coefficient k(command01).
  4. If k is effectively zero, use ballistic apogee:
       h + v^2/(2g)
  5. Otherwise use closed-form quadratic-drag coast prediction:
       h + ln(1 + k*v^2/g)/(2k)

FAILURE BEHAVIOR
  Non-finite input returns NAN. Invalid drag model falls back to ballistic
  prediction through k approximately zero.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
static float policy_predict_apogee_m(float h_m, float v_mps, float command01) {
  if (!is_finite_f(h_m) || !is_finite_f(v_mps)) {
    return NAN;
  }

  if (v_mps <= 0.0f) {
    // Once the vehicle is no longer climbing, the best apogee prediction is the
    // current altitude because upward coast has already ended.
    return h_m;
  }

  const float k = policy_drag_k(command01);
  const float v2 = v_mps * v_mps;

  if (!is_finite_f(k) || k < 1.0e-7f) {
    // As drag authority tends toward zero, the model smoothly approaches the
    // ballistic-energy result h + v^2/(2g).
    return h_m + v2 / (2.0f * kG);
  }

  const float argument = 1.0f + (k * v2) / kG;

  if (!is_finite_f(argument) || argument <= 0.0f) {
    return NAN;
  }

  return h_m + logf(argument) / (2.0f * k);
}

/*
policy_solve_command01(...)
------------------------------------------------------------------------------
ROLE
  Solve for the normalized deployment command that drives predicted apogee toward
  target apogee.

INPUT CONTRACT
  h_m and v_mps must be finite. target_apogee_m must be finite. The vehicle must
  be climbing for deployment to be meaningful.

OUTPUT CONTRACT
  Returns a normalized unslewed command in [0, POLICY_MAX_COMMAND01].

MECHANISM
  1. Predict apogee with no deployment.
  2. If no-deployment apogee is below target plus deadband, return zero.
  3. Predict apogee at maximum permitted deployment.
  4. If maximum deployment is still predicted above target, return maximum
     deployment.
  5. Otherwise use bisection to solve h_ap(u)=target.
  6. Return the midpoint command after a fixed iteration count.

FAILURE BEHAVIOR
  Invalid state or invalid prediction returns zero command. This prevents model
  failures from producing non-idle deployment intent.

DETERMINISM
  Fixed-count bisection loop. No hardware I/O. No dynamic allocation.
*/
static float policy_solve_command01(
  float h_m,
  float v_mps,
  float target_apogee_m) {
  if (!is_finite_f(h_m) || !is_finite_f(v_mps) || !is_finite_f(target_apogee_m)) {
    return 0.0f;
  }

  if (v_mps <= 0.0f) {
    return 0.0f;
  }

  const float u_max = clamp01(POLICY_MAX_COMMAND01);

  const float apogee_u0 = policy_predict_apogee_m(h_m, v_mps, 0.0f);

  if (!is_finite_f(apogee_u0)) {
    return 0.0f;
  }

  /*
    If the closed-brake prediction is already at or below target plus deadband,
    deployment is unnecessary.
  */
  if (apogee_u0 <= target_apogee_m + POLICY_APOGEE_DEADBAND_M) {
    return 0.0f;
  }

  const float apogee_umax = policy_predict_apogee_m(h_m, v_mps, u_max);

  if (!is_finite_f(apogee_umax)) {
    return 0.0f;
  }

  /*
    If full deployment still predicts an overshoot, the best available action is
    maximum allowed deployment.
  */
  if (apogee_umax > target_apogee_m) {
    return u_max;
  }

  float lo = 0.0f;
  float hi = u_max;

  // Because predicted apogee decreases monotonically with increasing drag
  // command in this model, simple bisection gives a deterministic bounded-cost
  // root find with no iteration-history sensitivity.
  for (uint8_t i = 0; i < POLICY_BISECTION_STEPS; ++i) {
    const float mid = 0.5f * (lo + hi);
    const float apogee_mid = policy_predict_apogee_m(h_m, v_mps, mid);

    if (!is_finite_f(apogee_mid)) {
      return 0.0f;
    }

    /*
      More deployment increases drag and lowers predicted apogee.

      If midpoint still predicts too high, increase the lower command bound.
      If midpoint predicts at or below target, decrease the upper command bound.
    */
    if (apogee_mid > target_apogee_m) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  return clamp01(0.5f * (lo + hi));
}

/*
policy_apply_slew_limit(...)
------------------------------------------------------------------------------
ROLE
  Bound normalized command rate so actuator demand changes gradually.

INPUT CONTRACT
  desired_command01 may be any float. dt_s must be finite and non-negative for
  normal slew limiting.

OUTPUT CONTRACT
  Returns a command no farther than POLICY_SLEW_PER_SEC * dt_s from the previous
  command memory.

MECHANISM
  1. Clamp desired command to [0,1].
  2. Reject invalid dt by freezing command at previous value.
  3. Compute maximum allowed command step.
  4. Clamp requested command around previous command.
  5. Store and return the limited command.

FAILURE BEHAVIOR
  Invalid dt freezes the command. Non-finite desired command clamps to zero.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
static float policy_apply_slew_limit(float desired_command01, float dt_s) {
  const float desired = clamp01(desired_command01);

  if (!is_finite_f(dt_s) || dt_s < 0.0f) {
    // Freezing at the previous command is safer than allowing a potentially
    // unbounded step after corrupted timing.
    return g_prev_command01;
  }

  // max_step is the largest command change allowed over this exact elapsed time.
  const float max_step = POLICY_SLEW_PER_SEC * dt_s;

  float limited = desired;

  if (limited > g_prev_command01 + max_step) {
    limited = g_prev_command01 + max_step;
  }

  if (limited < g_prev_command01 - max_step) {
    limited = g_prev_command01 - max_step;
  }

  limited = clamp01(limited);
  g_prev_command01 = limited;

  return limited;
}

/*
policy_reset_memory(...)
------------------------------------------------------------------------------
ROLE
  Reset private command memory without requiring mutable access to public state.

INPUT CONTRACT
  now_ms must be the current scheduler timestamp.

OUTPUT CONTRACT
  Previous command memory becomes zero and previous timestamp becomes now_ms.

MECHANISM
  1. Clear previous normalized command.
  2. Store timestamp for future slew-limit dt.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
static void policy_reset_memory(uint32_t now_ms) {
  g_prev_command01 = 0.0f;
  g_prev_ms = now_ms;
}




/*
policy_uncertainty_margin_m(...)
------------------------------------------------------------------------------
ROLE
  Compute conservative target reduction from altitude covariance.

INPUT CONTRACT
  state.est.P00 should be finite and non-negative when estimator covariance is
  valid.

OUTPUT CONTRACT
  Returns margin in meters. The margin is bounded to
  POLICY_MAX_UNCERTAINTY_MARGIN_M.

MECHANISM
  1. Reject non-finite or negative P00.
  2. Compute sigma_h = sqrt(P00).
  3. Compute margin = POLICY_SIGMA_MARGIN_N * sigma_h.
  4. Clamp margin to [0, POLICY_MAX_UNCERTAINTY_MARGIN_M].

FAILURE BEHAVIOR
  Invalid covariance returns zero margin.

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
static float policy_uncertainty_margin_m(const SystemState &state) {
  if (!is_finite_f(state.est.P00) || state.est.P00 < 0.0f) {
    return 0.0f;
  }

  // P00 is altitude variance, so sqrt(P00) is one-sigma altitude uncertainty.
  // The policy shifts the target downward by a configurable number of sigma to
  // make braking more conservative when state uncertainty is high.
  float margin_m = POLICY_SIGMA_MARGIN_N * sqrtf(state.est.P00);

  if (!is_finite_f(margin_m) || margin_m < 0.0f) {
    return 0.0f;
  }

  if (margin_m > POLICY_MAX_UNCERTAINTY_MARGIN_M) {
    margin_m = POLICY_MAX_UNCERTAINTY_MARGIN_M;
  }

  return margin_m;
}




/*
airbrake_policy_compute(...)
------------------------------------------------------------------------------
ROLE
  Compute drag-aware apogee-targeting airbrake deployment intent.

INPUT CONTRACT
  state.est.valid must be true. state.est.h_m and state.est.v_mps must be finite.
  The estimator timestamp must be fresh enough for policy use.

OUTPUT CONTRACT
  Returns valid=false when actuation intent is not authorized. Returns valid=true
  only when the vehicle is above the policy altitude gate, still climbing above
  the minimum vertical-speed gate, and apogee prediction requests deployment.

MECHANISM
  1. Initialize fail-safe output.
  2. Enforce compile-time policy gate.
  3. Enforce estimator validity, finiteness, and freshness.
  4. Enforce minimum altitude gate.
  5. Enforce upward-coast vertical-speed gate.
  6. Solve for unslewed deployment from drag-aware apogee prediction.
  7. Apply command slew limit.
  8. Publish valid output only when command is positive.

FAILURE BEHAVIOR
  Disabled policy, invalid estimator, stale estimator, non-finite state, or
  below-gate flight condition returns invalid output and resets command memory.

DETERMINISM
  Bounded scalar math with fixed-count bisection. No sensor I/O. No actuator I/O.
  No dynamic allocation.
*/
AirbrakePolicyOutput airbrake_policy_compute(const SystemState &state) {
  AirbrakePolicyOutput out;

  out.valid = false;
  out.command01 = 0.0f;

  out.predicted_apogee_no_brake_m = NAN;
  out.predicted_apogee_full_brake_m = NAN;
  out.target_apogee_m = NAN;
  out.apogee_error_m = NAN;

  out.target_nominal_m = POLICY_TARGET_APOGEE_M;
  out.target_effective_m = NAN;
  out.uncertainty_margin_m = NAN;

#if AIRBRAKE_POLICY_ENABLED
  const uint32_t now_ms = millis();

  // These early returns all explicitly reset command memory so that when the
  // policy later re-enters a valid state it ramps from zero rather than from an
  // obsolete pre-gate command.
  if (!state.policy_runtime_enabled) {
    policy_reset_memory(now_ms);
    return out;
  }

  if (state.arm_state != ArmingState::ARMED) {
    policy_reset_memory(now_ms);
    return out;
  }

  if (!state.software_arm_token) {
    policy_reset_memory(now_ms);
    return out;
  }

  if (!(state.phase == FlightPhase::COAST || state.phase == FlightPhase::BRAKE)) {
    policy_reset_memory(now_ms);
    return out;
  }

  if (!state.est.valid ||
      !is_finite_f(state.est.h_m) ||
      !is_finite_f(state.est.v_mps)) {
    policy_reset_memory(now_ms);
    return out;
  }

  if ((now_ms - state.est.t_ms) > POLICY_MAX_EST_AGE_MS) {
    policy_reset_memory(now_ms);
    return out;
  }

  const float h_m = state.est.h_m;
  const float v_mps = state.est.v_mps;

  // The controller is intentionally a coast-phase law. Below this altitude or
  // below this upward speed, the aerodynamic model is not trusted enough to
  // justify deployment.
  if (h_m < POLICY_MIN_ALT_M) {
    policy_reset_memory(now_ms);
    return out;
  }

  if (v_mps < POLICY_MIN_VZ_MPS) {
    policy_reset_memory(now_ms);
    return out;
  }

  const float uncertainty_margin_m = policy_uncertainty_margin_m(state);
  float target_eff_m = POLICY_TARGET_APOGEE_M - uncertainty_margin_m;

  if (!is_finite_f(target_eff_m)) {
    policy_reset_memory(now_ms);
    return out;
  }

  if (target_eff_m < 0.0f) {
    target_eff_m = 0.0f;
  }

  const float u_max = clamp01(POLICY_MAX_COMMAND01);

  const float apogee_no_brake =
    policy_predict_apogee_m(h_m, v_mps, 0.0f);

  const float apogee_full_brake =
    policy_predict_apogee_m(h_m, v_mps, u_max);

  // These fields are exported even when command stays zero so telemetry and SD
  // logs can show why the controller chose not to deploy.
  out.predicted_apogee_no_brake_m = apogee_no_brake;
  out.predicted_apogee_full_brake_m = apogee_full_brake;
  out.target_apogee_m = target_eff_m;
  out.target_nominal_m = POLICY_TARGET_APOGEE_M;
  out.target_effective_m = target_eff_m;
  out.uncertainty_margin_m = uncertainty_margin_m;

  if (is_finite_f(apogee_no_brake)) {
    out.apogee_error_m = apogee_no_brake - target_eff_m;
  }

  float dt_s = (now_ms - g_prev_ms) * 0.001f;
  g_prev_ms = now_ms;

  if (!is_finite_f(dt_s) || dt_s < 0.0f || dt_s > 1.0f) {
    // Very large dt values usually indicate startup or a long gap. Collapsing
    // them to zero prevents one delayed cycle from authorizing an oversized
    // slew-limited command step.
    dt_s = 0.0f;
  }

  const float desired_command01 =
    policy_solve_command01(
      h_m,
      v_mps,
      target_eff_m
    );

  const float command01 =
    policy_apply_slew_limit(desired_command01, dt_s);

  if (command01 > 0.0f) {
    // valid becomes true only when the controller has both authority and a
    // positive non-idle command to request. Zero command remains an intentional
    // fail-safe/no-deployment outcome.
    out.valid = true;
    out.command01 = command01;
  }
#else
  (void)state;
#endif

  return out;
}


#if AIRBRAKE_POLICY_TEST_API

/*
airbrake_policy_predict_apogee_m(...)
------------------------------------------------------------------------------
ROLE
  Public test wrapper for the internal apogee predictor.

INPUT CONTRACT
  h_m is altitude in meters. v_mps is vertical speed in m/s. command01 is
  normalized deployment command.

OUTPUT CONTRACT
  Returns predicted apogee in meters, or NAN if the internal predictor rejects
  the inputs.

MECHANISM
  Delegates to policy_predict_apogee_m(...).

FAILURE BEHAVIOR
  Same as policy_predict_apogee_m(...).

DETERMINISM
  Constant-time scalar math. No loops. No hardware I/O. No dynamic allocation.
*/
float airbrake_policy_predict_apogee_m(float h_m, float v_mps, float command01) {
  return policy_predict_apogee_m(h_m, v_mps, command01);
}

/*
airbrake_policy_solve_command01(...)
------------------------------------------------------------------------------
ROLE
  Public test wrapper for the internal apogee-command solver.

INPUT CONTRACT
  h_m is altitude in meters. v_mps is vertical speed in m/s. target_m is target
  apogee in meters.

OUTPUT CONTRACT
  Returns normalized deployment command in [0, POLICY_MAX_COMMAND01].

MECHANISM
  Delegates to policy_solve_command01(...).

FAILURE BEHAVIOR
  Same as policy_solve_command01(...).

DETERMINISM
  Fixed-count bisection through the internal solver. No hardware I/O. No dynamic
  allocation.
*/
float airbrake_policy_solve_command01(float h_m, float v_mps, float target_m) {
  return policy_solve_command01(h_m, v_mps, target_m);
}

#endif

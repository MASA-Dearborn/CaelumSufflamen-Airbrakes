#include "airbrake_policy.h"

#include "config.h"
#include "math_utils.h"

/*
airbrake_policy.cpp
===============================================================================
PURPOSE
  Compute bounded normalized airbrake command intent from estimator state.

SAFETY DEFAULT
  Policy output is invalid unless AIRBRAKE_POLICY_ENABLED is set at compile time
  and estimator state is valid.

CURRENT POLICY
  Simple vertical-speed schedule:
    - v <= POLICY_START_VZ_MPS : no deployment
    - v >  POLICY_START_VZ_MPS : deployment increases linearly
===============================================================================
*/

/*
airbrake_policy_reset(...)
------------------------------------------------------------------------------
ROLE
  Reset policy output to the fail-safe invalid state.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  state.policy.valid is false and command01 is zero.

MECHANISM
  1. Clear authorization flag.
  2. Clear normalized command.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
void airbrake_policy_reset(SystemState &state) {
  state.policy.valid = false;
  state.policy.command01 = 0.0f;
}

/*
airbrake_policy_compute(...)
------------------------------------------------------------------------------
ROLE
  Compute normalized airbrake command intent from current estimator state.

INPUT CONTRACT
  state.est.valid must be true and state.est.v_mps must be finite.

OUTPUT CONTRACT
  Returns valid=false for no authorization. Returns valid=true only when the
  policy is compiled in and command computation passes input checks.

MECHANISM
  1. Initialize fail-safe invalid output.
  2. Enforce compile-time policy gate.
  3. Reject invalid or non-finite estimator velocity.
  4. Map vertical speed above threshold to normalized deployment.
  5. Clamp to [0,1].
  6. Apply maximum policy command.

FAILURE BEHAVIOR
  Disabled policy, invalid estimator, or non-finite velocity returns invalid
  output with zero command.

DETERMINISM
  Constant-time scalar math. No loops. No sensor I/O. No actuator I/O. No dynamic
  allocation.
*/
AirbrakePolicyOutput airbrake_policy_compute(const SystemState &state) {
  AirbrakePolicyOutput out;

  out.valid = false;
  out.command01 = 0.0f;

#if AIRBRAKE_POLICY_ENABLED
  if (!state.est.valid || !is_finite_f(state.est.v_mps)) {
    return out;
  }

  if (state.est.v_mps > POLICY_START_VZ_MPS) {
    out.valid = true;
    out.command01 = clamp01(
      (state.est.v_mps - POLICY_START_VZ_MPS) / POLICY_FULL_SCALE_VZ_MPS
    );

    if (out.command01 > POLICY_MAX_COMMAND01) {
      out.command01 = POLICY_MAX_COMMAND01;
    }
  }
#else
  (void)state;
#endif

  return out;
}
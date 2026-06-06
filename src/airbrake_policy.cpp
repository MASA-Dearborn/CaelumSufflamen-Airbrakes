/*
policy_compute(...)
------------------------------------------------------------------------------
ROLE
  Produce the normalized airbrake deployment command and policy-level validity
  flag for the current scheduler cycle.

INPUT CONTRACT
  - All inputs are read from SystemState snapshots.
  - flight fields are meaningful only when flight.valid == true.
  - arm_state must already have been updated this cycle.

OUTPUT CONTRACT
  - Returns valid=false for all non-actuating states.
  - Returns valid=true only in BRAKE mode after estimator freshness checks pass.
  - command01 is always bounded to [0, POL_MAX_CMD01] before publication.

MECHANISM
  1. Initialize fail-safe output.
  2. Enforce compile-time policy gate.
  3. Enforce runtime policy enable.
  4. Enforce arming state.
  5. Enforce estimator validity and freshness.
  6. Advance finite-state policy mode.
  7. Map vertical speed to deployment command.
  8. Apply slew-rate limiting.
  9. Publish valid output only in BRAKE mode.

SAFETY / FAILURE BEHAVIOR
  Any invalid, stale, disarmed, or disabled state returns valid=false. Most such
  states also reset policy memory to prevent latent actuation on re-arm.

DETERMINISM
  Bounded scalar math. No sensor I/O. No actuator I/O. No heap allocation.
*/
AirbrakePolicyOutput policy_compute(const SystemState& sys) {
  AirbrakePolicyOutput out;

  // Fail-safe default: no command and no authorization.
  out.command01 = 0.0f;
  out.valid = false;

#if !AIRBRAKE_POLICY_ENABLED
  // Compile-time gate. In safe builds, no runtime command can activate policy.
  (void)sys;
  return out;
#else
  const uint32_t now_ms = millis();

  // Runtime gate. This allows bench firmware to include the policy code while
  // keeping policy output explicitly disabled until commanded.
  if (!g_policy_runtime_enabled) {
    return out;
  }

  // Arming gate. Policy memory is reset so old BRAKE-mode state cannot survive
  // a disarm/re-arm transition.
  if (sys.arm_state != ArmingState::ARMED) {
    policy_reset();
    return out;
  }

  // Estimator validity gate. The policy must never infer flight state from stale
  // numeric values left inside an invalid snapshot.
  if (!sys.flight.valid) {
    policy_reset();
    return out;
  }

  if (!isfinite(sys.flight.altitude_m) || !isfinite(sys.flight.vz_mps)) {
    policy_reset();
    return out;
  }

  // Freshness gate. A valid-but-old estimate is unsafe for actuation because the
  // vehicle can change state rapidly during coast.
  if ((now_ms - sys.flight.t_ms) > POL_MAX_EST_AGE_MS) {
    policy_reset();
    return out;
  }

  // dt is used only for slew limiting. Unsigned millis subtraction remains
  // defined across wraparound.
  const float dt_s = (now_ms - g_prev_ms) * 0.001f;
  g_prev_ms = now_ms;

  if (g_policy_mode == PolicyMode::DISABLED) {
    g_policy_mode = PolicyMode::ARMED_WAIT;
  }

  // FSM comments should remain inside each case in the final source.
  // ...
#endif
}
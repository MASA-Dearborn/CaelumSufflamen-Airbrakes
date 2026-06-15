#include "safety.h"

#include "config.h"

/*
safety.cpp
===============================================================================
PURPOSE
  Provide centralized safety predicates for policy and actuator gating.

DESIGN NOTE
  Compile-time actuation disabling is checked here and also inside actuator.cpp.
  Redundant checks are intentional for safety-relevant outputs.
===============================================================================
*/

/*
safety_runtime_ok(...)
------------------------------------------------------------------------------
ROLE
  Confirm minimum runtime state required before non-idle actuation is considered.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  Returns true only when configuration is valid, estimator is valid, and
  estimator age is within the configured freshness window.

MECHANISM
  1. Reject invalid configuration.
  2. Reject invalid estimator.
  3. Compare current millis() against estimator timestamp.
  4. Return true only when all gates pass.

FAILURE BEHAVIOR
  Any invalid or stale condition returns false and keeps actuator idle upstream.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
bool safety_runtime_ok(const SystemState &state) {
  // Configuration validity protects against using an unknown atmospheric
  // reference or partially configured runtime state.
  if (!state.cfg.valid) return false;

  // Actuation may not rely on an estimator that has never been successfully
  // seeded and published.
  if (!state.est.valid) return false;

  const uint32_t now_ms = millis();

  // Freshness is evaluated at the last moment before actuation because even a
  // previously valid estimate becomes unsafe if it is allowed to age out.
  if ((now_ms - state.est.t_ms) > EST_MAX_AGE_MS) return false;

  return true;
}

/*
safety_allows_actuation(...)
------------------------------------------------------------------------------
ROLE
  Final runtime predicate for non-idle actuator application.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  Returns true only if ACTUATION_ENABLED is nonzero and runtime safety checks
  pass.

MECHANISM
  1. Enforce compile-time hard gate.
  2. If actuation is compiled in, evaluate runtime predicate.
  3. If actuation is compiled out, return false unconditionally.

FAILURE BEHAVIOR
  Safe builds force this predicate false. Invalid runtime state also returns
  false.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
bool safety_allows_actuation(const SystemState &state) {
#if ACTUATION_ENABLED
  return safety_runtime_ok(state);
#else
  (void)state;
  return false;
#endif
}

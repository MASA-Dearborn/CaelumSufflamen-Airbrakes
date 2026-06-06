#include "safety.h"

#include "config.h"

/*
safety.cpp
===============================================================================
PURPOSE
  Provide centralized safety predicates for policy and actuator gating.

DESIGN NOTE
  Compile-time actuation disabling is checked here and also inside actuator.cpp.
  Redundant checks are intentional for safety-critical outputs.
===============================================================================
*/

/*
safety_runtime_ok(...)
------------------------------------------------------------------------------
ROLE
  Confirm that the minimum runtime state required for policy/actuation is valid.
*/
bool safety_runtime_ok(const SystemState &state) {
  return state.cfg.valid && state.est.valid;
}

/*
safety_allows_actuation(...)
------------------------------------------------------------------------------
ROLE
  Final runtime predicate for non-idle actuator application.

MECHANISM
  ACTUATION_ENABLED is a compile-time hard gate. Even if runtime state is valid,
  a safe build returns false.
*/
bool safety_allows_actuation(const SystemState &state) {
#if ACTUATION_ENABLED
  return safety_runtime_ok(state);
#else
  (void)state;
  return false;
#endif
}
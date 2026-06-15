#pragma once

#include "data_types.h"

/*
safety.h
===============================================================================
ROLE
  Central runtime safety predicates.

CURRENT SCOPE
  The verified implementation checks configuration validity, estimator validity,
  estimator freshness, and the compile-time actuation gate before non-idle
  actuation may be considered.
===============================================================================
*/

bool safety_runtime_ok(const SystemState &state);
bool safety_allows_actuation(const SystemState &state);
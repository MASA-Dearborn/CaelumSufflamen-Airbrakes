#pragma once

#include "data_types.h"

/*
safety.h
===============================================================================
ROLE
  Central runtime safety predicates.

CURRENT SCOPE
  This branch exposes a minimal runtime gate: configuration and estimator must
  be valid before non-idle actuation can be considered.
===============================================================================
*/

bool safety_runtime_ok(const SystemState &state);
bool safety_allows_actuation(const SystemState &state);
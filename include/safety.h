#pragma once

#include "data_types.h"

/*
safety.h
===============================================================================
ROLE
  Central runtime safety predicates.
===============================================================================
*/

bool safety_runtime_ok(const SystemState &state);
bool safety_allows_actuation(const SystemState &state);
#pragma once

#include "data_types.h"

/*
airbrake_policy.h
===============================================================================
ROLE
  Airbrake command policy interface.

SAFETY CONTRACT
  The policy computes command intent only. The actuator module and safety module
  still control whether command intent reaches hardware.

IMPLEMENTATION MATCH
  This header matches the verified airbrake_policy.cpp public symbols:
    airbrake_policy_reset(...)
    airbrake_policy_compute(...)
===============================================================================
*/

void airbrake_policy_reset(SystemState &state);
AirbrakePolicyOutput airbrake_policy_compute(const SystemState &state);
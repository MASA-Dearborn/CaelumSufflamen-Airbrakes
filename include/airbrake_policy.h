#pragma once

#include "data_types.h"

/*
airbrake_policy.h
===============================================================================
ROLE
  Airbrake command policy interface.

SAFETY CONTRACT
  The policy computes intent only. The actuator module and safety module still
  control whether command intent reaches hardware.
===============================================================================
*/

void airbrake_policy_reset(SystemState &state);
AirbrakePolicyOutput airbrake_policy_compute(const SystemState &state);

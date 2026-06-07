#pragma once

#include "data_types.h"

/*
airbrake_policy.h
===============================================================================
ROLE
  Apogee-prediction airbrake command policy.

CONTROL LAW
  The policy predicts coast apogee using a vertical quadratic-drag model:

    dv/dt = -g - k(u)v^2

  where:

    k(u) = rho * (CDA_body + u*CDA_brake) / (2*m)

  It selects the smallest normalized deployment command u in [0,1] that brings
  predicted apogee down to the target apogee, subject to estimator validity,
  coast-phase gates, model authority, deadband, and slew-rate limiting.

SAFETY CONTRACT
  This module computes command intent only. The safety module and actuator module
  still determine whether command intent reaches hardware.
===============================================================================
*/

void airbrake_policy_reset(SystemState &state);
AirbrakePolicyOutput airbrake_policy_compute(const SystemState &state);

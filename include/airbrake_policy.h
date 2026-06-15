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
  predicted apogee down to the effective target apogee, subject to estimator
  validity, phase gates, coast gates, model authority, deadband, uncertainty
  margin, and slew-rate limiting.

SAFETY CONTRACT
  This module computes command intent only. The safety module and actuator module
  still determine whether command intent reaches hardware.
===============================================================================
*/

void airbrake_policy_reset(SystemState &state);
AirbrakePolicyOutput airbrake_policy_compute(const SystemState &state);

#if AIRBRAKE_POLICY_TEST_API
float airbrake_policy_predict_apogee_m(float h_m, float v_mps, float command01);
float airbrake_policy_solve_command01(float h_m, float v_mps, float target_m);
#endif
#pragma once

#include "data_types.h"

/*
actuator.h
===============================================================================
ROLE
  Servo actuator abstraction.

SAFETY CONTRACT
  Non-idle commands are accepted only when ACTUATION_ENABLED is true at compile
  time. actuator_force_idle() remains available.

IMPLEMENTATION MATCH
  The actuator.cpp stores ActuatorConfig, maps normalized commands to explicit
  pulse-width writes, exposes the last commanded pulse width, and always
  supports idle forcing.
===============================================================================
*/

void actuator_begin(const ActuatorConfig &cfg);
void actuator_force_idle(void);
void actuator_write_command01(float command01);
int actuator_last_us(void);

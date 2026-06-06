#pragma once

#include <Arduino.h>
#include "data_types.h"

/*
attitude.h
===============================================================================
ROLE
  Attitude-adjacent derived computations.

CURRENT SCOPE
  The firmware branch estimates vertical acceleration by learning the
  apparent gravity vector in body coordinates and projecting acceleration onto
  that learned direction. This module owns that gravity vector and projection.
===============================================================================
*/

void attitude_reset_gravity_reference(SystemState &state);
void attitude_update_gravity_reference(SystemState &state, float ax, float ay, float az);
float attitude_compute_vertical_accel(const SystemState &state, float ax, float ay, float az);
bool attitude_update_aux_vertical(SystemState &state, uint32_t now_ms);

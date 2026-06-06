#pragma once

#include <Arduino.h>
#include "data_types.h"

/*
sensors.h
===============================================================================
ROLE
  Hardware acquisition API.

OWNERSHIP
  sensors.cpp owns hardware sensor objects. Each poll function writes exactly
  one snapshot type.

DETERMINISM
  Each poll function performs at most one acquisition attempt per call.
===============================================================================
*/

bool sensors_begin(SystemState &state);
void sensors_print_status(const SystemState &state);
bool sensors_bmp_data_ready_within(uint32_t timeout_ms);

bool sensors_poll_baro(SystemState &state, uint32_t now_ms);
bool sensors_poll_imu(SystemState &state, uint32_t now_ms);
bool sensors_poll_aux(SystemState &state, uint32_t now_ms);
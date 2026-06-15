#pragma once

#include <Arduino.h>
#include "data_types.h"

/*
estimation.h
===============================================================================
ROLE
  Live barometer/IMU fusion into EstimatorSample.

BOUNDARY
  This module consumes published snapshots and publishes EstimatorSample. It does
  not read hardware directly.

===============================================================================
*/

void estimation_reset(SystemState &state);
float estimation_relative_altitude(SystemState &state);
bool estimation_update(SystemState &state, uint32_t now_ms);
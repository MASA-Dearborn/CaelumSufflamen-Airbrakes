#pragma once

#include <Arduino.h>
#include "data_types.h"

/*
attitude.h
===============================================================================
ROLE
  Quaternion attitude-estimation interface for Caelum Sufflamen.

ENGINEERING INTENT
  This module owns the Madgwick IMU update and quaternion-based vertical
  acceleration projection. It replaces the earlier gravity-vector-only projection
  approach with the validated estimator kernel's attitude model.
===============================================================================
*/

void attitude_begin(SystemState &state);
void attitude_reset(SystemState &state);

bool attitude_update_imu(
  SystemState &state,
  float gx,
  float gy,
  float gz,
  float ax,
  float ay,
  float az,
  float dt_s,
  uint32_t now_us,
  uint32_t now_ms
);

float attitude_compute_vertical_accel(
  const SystemState &state,
  float ax,
  float ay,
  float az
);

bool attitude_update_aux_vertical(SystemState &state, uint32_t now_us, uint32_t now_ms);
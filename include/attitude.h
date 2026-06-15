#pragma once

#include <Arduino.h>
#include "data_types.h"

/*
attitude.h
===============================================================================
ROLE
  Madgwick IMU attitude estimator and quaternion-based vertical acceleration.

CURRENT SCOPE
  attitude.cpp owns a private quaternion state. It publishes AttitudeSample into
  SystemState and computes world-frame vertical linear acceleration from the
  current quaternion and IMU acceleration.
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
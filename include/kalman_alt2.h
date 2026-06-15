#pragma once

#include <Arduino.h>
#include "data_types.h"

/*
kalman_alt2.h
===============================================================================
ROLE
  Scalar 2-state altitude Kalman filter interface.

STATE
  h_m   = altitude above selected reference [m]
  v_mps = vertical velocity [m/s]

MODEL
  The model assumes constant acceleration over the measured sample interval.
  Acceleration enters as an external input when available. Barometric altitude is
  used as the measurement.

IMPLEMENTATION MATCH
  This header matches kalman_alt2.cpp, whose prediction function accepts measured
  acceleration and measured dt_s:
    kf_alt2_predict(kf, a_vertical_mps2, dt_s)
===============================================================================
*/

void kf_alt2_reset(KfAlt2State &kf);
void kf_alt2_seed(KfAlt2State &kf, float h0_m);
void kf_alt2_predict(KfAlt2State &kf, float a_vertical_mps2, float dt_s);
void kf_alt2_update(KfAlt2State &kf, float z_meas_m);
bool kf_alt2_is_valid(const KfAlt2State &kf);
#pragma once

#include <Arduino.h>

/*
kalman_alt2.h
===============================================================================
ROLE
  Scalar 2-state altitude Kalman filter interface.

STATE
  h_m   = altitude above selected reference [m]
  v_mps = vertical velocity [m/s]

MODEL
  The model assumes constant acceleration over one measured sample interval.
  Acceleration enters as an external input when available. Barometric altitude is
  used as the measurement.
===============================================================================
*/

struct KfAlt2State {
  bool seeded = false;

  float h_m = 0.0f;
  float v_mps = 0.0f;

  float P00 = 1.0f;
  float P01 = 0.0f;
  float P10 = 0.0f;
  float P11 = 1.0f;
};

void kf_alt2_reset(KfAlt2State &kf);
void kf_alt2_seed(KfAlt2State &kf, float h0_m);
void kf_alt2_predict(KfAlt2State &kf, float a_vertical_mps2, float dt_s);
void kf_alt2_update(KfAlt2State &kf, float z_meas_m);
bool kf_alt2_is_valid(const KfAlt2State &kf);
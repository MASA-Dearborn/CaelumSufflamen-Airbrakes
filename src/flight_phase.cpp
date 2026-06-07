#include "flight_phase.h"

#include <math.h>

#include "config.h"
#include "math_utils.h"

/*
flight_phase.cpp
===============================================================================
PURPOSE
  Provide a simple conservative flight-phase detector.

DESIGN INTENT
  This first implementation is intentionally readable and conservative. It uses
  estimator altitude, estimator vertical speed, and IMU acceleration norm.

  Later versions can add hysteresis, launch-detect latching, burnout detection,
  phase dwell timers, and descent confirmation.

PHASE POLICY
  - IDLE is the fail-safe fallback.
  - BOOST is detected from high acceleration after small altitude rise.
  - COAST is upward flight above the policy altitude/speed gates.
  - BRAKE is reserved for later integration with active braking state.
  - DESCENT begins after positive coast has ended.
===============================================================================
*/

/*
flight_phase_reset(...)
------------------------------------------------------------------------------
ROLE
  Reset phase state to IDLE.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  state.phase becomes FlightPhase::IDLE.

MECHANISM
  1. Assign IDLE phase.
  2. Leave estimator, sensor, policy, and actuator state unchanged.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
void flight_phase_reset(SystemState &state) {
  state.phase = FlightPhase::IDLE;
}

/*
flight_phase_update(...)
------------------------------------------------------------------------------
ROLE
  Classify current flight phase from estimator and IMU snapshots.

INPUT CONTRACT
  state.est and state.imu must be valid before their numeric payloads can be
  interpreted. now_ms must be the current scheduler timestamp.

OUTPUT CONTRACT
  state.phase is updated to a conservative phase estimate.

MECHANISM
  1. Fall back to IDLE when required snapshots are invalid.
  2. Extract altitude, vertical speed, and acceleration norm.
  3. Reject non-finite values.
  4. Classify BOOST from high acceleration after a small altitude rise.
  5. Classify COAST from sufficient altitude and upward speed.
  6. Classify DESCENT from sufficient altitude and non-positive vertical speed.
  7. Otherwise classify IDLE.

FAILURE BEHAVIOR
  Invalid, missing, or non-finite data produces IDLE.

DETERMINISM
  Constant-time scalar checks. No loops. No hardware I/O. No dynamic allocation.
*/
void flight_phase_update(SystemState &state, uint32_t now_ms) {
  (void)now_ms;

  if (!state.est.valid || !state.imu.valid) {
    state.phase = FlightPhase::IDLE;
    return;
  }

  const float h_m = state.est.h_m;
  const float v_mps = state.est.v_mps;
  const float a_norm = state.imu.a_norm;

  if (!is_finite_f(h_m) || !is_finite_f(v_mps) || !is_finite_f(a_norm)) {
    state.phase = FlightPhase::IDLE;
    return;
  }

  if (a_norm > FLIGHT_PHASE_BOOST_ACCEL_NORM_MPS2 &&
      h_m > FLIGHT_PHASE_BOOST_MIN_ALT_M) {
    state.phase = FlightPhase::BOOST;
    return;
  }

  if (h_m > POLICY_MIN_ALT_M && v_mps > POLICY_MIN_VZ_MPS) {
    state.phase = FlightPhase::COAST;
    return;
  }

  if (h_m > POLICY_MIN_ALT_M && v_mps <= FLIGHT_PHASE_DESCENT_VZ_MPS) {
    state.phase = FlightPhase::DESCENT;
    return;
  }

  state.phase = FlightPhase::IDLE;
}

/*
flight_phase_name(...)
------------------------------------------------------------------------------
ROLE
  Convert FlightPhase enum to a compact string.

INPUT CONTRACT
  phase may contain any FlightPhase value.

OUTPUT CONTRACT
  Returns a static string literal.

MECHANISM
  1. Switch on phase.
  2. Return a string literal for known values.
  3. Return UNKNOWN for unexpected values.

FAILURE BEHAVIOR
  Unexpected enum values return UNKNOWN.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
const char *flight_phase_name(FlightPhase phase) {
  switch (phase) {
    case FlightPhase::IDLE:
      return "IDLE";

    case FlightPhase::BOOST:
      return "BOOST";

    case FlightPhase::COAST:
      return "COAST";

    case FlightPhase::BRAKE:
      return "BRAKE";

    case FlightPhase::DESCENT:
      return "DESCENT";

    default:
      return "UNKNOWN";
  }
}
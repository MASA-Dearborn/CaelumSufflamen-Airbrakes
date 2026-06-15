#include "flight_phase.h"

#include <math.h>

#include "config.h"
#include "math_utils.h"

/*
flight_phase.cpp
===============================================================================
PURPOSE
  Provide a conservative stateful flight-phase detector.

DESIGN INTENT
  The detector uses estimator altitude, estimator vertical speed, and IMU
  acceleration norm, then applies latching and dwell timers so individual noisy
  samples cannot immediately move the phase machine backward or forward.

PHASE POLICY
  - IDLE is the fail-safe fallback.
  - BOOST is latched after confirmed launch acceleration/vertical motion.
  - COAST begins only after confirmed burnout and minimum BOOST dwell.
  - BRAKE reflects active airbrake command intent from the previous policy pass.
  - DESCENT begins only after confirmed non-positive vertical speed.
===============================================================================
*/

/*
Detector tuning
-------------------------------------------------------------------------------
These constants are local to the detector because they describe phase-machine
debounce behavior, not estimator physics or actuator policy. They should be tuned
from logged flight data after the first representative flights.
*/
static const float FLIGHT_PHASE_LAUNCH_MIN_VZ_MPS = 5.0f;
static const float FLIGHT_PHASE_BURNOUT_ACCEL_NORM_MPS2 = 16.0f;
static const float FLIGHT_PHASE_COAST_MIN_ALT_M = 5.0f;
static const float FLIGHT_PHASE_COAST_MIN_VZ_MPS = 1.0f;
static const float FLIGHT_PHASE_BRAKE_MIN_COMMAND01 = 0.01f;

static const uint32_t FLIGHT_PHASE_LAUNCH_CONFIRM_MS = 60UL;
static const uint32_t FLIGHT_PHASE_BURNOUT_CONFIRM_MS = 120UL;
static const uint32_t FLIGHT_PHASE_DESCENT_CONFIRM_MS = 300UL;
static const uint32_t FLIGHT_PHASE_MIN_BOOST_DWELL_MS = 250UL;
static const uint32_t FLIGHT_PHASE_MIN_COAST_DWELL_MS = 250UL;

static bool g_launch_latched = false;
static bool g_burnout_latched = false;
static bool g_descent_latched = false;

static uint32_t g_launch_latch_ms = 0;
static uint32_t g_burnout_latch_ms = 0;

static uint32_t g_launch_candidate_ms = 0;
static uint32_t g_burnout_candidate_ms = 0;
static uint32_t g_descent_candidate_ms = 0;

/*
elapsed_ms(...)
------------------------------------------------------------------------------
ROLE
  Compute unsigned elapsed time with rollover-safe arithmetic.
*/
static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms) {
  return now_ms - then_ms;
}

/*
condition_confirmed(...)
------------------------------------------------------------------------------
ROLE
  Debounce a boolean condition for a fixed confirmation dwell.

MECHANISM
  The first true sample starts the candidate timer. Continuous true samples keep
  the timer alive. A false sample clears the timer and forces confirmation to
  start over.
*/
static bool condition_confirmed(
  bool condition,
  uint32_t now_ms,
  uint32_t dwell_ms,
  uint32_t &candidate_ms
) {
  if (!condition) {
    candidate_ms = 0;
    return false;
  }

  if (candidate_ms == 0U) {
    candidate_ms = now_ms;
  }

  return elapsed_ms(now_ms, candidate_ms) >= dwell_ms;
}

/*
set_phase(...)
------------------------------------------------------------------------------
ROLE
  Update phase only when the requested phase differs from the current phase.
*/
static void set_phase(SystemState &state, FlightPhase next_phase) {
  if (state.phase != next_phase) {
    state.phase = next_phase;
  }
}

/*
candidate_elapsed_ms(...)
------------------------------------------------------------------------------
ROLE
  Report active confirmation timer age for telemetry.
*/
static uint32_t candidate_elapsed_ms(uint32_t now_ms, uint32_t candidate_ms) {
  if (candidate_ms == 0U) {
    return 0U;
  }

  return elapsed_ms(now_ms, candidate_ms);
}

/*
publish_phase_diag(...)
------------------------------------------------------------------------------
ROLE
  Publish latch and timer observability into SystemState.
*/
static void publish_phase_diag(
  SystemState &state,
  uint32_t now_ms,
  bool boost_dwell_met,
  bool coast_dwell_met,
  bool brake_active
) {
  state.phase_diag.valid = true;
  state.phase_diag.updated = true;
  state.phase_diag.t_ms = now_ms;
  ++state.phase_diag.seq;

  state.phase_diag.launch_latched = g_launch_latched;
  state.phase_diag.burnout_latched = g_burnout_latched;
  state.phase_diag.descent_latched = g_descent_latched;

  state.phase_diag.launch_candidate = (g_launch_candidate_ms != 0U);
  state.phase_diag.burnout_candidate = (g_burnout_candidate_ms != 0U);
  state.phase_diag.descent_candidate = (g_descent_candidate_ms != 0U);

  state.phase_diag.boost_dwell_met = boost_dwell_met;
  state.phase_diag.coast_dwell_met = coast_dwell_met;
  state.phase_diag.brake_active = brake_active;

  state.phase_diag.launch_confirm_ms =
    candidate_elapsed_ms(now_ms, g_launch_candidate_ms);

  state.phase_diag.burnout_confirm_ms =
    candidate_elapsed_ms(now_ms, g_burnout_candidate_ms);

  state.phase_diag.descent_confirm_ms =
    candidate_elapsed_ms(now_ms, g_descent_candidate_ms);

  state.phase_diag.since_launch_ms =
    g_launch_latched ? elapsed_ms(now_ms, g_launch_latch_ms) : 0xFFFFFFFFUL;

  state.phase_diag.since_burnout_ms =
    g_burnout_latched ? elapsed_ms(now_ms, g_burnout_latch_ms) : 0xFFFFFFFFUL;
}

/*
flight_phase_reset(...)
------------------------------------------------------------------------------
ROLE
  Reset phase state and all private detector memory to IDLE.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  state.phase becomes FlightPhase::IDLE. Launch, burnout, descent, and candidate
  timers are cleared.

MECHANISM
  1. Assign IDLE phase.
  2. Clear launch, burnout, and descent latches.
  3. Clear all phase-confirmation timers.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
void flight_phase_reset(SystemState &state) {
  state.phase = FlightPhase::IDLE;

  g_launch_latched = false;
  g_burnout_latched = false;
  g_descent_latched = false;

  g_launch_latch_ms = 0;
  g_burnout_latch_ms = 0;

  g_launch_candidate_ms = 0;
  g_burnout_candidate_ms = 0;
  g_descent_candidate_ms = 0;

  state.phase_diag = FlightPhaseDiag();
}

/*
flight_phase_update(...)
------------------------------------------------------------------------------
ROLE
  Update the stateful flight-phase machine from estimator and IMU snapshots.

INPUT CONTRACT
  state.est and state.imu must be valid before their numeric payloads can be
  interpreted. now_ms must be the current scheduler timestamp.

OUTPUT CONTRACT
  state.phase is updated to a conservative phase estimate. Once launch is
  latched, the detector does not return to IDLE until flight_phase_reset(...).

MECHANISM
  1. Fall back to IDLE before launch when required snapshots are invalid.
  2. Extract altitude, vertical speed, and acceleration norm.
  3. Reject non-finite values before launch and preserve latched phase after
     launch.
  4. Confirm and latch launch before entering BOOST.
  5. Hold BOOST for a minimum dwell and confirm burnout from reduced acceleration.
  6. Enter COAST after burnout.
  7. Enter BRAKE when prior-cycle policy intent reports active braking.
  8. Confirm DESCENT from sustained non-positive vertical speed.

FAILURE BEHAVIOR
  Invalid, missing, or non-finite data produces IDLE before launch. After launch,
  the previous latched phase is preserved so transient sensor invalidity does not
  erase flight history; independent safety freshness checks still gate actuation.

DETERMINISM
  Constant-time scalar checks. No loops. No hardware I/O. No dynamic allocation.
*/
void flight_phase_update(SystemState &state, uint32_t now_ms) {
  bool boost_dwell_met = false;
  bool coast_dwell_met = false;
  bool brake_active = false;

  if (!state.est.valid || !state.imu.valid) {
    if (!g_launch_latched) {
      g_launch_candidate_ms = 0;
      set_phase(state, FlightPhase::IDLE);
    } else if (!g_burnout_latched) {
      g_burnout_candidate_ms = 0;
      g_descent_candidate_ms = 0;
    } else if (!g_descent_latched) {
      g_descent_candidate_ms = 0;
    }

    publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
    return;
  }

  const float h_m = state.est.h_m;
  const float v_mps = state.est.v_mps;
  const float a_norm = state.imu.a_norm;

  if (!is_finite_f(h_m) || !is_finite_f(v_mps) || !is_finite_f(a_norm)) {
    if (!g_launch_latched) {
      g_launch_candidate_ms = 0;
      set_phase(state, FlightPhase::IDLE);
    } else if (!g_burnout_latched) {
      g_burnout_candidate_ms = 0;
      g_descent_candidate_ms = 0;
    } else if (!g_descent_latched) {
      g_descent_candidate_ms = 0;
    }

    publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
    return;
  }

  const bool launch_by_accel =
    (a_norm >= FLIGHT_PHASE_BOOST_ACCEL_NORM_MPS2) &&
    (h_m >= FLIGHT_PHASE_BOOST_MIN_ALT_M);

  const bool launch_by_motion =
    (h_m >= FLIGHT_PHASE_BOOST_MIN_ALT_M) &&
    (v_mps >= FLIGHT_PHASE_LAUNCH_MIN_VZ_MPS);

  if (!g_launch_latched) {
    if (condition_confirmed(
          launch_by_accel || launch_by_motion,
          now_ms,
          FLIGHT_PHASE_LAUNCH_CONFIRM_MS,
          g_launch_candidate_ms)) {
      g_launch_latched = true;
      g_launch_latch_ms = now_ms;
      set_phase(state, FlightPhase::BOOST);
    } else {
      set_phase(state, FlightPhase::IDLE);
    }

    publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
    return;
  }

  if (!g_burnout_latched) {
    boost_dwell_met =
      elapsed_ms(now_ms, g_launch_latch_ms) >= FLIGHT_PHASE_MIN_BOOST_DWELL_MS;

    const bool pre_burnout_descent_candidate =
      boost_dwell_met &&
      (h_m >= FLIGHT_PHASE_COAST_MIN_ALT_M) &&
      (v_mps <= FLIGHT_PHASE_DESCENT_VZ_MPS);

    if (condition_confirmed(
          pre_burnout_descent_candidate,
          now_ms,
          FLIGHT_PHASE_DESCENT_CONFIRM_MS,
          g_descent_candidate_ms)) {
      g_burnout_latched = true;
      g_descent_latched = true;
      g_burnout_latch_ms = now_ms;
      set_phase(state, FlightPhase::DESCENT);
      publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
      return;
    }

    const bool burnout_candidate =
      boost_dwell_met &&
      (a_norm <= FLIGHT_PHASE_BURNOUT_ACCEL_NORM_MPS2) &&
      (h_m >= FLIGHT_PHASE_COAST_MIN_ALT_M) &&
      (v_mps >= FLIGHT_PHASE_COAST_MIN_VZ_MPS);

    if (condition_confirmed(
          burnout_candidate,
          now_ms,
          FLIGHT_PHASE_BURNOUT_CONFIRM_MS,
          g_burnout_candidate_ms)) {
      g_burnout_latched = true;
      g_burnout_latch_ms = now_ms;
      set_phase(state, FlightPhase::COAST);
    } else {
      set_phase(state, FlightPhase::BOOST);
    }

    publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
    return;
  }

  if (!g_descent_latched) {
    coast_dwell_met =
      elapsed_ms(now_ms, g_burnout_latch_ms) >= FLIGHT_PHASE_MIN_COAST_DWELL_MS;

    const bool descent_candidate =
      coast_dwell_met &&
      (h_m >= FLIGHT_PHASE_COAST_MIN_ALT_M) &&
      (v_mps <= FLIGHT_PHASE_DESCENT_VZ_MPS);

    if (condition_confirmed(
          descent_candidate,
          now_ms,
          FLIGHT_PHASE_DESCENT_CONFIRM_MS,
          g_descent_candidate_ms)) {
      g_descent_latched = true;
      set_phase(state, FlightPhase::DESCENT);
      publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
      return;
    }
  }

  if (g_descent_latched) {
    set_phase(state, FlightPhase::DESCENT);
    publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
    return;
  }

  brake_active =
    state.policy.valid &&
    (state.policy.command01 >= FLIGHT_PHASE_BRAKE_MIN_COMMAND01);

  if (brake_active) {
    set_phase(state, FlightPhase::BRAKE);
  } else {
    set_phase(state, FlightPhase::COAST);
  }

  publish_phase_diag(state, now_ms, boost_dwell_met, coast_dwell_met, brake_active);
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

#include "sd_logger.h"

#include <SD.h>
#include <string.h>
#include <math.h>

#include "config.h"
#include "math_utils.h"
#include "telemetry.h"
#include "actuator.h"

/*
sd_logger.cpp
===============================================================================
PURPOSE
  Provide persistent CSV logging to the Teensy built-in SD card.

DESIGN CHANGE
  The logger now records the live estimator and live attitude outputs rather than
  maintaining a second logger-local gravity-vector/Kalman estimator. This keeps
  the SD record aligned with the same estimator state used by policy, safety, and
  telemetry.

FAILURE POLICY
  SD logging is non-critical. Any SD fault disables logging and increments a
  failure counter. Flight computation, telemetry, and actuation safety continue.
===============================================================================
*/

/*
make_next_log_filename(...)
------------------------------------------------------------------------------
ROLE
  Find the first unused LOG###.CSV filename.

INPUT CONTRACT
  out_name must point to a writable character buffer. out_size must be large
  enough for "LOG999.CSV" plus null terminator.

OUTPUT CONTRACT
  On success, out_name contains the first unused filename and true is returned.

MECHANISM
  1. Reject invalid output buffer.
  2. Iterate LOG000.CSV through LOG999.CSV.
  3. Return first candidate not present on SD.

FAILURE BEHAVIOR
  Invalid buffer or exhausted filename space returns false.

DETERMINISM
  Bounded boot-time loop with at most 1000 iterations. No dynamic allocation by
  project code.
*/
static bool make_next_log_filename(char *out_name, size_t out_size) {
  if (!out_name || out_size < 11U) {
    return false;
  }

  // A fixed bounded scan is preferable to dynamic naming logic in flight code:
  // it is deterministic, reviewable, and easy to audit on the SD card later.
  for (int i = 0; i < 1000; ++i) {
    snprintf(out_name, out_size, "LOG%03d.CSV", i);

    if (!SD.exists(out_name)) {
      return true;
    }
  }

  return false;
}

/*
sd_write_header(...)
------------------------------------------------------------------------------
ROLE
  Write the fixed SD CSV schema.

INPUT CONTRACT
  f must refer to an open File object.

OUTPUT CONTRACT
  One CSV header row is written.

MECHANISM
  1. Emit a comma-separated schema.
  2. Keep field order synchronized with sd_logger_service(...).

FAILURE BEHAVIOR
  File write errors are checked later through the File object state.

DETERMINISM
  Bounded file output. No loops. No sensor I/O. No dynamic allocation.
*/
static void sd_write_header(File &f) {
  f.println(
    "t_us,"
    "bmp_T,bmp_P,bmp_alt,"
    "ax,ay,az,gx,gy,gz,"
    "lis_ax,lis_ay,lis_az,"
    "q0,q1,q2,q3,"
    "a_vertical,"
    "est_h,est_v,est_a,"
    "P00,P01,P10,P11,"
    "arm_state,policy_runtime_enabled,software_arm_token,phase,actuator_us,"
    "phase_diag_valid,phase_diag_updated,phase_diag_seq,"
    "phase_diag_t_ms,phase_diag_age_ms,"
    "phase_launch_latched,phase_burnout_latched,phase_descent_latched,"
    "phase_launch_candidate,phase_burnout_candidate,phase_descent_candidate,"
    "phase_boost_dwell_met,phase_coast_dwell_met,phase_brake_active,"
    "phase_launch_confirm_ms,phase_burnout_confirm_ms,phase_descent_confirm_ms,"
    "phase_since_launch_ms,phase_since_burnout_ms,"
    "policy_valid,policy_cmd,"
    "apogee_no_brake,apogee_full_brake,target_apogee,apogee_error,"
    "warn_mask");
}

/*
sd_write_boot_marker(...)
------------------------------------------------------------------------------
ROLE
  Record a comment-style boot marker at the beginning of the SD log.

INPUT CONTRACT
  f must refer to an open File object.

OUTPUT CONTRACT
  One boot marker line is written.

MECHANISM
  1. Write marker prefix.
  2. Record micros().
  3. Record millis().
  4. Terminate line.

FAILURE BEHAVIOR
  File write errors are handled later through file-state checks.

DETERMINISM
  Bounded file output. No loops. No sensor I/O. No dynamic allocation.
*/
static void sd_write_boot_marker(File &f) {
  f.print(F("#BOOT,"));
  f.print(F("t_us="));
  f.print(micros());
  f.print(F(",ms="));
  f.println(millis());
}

/*
sd_disable_runtime(...)
------------------------------------------------------------------------------
ROLE
  Disable SD logging after a runtime fault.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  runtime_failed is latched, file_open is cleared, fail_count increments, and an
  open file is closed.

MECHANISM
  1. Latch runtime failure.
  2. Mark file closed.
  3. Increment failure count.
  4. Close File object when valid.

FAILURE BEHAVIOR
  No recovery is attempted. Repeated SD writes after failure are avoided.

DETERMINISM
  Constant-time except bounded file close. No loops. No dynamic allocation.
*/
static void sd_disable_runtime(SystemState &state) {
  state.sdlog.runtime_failed = true;
  state.sdlog.file_open = false;
  ++state.sdlog.fail_count;

  if (state.sdlog.file) {
    state.sdlog.file.close();
  }
}

/*
sd_logger_reset_reference_and_kf(...)
------------------------------------------------------------------------------
ROLE
  Preserve compatibility with command paths that reset altitude-reference state.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  Logger-local state is not used in this migrated logger. The function clears
  timing counters that depend on the logging stream but leaves file state intact.

MECHANISM
  1. Reset line count.
  2. Reset flush timestamp.
  3. Preserve SD file state.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No loops. No SD I/O. No dynamic allocation.
*/
void sd_logger_reset_reference_and_kf(SystemState &state) {
  state.sdlog.line_count = 0;
  state.sdlog.last_flush_ms = millis();
}

/*
sd_logger_init(...)
------------------------------------------------------------------------------
ROLE
  Initialize the SD logger as a non-fatal subsystem.

INPUT CONTRACT
  state must refer to the shared SystemState object. Serial should already be
  initialized because boot diagnostics are emitted.

OUTPUT CONTRACT
  On success, logging is enabled and a new file is open. On failure, logging
  remains disabled and boot diagnostics report failure.

MECHANISM
  1. Clear runtime SD state.
  2. Initialize timing and filename.
  3. Attempt SD.begin(BUILTIN_SDCARD).
  4. Allocate next unused LOG###.CSV filename.
  5. Open file.
  6. Write header and boot marker.
  7. Flush initial data.
  8. Store file and mark logger enabled.

FAILURE BEHAVIOR
  Any failure leaves logging disabled. Flight runtime may continue.

DETERMINISM
  Boot-time bounded work. Filename scan is bounded. No dynamic allocation by
  project code.
*/
void sd_logger_init(SystemState &state) {
  state.sdlog.enabled = false;
  state.sdlog.card_ok = false;
  state.sdlog.file_open = false;
  state.sdlog.runtime_failed = false;
  state.sdlog.fail_count = 0;
  state.sdlog.next_log_us = micros() + SD_LOG_PERIOD_US;
  state.sdlog.line_count = 0;
  state.sdlog.last_flush_ms = millis();

  strncpy(state.sdlog.filename, "NONE", sizeof(state.sdlog.filename));
  state.sdlog.filename[sizeof(state.sdlog.filename) - 1] = '\0';

#ifndef BUILTIN_SDCARD
  Serial.println(F("BOOT,SD,FAIL"));
  Serial.println(F("BOOT,SD_FILE,NONE"));
  return;
#else
  // Initialization is intentionally attempted once at boot; runtime does not
  // keep retrying SD setup because logging is non-critical and retries would add
  // complexity and timing variability.
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("BOOT,SD,FAIL"));
    Serial.println(F("BOOT,SD_FILE,NONE"));
    return;
  }
#endif

  state.sdlog.card_ok = true;

  char fname[16];

  if (!make_next_log_filename(fname, sizeof(fname))) {
    Serial.println(F("BOOT,SD,FAIL"));
    Serial.println(F("BOOT,SD_FILE,NONE"));
    return;
  }

  File f = SD.open(fname, FILE_WRITE);

  if (!f) {
    Serial.println(F("BOOT,SD,FAIL"));
    Serial.println(F("BOOT,SD_FILE,NONE"));
    return;
  }

  sd_write_header(f);
  sd_write_boot_marker(f);
  f.flush();

  state.sdlog.file = f;
  state.sdlog.file_open = true;
  state.sdlog.enabled = true;

  strncpy(state.sdlog.filename, fname, sizeof(state.sdlog.filename) - 1);
  state.sdlog.filename[sizeof(state.sdlog.filename) - 1] = '\0';

  Serial.println(F("BOOT,SD,OK"));
  Serial.print(F("BOOT,SD_FILE,"));
  Serial.println(state.sdlog.filename);
}

/*
sd_logger_service(...)
------------------------------------------------------------------------------
ROLE
  Write one SD log row when the SD logging period is due.

INPUT CONTRACT
  state must refer to the shared SystemState object. now_us and now_ms must be
  current monotonic timestamps. Sensor and estimator snapshots should have been
  updated earlier in the scheduler pass.

OUTPUT CONTRACT
  If logging is enabled and due, one CSV row is appended. Flush cadence and
  failure flags may be updated.

MECHANISM
  1. Return if logging is disabled, unavailable, closed, or failed.
  2. Return if the next log timestamp has not arrived.
  3. Advance next_log_us.
  4. Extract values from published snapshots, using NAN for invalid streams.
  5. Write one CSV row following sd_write_header(...) order.
  6. Flush on line-count cadence.
  7. Flush on elapsed-time cadence.
  8. Disable logger if File enters failure state.

FAILURE BEHAVIOR
  Invalid snapshots contribute NAN fields rather than shifting CSV columns. SD
  runtime failure latches runtime_failed and disables future logging.

DETERMINISM
  Bounded scalar extraction and bounded file output when due. No SD initialization
  at runtime. No dynamic allocation by project code.
*/
void sd_logger_service(SystemState &state, uint32_t now_us, uint32_t now_ms) {
  if (!state.sdlog.enabled || !state.sdlog.card_ok || !state.sdlog.file_open || state.sdlog.runtime_failed) {
    return;
  }

  if ((int32_t)(now_us - state.sdlog.next_log_us) < 0) {
    return;
  }

  // As with the main scheduler, advancing by a fixed period preserves average
  // cadence even if a particular call is slightly late.
  state.sdlog.next_log_us += SD_LOG_PERIOD_US;

  const float bmpT = state.baro.valid ? state.baro.temp_c : NAN;
  const float bmpP = state.baro.valid ? state.baro.press_hpa : NAN;
  const float bmpAlt = state.baro.valid ? state.baro.alt_m : NAN;

  const float ax = state.imu.valid ? state.imu.ax : NAN;
  const float ay = state.imu.valid ? state.imu.ay : NAN;
  const float az = state.imu.valid ? state.imu.az : NAN;
  const float gx = state.imu.valid ? state.imu.gx : NAN;
  const float gy = state.imu.valid ? state.imu.gy : NAN;
  const float gz = state.imu.valid ? state.imu.gz : NAN;

  const float lis_ax = state.aux.valid ? state.aux.ax : NAN;
  const float lis_ay = state.aux.valid ? state.aux.ay : NAN;
  const float lis_az = state.aux.valid ? state.aux.az : NAN;

  const float q0 = state.attitude.valid ? state.attitude.q0 : NAN;
  const float q1 = state.attitude.valid ? state.attitude.q1 : NAN;
  const float q2 = state.attitude.valid ? state.attitude.q2 : NAN;
  const float q3 = state.attitude.valid ? state.attitude.q3 : NAN;

  const float a_vertical = state.auxvz.valid ? state.auxvz.a_vertical : NAN;

  const float est_h = state.est.valid ? state.est.h_m : NAN;
  const float est_v = state.est.valid ? state.est.v_mps : NAN;
  const float est_a = state.est.valid ? state.est.a_mps2 : NAN;

  const float P00 = state.est.valid ? state.est.P00 : NAN;
  const float P01 = state.est.valid ? state.est.P01 : NAN;
  const float P10 = state.est.valid ? state.est.P10 : NAN;
  const float P11 = state.est.valid ? state.est.P11 : NAN;


  const uint32_t warn_mask = telemetry_warn_mask(state);

  state.sdlog.file.print(now_us);
  state.sdlog.file.print(',');

  state.sdlog.file.print(bmpT);
  state.sdlog.file.print(',');
  state.sdlog.file.print(bmpP);
  state.sdlog.file.print(',');
  state.sdlog.file.print(bmpAlt);
  state.sdlog.file.print(',');

  state.sdlog.file.print(ax);
  state.sdlog.file.print(',');
  state.sdlog.file.print(ay);
  state.sdlog.file.print(',');
  state.sdlog.file.print(az);
  state.sdlog.file.print(',');
  state.sdlog.file.print(gx);
  state.sdlog.file.print(',');
  state.sdlog.file.print(gy);
  state.sdlog.file.print(',');
  state.sdlog.file.print(gz);
  state.sdlog.file.print(',');

  state.sdlog.file.print(lis_ax);
  state.sdlog.file.print(',');
  state.sdlog.file.print(lis_ay);
  state.sdlog.file.print(',');
  state.sdlog.file.print(lis_az);
  state.sdlog.file.print(',');

  state.sdlog.file.print(q0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(q1);
  state.sdlog.file.print(',');
  state.sdlog.file.print(q2);
  state.sdlog.file.print(',');
  state.sdlog.file.print(q3);
  state.sdlog.file.print(',');

  state.sdlog.file.print(a_vertical);
  state.sdlog.file.print(',');

  state.sdlog.file.print(est_h);
  state.sdlog.file.print(',');
  state.sdlog.file.print(est_v);
  state.sdlog.file.print(',');
  state.sdlog.file.print(est_a);
  state.sdlog.file.print(',');

  state.sdlog.file.print(P00);
  state.sdlog.file.print(',');
  state.sdlog.file.print(P01);
  state.sdlog.file.print(',');
  state.sdlog.file.print(P10);
  state.sdlog.file.print(',');
  state.sdlog.file.print(P11);
  state.sdlog.file.print(',');

  state.sdlog.file.print((uint8_t)state.arm_state);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.policy_runtime_enabled ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.software_arm_token ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print((uint8_t)state.phase);
  state.sdlog.file.print(',');
  state.sdlog.file.print(actuator_last_us());
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.valid ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.updated ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.seq);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.t_ms);
  state.sdlog.file.print(',');
  state.sdlog.file.print(age_ms(now_ms, state.phase_diag.t_ms, state.phase_diag.valid));
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.launch_latched ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.burnout_latched ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.descent_latched ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.launch_candidate ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.burnout_candidate ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.descent_candidate ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.boost_dwell_met ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.coast_dwell_met ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.brake_active ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.launch_confirm_ms);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.burnout_confirm_ms);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.descent_confirm_ms);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.since_launch_ms);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.phase_diag.since_burnout_ms);
  state.sdlog.file.print(',');

  state.sdlog.file.print(state.policy.valid ? 1 : 0);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.policy.command01);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.policy.predicted_apogee_no_brake_m);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.policy.predicted_apogee_full_brake_m);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.policy.target_apogee_m);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.policy.apogee_error_m);
  state.sdlog.file.print(',');


  state.sdlog.file.println(warn_mask);

  ++state.sdlog.line_count;

  // Line-count flushing bounds the number of unwritten rows at risk after power
  // loss without forcing an SD flush on every sample.
  if ((state.sdlog.line_count % SD_FLUSH_EVERY_LINES) == 0U) {
    state.sdlog.file.flush();
    state.sdlog.last_flush_ms = now_ms;
  }

  if ((now_ms - state.sdlog.last_flush_ms) >= SD_FLUSH_EVERY_MS) {
    state.sdlog.file.flush();
    state.sdlog.last_flush_ms = now_ms;
  }

  // The File object converts many write failures into a falsy state. Latching
  // that failure here prevents repeated expensive writes after the first fault.
  if (!state.sdlog.file) {
    sd_disable_runtime(state);
  }
}

/*
sd_logger_ok(...)
------------------------------------------------------------------------------
ROLE
  Report whether SD logging is currently operational.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  Returns true only when logger is enabled, card initialized, file open, and no
  runtime fault has been latched.

MECHANISM
  1. Check enabled flag.
  2. Check card_ok flag.
  3. Check file_open flag.
  4. Check runtime_failed latch.

FAILURE BEHAVIOR
  Any unavailable or failed condition returns false.

DETERMINISM
  Constant-time. No loops. No SD I/O. No dynamic allocation.
*/
bool sd_logger_ok(const SystemState &state) {
  return state.sdlog.enabled && state.sdlog.card_ok && state.sdlog.file_open && !state.sdlog.runtime_failed;
}

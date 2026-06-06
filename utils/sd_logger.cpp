#include "sd_logger.h"

#include <SD.h>
#include <string.h>
#include <math.h>

#include "config.h"
#include "math_utils.h"
#include "attitude.h"

/*
sd_logger.cpp
===============================================================================
PURPOSE
  Provide persistent CSV logging to the Teensy built-in SD card.

SUBSYSTEM BOUNDARY
  This module owns the SD file object and all SD failure handling. Other modules
  may inspect SdLoggerState through SystemState but should not directly write to
  state.sdlog.file or mutate SD logger internals.

LOGGER-LOCAL ESTIMATOR
  The uploaded monolithic branch included a logger-local gravity estimate and
  logger-local scalar Kalman filter. This module preserves that behavior so the
  SD log can record an independently computed altitude/velocity trace alongside
  live telemetry.

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
  out_name must point to a writable character buffer.
  out_size must be large enough for "LOG999.CSV" plus null terminator.

MECHANISM
  The function scans LOG000.CSV through LOG999.CSV and returns the first filename
  that does not already exist on the SD card.

FAILURE BEHAVIOR
  If all 1000 names are occupied or out_name is invalid, false is returned.

DETERMINISM
  Bounded loop with a hard upper limit of 1000 iterations. This function is used
  only at boot, not inside the high-rate flight loop.
*/
static bool make_next_log_filename(char *out_name, size_t out_size) {
  if (!out_name || out_size < 11U) {
    return false;
  }

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

MAINTENANCE CONTRACT
  Any change to this header must be mirrored exactly in sd_logger_service(...).
  The SD log is intended for post-test parsing, so field order is a contract.
*/
static void sd_write_header(File &f) {
  f.println(
    "t_us,"
    "bmp_T,bmp_P,bmp_alt,bmp_alt_rel,"
    "ax,ay,az,gx,gy,gz,"
    "lis_ax,lis_ay,lis_az,"
    "g_bx,g_by,g_bz,"
    "a_vertical,"
    "kf_h,kf_v,"
    "P00,P01,P10,P11"
  );
}

/*
sd_write_boot_marker(...)
------------------------------------------------------------------------------
ROLE
  Record a comment-style boot marker at the beginning of the SD log.

MECHANISM
  The marker includes both micros() and millis() so the log has an absolute
  runtime origin for later inspection.
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

OUTPUT CONTRACT
  - runtime_failed becomes true.
  - file_open becomes false.
  - fail_count increments.
  - any open file is closed.

SAFETY
  This function does not stop the flight loop. It only prevents future SD writes.
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
  Reset logger-local reference altitude, gravity vector, and scalar Kalman state.

WHEN CALLED
  This should be called during boot and whenever altitude reference pressure
  changes, because old relative-altitude state is no longer on the same basis.

MECHANISM
  The reset preserves current configuration basis values so the logger can detect
  later sea-level/baseline changes.
*/
void sd_logger_reset_reference_and_kf(SystemState &state) {
  state.sdlog.baro_ref_set = false;
  state.sdlog.baro_ref_alt_m = NAN;

  state.sdlog.ref_basis_baseline_hpa = state.cfg.baro_baseline_hpa;
  state.sdlog.ref_basis_slp_hpa = state.cfg.sea_level_hpa;

  state.sdlog.g_bx = 0.0f;
  state.sdlog.g_by = 0.0f;
  state.sdlog.g_bz = -kG;

  state.sdlog.kf_h = 0.0f;
  state.sdlog.kf_v = 0.0f;

  state.sdlog.P00 = 1.0f;
  state.sdlog.P01 = 0.0f;
  state.sdlog.P10 = 0.0f;
  state.sdlog.P11 = 1.0f;
}

/*
sd_logger_init(...)
------------------------------------------------------------------------------
ROLE
  Initialize the SD logger as a non-fatal subsystem.

STEP-BY-STEP
  1. Clear SD runtime state.
  2. Reset logger-local reference and Kalman state.
  3. Attempt SD.begin(BUILTIN_SDCARD).
  4. Allocate the next LOG###.CSV filename.
  5. Open the file.
  6. Write CSV header and boot marker.
  7. Mark the logger enabled.

FAILURE BEHAVIOR
  Any failure prints a boot diagnostic and leaves logging disabled. The rest of
  the firmware may continue operating.
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

  sd_logger_reset_reference_and_kf(state);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("BOOT,SD,FAIL"));
    Serial.println(F("BOOT,SD_FILE,NONE"));
    return;
  }

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
logger_relative_altitude(...)
------------------------------------------------------------------------------
ROLE
  Convert absolute barometric altitude into logger-local relative altitude.

REFERENCE POLICY
  - If cfg.baro_baseline_hpa is finite, relative altitude is computed against
    the baseline pressure.
  - Otherwise, the first valid barometric altitude observed by the logger becomes
    the local zero reference.

BASIS CHANGE DETECTION
  If sea-level pressure or baseline pressure changes, the logger resets its
  reference and local filter. This prevents mixing samples from incompatible
  altitude bases.
*/
static float logger_relative_altitude(SystemState &state, float bmp_alt) {
  if (!is_finite_f(bmp_alt)) {
    return NAN;
  }

  bool basis_changed = false;

  if (state.sdlog.ref_basis_slp_hpa != state.cfg.sea_level_hpa) {
    basis_changed = true;
  }

  const bool old_baseline_finite = is_finite_f(state.sdlog.ref_basis_baseline_hpa);
  const bool new_baseline_finite = is_finite_f(state.cfg.baro_baseline_hpa);

  if (old_baseline_finite != new_baseline_finite) {
    basis_changed = true;
  }

  if (old_baseline_finite &&
      new_baseline_finite &&
      state.sdlog.ref_basis_baseline_hpa != state.cfg.baro_baseline_hpa) {
    basis_changed = true;
  }

  if (basis_changed) {
    sd_logger_reset_reference_and_kf(state);
  }

  if (is_finite_f(state.cfg.baro_baseline_hpa) &&
      state.cfg.baro_baseline_hpa > 0.0f) {
    const float baseline_alt =
      pressure_to_altitude_m(state.cfg.baro_baseline_hpa, state.cfg.sea_level_hpa);

    return bmp_alt - baseline_alt;
  }

  if (!state.sdlog.baro_ref_set) {
    state.sdlog.baro_ref_alt_m = bmp_alt;
    state.sdlog.baro_ref_set = true;
  }

  return bmp_alt - state.sdlog.baro_ref_alt_m;
}

/*
logger_kf_predict(...)
------------------------------------------------------------------------------
ROLE
  Run the logger-local Kalman prediction step.

INPUT CONTRACT
  a_vertical must be finite.

MECHANISM
  This is the same constant-acceleration altitude/velocity model used by the
  monolithic logger branch:
    h = h + v*Ts + 0.5*a*Ts^2
    v = v + a*Ts
*/
static void logger_kf_predict(SystemState &state, float a_vertical) {
  if (!is_finite_f(a_vertical)) {
    return;
  }

  const float h_pred =
    state.sdlog.kf_h +
    state.sdlog.kf_v * kTs +
    0.5f * a_vertical * kTs * kTs;

  const float v_pred =
    state.sdlog.kf_v +
    a_vertical * kTs;

  const float P00_new =
    state.sdlog.P00 +
    kTs * (state.sdlog.P10 + state.sdlog.P01) +
    kTs * kTs * state.sdlog.P11 +
    kQ00;

  const float P01_new =
    state.sdlog.P01 +
    kTs * state.sdlog.P11 +
    kQ01;

  const float P10_new =
    state.sdlog.P10 +
    kTs * state.sdlog.P11 +
    kQ10;

  const float P11_new =
    state.sdlog.P11 +
    kQ11;

  state.sdlog.kf_h = h_pred;
  state.sdlog.kf_v = v_pred;

  state.sdlog.P00 = P00_new;
  state.sdlog.P01 = P01_new;
  state.sdlog.P10 = P10_new;
  state.sdlog.P11 = P11_new;
}

/*
logger_kf_update(...)
------------------------------------------------------------------------------
ROLE
  Run the logger-local Kalman correction step using barometric relative altitude.

MECHANISM
  Measurement model H = [1 0], so:
    S  = P00 + R
    K0 = P00 / S
    K1 = P10 / S

FAILURE BEHAVIOR
  If the innovation covariance is non-positive, the update is skipped to avoid
  division by zero.
*/
static void logger_kf_update(SystemState &state, float z_meas) {
  if (!is_finite_f(z_meas)) {
    return;
  }

  const float S = state.sdlog.P00 + kR;

  if (S < 1e-9f) {
    return;
  }

  const float K0 = state.sdlog.P00 / S;
  const float K1 = state.sdlog.P10 / S;
  const float innovation = z_meas - state.sdlog.kf_h;

  const float P00_old = state.sdlog.P00;
  const float P01_old = state.sdlog.P01;
  const float P10_old = state.sdlog.P10;
  const float P11_old = state.sdlog.P11;

  state.sdlog.kf_h = state.sdlog.kf_h + K0 * innovation;
  state.sdlog.kf_v = state.sdlog.kf_v + K1 * innovation;

  state.sdlog.P00 = (1.0f - K0) * P00_old;
  state.sdlog.P01 = (1.0f - K0) * P01_old;
  state.sdlog.P10 = P10_old - K1 * P00_old;
  state.sdlog.P11 = P11_old - K1 * P01_old;
}

/*
sd_logger_service(...)
------------------------------------------------------------------------------
ROLE
  Write one SD log row when the SD log period is due.

INPUT CONTRACT
  All consumed sensor values come from published snapshots in SystemState.
  Invalid snapshots contribute NAN payloads to preserve CSV field alignment.

STEP-BY-STEP
  1. Return immediately if SD logging is unavailable or disabled.
  2. Return immediately if the log period is not due.
  3. Extract sensor values or NAN based on snapshot validity.
  4. Compute relative barometric altitude.
  5. Compute vertical acceleration from IMU if valid.
  6. Predict logger-local Kalman state if acceleration is available.
  7. Update logger-local Kalman state if barometric altitude is available.
  8. Write one CSV row.
  9. Flush by line-count and time cadence.
  10. Disable logger if the file object reports failure.

DETERMINISM
  The function performs bounded scalar math and one bounded write sequence when
  due. It performs no filename scan and no SD initialization at runtime.
*/
void sd_logger_service(SystemState &state, uint32_t now_us, uint32_t now_ms) {
  if (!state.sdlog.enabled ||
      !state.sdlog.card_ok ||
      !state.sdlog.file_open ||
      state.sdlog.runtime_failed) {
    return;
  }

  if ((int32_t)(now_us - state.sdlog.next_log_us) < 0) {
    return;
  }

  state.sdlog.next_log_us += SD_LOG_PERIOD_US;

  const float bmpT = state.baro.valid ? state.baro.temp_c : NAN;
  const float bmpP = state.baro.valid ? state.baro.press_hpa : NAN;
  const float bmpAlt = state.baro.valid ? state.baro.alt_m : NAN;
  const float bmpAltRel = state.baro.valid ? logger_relative_altitude(state, state.baro.alt_m) : NAN;

  const float ax = state.imu.valid ? state.imu.ax : NAN;
  const float ay = state.imu.valid ? state.imu.ay : NAN;
  const float az = state.imu.valid ? state.imu.az : NAN;
  const float gx = state.imu.valid ? state.imu.gx : NAN;
  const float gy = state.imu.valid ? state.imu.gy : NAN;
  const float gz = state.imu.valid ? state.imu.gz : NAN;

  const float lis_ax = state.aux.valid ? state.aux.ax : NAN;
  const float lis_ay = state.aux.valid ? state.aux.ay : NAN;
  const float lis_az = state.aux.valid ? state.aux.az : NAN;

  float a_vertical = NAN;

  if (state.imu.valid &&
      is_finite_f(ax) &&
      is_finite_f(ay) &&
      is_finite_f(az)) {
    attitude_update_gravity_reference(state, ax, ay, az);
    a_vertical = attitude_compute_vertical_accel(state, ax, ay, az);

    if (is_finite_f(a_vertical)) {
      logger_kf_predict(state, a_vertical);
    }
  }

  if (state.baro.valid && is_finite_f(bmpAltRel)) {
    logger_kf_update(state, bmpAltRel);
  }

  state.sdlog.file.print(now_us);
  state.sdlog.file.print(',');
  state.sdlog.file.print(bmpT);
  state.sdlog.file.print(',');
  state.sdlog.file.print(bmpP);
  state.sdlog.file.print(',');
  state.sdlog.file.print(bmpAlt);
  state.sdlog.file.print(',');
  state.sdlog.file.print(bmpAltRel);
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

  state.sdlog.file.print(state.sdlog.g_bx);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.sdlog.g_by);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.sdlog.g_bz);
  state.sdlog.file.print(',');

  state.sdlog.file.print(a_vertical);
  state.sdlog.file.print(',');

  state.sdlog.file.print(state.sdlog.kf_h);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.sdlog.kf_v);
  state.sdlog.file.print(',');

  state.sdlog.file.print(state.sdlog.P00);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.sdlog.P01);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.sdlog.P10);
  state.sdlog.file.print(',');
  state.sdlog.file.print(state.sdlog.P11);
  state.sdlog.file.println();

  ++state.sdlog.line_count;

  if ((state.sdlog.line_count % SD_FLUSH_EVERY_LINES) == 0U) {
    state.sdlog.file.flush();
    state.sdlog.last_flush_ms = now_ms;
  }

  if ((now_ms - state.sdlog.last_flush_ms) >= SD_FLUSH_EVERY_MS) {
    state.sdlog.file.flush();
    state.sdlog.last_flush_ms = now_ms;
  }

  if (!state.sdlog.file) {
    sd_disable_runtime(state);
  }
}

/*
sd_logger_ok(...)
------------------------------------------------------------------------------
ROLE
  Compact predicate for telemetry/status checks.
*/
bool sd_logger_ok(const SystemState &state) {
  return state.sdlog.enabled &&
         state.sdlog.card_ok &&
         state.sdlog.file_open &&
         !state.sdlog.runtime_failed;
}
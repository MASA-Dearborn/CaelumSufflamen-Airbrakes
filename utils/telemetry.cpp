#include "telemetry.h"

#include "config.h"
#include "math_utils.h"

/*
telemetry.cpp
===============================================================================
PURPOSE
  Emit fixed-schema telemetry and diagnostics.

CSV CONTRACT
  Header and row field order must remain synchronized. Invalid values are not
  reinterpreted; they are emitted directly so downstream analysis can detect NAN
  or invalid flags.

WARNING MASK CONTRACT
  Warning bits compactly summarize hardware health, snapshot validity, config
  validity, and SD health.
===============================================================================
*/

/*
telemetry_warn_mask(...)
------------------------------------------------------------------------------
ROLE
  Build compact health and validity bitmask.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  Returns a uint32_t warning mask.

MECHANISM
  1. Start from zero.
  2. Set hardware-health bits.
  3. Set snapshot-validity bits.
  4. Set configuration and SD-fault bits.
  5. Return the accumulated mask.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time. No loops. No hardware I/O. No dynamic allocation.
*/
uint32_t telemetry_warn_mask(const SystemState &state) {
  uint32_t w = 0U;

  if (!state.health.bmp_ok) w |= (1UL << 0);
  if (!state.health.bmi_accel_ok) w |= (1UL << 1);
  if (!state.health.bmi_gyro_ok) w |= (1UL << 2);
  if (!state.health.lis_ok) w |= (1UL << 3);

  if (!state.baro.valid) w |= (1UL << 4);
  if (!state.imu.valid) w |= (1UL << 5);
  if (!state.aux.valid) w |= (1UL << 6);
  if (!state.auxvz.valid) w |= (1UL << 7);
  if (!state.attitude.valid) w |= (1UL << 8);
  if (!state.est.valid) w |= (1UL << 9);
  if (!state.cfg.valid) w |= (1UL << 10);

  if ((!state.sdlog.card_ok) || state.sdlog.runtime_failed) {
    w |= (1UL << WARN_SD_FAULT_BIT);
  }

  return w;
}

/*
telemetry_print_header(...)
------------------------------------------------------------------------------
ROLE
  Emit the compact CSV schema.

INPUT CONTRACT
  Serial should already be initialized.

OUTPUT CONTRACT
  A single header line is emitted.

MECHANISM
  1. Print one fixed header string.
  2. Maintain exact field order correspondence with telemetry_emit_tlm(...).

FAILURE BEHAVIOR
  Serial output is best-effort.

DETERMINISM
  Bounded Serial output. No loops. No hardware I/O. No dynamic allocation.
*/
void telemetry_print_header(void) {
  Serial.println(
    F("HDR,t_ms,"
      "baro_valid,baro_upd,baro_seq,bmp_T,bmp_P,bmp_alt,"
      "imu_valid,imu_upd,imu_seq,ax,ay,az,gx,gy,gz,"
      "aux_valid,aux_seq,lis_ax,lis_ay,lis_az,"
      "att_valid,att_seq,q0,q1,q2,q3,"
      "auxvz_valid,a_vertical,"
      "est_valid,est_seeded,est_seq,est_h,est_v,est_a,"
      "sea_level_hpa,baro_baseline_hpa,"
      "warn_mask")
  );
}

/*
telemetry_emit_tlm(...)
------------------------------------------------------------------------------
ROLE
  Emit one compact telemetry row.

INPUT CONTRACT
  Snapshot valid flags are emitted next to payload values so the log consumer can
  decide which numerical fields are meaningful.

OUTPUT CONTRACT
  A single TLM CSV row is emitted.

MECHANISM
  1. Print current timestamp.
  2. Print snapshot flags and payloads.
  3. Print attitude and estimator outputs.
  4. Print configuration references.
  5. Print warning mask.

FAILURE BEHAVIOR
  Invalid snapshots still occupy their CSV fields. Valid flags define payload
  usability.

DETERMINISM
  Bounded Serial output. No loops. No hardware I/O. No dynamic allocation.
*/
void telemetry_emit_tlm(const SystemState &s) {
  Serial.print(F("TLM,"));
  Serial.print(millis());
  Serial.print(',');

  Serial.print(s.baro.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.baro.updated ? 1 : 0);
  Serial.print(',');
  Serial.print(s.baro.seq);
  Serial.print(',');
  Serial.print(s.baro.temp_c);
  Serial.print(',');
  Serial.print(s.baro.press_hpa);
  Serial.print(',');
  Serial.print(s.baro.alt_m);
  Serial.print(',');

  Serial.print(s.imu.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.imu.updated ? 1 : 0);
  Serial.print(',');
  Serial.print(s.imu.seq);
  Serial.print(',');
  Serial.print(s.imu.ax);
  Serial.print(',');
  Serial.print(s.imu.ay);
  Serial.print(',');
  Serial.print(s.imu.az);
  Serial.print(',');
  Serial.print(s.imu.gx);
  Serial.print(',');
  Serial.print(s.imu.gy);
  Serial.print(',');
  Serial.print(s.imu.gz);
  Serial.print(',');

  Serial.print(s.aux.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.aux.seq);
  Serial.print(',');
  Serial.print(s.aux.ax);
  Serial.print(',');
  Serial.print(s.aux.ay);
  Serial.print(',');
  Serial.print(s.aux.az);
  Serial.print(',');

  Serial.print(s.attitude.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.attitude.seq);
  Serial.print(',');
  Serial.print(s.attitude.q0);
  Serial.print(',');
  Serial.print(s.attitude.q1);
  Serial.print(',');
  Serial.print(s.attitude.q2);
  Serial.print(',');
  Serial.print(s.attitude.q3);
  Serial.print(',');

  Serial.print(s.auxvz.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.auxvz.a_vertical);
  Serial.print(',');

  Serial.print(s.est.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.est.seeded ? 1 : 0);
  Serial.print(',');
  Serial.print(s.est.seq);
  Serial.print(',');
  Serial.print(s.est.h_m);
  Serial.print(',');
  Serial.print(s.est.v_mps);
  Serial.print(',');
  Serial.print(s.est.a_mps2);
  Serial.print(',');

  Serial.print(s.cfg.sea_level_hpa);
  Serial.print(',');
  Serial.print(s.cfg.baro_baseline_hpa);
  Serial.print(',');
  Serial.println(telemetry_warn_mask(s));
}

/*
telemetry_print_status(...)
------------------------------------------------------------------------------
ROLE
  Emit human-readable system status for command-line inspection.

INPUT CONTRACT
  state must refer to the shared SystemState object.

OUTPUT CONTRACT
  A compact status line is emitted.

MECHANISM
  1. Print sensor health flags.
  2. Print major snapshot validity flags.
  3. Print active SD filename.

FAILURE BEHAVIOR
  No recovery is attempted.

DETERMINISM
  Bounded Serial output. No loops. No hardware I/O. No dynamic allocation.
*/
void telemetry_print_status(const SystemState &s) {
  Serial.print(F("STATUS,bmp_ok="));
  Serial.print(s.health.bmp_ok ? 1 : 0);

  Serial.print(F(",bmi_accel_ok="));
  Serial.print(s.health.bmi_accel_ok ? 1 : 0);

  Serial.print(F(",bmi_gyro_ok="));
  Serial.print(s.health.bmi_gyro_ok ? 1 : 0);

  Serial.print(F(",lis_ok="));
  Serial.print(s.health.lis_ok ? 1 : 0);

  Serial.print(F(",baro_valid="));
  Serial.print(s.baro.valid ? 1 : 0);

  Serial.print(F(",imu_valid="));
  Serial.print(s.imu.valid ? 1 : 0);

  Serial.print(F(",att_valid="));
  Serial.print(s.attitude.valid ? 1 : 0);

  Serial.print(F(",est_valid="));
  Serial.print(s.est.valid ? 1 : 0);

  Serial.print(F(",baro_baseline_hpa="));
  Serial.print(s.cfg.baro_baseline_hpa);

  Serial.print(F(",sd_file="));
  Serial.println(s.sdlog.filename);
}

/*
telemetry_emit_diag(...)
------------------------------------------------------------------------------
ROLE
  Emit age-oriented diagnostics.

INPUT CONTRACT
  state must refer to the shared SystemState object. now_ms must be current
  millis() time.

OUTPUT CONTRACT
  A single diagnostic line is emitted.

MECHANISM
  1. Print current timestamp.
  2. Print age of major snapshots using invalid-sentinel semantics.
  3. Print warning mask.

FAILURE BEHAVIOR
  Invalid snapshots produce the age sentinel instead of a misleading fresh age.

DETERMINISM
  Bounded Serial output. No loops. No hardware I/O. No dynamic allocation.
*/
void telemetry_emit_diag(const SystemState &s, uint32_t now_ms) {
  Serial.print(F("DIAG,RUN,t_ms="));
  Serial.print(now_ms);

  Serial.print(F(",baro_age_ms="));
  Serial.print(age_ms(now_ms, s.baro.t_ms, s.baro.valid));

  Serial.print(F(",imu_age_ms="));
  Serial.print(age_ms(now_ms, s.imu.t_ms, s.imu.valid));

  Serial.print(F(",att_age_ms="));
  Serial.print(age_ms(now_ms, s.attitude.t_ms, s.attitude.valid));

  Serial.print(F(",auxvz_age_ms="));
  Serial.print(age_ms(now_ms, s.auxvz.t_ms, s.auxvz.valid));

  Serial.print(F(",est_age_ms="));
  Serial.print(age_ms(now_ms, s.est.t_ms, s.est.valid));

  Serial.print(F(",warn_mask="));
  Serial.println(telemetry_warn_mask(s));
}
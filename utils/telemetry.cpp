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

MECHANISM
  Each bit corresponds to one persistent diagnostic condition. The mask is meant
  for quick scanning of logs and automated post-processing.
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
  if (!state.est.valid) w |= (1UL << 8);
  if (!state.cfg.valid) w |= (1UL << 9);

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

MAINTENANCE RULE
  Any change here must be mirrored exactly in telemetry_emit_tlm(...).
*/
void telemetry_print_header(void) {
  Serial.println(
    F("HDR,t_ms,"
      "baro_valid,baro_seq,bmp_T,bmp_P,bmp_alt,"
      "imu_valid,imu_seq,ax,ay,az,gx,gy,gz,"
      "aux_valid,aux_seq,lis_ax,lis_ay,lis_az,"
      "auxvz_valid,a_vertical,"
      "est_valid,est_seq,est_h,est_v,"
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
*/
void telemetry_emit_tlm(const SystemState &s) {
  Serial.print(F("TLM,"));
  Serial.print(millis());
  Serial.print(',');

  Serial.print(s.baro.valid ? 1 : 0);
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

  Serial.print(s.auxvz.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.auxvz.a_vertical);
  Serial.print(',');

  Serial.print(s.est.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(s.est.seq);
  Serial.print(',');
  Serial.print(s.est.h_m);
  Serial.print(',');
  Serial.print(s.est.v_mps);
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

  Serial.print(F(",aux_valid="));
  Serial.print(s.aux.valid ? 1 : 0);

  Serial.print(F(",auxvz_valid="));
  Serial.print(s.auxvz.valid ? 1 : 0);

  Serial.print(F(",est_valid="));
  Serial.print(s.est.valid ? 1 : 0);

  Serial.print(F(",sd_file="));
  Serial.println(s.sdlog.filename);
}

/*
telemetry_emit_diag(...)
------------------------------------------------------------------------------
ROLE
  Emit age-oriented diagnostics.

MECHANISM
  age_ms(...) returns a sentinel for invalid snapshots. This makes invalid data
  visibly different from fresh data.
*/
void telemetry_emit_diag(const SystemState &s, uint32_t now_ms) {
  Serial.print(F("DIAG,RUN,t_ms="));
  Serial.print(now_ms);

  Serial.print(F(",baro_age_ms="));
  Serial.print(age_ms(now_ms, s.baro.t_ms, s.baro.valid));

  Serial.print(F(",imu_age_ms="));
  Serial.print(age_ms(now_ms, s.imu.t_ms, s.imu.valid));

  Serial.print(F(",aux_age_ms="));
  Serial.print(age_ms(now_ms, s.aux.t_ms, s.aux.valid));

  Serial.print(F(",auxvz_age_ms="));
  Serial.print(age_ms(now_ms, s.auxvz.t_ms, s.auxvz.valid));

  Serial.print(F(",est_age_ms="));
  Serial.print(age_ms(now_ms, s.est.t_ms, s.est.valid));

  Serial.print(F(",warn_mask="));
  Serial.println(telemetry_warn_mask(s));
}
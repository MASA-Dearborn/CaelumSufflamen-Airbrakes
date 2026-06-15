#include "sensors.h"

#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include "BMI088.h"
#include <LIS2DU12Sensor.h>

#include "config.h"
#include "math_utils.h"

/*
sensors.cpp
===============================================================================
PURPOSE
  Own physical sensor objects and publish validity-qualified snapshots.

DETERMINISM
  Runtime poll functions make at most one acquisition attempt per call. No
  runtime poll function waits for readiness. The only bounded waits are boot or
  operator-commanded diagnostics/calibrations.
===============================================================================
*/

static Adafruit_BMP5xx bmp;

#if BMI088_ENABLED
static Bmi088Accel accel(Wire, 0x18);
static Bmi088Gyro gyro(Wire, 0x68);
#endif

#if LIS2DU12_ENABLED
static LIS2DU12Sensor lis(&Wire);
#endif

/*
sensors_begin(...)
------------------------------------------------------------------------------
ROLE
  Initialize all enabled sensors and publish boot health flags.

INPUT CONTRACT
  state must refer to the shared SystemState object. I2C pins and bus hardware
  must match board wiring.

OUTPUT CONTRACT
  state.health fields are set according to hardware initialization results.
  Return value is true if at least one enabled sensor initializes successfully.

MECHANISM
  1. Start Wire.
  2. Attempt BMP5xx initialization at supported addresses.
  3. Configure BMP5xx oversampling, IIR filter, output rate, and pressure mode.
  4. Attempt BMI088 accelerometer and gyroscope initialization.
  5. Attempt LIS2DU12 initialization and enable acceleration output.
  6. Return aggregate hardware availability.

FAILURE BEHAVIOR
  Partial initialization is allowed. Failed sensors publish invalid snapshots
  during runtime and appear in telemetry warning masks.

DETERMINISM
  Bounded boot-time hardware initialization. No runtime retry loop. No dynamic
  allocation by project code.
*/
bool sensors_begin(SystemState &state) {
  // One shared I2C bus is brought up before any device-specific initialization.
  Wire.begin();

  if (bmp.begin(0x46, &Wire) || bmp.begin(0x47, &Wire)) {
    state.health.bmp_ok = true;

    // These BMP settings trade modest latency for cleaner pressure output at the
    // firmware's 50 Hz scheduler rate.
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    bmp.enablePressure(true);
  }

#if BMI088_ENABLED
  state.health.bmi_accel_ok = (accel.begin() >= 0);
  state.health.bmi_gyro_ok = (gyro.begin() >= 0);
#else
  state.health.bmi_accel_ok = false;
  state.health.bmi_gyro_ok = false;
#endif

#if LIS2DU12_ENABLED
  if (lis.begin() == 0) {
    lis.Enable_X();
    state.health.lis_ok = true;
  } else {
    state.health.lis_ok = false;
  }
#else
  state.health.lis_ok = false;
#endif

  return state.health.bmp_ok ||
         state.health.bmi_accel_ok ||
         state.health.bmi_gyro_ok ||
         state.health.lis_ok;
}

/*
sensors_print_status(...)
------------------------------------------------------------------------------
ROLE
  Print boot-time hardware availability in compact human-readable form.

INPUT CONTRACT
  state.health should already have been populated by sensors_begin(...).

OUTPUT CONTRACT
  Serial receives four status lines. SystemState is not modified.

MECHANISM
  1. Print BMP5xx health.
  2. Print BMI088 accelerometer health.
  3. Print BMI088 gyroscope health.
  4. Print LIS2DU12 health.

FAILURE BEHAVIOR
  No recovery is attempted. This function only reports existing health flags.

DETERMINISM
  Bounded Serial output. No loops. No sensor transactions.
*/
void sensors_print_status(const SystemState &state) {
  Serial.print(F("[ BMP5xx init ] : "));
  Serial.println(state.health.bmp_ok ? F("OK") : F("FAIL"));

  Serial.print(F("[ BMI088 accel ] : "));
  Serial.println(state.health.bmi_accel_ok ? F("OK") : F("FAIL"));

  Serial.print(F("[ BMI088 gyro  ] : "));
  Serial.println(state.health.bmi_gyro_ok ? F("OK") : F("FAIL"));

  Serial.print(F("[ LIS2DU12 ] : "));
  Serial.println(state.health.lis_ok ? F("OK") : F("FAIL"));
}

/*
sensors_bmp_data_ready_within(...)
------------------------------------------------------------------------------
ROLE
  Boot-only diagnostic for BMP5xx data-ready behavior.

INPUT CONTRACT
  timeout_ms is the maximum diagnostic wait duration.

OUTPUT CONTRACT
  Returns true if BMP data-ready becomes true before timeout.

MECHANISM
  1. Capture start time.
  2. Poll bmp.dataReady().
  3. Delay 1 ms between polls.
  4. Return false if timeout expires.

FAILURE BEHAVIOR
  Timeout returns false without mutating SystemState.

DETERMINISM
  Bounded blocking loop. This function must not be called from the flight runtime
  path.
*/
bool sensors_bmp_data_ready_within(uint32_t timeout_ms) {
  const uint32_t t0 = millis();

  while ((millis() - t0) < timeout_ms) {
    if (bmp.dataReady()) return true;
    delay(1);
  }

  return false;
}

/*
sensors_poll_baro(...)
------------------------------------------------------------------------------
ROLE
  Publish one BMP5xx barometer snapshot.

INPUT CONTRACT
  state must refer to the shared SystemState object. now_ms must be current
  millis() time. state.cfg.sea_level_hpa should be finite and positive.

OUTPUT CONTRACT
  state.baro.t_ms and t_us are updated for the acquisition attempt. updated is
  cleared before acquisition and asserted only after a valid new sample. valid is
  true only after successful finite temperature, pressure, and altitude outputs.

MECHANISM
  1. Timestamp the attempt.
  2. Clear updated flag.
  3. Reject missing BMP hardware.
  4. Attempt exactly one BMP reading.
  5. Convert pressure from Pa to hPa.
  6. Convert hPa pressure to altitude.
  7. Validate numerical outputs.
  8. Publish payload and increment sequence on success.

FAILURE BEHAVIOR
  Failed hardware initialization or failed reading sets valid=false and returns.
  Downstream readers must ignore payload unless valid is true.

DETERMINISM
  One hardware transaction attempt. No retry loop. No dynamic allocation.
*/
bool sensors_poll_baro(SystemState &state, uint32_t now_ms) {
  const uint32_t now_us = micros();

  state.baro.t_ms = now_ms;
  state.baro.t_us = now_us;
  state.baro.updated = false;

  if (!state.health.bmp_ok) {
    state.baro.valid = false;
    return true;
  }

  if (!bmp.performReading()) {
    state.baro.valid = false;
    return true;
  }

  // The Adafruit BMP driver reports pressure in pascals; the rest of this
  // firmware standardizes on hectopascals for atmospheric calculations and
  // logging.
  const float press_hpa = bmp.pressure / 100.0f;
  const float temp_c = bmp.temperature;
  const float alt_m = pressure_to_altitude_m(press_hpa, state.cfg.sea_level_hpa);

  state.baro.valid =
    is_finite_f(press_hpa) &&
    is_finite_f(temp_c) &&
    is_finite_f(alt_m);

  if (state.baro.valid) {
    state.baro.updated = true;
    ++state.baro.seq;

    state.baro.temp_c = temp_c;
    state.baro.press_hpa = press_hpa;
    state.baro.alt_m = alt_m;
  }

  return true;
}

/*
sensors_poll_imu(...)
------------------------------------------------------------------------------
ROLE
  Publish one BMI088 IMU snapshot.

INPUT CONTRACT
  state must refer to the shared SystemState object. now_ms must be current
  millis() time.

OUTPUT CONTRACT
  state.imu timestamps and updated flag are refreshed. valid is true when either
  accelerometer or gyroscope stream provides finite data.

MECHANISM
  1. Timestamp the attempt.
  2. Initialize local values to NAN.
  3. Read accelerometer once if available.
  4. Read gyroscope once if available.
  5. Validate accel and gyro streams independently.
  6. Publish available values when at least one stream is valid.
  7. Compute acceleration norm only when accelerometer data is valid.

FAILURE BEHAVIOR
  If both streams fail or are unavailable, state.imu.valid becomes false.

DETERMINISM
  At most one accelerometer read and one gyro read. No retry loop. No dynamic
  allocation.
*/
bool sensors_poll_imu(SystemState &state, uint32_t now_ms) {
#if BMI088_ENABLED
  const uint32_t now_us = micros();

  state.imu.t_ms = now_ms;
  state.imu.t_us = now_us;
  state.imu.updated = false;

  bool accel_valid = false;
  bool gyro_valid = false;

  float ax = NAN;
  float ay = NAN;
  float az = NAN;
  float gx = NAN;
  float gy = NAN;
  float gz = NAN;

  if (state.health.bmi_accel_ok) {
    // Exactly one read is attempted per cycle so timing remains bounded and the
    // publication semantics stay simple.
    accel.readSensor();
    ax = accel.getAccelX_mss();
    ay = accel.getAccelY_mss();
    az = accel.getAccelZ_mss();
    accel_valid = is_finite_f(ax) && is_finite_f(ay) && is_finite_f(az);
  }

  if (state.health.bmi_gyro_ok) {
    gyro.readSensor();
    gx = gyro.getGyroX_rads();
    gy = gyro.getGyroY_rads();
    gz = gyro.getGyroZ_rads();
    gyro_valid = is_finite_f(gx) && is_finite_f(gy) && is_finite_f(gz);
  }

  state.imu.valid = accel_valid || gyro_valid;

  if (!state.imu.valid) {
    return true;
  }

  state.imu.updated = true;
  ++state.imu.seq;

  state.imu.ax = ax;
  state.imu.ay = ay;
  state.imu.az = az;
  state.imu.gx = gx;
  state.imu.gy = gy;
  state.imu.gz = gz;
  state.imu.a_norm = accel_valid ? sqrtf(ax * ax + ay * ay + az * az) : NAN;

  return true;
#else
  (void)now_ms;
  state.imu.updated = false;
  state.imu.valid = false;
  return true;
#endif
}

/*
sensors_poll_aux(...)
------------------------------------------------------------------------------
ROLE
  Publish one LIS2DU12 auxiliary accelerometer snapshot.

INPUT CONTRACT
  state must refer to the shared SystemState object. now_ms must be current
  millis() time.

OUTPUT CONTRACT
  state.aux timestamps and updated flag are refreshed. valid is true only when
  converted x/y/z acceleration values are finite.

MECHANISM
  1. Timestamp the attempt.
  2. Reject missing LIS2DU12 hardware.
  3. Read raw integer axes once.
  4. Read sensitivity once.
  5. Convert raw counts to m/s^2.
  6. Validate converted values.
  7. Publish payload and sequence on success.

FAILURE BEHAVIOR
  Failed initialization or invalid conversion sets state.aux.valid=false.

DETERMINISM
  One raw-axis read and one sensitivity read. No retry loop. No dynamic allocation.
*/
bool sensors_poll_aux(SystemState &state, uint32_t now_ms) {
#if LIS2DU12_ENABLED
  const uint32_t now_us = micros();

  state.aux.t_ms = now_ms;
  state.aux.t_us = now_us;
  state.aux.updated = false;

  if (!state.health.lis_ok) {
    state.aux.valid = false;
    return true;
  }

  int16_t raw[3] = { 0, 0, 0 };
  float sensitivity = 0.0f;

  lis.Get_X_AxesRaw(raw);
  lis.Get_X_Sensitivity(&sensitivity);

  // Sensitivity is typically reported in mg/LSB, so the conversion is:
  //   raw * sensitivity -> mg
  //   mg * 0.001        -> g
  //   g * 9.80665       -> m/s^2
  const float ax = raw[0] * sensitivity * 0.001f * 9.80665f;
  const float ay = raw[1] * sensitivity * 0.001f * 9.80665f;
  const float az = raw[2] * sensitivity * 0.001f * 9.80665f;

  state.aux.valid = is_finite_f(ax) && is_finite_f(ay) && is_finite_f(az);

  if (state.aux.valid) {
    state.aux.updated = true;
    ++state.aux.seq;

    state.aux.ax = ax;
    state.aux.ay = ay;
    state.aux.az = az;
    state.aux.a_norm = sqrtf(ax * ax + ay * ay + az * az);
  }

  return true;
#else
  (void)now_ms;
  state.aux.updated = false;
  state.aux.valid = false;
  return true;
#endif
}

/*
sensors_calibrate_baro_base(...)
------------------------------------------------------------------------------
ROLE
  Capture an averaged local barometer baseline pressure.

INPUT CONTRACT
  state must refer to the shared SystemState object. BMP5xx must be initialized.
  sample_count should be positive. This routine is intended for boot or ground
  command use, not flight-loop runtime.

OUTPUT CONTRACT
  On success, state.cfg.baro_baseline_hpa receives the average pressure in hPa,
  state.cfg.valid is asserted, and true is returned.

MECHANISM
  1. Reject unavailable barometer or zero sample count.
  2. Repeatedly poll one barometer sample.
  3. Accumulate finite pressure samples in hPa.
  4. Delay between samples because this is a deliberate calibration routine.
  5. Publish average baseline pressure.

FAILURE BEHAVIOR
  If no valid samples are collected, baseline remains unchanged and false is
  returned.

DETERMINISM
  Bounded blocking loop with sample_count iterations. No dynamic allocation.
*/
bool sensors_calibrate_baro_base(SystemState &state, uint16_t sample_count) {
  if (!state.health.bmp_ok) return false;
  if (sample_count == 0U) return false;

  float sum_hpa = 0.0f;
  uint16_t count = 0U;

  for (uint16_t i = 0; i < sample_count; ++i) {
    const uint32_t now_ms = millis();

    sensors_poll_baro(state, now_ms);

    if (state.baro.valid && is_finite_f(state.baro.press_hpa)) {
      sum_hpa += state.baro.press_hpa;
      ++count;
    }

    delay(BARO_CALIB_SAMPLE_DELAY_MS);
  }

  if (count == 0U) {
    return false;
  }

  state.cfg.baro_baseline_hpa = sum_hpa / (float)count;
  state.cfg.valid = true;

  return true;
}

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
  runtime poll function waits for readiness. The only bounded wait is the boot
  diagnostic sensors_bmp_data_ready_within().
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

MECHANISM
  Each sensor initialization is attempted once. Success is stored in SensorHealth
  so later poll functions can distinguish hardware absence from transient sample
  failure.

FAILURE BEHAVIOR
  Partial initialization is allowed. Failed sensors simply publish invalid
  snapshots during runtime.
*/
bool sensors_begin(SystemState &state) {
  Wire.begin();

  if (bmp.begin(0x46, &Wire) || bmp.begin(0x47, &Wire)) {
    state.health.bmp_ok = true;

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
  Print boot-time hardware availability in a compact human-readable format.
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

DETERMINISM NOTE
  This function intentionally waits, but only during boot diagnostics. It must
  not be called from the flight runtime path.
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

OUTPUT CONTRACT
  t_ms is updated for the acquisition attempt. valid is true only after a
  successful reading with finite temperature, pressure, and altitude.

FAILURE BEHAVIOR
  Failed hardware initialization or failed reading sets valid=false and returns.
*/
bool sensors_poll_baro(SystemState &state, uint32_t now_ms) {
  state.baro.t_ms = now_ms;

  if (!state.health.bmp_ok) {
    state.baro.valid = false;
    return true;
  }

  if (!bmp.performReading()) {
    state.baro.valid = false;
    return true;
  }

  const float press_hpa = bmp.pressure / 100.0f;
  const float temp_c = bmp.temperature;
  const float alt_m = pressure_to_altitude_m(press_hpa, state.cfg.sea_level_hpa);

  state.baro.valid =
    is_finite_f(press_hpa) &&
    is_finite_f(temp_c) &&
    is_finite_f(alt_m);

  if (state.baro.valid) {
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

MECHANISM
  Accelerometer and gyro validity are evaluated independently. A snapshot is
  considered valid when either stream produces finite data. Missing stream fields
  remain NAN.

FAILURE BEHAVIOR
  If both streams fail, valid=false and numeric payload should be ignored.
*/
bool sensors_poll_imu(SystemState &state, uint32_t now_ms) {
#if BMI088_ENABLED
  state.imu.t_ms = now_ms;

  bool accel_valid = false;
  bool gyro_valid = false;

  float ax = NAN;
  float ay = NAN;
  float az = NAN;
  float gx = NAN;
  float gy = NAN;
  float gz = NAN;

  if (state.health.bmi_accel_ok) {
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
  state.imu.valid = false;
  return true;
#endif
}

/*
sensors_poll_aux(...)
------------------------------------------------------------------------------
ROLE
  Publish one LIS2DU12 auxiliary accelerometer snapshot.

MECHANISM
  Raw integer axes are multiplied by device sensitivity, converted from mg to g,
  then converted to m/s^2.

FAILURE BEHAVIOR
  Failed initialization or non-finite conversion publishes valid=false.
*/
bool sensors_poll_aux(SystemState &state, uint32_t now_ms) {
#if LIS2DU12_ENABLED
  state.aux.t_ms = now_ms;

  if (!state.health.lis_ok) {
    state.aux.valid = false;
    return true;
  }

  int16_t raw[3] = { 0, 0, 0 };
  float sensitivity = 0.0f;

  lis.Get_X_AxesRaw(raw);
  lis.Get_X_Sensitivity(&sensitivity);

  const float ax = raw[0] * sensitivity * 0.001f * 9.80665f;
  const float ay = raw[1] * sensitivity * 0.001f * 9.80665f;
  const float az = raw[2] * sensitivity * 0.001f * 9.80665f;

  state.aux.valid = is_finite_f(ax) && is_finite_f(ay) && is_finite_f(az);

  if (state.aux.valid) {
    ++state.aux.seq;
    state.aux.ax = ax;
    state.aux.ay = ay;
    state.aux.az = az;
    state.aux.a_norm = sqrtf(ax * ax + ay * ay + az * az);
  }

  return true;
#else
  (void)now_ms;
  state.aux.valid = false;
  return true;
#endif
}
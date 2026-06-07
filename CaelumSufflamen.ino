#include <Arduino.h>

#include "config.h"
#include "data_types.h"
#include "math_utils.h"
#include "sensors.h"
#include "attitude.h"
#include "estimation.h"
#include "telemetry.h"
#include "commands.h"
#include "safety.h"
#include "airbrake_policy.h"
#include "actuator.h"
#include "sd_logger.h"

/*
CaelumSufflamen.ino
===============================================================================
ROLE
  Top-level deterministic scheduler and orchestration layer.

ORDERING CONTRACT
  Runtime order is intentionally fixed:

    1. Heartbeat and command service.
    2. Time-gated loop admission.
    3. Sensor snapshot publication.
    4. Madgwick/Kalman estimator update.
    5. Safety and policy evaluation.
    6. Safety-gated actuator update.
    7. Telemetry, diagnostics, and SD logging.

  This order ensures downstream computations consume the freshest snapshots
  available in the current cycle and telemetry reflects the same state used for
  policy and actuator decisions.

DETERMINISM CONTRACT
  loop() contains no hidden waits, heap allocation, retry loops, or blocking
  sensor transactions. CAL_BASELINE and boot calibration are deliberate bounded
  ground/boot operations, not flight-loop operations.
===============================================================================
*/

static SystemState state;

static uint32_t last_heartbeat_ms = 0;
static bool led_state = true;

static uint32_t next_loop_us = 0;
static uint32_t last_diag_ms = 0;
static uint32_t last_tlm_ms = 0;

/*
initialize_state(...)
------------------------------------------------------------------------------
ROLE
  Populate runtime structures with known conservative defaults before hardware
  initialization.

INPUT CONTRACT
  No inputs are required. The global state object exists statically.

OUTPUT CONTRACT
  Configuration, actuator calibration, attitude, estimator, and policy state are
  initialized to safe defaults.

MECHANISM
  1. Set runtime configuration defaults.
  2. Set actuator calibration defaults.
  3. Reset attitude to identity.
  4. Reset estimator and policy outputs.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time assignments. No hardware I/O. No dynamic allocation.
*/
static void initialize_state(void) {
  state.cfg.valid = true;
  state.cfg.serial_header_enable = true;
  state.cfg.sea_level_hpa = DEFAULT_SEA_LEVEL_HPA;
  state.cfg.baro_baseline_hpa = NAN;

  state.actuator_cfg.servo_us_min = SERVO_US_MIN_DEFAULT;
  state.actuator_cfg.servo_us_max = SERVO_US_MAX_DEFAULT;
  state.actuator_cfg.servo_us_idle = SERVO_US_IDLE_DEFAULT;

  attitude_begin(state);
  estimation_reset(state);
  airbrake_policy_reset(state);
}

/*
heartbeat(...)
------------------------------------------------------------------------------
ROLE
  Toggle the status LED at a fixed low-rate cadence.

INPUT CONTRACT
  STATUS_LED must have been configured as OUTPUT.

OUTPUT CONTRACT
  LED state toggles whenever HEARTBEAT_MS has elapsed.

MECHANISM
  1. Read millis().
  2. Compare unsigned elapsed time against HEARTBEAT_MS.
  3. Toggle stored LED state.
  4. Write LED pin and update timestamp.

FAILURE BEHAVIOR
  No failure path exists.

DETERMINISM
  Constant-time pin update only when due.
*/
static void heartbeat(void) {
  const uint32_t now_ms = millis();

  if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_MS) {
    led_state = !led_state;
    digitalWrite(STATUS_LED, led_state ? HIGH : LOW);
    last_heartbeat_ms = now_ms;
  }
}

/*
setup(...)
------------------------------------------------------------------------------
ROLE
  Perform one-time initialization and publish boot diagnostics.

INPUT CONTRACT
  Arduino runtime must have started.

OUTPUT CONTRACT
  Sensors, estimator, SD logger, actuator, telemetry, and scheduler baselines are
  initialized. Sensor/SD failures are reported but are not fatal.

MECHANISM
  1. Initialize software state.
  2. Configure status LED and Serial.
  3. Initialize sensors and print health.
  4. Attempt boot barometer baseline calibration when BMP is available.
  5. Reset estimator after baseline capture.
  6. Initialize SD logger after baseline so logged basis matches estimator basis.
  7. Attach actuator and force idle.
  8. Print command help and telemetry header.
  9. Establish scheduler baselines.

FAILURE BEHAVIOR
  Sensor and SD initialization failures are stored in SystemState and reported by
  telemetry. The firmware continues with partial subsystem availability.

DETERMINISM
  Boot contains bounded waits only: Serial wait timeout and barometer calibration.
  Runtime loop remains non-blocking.
*/
void setup() {
  initialize_state();

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  led_state = true;

  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < BOOT_SERIAL_TIMEOUT_MS) {
    // Bounded boot-only wait for USB Serial. Runtime loop never blocks on Serial.
  }

  Serial.println(F("BOOT,BEGIN"));

  sensors_begin(state);
  sensors_print_status(state);

  if (state.health.bmp_ok) {
    if (sensors_calibrate_baro_base(state, BARO_CALIB_SAMPLES)) {
      estimation_reset(state);

      Serial.print(F("BOOT,BARO_BASELINE,"));
      Serial.println(state.cfg.baro_baseline_hpa);
    } else {
      Serial.println(F("BOOT,BARO_BASELINE,FAIL"));
    }
  }

  sd_logger_init(state);

  actuator_begin(state.actuator_cfg);
  actuator_force_idle();

  commands_print_help();
  telemetry_print_header();

  next_loop_us = micros() + LOOP_PERIOD_US;
  last_diag_ms = millis();
  last_tlm_ms = millis();

  Serial.println(F("BOOT,READY"));
}

/*
loop(...)
------------------------------------------------------------------------------
ROLE
  Execute one bounded scheduler pass.

INPUT CONTRACT
  setup() must have completed.

OUTPUT CONTRACT
  When the main period is due, fresh sensor snapshots are acquired, the estimator
  consumes updated flags, safety/policy are evaluated, actuator output is gated,
  and telemetry/logging are emitted according to cadence.

MECHANISM
  1. Service heartbeat and non-blocking commands.
  2. Return early if the main loop period is not due.
  3. Poll barometer, IMU, and auxiliary accelerometer once.
  4. Run migrated estimator kernel:
       IMU update -> Madgwick attitude -> vertical acceleration -> Kalman predict
       Baro update -> relative altitude -> Kalman correction
  5. Compute policy intent.
  6. Apply actuator command only if safety and policy gates pass.
  7. Emit telemetry and diagnostics by cadence.
  8. Service SD logger by cadence.

FAILURE BEHAVIOR
  Invalid estimator, invalid policy, disabled actuation, or stale state forces
  idle actuator output.

DETERMINISM
  Runtime pass is bounded. No blocking waits, retry loops, or dynamic allocation.
*/
void loop() {
  heartbeat();
  commands_service(state);

  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

  if ((int32_t)(now_us - next_loop_us) < 0) {
    if ((now_ms - last_diag_ms) >= DIAG_PERIOD_MS) {
      telemetry_emit_diag(state, now_ms);
      last_diag_ms = now_ms;
    }
    return;
  }

  next_loop_us += LOOP_PERIOD_US;

  sensors_poll_baro(state, now_ms);
  sensors_poll_imu(state, now_ms);
  sensors_poll_aux(state, now_ms);

  estimation_update(state, now_ms);

  state.policy = airbrake_policy_compute(state);

  if (safety_allows_actuation(state) && state.policy.valid) {
    actuator_write_command01(state.policy.command01);
  } else {
    actuator_force_idle();
  }

  if ((now_ms - last_tlm_ms) >= TLM_PERIOD_MS) {
    if (state.cfg.serial_header_enable) {
      telemetry_emit_tlm(state);
    }

    last_tlm_ms = now_ms;
  }

  if ((now_ms - last_diag_ms) >= DIAG_PERIOD_MS) {
    telemetry_emit_diag(state, now_ms);
    last_diag_ms = now_ms;
  }

  sd_logger_service(state, now_us, now_ms);
}
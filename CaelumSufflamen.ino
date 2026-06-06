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
  The runtime order is intentionally fixed:

    1. Heartbeat and command service.
    2. Time-gated loop admission.
    3. Sensor snapshot publication.
    4. Derived vertical acceleration update.
    5. Kalman estimator update.
    6. Safety and policy evaluation.
    7. Safety-gated actuator update.
    8. Telemetry, diagnostics, and SD logging.

  This order ensures that downstream computations consume the freshest published
  snapshots available in the current cycle and that telemetry reflects the same
  state used for policy and actuator decisions.

DETERMINISM CONTRACT
  loop() does not contain hidden waits, heap allocation, retry loops, or blocking
  sensor transactions. Each subsystem call performs bounded work.
===============================================================================
*/

static SystemState state;

static uint32_t last_heartbeat_ms = 0;
static bool led_state = true;

static uint32_t next_loop_us = 0;
static uint32_t last_diag_ms = 0;
static uint32_t last_tlm_ms = 0;

/*
initialize_state()
------------------------------------------------------------------------------
ROLE
  Populate all runtime structures with known conservative defaults before any
  hardware initialization occurs.

MECHANISM
  Default member initializers in data_types.h establish most safe defaults. This
  function then explicitly sets the few fields tied to compile-time constants and
  resets estimator/policy/gravity modules through their owning APIs.

SAFETY
  The actuator command starts invalid and the actuator configuration starts at
  idle-safe defaults. The servo is not attached here; attachment belongs to the
  actuator module.
*/
static void initialize_state(void) {
  state = SystemState();

  state.cfg.valid = true;
  state.cfg.serial_header_enable = true;
  state.cfg.sea_level_hpa = DEFAULT_SEA_LEVEL_HPA;
  state.cfg.baro_baseline_hpa = NAN;

  state.actuator_cfg.servo_us_min = SERVO_US_MIN_DEFAULT;
  state.actuator_cfg.servo_us_max = SERVO_US_MAX_DEFAULT;
  state.actuator_cfg.servo_us_idle = SERVO_US_IDLE_DEFAULT;

  attitude_reset_gravity_reference(state);
  estimation_reset(state);
  airbrake_policy_reset(state);
}

/*
heartbeat()
------------------------------------------------------------------------------
ROLE
  Toggle the status LED at a fixed low-rate cadence.

MECHANISM
  millis() subtraction is used so wraparound behavior remains well-defined for
  unsigned timestamps.

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
setup()
------------------------------------------------------------------------------
ROLE
  Perform one-time initialization and publish boot diagnostics.

STEP-BY-STEP
  1. Initialize all software state to conservative defaults.
  2. Configure LED and Serial.
  3. Initialize SD logger after Serial is ready because it prints boot status.
  4. Initialize sensors and print health results.
  5. Attach actuator and force idle.
  6. Print command help and telemetry header.
  7. Establish scheduler baselines.

FAILURE BEHAVIOR
  Sensor and SD initialization failures are recorded in SystemState. The system
  still boots so telemetry can expose partial subsystem availability.
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

  sd_logger_init(state);

  sensors_begin(state);
  sensors_print_status(state);

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
loop()
------------------------------------------------------------------------------
ROLE
  Execute one bounded scheduler pass.

MECHANISM
  The outer loop runs frequently, but the main flight pipeline is admitted only
  when LOOP_PERIOD_US has elapsed. Diagnostics may still emit while waiting.

SAFETY
  The actuator is never driven directly from the policy module. The policy must
  produce a valid command and the safety module must permit actuation; otherwise
  the actuator is forced to idle.
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

  if (state.imu.valid) {
    attitude_update_gravity_reference(state, state.imu.ax, state.imu.ay, state.imu.az);
  }

  attitude_update_aux_vertical(state, now_ms);
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
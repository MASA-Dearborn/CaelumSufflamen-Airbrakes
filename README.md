# Caelum Sufflamen Airbrakes

**Caelum Sufflamen** is a deterministic embedded avionics and airbrake-control firmware scaffold. The project emphasizes bounded execution, traceable sensor snapshots, quaternion-based attitude estimation, Kalman-filter-based vertical state estimation, safety-gated actuation, real-time telemetry, and persistent SD-card logging.

The name *Caelum Sufflamen* means “sky brake.” In this repository, the phrase refers to the software architecture supporting a rocket airbrake module: a flight-computer runtime that observes sensor data, estimates vertical state, evaluates an airbrake policy, and only permits actuator motion through explicit safety gates.

> **Safety posture:** non-idle airbrake actuation is disabled by default. A build must explicitly enable the compile-time actuation and policy switches before the firmware can command non-idle servo output.

---

## Table of Contents

- [Project Goals](#project-goals)
- [System Architecture](#system-architecture)
- [Repository Layout](#repository-layout)
- [Hardware Target](#hardware-target)
- [Core Runtime Contracts](#core-runtime-contracts)
- [Estimator Pipeline](#estimator-pipeline)
- [Safety Model](#safety-model)
- [Serial Command Interface](#serial-command-interface)
- [Telemetry and SD Logging](#telemetry-and-sd-logging)
- [Build and Integration Notes](#build-and-integration-notes)
- [Bring-Up Procedure](#bring-up-procedure)
- [Verification Checklist](#verification-checklist)
- [Known Integration Notes](#known-integration-notes)
- [Future Development](#future-development)
- [Safety Notice](#safety-notice)

---

## Project Goals

This repository is intended to support disciplined embedded avionics development rather than only producing a single monolithic flight sketch.

Primary goals:

- build a deterministic single-threaded flight-software runtime,
- publish sensor and estimator state through explicit validity-qualified snapshots,
- estimate altitude and vertical velocity from barometer and IMU-derived acceleration,
- provide transparent telemetry for bench testing and post-flight review,
- maintain persistent SD-card logs for estimator tuning and verification,
- isolate actuator output behind compile-time and runtime safety gates,
- preserve a modular architecture that can support future airbrake policy development.

---

## System Architecture

The firmware is organized as a fixed-order runtime pipeline:

```text
Heartbeat + Serial Commands
        |
        v
Time-Gated Main Loop Admission
        |
        v
Sensor Snapshot Publication
  - BMP5xx barometer
  - BMI088 accelerometer / gyroscope
  - LIS2DU12 auxiliary accelerometer
        |
        v
Madgwick Attitude Estimation
        |
        v
Quaternion-Based Vertical Acceleration
        |
        v
2-State Kalman Altitude / Vertical Velocity Estimator
        |
        v
Airbrake Policy Intent
        |
        v
Safety-Gated Actuator Output
        |
        v
Telemetry + Diagnostics + SD Logging
```

The top-level scheduler is intentionally conservative:

- sensor poll functions attempt at most one acquisition per call,
- estimator functions perform bounded scalar math,
- telemetry and logging run at configured cadences,
- unsafe or invalid states force the actuator to idle,
- blocking routines are reserved for boot-time or deliberate ground operations.

---

## Repository Layout

```text
CaelumSufflamen-Airbrakes/
├── CaelumSufflamen.ino      # Top-level Arduino/Teensy scheduler
├── include/                 # Core public interfaces and shared data contracts
│   ├── airbrake_policy.h
│   ├── actuator.h
│   ├── attitude.h
│   ├── config.h
│   ├── data_types.h
│   ├── estimation.h
│   ├── kalman_alt2.h
│   ├── safety.h
│   └── sensors.h
├── src/                     # Core subsystem implementations
│   ├── airbrake_policy.cpp
│   ├── actuator.cpp
│   ├── attitude.cpp
│   ├── config.cpp
│   ├── estimation.cpp
│   ├── kalman_alt2.cpp
│   ├── safety.cpp
│   └── sensors.cpp
└── utils/                   # Utility subsystems
    ├── commands.cpp
    ├── math_utils.h
    ├── sd_logger.cpp
    ├── sd_logger.h
    └── telemetry.cpp
```

### Module Responsibilities

| Module | Responsibility |
|---|---|
| `CaelumSufflamen.ino` | Top-level setup, scheduler, fixed runtime ordering |
| `config.h` | Compile-time switches, scheduler rates, tuning constants, safety defaults |
| `data_types.h` | Shared snapshot structs and runtime data contracts |
| `sensors.*` | Sensor initialization, health flags, barometer/IMU/aux snapshot publication |
| `attitude.*` | Madgwick quaternion update and vertical acceleration projection |
| `kalman_alt2.*` | Deterministic 2-state altitude/vertical-velocity Kalman filter |
| `estimation.*` | Estimator orchestration: IMU prediction plus barometer correction |
| `airbrake_policy.*` | Policy-level airbrake command intent |
| `safety.*` | Final safety predicate for actuation permission |
| `actuator.*` | Servo attachment, idle forcing, normalized command mapping |
| `commands.*` | Bounded Serial command parser |
| `telemetry.*` | Serial telemetry and diagnostic output |
| `sd_logger.*` | Persistent CSV logging and SD fault handling |
| `math_utils.h` | Small math/parsing helpers shared by modules |

---

## Hardware Target

The firmware is written for an Arduino/Teensy-style embedded environment, with the current design matching a Teensy 4.1-class avionics stack.

### Current Subsystems

| Subsystem | Device / Interface | Purpose |
|---|---|---|
| Barometer | BMP5xx over I2C | Pressure, temperature, and barometric altitude measurement |
| Primary IMU | BMI088 accelerometer + BMI088 gyroscope over I2C | Body-frame acceleration and angular rate |
| Auxiliary accelerometer | LIS2DU12 over I2C | Auxiliary acceleration stream for diagnostics or future redundancy |
| Storage | SD card | Persistent CSV log files |
| Actuator | PWM servo output on `PIN_AIRBRAKE_SERVO` | Airbrake deployment command output |
| Operator I/O | USB Serial | Commands, telemetry, diagnostics |

---

## Core Runtime Contracts

### 1. Published Snapshot Contract

Sensor and estimator data are exchanged through plain structs called snapshots. A snapshot represents the most recent known state of a subsystem and carries both payload values and metadata.

Common fields:

| Field | Meaning |
|---|---|
| `valid` | Payload values are semantically usable |
| `updated` | A new sample/event was published for the current consumer pass |
| `t_ms` | Millisecond timestamp for freshness checks and diagnostics |
| `t_us` | Microsecond timestamp for measured-`dt` estimator propagation |
| `seq` | Successful-publication counter |
| payload fields | Numeric values with module-specific units |

Reader rule:

```text
Payload values are meaningful only when:
  valid == true
  required numeric fields are finite
  data age is acceptable for the consuming subsystem
```

Invalid snapshots may contain stale numeric values. Consumers must gate on metadata, not only on payload values.

### 2. Deterministic Runtime Contract

The runtime is designed to avoid hidden work:

- no heap allocation in the flight loop,
- no unbounded retry loops in sensor polling,
- no blocking waits inside the main scheduler path,
- bounded string parsing for Serial commands,
- bounded scalar math for estimator and policy logic.

Boot-time and ground-command exceptions are explicitly documented. For example, barometer baseline calibration uses a bounded sample loop and delay because it is intended for boot or ground procedures, not for flight-loop execution.

### 3. Fail-Safe Actuator Contract

The actuator defaults to idle. Any invalid or unsafe condition must result in idle output, not a best-effort command.

Fail-idle causes include:

- actuation disabled at compile time,
- airbrake policy disabled at compile time,
- invalid estimator state,
- stale estimator state,
- invalid policy output,
- failed safety predicate,
- invalid command input.

---

## Estimator Pipeline

The current estimator kernel ports the validated airbrakes estimator design into the modular project architecture.

### Attitude Update

The attitude module uses a Madgwick-style IMU update:

1. gyroscope data integrates angular velocity,
2. accelerometer direction constrains gravity alignment,
3. the quaternion is normalized after each update,
4. the quaternion snapshot is published with validity and timestamps.

### Vertical Acceleration

After a valid attitude update, body-frame acceleration is rotated into the world vertical axis. Gravity is removed using the configured gravitational constant, producing a world-frame vertical linear acceleration estimate.

```text
a_vertical = world_z_component(body_accel, q) + g
```

The sign convention follows the validated estimator kernel used during migration.

### Kalman Filter State

The vertical Kalman filter estimates:

```text
x = [ h, v ]^T
```

where:

| State | Units | Meaning |
|---|---|---|
| `h` | meters | relative altitude |
| `v` | meters/second | vertical velocity, positive upward |

### Prediction Model

The filter uses measured IMU `dt`, not a fixed timestep:

```text
h(k+1) = h(k) + v(k) * dt + 0.5 * a * dt^2
v(k+1) = v(k) + a * dt
```

The covariance prediction uses a discretized white-acceleration process-noise model.

### Measurement Update

The barometer supplies the altitude measurement. Pressure is converted from Pa to hPa before altitude conversion. A captured baseline pressure can be used to make altitude relative to the launch-site baseline.

The Kalman update uses Joseph-form covariance correction for improved numerical robustness.

---

## Safety Model

Airbrake actuation is safety-critical and is therefore guarded by independent layers.

### Compile-Time Gates

The default build is non-actuating:

```cpp
#ifndef ACTUATION_ENABLED
#define ACTUATION_ENABLED 0
#endif

#ifndef AIRBRAKE_POLICY_ENABLED
#define AIRBRAKE_POLICY_ENABLED 0
#endif
```

A development or flight-test build must intentionally override these values. Leaving them unset keeps the actuator in an idle-safe behavior.

### Runtime Policy and Safety Gates

The airbrake policy computes intent only. The actuator layer applies output only after safety checks pass.

A command reaches hardware only when all relevant conditions are true:

```text
actuation compiled in
AND policy compiled in
AND estimator valid/fresh
AND policy output valid
AND safety predicate allows actuation
```

Otherwise, `actuator_force_idle()` is used.

---

## Serial Command Interface

The Serial command parser is intentionally small and bounded. It consumes only already-available Serial bytes and dispatches complete CR/LF-terminated command lines.

Supported commands:

```text
HELP
STATUS
HDR 0|1
SET_SLP <hpa>
CAP_BASELINE
CAL_BASELINE
```

| Command | Purpose |
|---|---|
| `HELP` | Print the supported command list |
| `STATUS` | Print compact system status |
| `HDR 0` | Disable periodic telemetry header printing |
| `HDR 1` | Enable telemetry header printing and print a header immediately |
| `SET_SLP <hpa>` | Set the sea-level pressure reference in hPa |
| `CAP_BASELINE` | Capture the current valid barometer pressure as the baseline |
| `CAL_BASELINE` | Run a bounded averaged barometer baseline calibration |

`CAL_BASELINE` is a deliberate bounded blocking ground operation. It should be used during setup, bench testing, or preflight calibration, not during active flight-loop execution.

---

## Telemetry and SD Logging

The firmware has two observation channels:

1. **Serial telemetry** for live monitoring and debugging.
2. **SD logging** for persistent post-test or post-flight analysis.

Logged and telemetered data include:

- sensor validity flags,
- barometer pressure, temperature, and altitude,
- IMU acceleration and angular rate,
- auxiliary acceleration,
- quaternion attitude state,
- vertical acceleration,
- Kalman altitude and vertical velocity,
- covariance terms,
- configuration reference pressures,
- policy output,
- actuator command state,
- warning masks and diagnostics.

SD-card failure is non-fatal. If initialization or runtime writes fail, the logger disables itself and records the fault in runtime state so telemetry can report the condition.

---

## Bring-Up Procedure

Recommended bench bring-up sequence:

1. Build with default safety settings:

   ```cpp
   ACTUATION_ENABLED = 0
   AIRBRAKE_POLICY_ENABLED = 0
   ```

2. Connect the board over USB Serial at `115200` baud.
3. Boot the firmware and confirm:

   ```text
   BOOT,BEGIN
   BOOT,READY
   ```

4. Confirm sensor health output matches connected hardware.
5. Run:

   ```text
   STATUS
   ```

6. Calibrate the local barometer baseline:

   ```text
   CAL_BASELINE
   ```

7. Confirm the response:

   ```text
   ACK,CAL_BASELINE,<pressure_hpa>
   ```

8. Confirm telemetry rows are emitted at the expected cadence.
9. Confirm SD logging creates a log file when SD hardware is present.
10. Confirm the actuator remains idle with default safety settings.
11. Perform estimator bench checks before enabling any actuation-related build flags.

---

## Verification Checklist

### Static and Compile Verification

- [ ] All headers resolve under the selected build system.
- [ ] Sensor libraries are installed and version-compatible.
- [ ] Servo backend compiles for the target board.
- [ ] Compile-time actuation flags are intentionally set.

### Sensor Verification

- [ ] BMP5xx initializes successfully when connected.
- [ ] BMI088 accelerometer initializes successfully when connected.
- [ ] BMI088 gyroscope initializes successfully when connected.
- [ ] LIS2DU12 initializes successfully when connected.
- [ ] Barometer pressure appears in hPa.
- [ ] IMU acceleration appears in m/s^2.
- [ ] Gyroscope output appears in rad/s.

### Estimator Verification

- [ ] Quaternion values remain finite.
- [ ] Quaternion norm remains near one after updates.
- [ ] Vertical acceleration remains finite at rest.
- [ ] Kalman estimator seeds only after a valid barometric altitude exists.
- [ ] Estimated altitude remains near zero after baseline calibration at rest.
- [ ] Estimated vertical velocity remains bounded at rest.
- [ ] Covariance terms remain finite and non-explosive.

### Telemetry and Logging Verification

- [ ] Telemetry header and row field order are stable.
- [ ] Invalid floating-point values print clearly as `nan` where applicable.
- [ ] Warning masks change when sensors are disconnected or invalid.
- [ ] SD logger creates unique log files.
- [ ] SD write or initialization failure does not stop the main runtime.

### Actuation Verification

- [ ] Default build keeps actuator idle.
- [ ] Invalid estimator state forces idle.
- [ ] Disabled policy forces idle.
- [ ] Safety predicate failure forces idle.
- [ ] Servo pulse range is calibrated before any non-idle test.
- [ ] Non-idle testing occurs only in a controlled bench setup.

---

## Known Integration Notes

- `CAL_BASELINE` intentionally blocks for a bounded number of samples; it is a calibration routine, not a flight-loop operation.
- The current airbrake policy constants are placeholders for future validated control-law development.
- Compile-time actuation gates should remain disabled until estimator, sensor, and actuator behavior have been verified on the bench.

---

## Future Development

Potential extensions include:

- formal flight-state machine integration,
- GPS or GNSS sensor fusion,
- magnetometer heading integration,
- redundant sensor validation and voting,
- airbrake deployment law based on apogee prediction,
- hardware-in-the-loop simulation,
- automated compile checks in CI,
- telemetry parser and plotting tools,
- post-flight estimator tuning notebooks,
- actuator envelope validation and calibration documentation.

---

## Safety Notice

This repository contains experimental aerospace firmware. It is not a certified flight controller and must not be treated as flight-ready without independent review, ground testing, simulation, range-safety review, and team approval.

Actuator output can create mechanical hazards. Airbrake hardware should be restrained or disconnected during early software testing. Non-idle actuation should be enabled only after sensor validity, estimator behavior, safety gating, and servo calibration have been verified in a controlled environment.

---

## Author / Organization

Developed for the MASA Dearborn airbrakes / avionics effort.

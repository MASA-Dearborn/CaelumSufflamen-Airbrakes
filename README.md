# Caelum Sufflamen Airbrakes

**Caelum Sufflamen** is a deterministic embedded avionics and airbrake-control software scaffold for high-power rocketry research. The project focuses on bounded scheduling, sensor snapshot traceability, Kalman-filter-based vertical state estimation, safety-gated actuation, real-time telemetry, and persistent SD-card flight logging.

The current implementation is designed as an engineering and verification platform. Non-idle airbrake motion is disabled by default unless the build explicitly enables the compile-time actuation and policy gates.

---

## System Overview

The firmware is organized as a deterministic single-threaded pipeline:

```text
Heartbeat + Commands
        |
        v
Time-Gated Scheduler
        |
        v
Sensor Snapshot Publication
        |
        v
Madgwick Attitude + Vertical Acceleration
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
Serial Telemetry + Diagnostics + SD Logging
```

The top-level `loop()` performs bounded work only. Sensor calls are one-attempt polling calls, estimator work is scalar math, and unsafe or invalid states force the actuator to idle.

---

## Repository Layout

```text
CaelumSufflamen-Airbrakes/
├── CaelumSufflamen.ino      # Top-level Arduino/Teensy scheduler
├── include/                 # Core public interfaces and data contracts
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
└── utils/                   # Telemetry, commands, SD logging, math helpers
    ├── commands.cpp
    ├── math_utils.h
    ├── sd_logger.cpp
    ├── sd_logger.h
    └── telemetry.cpp
```

> **Integration note:** the top-level sketch includes utility headers such as `commands.h`, `telemetry.h`, `sd_logger.h`, and `math_utils.h`. The implementation should keep header locations and include paths consistent with the selected build system.

---

## Hardware Target

The project is intended for a Teensy-class embedded target, with the current architecture matching a Teensy 4.1 style avionics stack.

### Current Sensor Suite

| Subsystem | Device / Interface | Purpose |
|---|---|---|
| Barometer | BMP5xx over I2C | Pressure, temperature, altitude measurement |
| Primary IMU | BMI088 accelerometer + gyroscope over I2C | Body-frame acceleration and angular rate |
| Auxiliary accelerometer | LIS2DU12 over I2C | Auxiliary acceleration stream |
| Storage | Built-in / attached SD interface | Persistent CSV flight/test logging |
| Actuator | Servo PWM on `PIN_AIRBRAKE_SERVO` | Airbrake deployment output |

---

## Core Software Concepts

### Published Snapshots

The shared runtime state is represented through validity-qualified snapshots. Each major measurement or estimate contains:

- `valid`: whether the numeric payload is semantically usable,
- `updated`: whether a new sample/event was published for estimator consumption,
- `t_ms`: millisecond timestamp for diagnostics and freshness checks,
- `t_us`: microsecond timestamp for measured-`dt` propagation,
- `seq`: monotonically increasing publication counter,
- numeric payload fields with explicit units.

Downstream modules should consume payload values only when `valid == true` and the required numeric fields are finite.

### Deterministic Scheduling

The top-level scheduler runs at 50 Hz:

```cpp
static const uint32_t LOOP_HZ = 50UL;
static const uint32_t LOOP_PERIOD_US = 1000000UL / LOOP_HZ;
```

Telemetry is emitted at 10 Hz, diagnostics at 1 Hz, and SD logging at 50 Hz by default.

### Attitude and Vertical Acceleration

The attitude module implements a Madgwick-style quaternion update using gyro and accelerometer input. The quaternion is used to rotate body-frame acceleration into world vertical acceleration. Gravity is then removed so the estimator can use the resulting vertical linear acceleration as a Kalman prediction input.

### Kalman Estimator

The vertical estimator uses a 2-state Kalman filter:

```text
x = [ h, v ]^T
```

where:

- `h` is altitude in meters,
- `v` is vertical velocity in meters per second.

The estimator uses:

- barometric altitude as the measurement update,
- quaternion-derived vertical acceleration as the prediction input,
- measured IMU `dt` rather than a fixed estimator timestep.

---

## Safety Model

Airbrake actuation is intentionally gated at multiple layers.

### Compile-Time Gates

By default, both are disabled:

```cpp
#ifndef ACTUATION_ENABLED
#define ACTUATION_ENABLED 0
#endif

#ifndef AIRBRAKE_POLICY_ENABLED
#define AIRBRAKE_POLICY_ENABLED 0
#endif
```

With these defaults, the firmware may compute estimates and telemetry, but it will not command non-idle actuator motion.

### Runtime Gates

The actuator is forced idle unless:

1. actuation is compiled in,
2. the estimator is valid and fresh,
3. the airbrake policy returns a valid command,
4. the safety predicate allows actuation.

Invalid estimator data, stale state, disabled policy, SD faults, or sensor faults are surfaced through telemetry and diagnostics.

---

## Serial Commands

The current command parser supports:

```text
HELP
STATUS
HDR 0|1
SET_SLP <hpa>
CAP_BASELINE
CAL_BASELINE
```

### Command Descriptions

| Command | Purpose |
|---|---|
| `HELP` | Print supported command list |
| `STATUS` | Print compact health/status information |
| `HDR 0` | Disable telemetry header emission |
| `HDR 1` | Enable telemetry header emission and print header |
| `SET_SLP <hpa>` | Set sea-level reference pressure in hPa |
| `CAP_BASELINE` | Capture current valid barometer pressure as baseline |
| `CAL_BASELINE` | Run bounded averaged barometer baseline calibration |

`CAL_BASELINE` is an intentional bounded blocking ground operation and should not be used during flight runtime.

---

## Telemetry and Logging

The firmware emits CSV-style serial telemetry and diagnostics. It also logs synchronized flight/test data to SD when available.

Logged and telemetered fields include:

- barometer validity, pressure, temperature, and altitude,
- IMU validity, acceleration, and angular rate,
- auxiliary acceleration,
- quaternion attitude state,
- vertical acceleration,
- Kalman altitude and vertical velocity,
- covariance terms,
- configuration references,
- warning masks.

SD failure is non-fatal. If SD initialization or runtime writes fail, the logger disables itself and the rest of the firmware continues operating.

---

## Build Notes

This project uses Arduino-style C++ targeting Teensy-class hardware.

### Expected Tooling

- Arduino IDE or Arduino CLI with Teensy board support
- Teensyduino / Teensy board package
- Compatible sensor libraries:
  - `Adafruit_BMP5xx`
  - `BMI088`
  - `LIS2DU12Sensor`
  - `SD`
  - `PWMServo` or `Servo`

### Include Path Note

The repository currently separates source files into `include/`, `src/`, and `utils/`. The build environment must compile the `.cpp` files in both implementation folders and make the header folders available on the include path.

For an Arduino IDE sketch-only workflow, this may require moving files into the sketch folder or configuring a supported build system that understands the current folder layout.

---

## Suggested Verification Checklist

### Static / Compile Verification

- Confirm every `#include` resolves under the selected build system.
- Confirm all `.cpp` files in `src/` and `utils/` are compiled.
- Confirm the selected board target provides required Teensy macros and SD definitions.
- Confirm `ACTUATION_ENABLED` and `AIRBRAKE_POLICY_ENABLED` are intentionally set.

### Bench Verification

- Boot with actuation disabled.
- Confirm `BOOT,BEGIN` and `BOOT,READY` appear on Serial.
- Confirm sensor health lines match connected hardware.
- Confirm `CAL_BASELINE` returns `ACK,CAL_BASELINE,<pressure>` when the barometer is present.
- Confirm telemetry rows preserve a stable CSV field order.
- Confirm SD logging creates unique `LOG###.CSV` files when SD is available.

### Estimator Verification

- At rest, confirm quaternion values remain finite and normalized.
- Confirm vertical acceleration is finite after valid IMU and attitude updates.
- Confirm the Kalman estimator seeds only after valid barometric altitude exists.
- Confirm estimator altitude remains near zero after baseline calibration at rest.
- Confirm covariance terms remain finite.

### Actuation Verification

- With default compile-time safety settings, confirm the actuator remains idle.
- Enable actuation only for controlled bench testing.
- Confirm invalid estimator state forces idle.
- Confirm disabled or invalid policy output forces idle.

---

## Current Development Status

This repository represents a modular embedded avionics research scaffold. It is suitable for architecture review, bench testing, estimator validation, telemetry/logging development, and future airbrake-control policy work.

Before flight use, the system should undergo:

- full compile verification in the target build system,
- hardware-in-the-loop bench testing,
- sensor calibration validation,
- estimator tuning from logged data,
- actuator range and failsafe validation,
- simulation-based airbrake policy validation,
- documented preflight and postflight review procedures.

---

## Safety Notice

This software is experimental research firmware for aerospace systems. It is not a certified flight controller. Any use in a flight article must be preceded by independent review, ground testing, simulation, and compliance with all applicable range safety and team procedures.

---

## Author / Organization

Developed for the MASA Dearborn airbrakes / avionics effort.

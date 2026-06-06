# Caelum Sufflamen

### Deterministic Flight Software, Avionics, and State Estimation Framework

Caelum Sufflamen is a flight-computer and avionics software platform developed to explore deterministic embedded systems, state estimation, telemetry, and safety-oriented flight architectures for amateur aerospace applications.

The project combines real-time sensor acquisition, Kalman-filter-based state estimation, onboard data logging, telemetry generation, and health monitoring into a unified avionics framework designed around reliability, observability, and reproducibility.

---

## Project Objectives

The primary goals of the project were:

* Develop a deterministic flight-software architecture.
* Implement real-time multi-sensor data acquisition.
* Explore flight-state estimation using Kalman filtering.
* Design a telemetry and diagnostics framework.
* Build a robust SD-card flight logging pipeline.
* Investigate safety-oriented avionics design principles.
* Create a platform suitable for future control-law and airbrake research.

---

## System Architecture

### Sensor Suite

| Sensor               | Purpose                                     |
| -------------------- | ------------------------------------------- |
| BMP5xx               | Barometric pressure and altitude estimation |
| BMI088 Accelerometer | Linear acceleration measurement             |
| BMI088 Gyroscope     | Angular rate measurement                    |
| LIS2DU12             | Auxiliary acceleration measurement          |

The software continuously acquires data from all sensors and produces synchronized state information at fixed update rates.

---

## State Estimation

A two-state Kalman filter estimates:

[
x =
\begin{bmatrix}
h \
v
\end{bmatrix}
]

where:

* (h) = altitude
* (v) = vertical velocity

The estimator fuses:

* Barometric altitude measurements
* Vertical acceleration measurements derived from IMU data

Features include:

* Prediction/update architecture
* Covariance propagation
* Real-time innovation correction
* Relative-altitude reference management
* Estimator confidence tracking

---

## Avionics Features

### Deterministic Scheduling

The flight software operates using fixed-rate execution loops:

* Main control loop: 50 Hz
* Telemetry output: 10 Hz
* Diagnostics output: 1 Hz
* SD logging: 50 Hz

This architecture ensures predictable execution timing and simplifies system validation.

### Health Monitoring

Continuous monitoring of:

* Sensor initialization status
* Data validity
* Sensor freshness
* Estimator validity
* SD-card health
* Logging status

A consolidated warning-mask system provides rapid fault visibility.

### Telemetry

Real-time telemetry includes:

* Altitude
* Pressure
* Temperature
* Acceleration
* Angular velocity
* Vertical velocity
* Estimator state
* Warning flags
* Configuration state

The telemetry interface is designed for both machine parsing and live operator monitoring.

---

## Flight Data Logging

The onboard logger automatically:

* Creates unique log files
* Records synchronized sensor data
* Logs estimator states
* Stores covariance information
* Periodically flushes data to reduce loss risk

Recorded datasets support post-flight analysis, estimator tuning, and validation.

---

## Visualization and Diagnostics

The software includes multiple real-time plotting modes:

### Overview Mode

Flight-state overview and health indicators.

### IMU Mode

Raw inertial sensor monitoring.

### Apogee Mode

Altitude, vertical velocity, residuals, and projected apogee.

### Unified Mode

Full system visualization for development and tuning.

---

## Command Interface

Runtime configuration is supported through a serial command console.

Available commands include:

```text
HELP
STATUS
HDR 0|1
PLOT OFF|OVERVIEW|IMU|APOGEE|UNIFIED
SET_SLP <hpa>
CAP_BASELINE
```

This interface enables live debugging and estimator tuning without recompilation.

---

## Engineering Focus

This project emphasizes:

* Embedded systems engineering
* Aerospace avionics
* Sensor fusion
* Kalman filtering
* State estimation
* Real-time software
* Safety-oriented architectures
* Telemetry systems
* Flight-data analysis

Rather than focusing solely on flight functionality, the project investigates how rigorous software architecture, deterministic execution, and mathematical estimation techniques can improve the reliability and observability of embedded aerospace systems.

---

## Technologies

* C++
* Arduino Framework
* BMP5xx Sensor Suite
* BMI088 IMU
* LIS2DU12 Accelerometer
* SD Storage Interface
* Kalman Filtering
* Serial Telemetry
* Embedded State Estimation

---

## Future Development

Planned extensions include:

* Airbrake deployment control
* Formal flight-state machine integration
* Redundant sensor fusion
* Magnetometer-based heading estimation
* GPS integration
* Wireless telemetry
* Adaptive filtering techniques
* Advanced flight-event detection
* Closed-loop control algorithms

---

## Author

**David Richardson**

Computer Engineering Student
University of Michigan–Dearborn

Focused on FPGA systems, embedded software, computational physics, state estimation, and aerospace-oriented engineering systems.

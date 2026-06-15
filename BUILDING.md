# Building and Flashing

This repository targets `Teensy 4.1` and now defines one canonical Arduino CLI build path for the current split `include/`, `src/`, and `utils/` layout.

## Board and FQBN

- Board: `Teensy 4.1`
- Arduino CLI FQBN: `teensy:avr:teensy41`

## Required toolchain surfaces

- `arduino-cli`
- a Teensy board package that provides `teensy:avr:teensy41`
- sensor libraries matching these headers:
  - `Adafruit_BMP5xx.h`
  - `Adafruit_Sensor.h`
  - `BMI088.h`
  - `LIS2DU12Sensor.h`

## Canonical build command

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\teensy41_arduino_cli.ps1 -ArduinoCli arduino-cli
```

The wrapper stages a normalized sketch under `.build/teensy41/staged_sketch/` by:

1. copying `CaelumSufflamen.ino` into the staged sketch root
2. copying headers from `include/` and `utils/` into the staged sketch root
3. copying implementation files from `src/` and `utils/` into `staged_sketch/src/`
4. invoking `arduino-cli compile --fqbn teensy:avr:teensy41`

This keeps the source tree reviewable while still giving the repository one explicit build entrypoint.

## Canonical upload command

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\teensy41_arduino_cli.ps1 -ArduinoCli arduino-cli -Upload -Port COM7
```

Replace `COM7` with the port reported for the connected Teensy by `arduino-cli board list`.

## Outputs

- Staged sketch: `.build/teensy41/staged_sketch/`
- Compiled binaries: `.build/teensy41/output/`

## Current limitations

- exact Teensy core version is still unpinned
- exact library versions are still unpinned
- the wrapper assumes `arduino-cli` can already resolve the Teensy platform and required libraries
- this is a canonical build and flash workflow, not a claim of cross-machine bit-for-bit reproducibility

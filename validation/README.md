# Empirical Validation Workflow

This directory is the committed home for flight-log-based aerodynamic identification and policy validation artifacts. It may contain real Teensy 4.1 SD logs and generated result summaries, but each artifact must state its provenance clearly.

## Goal

The control law in `src/airbrake_policy.cpp` depends on two aerodynamic coefficients in `utils/config.h`:

- `POLICY_CDA_BODY_M2`
- `POLICY_CDA_BRAKE_M2`

The current coefficients remain placeholders. The previous-year data under `flight data/` has been audited, but it does not contain enough information to identify the current airbrake-policy drag coefficients. This workflow gives the repository a repeatable place to replace the placeholders with real flight evidence when available.

## Directory Contract

```text
validation/
|- README.md
|- data/
|  `- .gitkeep
`- results/
   `- .gitkeep
```

- `validation/data/` is for raw or minimally processed SD log captures that are safe to commit.
- `validation/results/` is for machine-readable and human-readable summaries produced from those logs.
- `flight data/` currently contains previous-year logs used by `validation/results/previous_year_flight_data_audit.json`.

## Required Input

The fitter uses the SD logger schema already emitted by `utils/sd_logger.cpp`. It expects these columns:

- `t_us`
- `phase`
- `est_h`
- `est_v`
- `policy_cmd`
- `policy_valid`

The current fitter assumes:

- `est_h` and `est_v` are the best available vertical state for the flight segment under review.
- `phase` uses the numeric enum values documented in the firmware.
- actual apogee can be approximated from the maximum logged estimator altitude in the active flight window.
- the policy drag model remains `k(u) = rho * (CDA_body + u * CDA_brake) / (2m)`.
- local density is treated as constant over the fitted segment.

## Recommended Workflow

1. Copy one or more SD logs into `validation/data/<flight_id>/`.
2. Run the host-side fitter against those logs.
3. Review the recommended `POLICY_CDA_BODY_M2` and `POLICY_CDA_BRAKE_M2`.
4. Compare current-constant prediction error against fitted-constant prediction error.
5. Update `utils/config.h` only when the fit is physically credible and repeatable across more than one flight.
6. Commit the raw log subset, the generated JSON summary, and any supporting notes in the same change.

## Example Command

```powershell
python tests/host/policy_aero_empirical_fit.py `
  validation/data/flight_001/LOG000.CSV `
  --json-out validation/results/flight_001_policy_fit.json
```

For multiple logs:

```powershell
python tests/host/policy_aero_empirical_fit.py `
  validation/data/flight_001/LOG000.CSV `
  validation/data/flight_002/LOG003.CSV `
  --json-out validation/results/multi_flight_policy_fit.json
```

Previous-year data audit:

```powershell
python tests/host/audit_previous_year_flight_data.py `
  --data-dir "flight data" `
  --json-out validation/results/previous_year_flight_data_audit.json
```

## What To Commit

For each empirical constants update, commit at least:

- the source SD log or curated subset used for fitting
- the JSON summary emitted by `tests/host/policy_aero_empirical_fit.py`
- the resulting constant change in `utils/config.h`
- a short note describing vehicle configuration, mass, atmospheric assumption, and any anomalies

## Acceptance Checklist

Before replacing placeholder constants, confirm:

- the fit used rows from real `COAST` or `BRAKE` segments
- the estimator remained valid through the fitted region
- the fitted constants are positive and physically plausible
- multiple logs give comparable recommendations when real flight data is available
- fitted prediction bias and RMSE improve over the current constants
- the updated constants are traceable to committed artifacts in this directory

## Current Status

This repository now contains previous-year CSV logs under `flight data/` and an audit result under `validation/results/`. The audit finds that these logs cannot identify the current aerodynamic constants because they lack deployment command state and observed coast-through-apogee data.
